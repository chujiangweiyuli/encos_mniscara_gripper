import asyncio
import json
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import serial

from zlgcan_interface import ZLGCANManager

_BASE = Path(__file__).resolve().parent
JOINT_MODULE_CTRL_CONTROLLER_CONFIG = _BASE / "controller_config.json"
JOINT_MODULE_CTRL_MODULE_CONFIG = _BASE / "module_config.json"


class EncosCanController():
    def __init__(self, com_port, controller_name):
        self.controller_name = controller_name
        # 获取关节模组控制参数
        with open(JOINT_MODULE_CTRL_CONTROLLER_CONFIG, 'r') as f_ctrl_conf:
            controller_data = json.load(f_ctrl_conf).get(controller_name, {})
            module_name = controller_data.get("module_name")
            com_protocol = controller_data.get("com_protocol")
            if com_protocol == "CAN":
                self.com_port = com_port
                self.can_id = controller_data.get("can_id")
                self.baudrate = controller_data.get("baudrate")
            self.movement_range = controller_data.get("movement_range")
            self.init_config = controller_data.get("initialization")
            self.acc_val = self.init_config["acceleration"]

        # 获取关节模组电机参数
        with open(JOINT_MODULE_CTRL_MODULE_CONFIG, 'r') as f_mod_conf:
            module_data = json.load(f_mod_conf).get(module_name, {})
            self.gear_rate = module_data.get("gear_ratio")
            self.encoder_resolution = module_data.get("encoder_resolution")
            self.rated_speed = module_data.get("rated_speed")
            self.rated_speed = 6 * self.rated_speed  # Convert to DPS (Degrees Per Second)
            self.peak_speed = module_data.get("peak_speed")
            self.peak_speed = 6 * self.peak_speed  # Convert to DPS (Degrees Per Second)
            self.ctrl_frequency = module_data.get("ctrl_frequency")  # Default to 1000 Hz if not specified

        brand_name = module_data.get("brand")
        self.brand_name = brand_name
        self.com_protocol = com_protocol
        print(f"Initialized Encos Can Controller for {controller_name} on port {self.com_port} with CAN ID {self.can_id}.")

        if self.com_port.startswith("ZLG"):
            device_serial = self.com_port.split('_')[1] # CAN设备序列号
            self.can_interface = ZLGCANManager.get(device_serial)
            self.can_index = int(self.com_port.split('_')[2])  # CAN通道索引号
            print(f"Using ZLG CAN interface for controller {controller_name}.")
        else:
            raise ValueError(f"Invalid COM port: {self.com_port}")

        self.ENABLE = None

    # 初始化
    def initialize(self):
        self.set_acceleration(self.acc_val)
        print(f"<Initialization> Set acceleration to {self.init_config['acceleration']} rad/s².")
        if self.init_config["up_enable"]:
            self.up_enable_motor()
            print("<Initialization> Motor up-enabled.")
        else:
            self.down_enable_motor()
            print("<Initialization> Motor down-enabled.")

    # 获取超时保护时间
    def get_timeout_protection_time(self):
        """
        :指令格式:
        uint3 - 111 默认电机模式
        uint5 - 11111 （无效位）
        uint8 - 0001 1111 (超时保护时间查询)

        :return:
        超时保护时间，单位 ms
        """
        try:
            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=[0xff, 0x1f],
                can_index=self.can_index
            )
            if success:
                valid_data = response[:4]
                tp_time = int.from_bytes(valid_data[2:], byteorder='big', signed=False)
                print(f"Timeout protection time: {tp_time} ms")
                return {
                    "success": True,
                    "content": tp_time
                }
            else:
                print(f"Failed to get timeout protection time")
                return {
                    "success": False,
                    "content": "Failed to get timeout protection time"
                }
        except Exception as e:
            print(f"Error occurred while getting timeout protection time: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

    # 获取当前位置
    def get_instantaneous_position(self):
        """
        :指令格式:
        uint3 - 111 默认电机模式
        uint5 - 11111 （无效位）
        uint8 - 0000 0001 （位置查询）

        :return:
        当前位置，单位 °
        """
        try:
            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=[0xff, 0x01],
                can_index=self.can_index
            )
            if success:
                valid_data = response[:6]
                position = struct.unpack('>f', valid_data[2:])[0]
                print(f"Current instantaneous position: {position} °")
                return {
                    "success": True,
                    "content": position
                }
            else:
                print(f"Failed to get instantaneous position")
                return {
                    "success": False,
                    "content": "Failed to get instantaneous position"
                }
        except Exception as e:
            print(f"Error occurred while getting instantaneous position: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }
        
    # 获取当前速度
    def get_instantaneous_speed(self, quiet: bool = False):
        try:
            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=[0xff, 0x02],
                can_index=self.can_index
            )
            if success:
                valid_data = response[:6]
                speed = struct.unpack('>f', valid_data[2:])[0]
                if not quiet:
                    print(f"Current instantaneous speed: {speed} RPM")
                return {
                    "success": True,
                    "content": speed
                }
            else:
                print(f"Failed to get instantaneous speed")
                return {
                    "success": False,
                    "content": "Failed to get instantaneous speed"
                }
        except Exception as e:
            print(f"Error occurred while getting instantaneous speed: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

    # 获取当前相电流
    def get_instantaneous_current(self):
        """
        :指令格式:
        uint3 - 111 默认电机模式
        uint5 - 11111 （无效位）
        uint8 - 0000 0011 （相电流查询）

        :return:
        当前相电流，单位 A
        """
        try:
            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=[0xff, 0x03],
                can_index=self.can_index
            )
            if success:
                valid_data = response[:6]
                current = struct.unpack('>f', valid_data[2:])[0]
                print(f"Current instantaneous current: {current} A")
                return {
                    "success": True,
                    "content": current
                }
            else:
                print(f"Failed to get instantaneous current")
                return {
                    "success": False,
                    "content": "Failed to get instantaneous current"
                }
        except Exception as e:
            print(f"Error occurred while getting instantaneous current: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

    # 获取当前加速度
    def get_acceleration(self):
        """
        :指令格式:
        uint3 - 111 默认电机模式
        uint5 - 11111 （无效位）
        uint8 - 0000 0101 （加速度查询）

        :return:
        角加速度值，单位 rad/s²
        """
        try:
            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=[0xff, 0x05],
                can_index=self.can_index
            )
            if success:
                valid_data = response[:4]
                acceleration = int.from_bytes(valid_data[2:], byteorder='big', signed=False) / 100
                print(f"Current acceleration: {acceleration} rad/s²")
                return {
                    "success": True,
                    "content": acceleration
                }
            else:
                print(f"Failed to get acceleration")
                return {
                    "success": False,
                    "content": "Failed to get acceleration"
                }
        except Exception as e:
            print(f"Error occurred while getting acceleration: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

    @staticmethod
    def _extract_pid_payload_6(response: bytes) -> Optional[bytes]:
        """从应答中取出 6 字节 PID 载荷：优先与位置等查询一致跳过前 2 字节，否则使用首 6 字节。"""
        if len(response) >= 8:
            return bytes(response[2:8])
        if len(response) >= 6:
            return bytes(response[:6])
        return None

    # 获取电流环 KP、KI
    def get_current_loop_pid(self):
        """
        发送 [0xff, 0x32]，返回 6 个有效字节：前 3 字节中 uint16 小端 /10000 为 KP，
        后 3 字节中 int16 小端 /10 为 KI（每组前 2 字节为整数，第 3 字节预留）。
        """
        try:
            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=[0xFF, 32],
                can_index=self.can_index,
            )
            if not success:
                return {"success": False, "content": "Failed to get current loop PID"}
            payload = self._extract_pid_payload_6(response)
            if payload is None:
                return {"success": False, "content": "Response too short for current loop PID"}
            kp = int.from_bytes(payload[2:4], byteorder="big", signed=False) / 10000.0
            ki = int.from_bytes(payload[4:6], byteorder="big", signed=False) / 10.0
            out = {"kp": kp, "ki": ki}
            print(f"Current loop PID: KP={kp}, KI={ki}")
            return {"success": True, "content": out}
        except Exception as e:
            print(f"Error occurred while getting current loop PID: {e}")
            return {"success": False, "content": f"Error occurred: {e}"}

    # 获取速度环 KP、KI
    def get_speed_loop_pid(self):
        """
        发送 [0xff, 0x33]，返回 6 字节：前/后各 3 字节中 uint16 小端 /100000 为 KP、KI。
        """
        try:
            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=[0xFF, 33],
                can_index=self.can_index,
            )
            if not success:
                return {"success": False, "content": "Failed to get speed loop PID"}
            payload = self._extract_pid_payload_6(response)
            if payload is None:
                return {"success": False, "content": "Response too short for speed loop PID"}
            kp = int.from_bytes(payload[2:4], byteorder="big", signed=False) / 100000.0
            ki = int.from_bytes(payload[4:6], byteorder="big", signed=False) / 100000.0
            out = {"kp": kp, "ki": ki}
            print(f"Speed loop PID: KP={kp}, KI={ki}")
            return {"success": True, "content": out}
        except Exception as e:
            print(f"Error occurred while getting speed loop PID: {e}")
            return {"success": False, "content": f"Error occurred: {e}"}

    # 获取位置环 KP、KD
    def get_position_loop_pid(self):
        """
        发送 [0xff, 0x34]，返回 6 字节：前/后各 3 字节中 uint16 小端 /100000 为 KP、KD。
        """
        try:
            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=[0xFF, 34],
                can_index=self.can_index,
            )
            if not success:
                return {"success": False, "content": "Failed to get position loop PID"}
            payload = self._extract_pid_payload_6(response)
            if payload is None:
                return {"success": False, "content": "Response too short for position loop PID"}
            kp = int.from_bytes(payload[2:4], byteorder="big", signed=False) / 100000.0
            kd = int.from_bytes(payload[4:6], byteorder="big", signed=False) / 100000.0
            out = {"kp": kp, "kd": kd}
            print(f"Position loop PID: KP={kp}, KD={kd}")
            return {"success": True, "content": out}
        except Exception as e:
            print(f"Error occurred while getting position loop PID: {e}")
            return {"success": False, "content": f"Error occurred: {e}"}

    def get_three_loop_pid(self):
        """依次读取电流环、速度环、位置环 PID，合成一个结果。"""
        cur = self.get_current_loop_pid()
        spd = self.get_speed_loop_pid()
        pos = self.get_position_loop_pid()
        ok = cur["success"] and spd["success"] and pos["success"]
        content = {
            "current": cur.get("content"),
            "speed": spd.get("content"),
            "position": pos.get("content"),
        }
        return {"success": ok, "content": content}

    @staticmethod
    def _pid_scaled_u16_be(value: float, scale: float) -> bytes:
        """物理量乘 scale 后取整，按 uint16 大端编码（负数按 16 位补码写入）。"""
        v = int(round(value * scale)) & 0xFFFF
        return v.to_bytes(2, byteorder="big", signed=False)

    # 配置电流环 PI（KP、KI）
    def set_current_loop_pid(self, kp: float, ki: float):
        """
        6 字节指令：0xc0 0x0c | KP(uint16 大端, kp×10000) | KI(uint16 大端, ki×10)
        """
        try:
            b = (
                bytes([0xC0, 0x0C])
                + self._pid_scaled_u16_be(kp, 10000.0)
                + self._pid_scaled_u16_be(ki, 10.0)
            )
            success = self.can_interface.transmit(
                can_id=self.can_id,
                data=b,
                can_index=self.can_index,
            )
            if success:
                print(f"Set current loop PID: KP={kp}, KI={ki}")
                return {"success": True, "content": True}
            return {"success": False, "content": "Failed to set current loop PID"}
        except Exception as e:
            print(f"Error occurred while setting current loop PID: {e}")
            return {"success": False, "content": f"Error occurred: {e}"}

    # 配置速度环 PI（KP、KI）
    def set_speed_loop_pid(self, kp: float, ki: float):
        """
        6 字节指令：0xc0 0x0d | KP(uint16 大端, ×100000) | KI(uint16 大端, ×100000)
        """
        try:
            b = (
                bytes([0xC0, 0x0D])
                + self._pid_scaled_u16_be(kp, 100000.0)
                + self._pid_scaled_u16_be(ki, 100000.0)
            )
            success = self.can_interface.transmit(
                can_id=self.can_id,
                data=b,
                can_index=self.can_index,
            )
            if success:
                print(f"Set speed loop PID: KP={kp}, KI={ki}")
                return {"success": True, "content": True}
            return {"success": False, "content": "Failed to set speed loop PID"}
        except Exception as e:
            print(f"Error occurred while setting speed loop PID: {e}")
            return {"success": False, "content": f"Error occurred: {e}"}

    # 配置位置环 PD（KP、KD）
    def set_position_loop_pid(self, kp: float, kd: float):
        """
        6 字节指令：0xc0 0x0e | KP(uint16 大端, ×100000) | KD(uint16 大端, ×100000)
        """
        try:
            b = (
                bytes([0xC0, 0x0E])
                + self._pid_scaled_u16_be(kp, 100000.0)
                + self._pid_scaled_u16_be(kd, 100000.0)
            )
            success = self.can_interface.transmit(
                can_id=self.can_id,
                data=b,
                can_index=self.can_index,
            )
            if success:
                print(f"Set position loop PID: KP={kp}, KD={kd}")
                return {"success": True, "content": True}
            return {"success": False, "content": "Failed to set position loop PID"}
        except Exception as e:
            print(f"Error occurred while setting position loop PID: {e}")
            return {"success": False, "content": f"Error occurred: {e}"}

    # 设置零点
    def set_zero_point(self):
        """
        :指令格式:
        byte0 - CAN ID (HIGH)
        byte1 - CAN ID (LOW)
        byte2 - 0000 0000 (默认字节)
        byte3 - 0000 0011 (默认字节)

        :return:
        True or False
        """
        try:
            can_id_str = format(self.can_id, '016b')
            rest_bit = '0000000000000011'

            value_str = can_id_str + rest_bit
            can_data = int(value_str, 2).to_bytes(4, byteorder='big')
            
            success, response = self.can_interface.transceive(
                can_id=0x07ff,
                data=can_data,
                can_index=self.can_index
            )
            if success:
                valid_data = response[:4]
                if valid_data.hex()[4:] == "0103":
                    print(f"Set zero point successfully")
                    return {
                        "success": True,
                        "content": True
                    }
                elif valid_data.hex()[4:] == "0100":
                    print(f"Failed to set zero point")
                    return {
                        "success": True,
                        "content": False
                    }
                else:
                    print(f"Received unknown response when setting zero point")
                    return {
                        "success": False,
                        "content": f"Received unknown response"
                    }
            else:
                print(f"Failed to set zero point")
                return {
                    "success": False,
                    "content": "Failed to set zero point"
                }
        except Exception as e:
            print(f"Error occurred while setting zero point: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

    # 位置控制
    def set_target_position(self, pos_deg, spd_val, cur_thr = 10):
        """
        pos_deg : 目标位置（单位：度）
        spd_val : 目标速度（单位：RPM）
        cur_thr : 目标电流阈值（单位：A）

        :return:
        True or False
        """
        # 如果没有上使能，那无法发送运动指令
        if not self.ENABLE:
            return {"success": False,
                    "content": f"The motor is not enabled. Please call up_enable_motor() before sending commands."}

        try:
            ack_val = 0 # 报文返回状态（0：不返回，1：返回报文类型1，2：返回报文类型2，3：返回报文类型3）

            motor_mode_str = '001'
            pos_deg_f = struct.pack('>f', pos_deg)
            pos_deg_str = ''.join(f'{byte:08b}' for byte in pos_deg_f)
            spd_val_str = format(spd_val*10, '015b')
            cur_thr_str = format(cur_thr*10, '012b')
            ack_val_str = format(ack_val, '02b')

            if pos_deg is None:
                return self.UNSUPPORTED_OP_MSG

            value_str = motor_mode_str + pos_deg_str + spd_val_str + cur_thr_str + ack_val_str
            can_data = int(value_str, 2).to_bytes(8, byteorder='big')
            
            success = self.can_interface.transmit(
                can_id=self.can_id,
                data=can_data,
                can_index=self.can_index
            )
            if success:
                print(f"Set target position to {pos_deg} ° at {spd_val} RPM")
                return {
                    "success": True,
                    "content": True
                }
            else:
                print(f"Failed to send target position command")
                return {
                    "success": False,
                    "content": "Failed to send target position command"
                }
        except Exception as e:
            print(f"Error occurred while setting target position: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

    # 速度控制
    def set_target_speed(self, spd_val, cur_thr = 10):
        """
        spd_val : 目标速度（单位：RPM）
        cur_thr : 目标电流阈值（单位：A）

        :return:
        True or False
        """
        # 如果没有上使能，那无法发送运动指令
        if not self.ENABLE:
            return {"success": False,
                    "content": f"The motor is not enabled. Please call up_enable_motor() before sending commands."}

        try:
            ack_val = 0 # 报文返回状态（0：不返回，1：返回报文类型1，2：返回报文类型2，3：返回报文类型3）

            motor_mode_str = '010'
            reserve = '000'
            ack_val_str = format(ack_val, '02b')
            spd_val_f = struct.pack('>f', spd_val)
            spd_val_str = ''.join(f'{byte:08b}' for byte in spd_val_f)
            cur_thr_str = format(cur_thr*10, '016b')

            value_str = motor_mode_str + reserve + ack_val_str + spd_val_str + cur_thr_str
            can_data = int(value_str, 2).to_bytes(7, byteorder='big')

            success = self.can_interface.transmit(
                can_id=self.can_id,
                data=can_data,
                can_index=self.can_index
            )

            if success:
                print(f"Set target speed to {spd_val} RPM with current threshold {cur_thr} A")
                return {
                    "success": True,
                    "content": True
                }
            else:
                print(f"Failed to send target speed command")
                return {
                    "success": False,
                    "content": "Failed to send target speed command"
                }
        except Exception as e:
            print(f"Error occurred while setting target speed: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

    # 力矩控制
    def set_target_torque(self, torque_val):
        """
        torque_val : 目标力矩（浮点，协议侧为 int16，数值 = 力矩 × 100，大端）

        报文共 3 字节：第 1 字节固定 0b01100100（0x64）；第 2、3 字节为
        int(round(torque_val * 100)) 的 int16 **大端** 编码。

        :return:
        与 set_target_position / set_target_speed 相同结构的 dict
        """
        if not self.ENABLE:
            return {
                "success": False,
                "content": (
                    "The motor is not enabled. Please call up_enable_motor() "
                    "before sending commands."
                ),
            }

        try:
            scaled = int(round(float(torque_val) * 100))
            scaled = max(-32768, min(32767, scaled))
            # 首字节 0x64，后两字节 int16 大端
            can_data = struct.pack(">Bh", 0x64, scaled)

            success = self.can_interface.transmit(
                can_id=self.can_id,
                data=can_data,
                can_index=self.can_index,
            )
            if success:
                print(f"Set target torque to {torque_val} (payload int16={scaled})")
                return {"success": True, "content": True}
            print("Failed to send target torque command")
            return {
                "success": False,
                "content": "Failed to send target torque command",
            }
        except Exception as e:
            print(f"Error occurred while setting target torque: {e}")
            return {"success": False, "content": f"Error occurred: {e}"}

    # 设置加速度，断电失忆
    def set_acceleration(self, acc_val):
        """
        acc_val : 目标加速度（单位：rad/s）

        :return:
        返回的报文内容
        """
        try:
            motor_mode = '110'
            reserve = '000'
            ack_val = 0 # 报文返回状态（0：不返回，1：返回报文类型4）
            ack_val_str = format(ack_val, '02b')
            config_code = '00000001'
            acc_val_str = format(acc_val*100, '016b')

            value_str = motor_mode + reserve + ack_val_str + config_code + acc_val_str
            can_data = int(value_str, 2).to_bytes(4, byteorder='big')
            
            success = self.can_interface.transmit(
                can_id=self.can_id,
                data=can_data,
                can_index=self.can_index
            )
            if success:
                acc_get = self.get_acceleration().get("content")
                if acc_get == acc_val:
                    print(f"Set acceleration to {acc_val} rad/s² successfully")
                    return {
                        "success": True,
                        "content": True
                    }
                return {
                    "success": True,
                    "content": f"Set acceleration to {acc_val} rad/s², but got {acc_get} rad/s²"
                }
            else:
                print(f"Failed to set acceleration")
                return {
                    "success": False,
                    "content": "Failed to set acceleration"
                }
        except Exception as e:
            print(f"Error occurred while setting acceleration: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

    # 间接下使能
    def down_enable_motor(self):
        """
        uint3 - 110 默认电机模式
        uint5 - 00000 （预留位）
        uint8 - 0000 1011 （配置代码）
        uint16 - 0000 0000 | 0000 0010 （配置数值，1ms心跳保护延时）
        """
        try:
            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=[0xc0, 0x0b, 0x00, 0x01],
                can_index=self.can_index
            )
            if success:
                valid_data = response[:5]
                self.ENABLE = False
                if valid_data.hex() == "fffe0b0001":
                    print(f"Down enabled motor successfully")
                    return {
                        "success": True,
                        "content": True
                    }
                else:
                    print(f"Failed to down enable motor")
                    return {
                        "success": True,
                        "content": False
                    }
            else:
                print(f"Failed to down enable motor")
                return {
                    "success": False,
                    "content": "Failed to down enable motor"
                }
        except Exception as e:
            print(f"Error occurred while down enabling motor: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

    # 间接上使能
    def up_enable_motor(self):
        """
        uint3 - 110 默认电机模式
        uint5 - 00000 （预留位）
        uint8 - 0000 1011 （配置代码）
        uint16 - 0000 00000 | 0000 0000 （配置数值，0ms心跳保护延时）
        """
        try:
            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=[0xc0, 0x0b, 0x00, 0x00],
                can_index=self.can_index
            )
            if success:
                valid_data = response[:5]
                self.ENABLE = True
                # 需要重新发送运动指令才能进入上使能状态
                if valid_data.hex() == "fffe0b0000":
                    current_pos = self.get_instantaneous_position().get("content")
                    self.set_target_position(pos_deg=current_pos, spd_val=30)
                    print(f"Up enabled motor successfully")
                    return {
                        "success": True,
                        "content": True
                    }
                else:
                    print(f"Failed to up enable motor")
                    return {
                        "success": True,
                        "content": False
                    }
            else:
                print(f"Failed to up enable motor")
                return {
                    "success": False,
                    "content": "Failed to up enable motor"
                }
        except Exception as e:
            print(f"Error occurred while up enabling motor: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

    # 停止关节模组
    def stop_motor(self, brake_on: bool):
        """
        :指令格式:
        uint3 - 100 默认电机模式
        uint5 - 00000（预留位）
        uint8 - 0000 0001（期望打开刹车）/ 0000 0000（期望关闭刹车）
        brake_on : True 打开刹车，False 关闭刹车

        :return:
        刹车状态，True表示刹车打开，False表示刹车关闭
        """
        # 目前关节模组并不支持刹车功能
        return {
            "success": False,
            "content": "This joint module does not support brake functionality"
        }
        try:
            motor_mode = '100'
            reserve = '00000'
            brake_on_str = format(int(brake_on), '08b')

            value_str = motor_mode + reserve + brake_on_str
            can_data = int(value_str, 2).to_bytes(4, byteorder='big')

            success, response = self.can_interface.transceive(
                can_id=self.can_id,
                data=can_data,
                can_index=self.can_index
            )
            if success and response:
                valid_data = response[:2]
                if valid_data.hex() == "b201" and brake_on:
                    print(f"Set brake status to {brake_on} successfully")
                    return {
                        "success": True,
                        "content": True
                    }
                elif valid_data.hex() == "b200" and not brake_on:
                    print(f"Failed to set brake status to {brake_on} ")
                    return {
                        "success": True,
                        "content": False
                    }
                else:
                    print(f"Received unexpected response when setting brake status")
                    return {
                        "success": True,
                        "content": f"Received unexpected response"
                    }
            else:
                print(f"Failed to set brake status")
                return {
                    "success": False,
                    "content": "Failed to set brake status"
                }
        except Exception as e:
            print(f"Error occurred while setting brake status: {e}")
            return {
                "success": False,
                "content": f"Error occurred: {e}"
            }

if __name__ == "__main__":
    """
    旋转模组： 0 → 70 → 130 → 0
    """
    SAMPLE_INTERVAL_S = 0.005
    CAPTURE_DURATION_S = 5.0
    MOTION_DELAY_S = 0.5

    controller = EncosCanController(com_port="ZLG_31F10005727_1", controller_name="test1")
    controller.set_zero_point()
    # time.sleep(1)

    controller.initialize()

    start_time = time.time()
    controller.set_target_position(pos_deg=172, spd_val=200)
    time.sleep(0.3)
    controller.set_target_position(pos_deg=120, spd_val=200)
    input("Press Enter to continue...")
    controller.set_target_position(pos_deg=-53, spd_val=200)
    time.sleep(0.3)
    controller.set_target_position(pos_deg=0, spd_val=200)
    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")

    # rot_controller = EncosCanController(com_port="ZLG_31F100058E9_0", controller_name="test1")
    # grip_controller = EncosCanController(com_port="ZLG_31F100058E9_1", controller_name="test2")
    # print(grip_controller.get_instantaneous_position().get("content"))
    # print(rot_controller.get_instantaneous_position().get("content"))
    # exit()

    # grip_controller.initialize()
    # rot_controller.initialize()

    # time.sleep(3)

    # nums = 3

    # while nums > 0:
    #     # 快速开合
    #     grip_controller.set_target_torque(torque_val=0.3)
    #     time.sleep(0.2)
    #     grip_controller.set_target_position(pos_deg= -7, spd_val=200)
    #     time.sleep(0.08)
    #     grip_controller.set_target_torque(torque_val=0.3)
    #     time.sleep(0.4)

    #     # 前往相机切根位置
    #     rot_controller.set_target_position(pos_deg=70 + 153, spd_val=120)
    #     time.sleep(0.10)
    #     rot_controller.set_target_position(pos_deg=70, spd_val=120)
    #     time.sleep(0.5)

    #     # 前往放置位置
    #     rot_controller.set_target_position(pos_deg=130 + 146, spd_val=120)
    #     time.sleep(0.09)
    #     rot_controller.set_target_position(pos_deg=130, spd_val=120)
    #     time.sleep(0.5)
    #     grip_controller.set_target_position(pos_deg= -7, spd_val=100)
    #     time.sleep(0.3)

    #     # 前往零位
    #     rot_controller.set_target_position(pos_deg=0 - 140, spd_val=120)
    #     time.sleep(0.19)
    #     rot_controller.set_target_position(pos_deg=0, spd_val=120)
    #     time.sleep(1.5)

    #     nums -= 1

    # grip_controller.set_target_torque(torque_val=0.3)
    # time.sleep(0.2)
    # grip_controller.set_target_position(pos_deg= -7, spd_val=200)
    # time.sleep(0.08)
    # grip_controller.set_target_torque(torque_val=0.3)
    # time.sleep(1)

    # grip_controller.set_target_position(pos_deg= -7, spd_val=200)
    # time.sleep(0.5)
    # grip_controller.down_enable_motor()

    # controller.set_current_loop_pid(kp=0.1, ki=1200.0)
    # controller.set_speed_loop_pid(kp=0.006, ki=0.05)
    # controller.set_position_loop_pid(kp=0.01, kd=0.002)    

    # time.sleep(1)

    # print(controller.get_current_loop_pid().get("content"))
    # print(controller.get_speed_loop_pid().get("content"))
    # print(controller.get_position_loop_pid().get("content"))
    """
    Current loop PID: KP=0.1, KI=1200.0
    {'kp': 0.1, 'ki': 1200.0}
    Speed loop PID: KP=0.006, KI=0.05
    {'kp': 0.006, 'ki': 0.05}
    Position loop PID: KP=0.01, KD=0.002
    {'kp': 0.01, 'kd': 0.002}
    """

    exit()


    pos = controller.get_instantaneous_position().get("content")
    print(f"Current position: {pos} °")

    controller.set_acceleration(acc_val=20)

    target_pos = pos - 180

    # (相对开始时刻的时间差 s, 瞬时速度绝对值 RPM)；先采样，0.5s 后再下发运动以记录静止段与运动段
    speed_log: List[Tuple[float, float]] = []
    t0 = time.perf_counter()
    next_sample_t = t0
    motion_issued = False

    while True:
        now = time.perf_counter()
        if now - t0 >= CAPTURE_DURATION_S:
            break
        if now < next_sample_t:
            time.sleep(min(next_sample_t - now, 0.001))
            continue

        if not motion_issued and (now - t0) >= MOTION_DELAY_S:
            motion_issued = True
            controller.set_target_position(pos_deg=target_pos, spd_val=100)

        res = controller.get_instantaneous_speed(quiet=True)
        dt = now - t0
        if res.get("success"):
            spd = res["content"]
            speed_log.append((dt, abs(float(spd))))
        else:
            speed_log.append((dt, float("nan")))

        next_sample_t += SAMPLE_INTERVAL_S
        if next_sample_t < now:
            next_sample_t = now + SAMPLE_INTERVAL_S

    print(
        f"Speed capture done: {len(speed_log)} samples over {CAPTURE_DURATION_S}s "
        f"(interval {SAMPLE_INTERVAL_S * 1000:.0f} ms, motion at {MOTION_DELAY_S}s)."
    )

    ts = [p[0] for p in speed_log]
    vs = [p[1] for p in speed_log]
    _, ax = plt.subplots(figsize=(8, 3))
    ax.plot(
        ts,
        vs,
        linestyle="-",
        linewidth=0.25,
        marker=".",
        markersize=1,
        markeredgewidth=0,
        color="C0",
    )
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("|速度| / RPM")
    ax.grid(True, alpha=0.25, linewidth=0.3)
    ax.tick_params(axis="both", labelsize=8, width=0.4, length=2)
    for spine in ax.spines.values():
        spine.set_linewidth(0.4)
    plt.tight_layout()
    plt.show()