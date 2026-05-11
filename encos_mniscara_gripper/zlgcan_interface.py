from ctypes import *
from pathlib import Path
import os
import threading
from queue import Queue
import asyncio
import time
from dataclasses import dataclass
from typing import ClassVar, Dict

_BASE = Path(__file__).resolve().parent
ZLGCAN_CONTROL_DLL = _BASE / "ControlCAN.dll"
ZLGCAN_CONTROL_SO = _BASE / "libcontrolcan.so"

class ZLGCANInterface:
    # 设备信息
    class VCI_BOARD_INFO(Structure):
        _fields_ = [
            ("hw_Version", c_ushort),
            ("fw_Version", c_ushort),
            ("dr_Version", c_ushort),
            ("in_Version", c_ushort),
            ("irq_Num", c_ushort),
            ("can_Num", c_ubyte),
            ("str_Serial_Num", c_char * 20),
            ("str_hw_Type", c_char * 40),
            ("Reserved", c_ushort * 4),
        ]
    # 初始化参数
    class VCI_INIT_CONFIG(Structure):  
        _fields_ = [
            ("AccCode", c_uint),
            ("AccMask", c_uint),
            ("Reserved", c_uint),
            ("Filter", c_ubyte),
            ("Timing0", c_ubyte),
            ("Timing1", c_ubyte),
            ("Mode", c_ubyte)
        ]
    # 收发帧信息
    class VCI_CAN_OBJ(Structure):  
        _fields_ = [
            ("ID", c_uint),
            ("TimeStamp", c_uint),
            ("TimeFlag", c_ubyte),
            ("SendType", c_ubyte),
            ("RemoteFlag", c_ubyte),
            ("ExternFlag", c_ubyte),
            ("DataLen", c_ubyte),
            ("Data", c_ubyte*8),
            ("Reserved", c_ubyte*3)
        ]

    # 设备参数
    VCI_USBCAN2 = 4
    # 响应参数
    STATUS_OK = 1
    # 初始化参数
    ACC_CODE = 0x00000000
    ACC_MASK = 0xFFFFFFFF
    TIMING_MAP = {
        20000 : (0x18,0x1C),
        40000 : (0x87,0xFF),
        50000 : (0x09,0x1C),
        80000 : (0x83,0xFF),
        100000 : (0x04,0x1C),
        125000 : (0x03,0x1C),
        200000 : (0x81,0xFA),
        250000 : (0x01,0x1C),
        400000 : (0x80,0xFA),
        500000 : (0x00,0x1C),
        666000 : (0x80,0xB6),
        800000 : (0x00,0x16),
        1000000 : (0x00,0x14),
        33330 : (0x09,0x6F),
        66660 : (0x04,0x6F),
        83330 : (0x03,0x6F)}
    FILTER = 1
    MODE = 0

    # 接收参数
    MAX_FRAMES = 2500
    WAIT_TIME_MS = 10

    # 判断当前操作系统类型选择加载的库
    if os.name == 'nt':
        CAN_LIB = windll.LoadLibrary(str(ZLGCAN_CONTROL_DLL)) if ZLGCAN_CONTROL_DLL.exists() else None
        print(f"Loaded ZLG CAN Library from {ZLGCAN_CONTROL_DLL}")
    elif os.name == 'posix':
        CAN_LIB = cdll.LoadLibrary(str(ZLGCAN_CONTROL_SO)) if ZLGCAN_CONTROL_SO.exists() else None
        print(f"Loaded ZLG CAN Library from {ZLGCAN_CONTROL_SO}")
    else:
        CAN_LIB = None
        print("No suitable ZLG CAN library found for this Operating System.")

    def __init__(self, port: str = "ZLG_0", baudrate: int = 1000000):
        index = int(port.split('_')[1])
        self.DEV_INDEX = index
        self.com_port = port
        self.baudrate = baudrate

        # 打开设备
        open_res = self.CAN_LIB.VCI_OpenDevice(self.VCI_USBCAN2, self.DEV_INDEX, 0)
        assert open_res == self.STATUS_OK, f"Failed to open ZLG CAN device {self.DEV_INDEX}."

        # 读取设备序列号
        self.device_serial = self.get_device_serial_number()

        # 连接设备
        self.connect()

    # 获取连接状态
    def get_connection_status(self, can_index):
        ret = self.CAN_LIB.VCI_GetReceiveNum(self.VCI_USBCAN2, self.DEV_INDEX, can_index)
        if ret != -1:
            print(f"ZLG CAN device {self.DEV_INDEX} channel {can_index} connection status: Connected.")
            return True
        else:
            print(f"ZLG CAN device {self.DEV_INDEX} channel {can_index} connection status: Disconnected.")
            return False

    # 连接设备
    def connect(self):
        # 初始化设备
        timing_0, timing_1 = self.TIMING_MAP[self.baudrate]
        vci_initconfig = self.VCI_INIT_CONFIG(self.ACC_CODE, self.ACC_MASK, 0, self.FILTER, timing_0, timing_1, self.MODE)

        # 初始化并启动CAN通道
        try:
            # 初始化CAN通道0
            ret_init = self.CAN_LIB.VCI_InitCAN(self.VCI_USBCAN2, self.DEV_INDEX, 0, byref(vci_initconfig))
            if ret_init == self.STATUS_OK:
                print(f"CAN INDEX 0 initialized successfully.")
            if ret_init != self.STATUS_OK:
                print(f"Failed to initialize CAN INDEX 0.")
            ret_start = self.CAN_LIB.VCI_StartCAN(self.VCI_USBCAN2, self.DEV_INDEX, 0)
            if ret_start == self.STATUS_OK:
                print(f"CAN INDEX 0 started successfully.")
            if ret_start != self.STATUS_OK:
                print(f"Failed to start CAN INDEX 0.")

            # 初始化CAN通道1
            ret_init = self.CAN_LIB.VCI_InitCAN(self.VCI_USBCAN2, self.DEV_INDEX, 1, byref(vci_initconfig))
            if ret_init == self.STATUS_OK:
                print(f"CAN INDEX 1 initialized successfully.")
            if ret_init != self.STATUS_OK:
                print(f"Failed to initialize CAN INDEX 1.")
            ret_start = self.CAN_LIB.VCI_StartCAN(self.VCI_USBCAN2, self.DEV_INDEX, 1)
            if ret_start == self.STATUS_OK:
                print(f"CAN INDEX 1 started successfully.")
            if ret_start != self.STATUS_OK:
                print(f"Failed to start CAN INDEX 1.")

        except Exception as e:
            print(f"Error during CAN initialization: {e}")
            return False

        # 连接后清除缓冲区
        self.clear_buffer(0)
        self.clear_buffer(1)

        return True
    
    # 断开设备
    def disconnect(self):
        for can_index in list(self._recv_threads.keys()):
            self.stop_receive_thread(can_index)

        close_res = self.CAN_LIB.VCI_CloseDevice(self.VCI_USBCAN2, self.DEV_INDEX)
        if close_res == self.STATUS_OK:
            print(f"ZLG CAN device {self.DEV_INDEX} disconnected successfully.")
        else:
            print(f"Failed to disconnect ZLG CAN device {self.DEV_INDEX}.")

    # 重连设备
    def reconnect(self):
        self.connect()
        # print(f"ZLG CAN device {self.DEV_INDEX} reconnected successfully.")

    # 启动设备
    def start_device(self):
        # 后台接收线程相关
        self._recv_queues = {0: Queue(), 1: Queue()}
        self._recv_stop_events = {0: threading.Event(), 1: threading.Event()}
        self._recv_threads = {}

        # 首次创建时启动接收线程
        self.start_receive_thread(0)
        self.start_receive_thread(1)

    # 读取设备唯一序列号
    def get_device_serial_number(self):
        info = self.VCI_BOARD_INFO()
        ret = int(self.CAN_LIB.VCI_ReadBoardInfo(self.VCI_USBCAN2, self.DEV_INDEX, byref(info)))

        if ret == 1:
            return info.str_Serial_Num.decode('utf-8').rstrip('\x00')
        else:
            return None

    # 一次性取出后台线程已经解析好的所有帧，并清空对应队列
    def pop_all_received_frames(self, can_index: int):
        frames = []
        q = self._recv_queues[can_index]

        while not q.empty():
            batch = q.get_nowait()
            frames.extend(batch)

        return frames

    # 接收循环
    def _receive_loop(self, can_index: int):
        obj_array_type = self.VCI_CAN_OBJ * self.MAX_FRAMES
        obj_array = obj_array_type()

        stop_event = self._recv_stop_events[can_index]

        while not stop_event.is_set():
            ret = self.CAN_LIB.VCI_Receive(
                self.VCI_USBCAN2,
                self.DEV_INDEX,
                can_index,
                obj_array,          # 数组本身即可作为指针传入
                self.MAX_FRAMES,
            )

            if ret < 0:
                # 出错
                # print(
                #     f"ZLG CAN device {self.DEV_INDEX} channel {can_index} receive error."
                # )
                continue

            if ret == 0:
                # 本轮没有数据，下一轮继续
                continue

            # 从本次读取的 obj_array[0..ret-1] 中提取并解析
            parsed_frames = []
            for i in range(ret):
                frame = obj_array[i]
                data_len = frame.DataLen
                data_bytes = bytes(frame.Data[:data_len])  # 只取有效长度

                parsed = {
                    "id": frame.ID,
                    "timestamp": frame.TimeStamp,
                    "dlc": data_len,
                    "data": data_bytes,
                    "remote": frame.RemoteFlag,
                    "extern": frame.ExternFlag,
                }
                parsed_frames.append(parsed)

            # 把这批解析好的帧推入队列，供主线程消费
            if parsed_frames:
                self._recv_queues[can_index].put(parsed_frames)

    # 开始指定通道的后台接收线程
    def start_receive_thread(self, can_index: int):
        if can_index in self._recv_threads and self._recv_threads[can_index].is_alive():
            # print(
            #     f"ZLG CAN device {self.DEV_INDEX} channel {can_index} receive thread already running."
            # )
            return

        if can_index not in self._recv_stop_events:
            self._recv_stop_events[can_index] = threading.Event()
            self._recv_queues[can_index] = Queue()

        self._recv_stop_events[can_index].clear()

        t = threading.Thread(
            target=self._receive_loop,
            args=(can_index,),
            daemon=True,
            name=f"ZLGCAN-Recv-{self.DEV_INDEX}-{can_index}",
        )
        self._recv_threads[can_index] = t
        t.start()

        # print(
        #     f"ZLG CAN device {self.DEV_INDEX} channel {can_index} receive thread started."
        # )

    # 停止指定通道的后台接收线程
    def stop_receive_thread(self, can_index: int):
        if can_index not in self._recv_threads:
            return

        self._recv_stop_events[can_index].set()
        t = self._recv_threads[can_index]
        if t.is_alive():
            t.join(timeout=1.0)

        # print(
        #     f"ZLG CAN device {self.DEV_INDEX} channel {can_index} receive thread stopped."
        # )

    # 数据发送
    def transmit(self, can_id, data, can_index, padding = False):
        """
        can_id: 设备 ID
        data_len: 数据长度
        data: 数据内容，类型为列表或元组，长度不超过8
        """
        cmd_data_temp = c_ubyte * 8
        data_len = len(data)

        # 8字节数据填充
        if padding and data_len < 8:
            pad_len = 8 - data_len if data_len < 8 else 0
            data += [0] * pad_len

        cmd = self.VCI_CAN_OBJ()
        cmd.ID = can_id
        cmd.TimeStamp = 0  # 时间戳，默认为0
        cmd.TimeFlag = 0   # 时间戳有效标志，0（无效）
        cmd.SendType = 0   # 发送类型，0（正常发送），1（单次发送）
        cmd.RemoteFlag = 0 # 帧类型，0（数据帧），1（远程帧）
        cmd.ExternFlag = 0 # 扩展类型，0（标准帧），1（扩展帧）
        cmd.DataLen = data_len
        cmd.Data = cmd_data_temp(*data)
        cmd.Reserved = (c_ubyte * 3)(0, 0, 0)

        ret_transmitteed = self.CAN_LIB.VCI_Transmit(self.VCI_USBCAN2, self.DEV_INDEX, can_index, byref(cmd), 1)
        if ret_transmitteed == 1:
            # print(f'ZLG CAN device {self.DEV_INDEX} channel {can_index} transmitted successfully.')
            return True
        if ret_transmitteed == -1:
            # print(f'Failed to transmit on ZLG CAN device {self.DEV_INDEX} channel {can_index}.')
            return False

    # 数据接收
    def receive(self, can_index):
        obj_temp = self.VCI_CAN_OBJ * 1
        obj_inst = obj_temp()
        ret_received = self.CAN_LIB.VCI_Receive(self.VCI_USBCAN2, self.DEV_INDEX, can_index, byref(obj_inst), 1, 0)

        if ret_received > 0:
            recv_data = bytes(obj_inst[0].Data[:])
            # print(f'ZLG CAN device {self.DEV_INDEX} channel {can_index} received data: {recv_data.hex()}.')
            return recv_data
        else:
            # print(f'No data received on ZLG CAN device {self.DEV_INDEX} channel {can_index}.')
            return None

    # 数据收发
    def transceive(self, can_id, data, can_index, padding = False):
        # 先清除缓冲区
        self.clear_buffer(can_index)
        # 发送数据
        trans_res = self.transmit(can_id, data, can_index, padding)
        if trans_res:
            # # 接收数据（非线程）
            # recv_data = self.receive(can_index)
            # 接收数据（线程）
            while True:
                recv_frames = self.pop_all_received_frames(can_index)
                if len(recv_frames) == 0:
                    # 必要等待时间，避免忙轮询
                    time.sleep(0.0005)
                    continue
                data_list = [f["data"] for f in recv_frames]
                recv_data = data_list[-1] if data_list else None
                return True, recv_data
        else:
            return False, None
        
    # 清除缓冲区
    def clear_buffer(self, can_index):
        clear_res = self.CAN_LIB.VCI_ClearBuffer(self.VCI_USBCAN2, self.DEV_INDEX, can_index)
        if clear_res == self.STATUS_OK:
            # print(f"Cleared buffer successfully for ZLG CAN device {self.DEV_INDEX} channel {can_index}.")
            return True
        else:
            # print(f"Failed to clear buffer for ZLG CAN device {self.DEV_INDEX} channel {can_index}.")
            return False


@dataclass(frozen=True)
class ZLGCANManager():
    _registry: ClassVar[Dict[str, "ZLGCANInterface"]] = {}  # serial -> instance
    _index_to_serial: ClassVar[Dict[int, str]] = {}  # device_index -> serial (避免重复打开)
    MAX_DEVICES: ClassVar[int] = 10

    @classmethod
    def get(cls, serial: str) -> "ZLGCANInterface":
        if serial in cls._registry:
            # print(f"ZLG CAN device with serial {serial} already registered, returning cached instance.")
            return cls._registry[serial]
        
        # print(f"ZLG CAN device with serial {serial} not found in registry, scanning devices...")
        for i in range(cls.MAX_DEVICES):
            # 检查该索引是否已经被扫描过
            if i in cls._index_to_serial:
                # 该索引已知，检查是否是目标设备
                if cls._index_to_serial[i] == serial:
                    # 应该已经在 registry 中，但以防万一
                    if serial in cls._registry:
                        return cls._registry[serial]
                # 不是目标设备，跳过此索引
                continue
            
            try:
                inst = ZLGCANInterface(f"ZLG_{i}_0", 1_000_000)
                found_serial = inst.get_device_serial_number()
                
                # 记录索引到序列号的映射
                cls._index_to_serial[i] = found_serial
                
                # 注册并启动设备
                inst.start_device()
                cls._registry[found_serial] = inst
                # print(f"Registered ZLG CAN device with serial: {found_serial} at index {i}")
                
                if found_serial == serial:
                    # 找到目标设备
                    return inst
                # 不是目标设备，继续搜索（但设备已注册，下次不会重复打开）
                
            except Exception as e:
                # print(f"No ZLG CAN device at index {i}: {e}")
                continue
        
        # print(f"ZLG CAN Interface with serial {serial} not found after scanning all devices.")
        raise KeyError(f"ZLG CAN device with serial {serial} not found. Available: {list(cls._registry)}")

    @classmethod
    def all(cls) -> Dict[str, "ZLGCANInterface"]:
        return dict(cls._registry)

    @classmethod
    def _register_presets(cls, max_devices_nums: int = None) -> None:
        max_nums = max_devices_nums if max_devices_nums is not None else cls.MAX_DEVICES

        if cls._registry:
            return

        for i in range(max_nums):
            try:
                inst = ZLGCANInterface(f"ZLG_{i}_0", 1_000_000)
                serial = inst.get_device_serial_number()
                inst.start_device()
                cls._registry[serial] = inst
                # print(f"Registered ZLG CAN device with serial: {serial}")
            except Exception as e:
                # print(f"Failed to register ZLG CAN device at index {i}: {e}")
                continue

ZLGCANManager._register_presets(1)