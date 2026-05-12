import asyncio
import json
import math
import random
import struct
import sys
import time
from pathlib import Path
from loguru import logger

_BASE = Path(__file__).resolve().parent.parent
if str(_BASE) not in sys.path:
    sys.path.insert(0, str(_BASE))

from encos_controller import EncosCanController
from scara_v1 import SCARA_V1_BASIC_CONFIG, ScaraArm, ScaraLiftingJoint

SCARA_V1_MINI_GRIPPER_BASIC_CONFIG = _BASE / "interface" / "scara_v1_mini_gripper_basic_config.json"


def _ensure_config_dict(config):
    """接受已加载的 dict 或 JSON 文件路径（str / Path），返回配置 dict。"""
    if isinstance(config, dict):
        return config
    with Path(config).open(encoding="utf-8") as f:
        return json.load(f)


def _basic_config_arm_slice(full_basic: dict, robot_id: int) -> dict:
    """从 scara_v1_basic_config.json 整文件中取出单臂的 arm 配置 dict。"""
    key = str(robot_id)
    try:
        return full_basic[key]["arm"]
    except (KeyError, TypeError) as e:
        raise KeyError(
            f'基本配置中缺少路径 [{key!r}]["arm"]，请检查 scara_v1_basic_config 结构'
        ) from e


def _gripper_config_robot_slice(full_gripper: dict, robot_id: int) -> dict:
    """从夹爪整文件配置中取出对应 robot_id 的一节。"""
    key = str(robot_id)
    try:
        return full_gripper[key]
    except (KeyError, TypeError) as e:
        raise KeyError(
            f"夹爪配置中缺少机械臂 id {key!r} 的配置段"
        ) from e


__all__ = [
    "EncosCanController",
    "SCARA_V1_BASIC_CONFIG",
    "ScaraArm",
    "ScaraLiftingJoint",
    "SCARA_V1_MINI_GRIPPER_BASIC_CONFIG",
]


class ScaraV1MiniGripper():
    def __init__(self, robot_id=0, gripper_com_port=None, gripper_config_data=None, basic_config_path=None):
        """
        Args:
            robot_id: 区分左右手
            gripper_com_port: 左右手夹爪CAN端口
            gripper_config_data: 左右手夹爪配置数据
            basic_config_path: 基本配置文件路径
        """
        self.robot_id = robot_id
        if gripper_com_port is None or gripper_config_data is None or basic_config_path is None:
            print("gripper_com_port and gripper_config_data and basic_config_path must be provided.")
            return

        logger.info(f"ScaraV1MiniGripper already with robot_id: {robot_id}, gripper_com_port: {gripper_com_port}, gripper_config_data: {gripper_config_data}, basic_config_path: {basic_config_path}")

        full_gripper = _ensure_config_dict(gripper_config_data)
        self.gripper_config_data = _gripper_config_robot_slice(full_gripper, robot_id)
        full_basic = _ensure_config_dict(basic_config_path)
        arm_config_data = _basic_config_arm_slice(full_basic, robot_id)

        #基座和中间关节
        self.scara_arm = ScaraArm(robot_id=robot_id, arm_com_port=gripper_com_port, arm_config_data=arm_config_data)
        logger.info(f"ScaraArm initialized with robot_id: {robot_id}, arm_com_port: {gripper_com_port}, basic_config_path: {basic_config_path}")



        #升降关节
        # self.scara_lifting = ScaraLiftingJoint(robot_id=robot_id, lifting_com_port=gripper_com_port, lifting_config_data=basic_config_path)
        # logger.info(f"ScaraLiftingJoint initialized with robot_id: {robot_id}, lifting_com_port: {gripper_com_port}, lifting_config_data: {basic_config_path}")
        


        self.arm_type = self.gripper_config_data.get("arm_type", "left")
        self.controller_names = self.gripper_config_data.get("controller_names")
        self.gripper_controller_name = self.controller_names[0]
        self.cutter_controller_name = self.controller_names[1]

        # 夹爪和切刀控制器
        self.gripper_controller = EncosCanController(com_port=gripper_com_port, controller_name=self.gripper_controller_name)
        logger.info(f"Gripper controller already with com_port: {gripper_com_port}, controller_name: {self.gripper_controller_name}")
        self.cutter_controller = EncosCanController(com_port=gripper_com_port, controller_name=self.cutter_controller_name)
        logger.info(f"Cutter controller already with com_port: {gripper_com_port}, controller_name: {self.cutter_controller_name}")

        # 初始化夹爪和切刀控制器
        self.gripper_controller.initialize()
        logger.info(f"Gripper controller initialized with com_port: {gripper_com_port}, controller_name: {self.gripper_controller_name}")
        self.cutter_controller.initialize()
        logger.info(f"Cutter controller initialized with com_port: {gripper_com_port}, controller_name: {self.cutter_controller_name}")

        # 夹爪和切刀配置参数
        self.gear_ratio = self.gripper_config_data.get("gear_ratio", 1)
        self.max_open_angle = self.gripper_config_data.get("max_open_angle",20)
        self.min_open_angle = self.gripper_config_data.get("min_open_angle", 0)


    
    def _homing_gripper(self, homing_torque=-0.5):
        '''
        Args:
            homing_torque: 回零力矩
        '''
        print("开始回零夹爪电机")
        self.gripper_controller.up_enable_motor()
        time.sleep(1)
        self.gripper_controller.set_target_torque(homing_torque)
        time.sleep(2)
        self.gripper_controller.set_zero_point()
        self.gripper_controller.down_enable_motor()
        print("回零夹爪电机完成")
        logger.info(f"Homing gripper completed")
        return True

    def _homing_cutter(self):
        print("开始回零切刀电机，请将切刀放置到夹爪圆孔正下方")
        self.cutter_controller.down_enable_motor()
        input("请按回车键继续")
        time.sleep(1)
        self.cutter_controller.set_zero_point()
        # self.cutter_controller.down_enable_motor()
        print("回零切刀电机完成")
        logger.info(f"Homing cutter completed")
        return True

    def _homing_scara(self):
        print("开始回零基座和中间关节")
        self.scara_arm.calibration()
        print("回零基座和中间关节完成")
        logger.info(f"Homing scara completed")
        return True


    # def _homing_lifting(self):
    #     print("开始回零升降关节")
    #     self.scara_lifting.calibration()
    #     print("回零升降关节完成")
    #     logger.info(f"Homing completed")
    #     return True

    def _up_enable_scara(self):
        self.scara_arm.up_enable();
        return True


    # def _up_enable_lifting(self):
    #     self.scara_lifting.up_enable();
    #     return True

    def _up_enable_gripper(self):
        self.gripper_controller.up_enable_motor();
        return True

    def _up_enable_cutter(self):
        self.cutter_controller.up_enable_motor();
        return True

    def _angle_to_motor_pos(self, open_angle):
        """
        将夹爪开合角度转换为电机目标位置
        如果齿轮比为1:1，电机旋转角度 = 夹爪开合角度
        如果齿轮比不为1:1，需要按比例换算

        args:
            open_angle: 夹爪开合角度
        """
        return open_angle * self.gear_ratio

    # 夹爪闭合
    def close_gripper(self, close_angle=0, speed_rpm=200):
        '''
        Args:
            speed_rpm: 夹爪闭合速度
        '''
        motor_pos = self._angle_to_motor_pos(close_angle)
        return self.gripper_controller.set_target_position(motor_pos, speed_rpm)
        

    # 夹爪打开
    def open_gripper(self, open_angle=14, speed_rpm=200):
        '''
        Args:
            open_angle: 夹爪打开角度
            speed_rpm: 夹爪打开速度
        '''
        open_angle = max(self.min_open_angle, min(self.max_open_angle, open_angle))
        motor_pos = self._angle_to_motor_pos(open_angle)
        return self.gripper_controller.set_target_position(motor_pos, speed_rpm)

    # 夹爪快速开合
    def quick_open_close_gripper(self,open_angle=2,speed_rpm=200,close_torque=-0.5):
        '''
        Args:
            open_angle: 夹爪打开角度（逻辑角，度）
            speed_rpm: 恒速阶段目标转速（RPM），位置阶段下发位置指令也用该 RPM
            close_torque: 闭合判据力矩（N·m），取绝对值换算为电流阈值
            torque_tolerance: 力矩容差（N·m）
            blocking: True 时按电流/位置判据等待，否则各阶段固定 sleep
            position_tolerance: 张开到位位置容差（度，电机位置同单位）
            timeout_s: blocking 时每段最大等待时间（秒）
        '''
        self.gripper_controller.set_target_torque(close_torque)
        time.sleep(0.2)
        self.gripper_controller.set_target_position(self._angle_to_motor_pos(open_angle), speed_rpm)
        time.sleep(0.15)
        self.gripper_controller.set_target_torque(close_torque)
        

    # 切削动作(不包括回刀)
    def cut(self, cut_angle=40, speed_rpm=200):
        '''
        Args:
            cut_angle: 切削角度
            speed_rpm: 切削速度
        '''
        self.cutter_controller.set_target_torque(2.0)
        time.sleep(0.048)
        self.cutter_controller.set_target_position(self._angle_to_motor_pos(cut_angle), speed_rpm)
              
        return True

    # 回刀动作
    def home_cut(self, start_angle=-40, speed_rpm=200):
        '''
        Args:
            start_angle: 回刀角度
            speed_rpm: 回刀速度
        '''
        self.cutter_controller.set_target_position(self._angle_to_motor_pos(start_angle), speed_rpm)

    # 回零动作
    def homing(self, type="all", homing_torque=-0.5):
        '''
        Args:
            type: 回零类型，all: 回零基座和中间关节、夹爪和切刀，scara: 回零基座和中间关节，lifting: 回零升降关节，gripper: 回零夹刀，cutter: 回零切刀
            homing_torque: 夹爪回零力矩
        '''
        if type == "all":
            self._homing_scara()
            time.sleep(2)
            # self._homing_lifting()
            # time.sleep(2)
            self._homing_gripper(homing_torque=homing_torque)
            time.sleep(1)
            self._homing_cutter()
        elif type == "scara":
            self._homing_scara()
        # elif type == "lifting":
        #     # self._homing_lifting()
            pass
        elif type == "gripper":
            self._homing_gripper(homing_torque=homing_torque)
        elif type == "cutter":
            self._homing_cutter()
        else:
            print("Invalid homing type")
            return False
        return True

    def move_to_position_by_angle(self, target_joints_pos=(0, 0), speed_ratio=(1.0,1.0), blocking: bool=False,base_blocking_joint_angle = 0.5,middle_blocking_joint_angle = 0.5):
        '''
        Args:
            target_joints_pos: 目标关节角度
            speed_ratio: 速度比例
            blocking: 是否阻塞
            base_blocking_joint_angle: 基座容忍阻塞角度，单位：度
            middle_blocking_joint_angle: 中间关节容忍阻塞角度，单位：度
        '''
        return self.scara_arm.reach_target_joints_pos(target_joints_pos=target_joints_pos,
        speed_ratio=speed_ratio, blocking=blocking, base_blocking_joint_angle=base_blocking_joint_angle,
        middle_blocking_joint_angle=middle_blocking_joint_angle)

    def move_to_position_by_position(self, target_position=(0, 0), speed_ratio=(1.0,1.0), blocking: bool=False, base_blocking_joint_angle = 0.5,middle_blocking_joint_angle = 0.5):
        '''
        Args:
            target_position: 目标位置
            speed_ratio: 速度比例
            blocking: 是否阻塞
            base_blocking_joint_angle: 基座容忍阻塞角度，单位：度
            middle_blocking_joint_angle: 中间关节容忍阻塞角度，单位：度
        '''
        return self.scara_arm.reach_target_position(target_position=target_position, speed_ratio=speed_ratio, 
        blocking=blocking, base_blocking_joint_angle=base_blocking_joint_angle,
        middle_blocking_joint_angle=middle_blocking_joint_angle)

    def up_enable(self, type="all"):
        '''
        Args:
            type: 使能类型，all: 使能基座和中间关节、夹爪和切刀，scara: 使能基座和中间关节，gripper: 使能夹爪，cutter: 使能切刀
        '''
        if type == "all":
            self._up_enable_scara()
            self._up_enable_gripper()
            self._up_enable_cutter()
        elif type == "scara":
            self._up_enable_scara()
        elif type == "gripper":
            self._up_enable_gripper()
        elif type == "cutter":
            self._up_enable_cutter()
        else:
            print("Invalid up enable type")
            return False
        return True

    def down_enable(self):
        '''
            下使能
        '''
        self.scara_arm.down_enable();
        self.gripper_controller.down_enable_motor();
        self.cutter_controller.down_enable_motor();
        return True

    def get_current_scara_pos(self):
        '''
        Returns:
            base_joint_pos: 基座关节角度
            middle_joint_pos: 中间关节角度
        '''
        return self.scara_arm.get_current_pos()

    def get_current_gripper_pos(self):
        '''
        Returns:
            position: 夹爪位置
        '''
        return self.gripper_controller.get_instantaneous_position()

    def get_current_cutter_pos(self):
        '''
        Returns:
            position: 切刀位置
        '''
        return self.cutter_controller.get_instantaneous_position()


if __name__ == "__main__":
    scara_v1_mini_gripper = ScaraV1MiniGripper(robot_id=0, gripper_com_port="ZLG_31F10005727_1", gripper_config_data=SCARA_V1_MINI_GRIPPER_BASIC_CONFIG, basic_config_path=SCARA_V1_BASIC_CONFIG)

    try:
        # scara_v1_mini_gripper.close_gripper(speed_rpm=30)
        # time.sleep(1)
        # scara_v1_mini_gripper.open_gripper(speed_rpm=30)
        # time.sleep(1)
        # scara_v1_mini_gripper.open_gripper(speed_rpm=30)
        # time.sleep(0.8)
        # scara_v1_mini_gripper.quick_open_close_gripper(open_angle=2, 
        # speed_rpm=30, close_torque=-0.8)
        # time.sleep(5)
       # scara_v1_mini_gripper.quick_open_close_gripper()
       

        # scara_v1_mini_gripper.up_enable(type="all")
        # scara_v1_mini_gripper.homing(type="all")

        for i in range(10):
            tmp = time.time()
            scara_v1_mini_gripper.quick_open_close_gripper(open_angle=2, speed_rpm=200, close_torque=-0.8)
            time.sleep(0.2)
            scara_v1_mini_gripper.cut(cut_angle=40, speed_rpm=200)
            time.sleep(0.2)

            R = 440.0
            R2 = R * R
            R1 = 220
            R12 = R1 * R1
            while True:
                theta = random.uniform(0.0, 2.0 * math.pi)
                r = R * math.sqrt(random.random())
                xf, yf = r * math.cos(theta), r * math.sin(theta)
                tmpx = math.floor(xf)
                tmpy = math.floor(yf)
                if R12 <= tmpx * tmpx + tmpy * tmpy <= R2 and tmpx > 0 and tmpy > 0:
                    break
            print(f"tmpx: {tmpx}, tmpy: {tmpy}")
            scara_v1_mini_gripper.move_to_position_by_position(target_position=(tmpx, tmpy), 
            speed_ratio=(0.15,0.15), blocking=True, base_blocking_joint_angle=30.5, 
            middle_blocking_joint_angle=30.5)
            scara_v1_mini_gripper.open_gripper(speed_rpm=200)
            time.sleep(0.5)
            scara_v1_mini_gripper.home_cut(start_angle=-40, speed_rpm=200)
            scara_v1_mini_gripper.open_gripper(speed_rpm=200)
            scara_v1_mini_gripper.move_to_position_by_angle(target_joints_pos=(190, -180), 
            speed_ratio=(0.3,0.3), blocking=True, base_blocking_joint_angle=10.5, 
            middle_blocking_joint_angle=10.5)
            print(f"Time: {time.time() - tmp}")
        # scara_v1_mini_gripper.move_to_position_by_angle(target_joints_pos=(-180, 180), 
        # speed_ratio=(0.5,0.5), blocking=True, base_blocking_joint_angle=1.5, 
        # middle_blocking_joint_angle=1.5)


        # print(scara_v1_mini_gripper.get_current_scara_pos())
        # print(scara_v1_mini_gripper.get_current_gripper_pos())
        # print(scara_v1_mini_gripper.get_current_cutter_pos())
        # print(scara_v1_mini_gripper.get_current_cutter_pos())
    except Exception as e:
        print(f"Error: {e}")
        raise e
    finally:
        scara_v1_mini_gripper.down_enable()
        print("Done.")