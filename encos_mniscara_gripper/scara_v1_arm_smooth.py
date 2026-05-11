from SCARA.paths import *
from SCARA.joint_module_ctrl.multi_controller import MultiController
from SCARA.log import MyLogger
import asyncio
import numpy as np
import json
import time
import threading
import collections

EXECUTION_TRAJECTORY = []

# SCARA单臂
class ScaraArm(MultiController):
    def __init__(
        self,
        robot_id=0,
        arm_com_port=None,
        arm_config_data=None,
    ):
        self.robot_id = robot_id

        # 传入信息判断
        if arm_com_port is None or arm_config_data is None:
            MyLogger.logger.error("arm_com_port and arm_config_data must be provided.")
            return

        self.com_port = arm_com_port
        # 从配置文件中读取机械臂基本参数
        self.arm_config_data = arm_config_data # 单臂配置数据
        self.arm_type = self.arm_config_data.get("arm_type", "left") # "left"表示左手坐标系，"right"表示右手坐标系
        controller_names = self.arm_config_data.get("controller_names") # 机械臂关节实例名称
        link_lengths = self.arm_config_data.get("link_lengths") # 机械臂连杆长度
        calibration_config = self.arm_config_data.get("calibration") # 机械臂标定配置

        # 创建机械臂关节的控制实例
        controller_maps = {
            "base_joint": controller_names[0],
            "middle_joint": controller_names[1],
        }
        super().__init__(controller_maps=controller_maps, com_port=arm_com_port)

        # 设置连杆的长度
        self.distance_base_link = link_lengths[0]
        self.distance_middle_link = link_lengths[1]

        # 读取校准配置数据
        self.offsets = calibration_config.get("offsets", None)
        self.signs = calibration_config.get("signs", [1, -1])

        # 设置机械臂默认折角方向
        self.default_bulge_direction = "CCW" if self.arm_type == "left" else "CW"
        self.bulge_direction = self.default_bulge_direction

        # 自动校准
        # self.calibration()

        # 初始化
        self.initialize()

        # 平滑轨迹规划相关
        # 执行频率
        self.smooth_trajectory_execution_frequency = 50.0
        # 末端最大线速度
        self.smooth_trajectory_execution_max_speed = 200.0
        # 跟踪阈值
        self.trajectory_tracking_threshold = self.smooth_trajectory_execution_max_speed / self.smooth_trajectory_execution_frequency
        # 折向方向
        self.smooth_trajectory_execution_bulge_direction = "CCW" if self.arm_type == "left" else "CW"
        # 执行池（队列）
        self.smooth_trajectory_execution_pool = collections.deque(maxlen=1000)
        # 池读写锁
        self.smooth_trajectory_execution_pool_lock = threading.Lock()
        # 末端历史散点，用于估计当前轨迹切向方向
        self.smooth_trajectory_history_points = None
        self.smooth_trajectory_history_lock = threading.Lock()
        # 当前参考轨迹状态，用于在线重规划时继承位置/速度/加速度
        self.smooth_trajectory_reference_state = None
        self.smooth_trajectory_reference_state_lock = threading.Lock()
        # 最新规划目标位置
        self.smooth_trajectory_planning_target_position = None
        self.smooth_trajectory_planning_target_position_lock = threading.Lock()
        # 规划启动信号
        self.smooth_trajectory_planning_signal = threading.Event()
        # 线程关闭信号
        self.smooth_trajectory_planning_thread_stop = threading.Event()
        self.smooth_trajectory_execution_thread_stop = threading.Event()
        # 线程
        self.smooth_trajectory_planning_thread = None
        self.smooth_trajectory_execution_thread = None

    # 初始化
    def initialize(self):
        print("初始化base_joint 111")
        print(self.base_joint)
        self.base_joint.initialize()
        self.middle_joint.initialize()
        print("初始化base_joint 222")

    # 返回机械臂通信设备的连接状态
    def get_connection_status(self):
        status = self.middle_joint.get_connection_status()
        return "Connecting" if status == "C" and status == "C" else "Disconnected"

    # 断开与机械臂通信设备的连接
    def disconnect(self):
        self.base_joint.disconnect()

    # 重连机械臂通信设备
    def reconnect(self):
        self.base_joint.reconnect()

    # 返回机械臂连接状态
    def get_communication_status(self):
        base_status = self.base_joint.get_communication_status()
        time.sleep(0.1)
        if base_status == "S":
            if self.middle_joint.get_communication_status() == "S":
                status = "Both Joints Successful"
            else:
                status = "Middle Joint Failed"
        else:
            if self.middle_joint.get_communication_status() == "S":
                status = "Base Joint Successful"
            else:
                status = "Both Joints Failed"

        return status

    # 校准
    def calibration(self, set_angles = None, auto = True):
        # 如果没有传入set_angles，则使用默认值
        if set_angles is None:
            set_angles = (-180, 180) if self.arm_type == "left" else (180, -180) # 通过伸直姿势来计算偏置
        set_b_joint_angle, set_m_joint_angle = set_angles

        # 初始化偏置如果是None
        if self.offsets is None:
            self.offsets = [0, 0]

        # 下使能Scara臂
        # self.down_enable()

        # 自动校准
        if auto:
            # 设置加速度
            self.base_joint.set_acceleration(3)
            self.middle_joint.set_acceleration(3)
            cali_sign = 1 if self.arm_type == "left" else -1
            # 先确定基座关节的物理卡销位置
            self.base_joint.up_enable_motor()
            self.base_joint.set_target_speed(9 * cali_sign, 3)
            time.sleep(0.1) # 避开启动时的峰值电流
            while True:
                curr_curr = self.base_joint.get_instantaneous_current().get("content")
                if abs(curr_curr) > 2.95:
                    time.sleep(2.0) # 稳定时间
                    base_mark_pos = self.base_joint.get_instantaneous_position().get("content")
                    time.sleep(0.5)
                    break
                time.sleep(0.01)
            delta_base = base_mark_pos - self.signs[0] * (-75) * cali_sign # ctrl - set

            # 再确定中间关节的物理校准点位置
            self.middle_joint.up_enable_motor()
            self.middle_joint.set_target_speed(9 * cali_sign, 3)
            time.sleep(0.1) # 避开启动时的峰值电流
            while True:
                curr_curr = self.middle_joint.get_instantaneous_current().get("content")
                if abs(curr_curr) > 2.95:
                    time.sleep(2.0) # 稳定时间
                    middle_mark_pos = self.middle_joint.get_instantaneous_position().get("content")
                    time.sleep(0.5)
                    break
                time.sleep(0.01)
            delta_middle = middle_mark_pos - self.signs[1] * 15 * cali_sign # ctrl - set

            # 移动到伸直姿势
            self.base_joint.set_target_position(self.signs[0] * set_b_joint_angle + delta_base, 9)
            self.middle_joint.set_target_position(self.signs[1] * set_m_joint_angle + delta_middle, 9)
            time.sleep(5) # 等待运动完成
        # 手动校准
        else:
            input(f"确保机械臂已摆放成完全伸直状态以便校准，并按下回车：")

        # 设置双关节零位
        self.base_joint.set_zero_point()
        self.middle_joint.set_zero_point()
        self.up_enable()

        # 获取当前位置
        current_base_joint_pos = self.base_joint.get_instantaneous_current().get("content")
        current_middle_joint_pos = self.middle_joint.get_instantaneous_current().get("content")

        # 计算偏置
        self.offsets[0] = current_base_joint_pos - self.signs[0] * set_b_joint_angle
        self.offsets[1] = current_middle_joint_pos - self.signs[1] * set_m_joint_angle

        # 写文件
        self.arm_config_data["calibration"]["offsets"] = self.offsets
        self.arm_config_data["calibration"]["signs"] = self.signs
        with open(SCARA_V1_BASIC_CONFIG, 'r') as f_config_r:
            config_data_full = json.load(f_config_r)
        config_data_full[str(self.robot_id)]["arm"] = self.arm_config_data
        with open(SCARA_V1_BASIC_CONFIG, 'w') as f_config_w:
            json.dump(config_data_full, f_config_w, indent=4)

    # 关节上使能
    def up_enable(self):
        self.base_joint.up_enable_motor()
        self.middle_joint.up_enable_motor()
        return True
    
    # 关节下使能
    def down_enable(self):
        self.base_joint.down_enable_motor()
        self.middle_joint.down_enable_motor()
        return True

    # 获取末端、关节位置
    def get_current_pos(self, joint_only: bool=False):
        base_pos_ctrl = self.base_joint.get_instantaneous_position().get("content")
        middle_pos_ctrl = self.middle_joint.get_instantaneous_position().get("content")
        base_pos = self.signs[0] * (base_pos_ctrl - self.offsets[0])
        middle_pos = self.signs[1] * (middle_pos_ctrl - self.offsets[1])

        if joint_only:
            return {
            "base_joint_pos": base_pos,
            "middle_joint_pos": middle_pos,
            }
        else:
            end_effector_x, end_effector_y = self.position_forward_kinematics(base_pos, middle_pos)
            return {
                "base_joint_pos": base_pos,
                "middle_joint_pos": middle_pos,
                "end_effector_x": end_effector_x,
                "end_effector_y": end_effector_y
            }

    # 位置正运动学解算，给定基座和中间关节的角度值，计算末端的xy位置
    def position_forward_kinematics(self, b_joint_angle, m_joint_angle):
        b_joint_angle_r = np.radians(b_joint_angle)
        m_joint_angle_r = np.radians(m_joint_angle)

        if self.arm_type == "left":
            b_temp_angle_r = b_joint_angle_r + 3*np.pi / 2
        elif self.arm_type == "right":
            b_temp_angle_r = b_joint_angle_r - np.pi / 2
        x = self.distance_base_link * np.cos(b_temp_angle_r) + self.distance_middle_link * np.cos(m_joint_angle_r + b_joint_angle_r + np.pi / 2)
        y = self.distance_base_link * np.sin(b_temp_angle_r) + self.distance_middle_link * np.sin(m_joint_angle_r + b_joint_angle_r + np.pi / 2)

        return x, y

    # 位置逆运动学解算，给定末端xy位置（单位mm），计算两个关节在各自位置坐标系的角度值
    def position_inverse_kinematics(self, x, y, bulge_direction = None):
        """
        bulge_direction: "CW" or "CCW", the bulge direction
        """
        # a = time.time()

        # 初始化bulge_direction
        if bulge_direction is None:
            bulge_direction = "CCW" if self.arm_type == "left" else "CW"

        # 计算等效直杆长度的平方
        straight_link_length_sqr = x**2 + y**2
        # 计算凸角度
        bulge_angle = np.arccos((self.distance_base_link**2 + self.distance_middle_link**2 - straight_link_length_sqr) / (2 * self.distance_base_link * self.distance_middle_link))
        # 计算基座关节处的偏角
        offset_angle = np.arccos((self.distance_base_link**2 + straight_link_length_sqr - self.distance_middle_link**2) / (2 * self.distance_base_link * np.sqrt(straight_link_length_sqr)))
        
        # 如果逆折，则中间关节角度等于凸角度；如果顺折，则中间关节角度等于360度减去凸角度
        if bulge_direction == "CCW":
            m_joint_angle = bulge_angle if self.arm_type == "left" else -2*np.pi + bulge_angle
            self.bulge_direction = "CCW"
        elif bulge_direction == "CW":
            m_joint_angle = 2*np.pi - bulge_angle if self.arm_type == "left" else -1 * bulge_angle
            self.bulge_direction = "CW"

        # 计算等效直杆与X轴夹角
        straight_link_angle = np.arctan2(y, x)
        if straight_link_angle < 0:
            straight_link_angle += 2*np.pi

        # 如果逆折
        if bulge_direction == "CCW":
            b_joint_angle_pre = straight_link_angle + offset_angle
            if self.arm_type == "left": # left arm
                b_joint_angle = b_joint_angle_pre if b_joint_angle_pre <= 2*np.pi else b_joint_angle_pre - 2*np.pi
                b_joint_angle -= 3*np.pi/2
            else:  # right arm
                b_joint_angle = b_joint_angle_pre if b_joint_angle_pre <= 3*np.pi/2 else b_joint_angle_pre - 2*np.pi
                b_joint_angle += np.pi/2

        elif bulge_direction == "CW":
            b_joint_angle_pre = straight_link_angle - offset_angle
            if self.arm_type == "left": # left arm
                b_joint_angle = b_joint_angle_pre if b_joint_angle_pre >= 0 else b_joint_angle_pre + 2*np.pi
                b_joint_angle -= 3*np.pi/2
            else:  # right arm
                b_joint_angle = b_joint_angle_pre if b_joint_angle_pre >= -np.pi/2 else b_joint_angle_pre + 2*np.pi
                b_joint_angle += np.pi/2
            
        b_joint_angle = np.rad2deg(b_joint_angle)
        m_joint_angle = np.rad2deg(m_joint_angle)

        if b_joint_angle < self.base_joint.movement_range["min"] or \
            b_joint_angle > self.base_joint.movement_range["max"]:
            b_joint_angle = None
        if m_joint_angle < self.middle_joint.movement_range["min"] or \
            m_joint_angle > self.middle_joint.movement_range["max"]:
            m_joint_angle = None
        # b = time.time()

        return (b_joint_angle, 
                m_joint_angle, 
                # b - a,
                )

    # 移动到目标位置，目标位置为mm单位
    def reach_target_position(self, target_position=(0, 0), bulge_direction=None, speed_ratio=(1.0,1.0), blocking: bool=False, base_blocking_joint_angle = 0.5,middle_blocking_joint_angle = 0.5):
        """
        Reach the target position (x, y) uniformly with linear speed.
        The target position is in mm.
        The speed_ratio is a list of [arm_speed_ratio, arm_speed_ratio] in range [0.0, 1.0].
        The bulge_direction: "CW" or "CCW".
        """
        # 初始化bulge_direction
        if bulge_direction is None:
            bulge_direction = self.default_bulge_direction

        # 目标位置初步检查
        target_x, target_y = target_position
        if target_x**2 + target_y**2 > (self.distance_base_link + self.distance_middle_link)**2:
            MyLogger.logger.error(f"Target position {target_position} is out of the arm's reachable workspace.")
            raise ValueError(f"Target position {target_position} is out of the arm's reachable workspace.")

        # 折角方向检查
        if bulge_direction not in ["CW", "CCW"]:
            MyLogger.logger.error(f"Bulge direction {bulge_direction} is not supported. Use 'CW' or 'CCW'.")
            raise ValueError(f"Bulge direction {bulge_direction} is not supported. Use 'CW' or 'CCW'.")

        # 速度检查
        b_speed_ratio, m_speed_ratio = speed_ratio
        if b_speed_ratio < 0 or m_speed_ratio < 0:
            MyLogger.logger.error(f"Speed ratio {speed_ratio} should be non-negative.")
            raise ValueError(f"Speed ratio {speed_ratio} should be non-negative.")
        if b_speed_ratio > 1.0 or m_speed_ratio > 1.0:
            MyLogger.logger.error(f"Speed ratio {speed_ratio} should be in the range of [0.0, 1.0].")
            raise ValueError(f"Speed ratio {speed_ratio} should be in the range of [0.0, 1.0].")
        b_joint_speed = self.base_joint.rated_speed * b_speed_ratio
        m_joint_speed = self.middle_joint.rated_speed * m_speed_ratio

        global EXECUTION_TRAJECTORY
        EXECUTION_TRAJECTORY.append((target_x, target_y, bulge_direction))

        # 判断运动成功距离数值的合理性
        if blocking and (base_blocking_joint_angle <= 0 or middle_blocking_joint_angle <= 0):
            MyLogger.logger.error(f"The blocking joint angle {base_blocking_joint_angle} and {middle_blocking_joint_angle} should be positive.")
            raise ValueError(f"The blocking joint angle {base_blocking_joint_angle} and {middle_blocking_joint_angle} should be positive.")
        
        # 上使能左臂
        if not self.base_joint.ENABLE or not self.middle_joint.ENABLE:
            self.up_enable()


        try:
            # 获取初始位姿
            start_joint_angles = self.get_current_pos(joint_only=True)
            start_b_joint_angle, start_m_joint_angle = start_joint_angles.get("base_joint_pos"), start_joint_angles.get("middle_joint_pos")

            # 计算目标关节坐标系角度
            target_b_joint_angle, target_m_joint_angle = self.position_inverse_kinematics(target_x, target_y, bulge_direction)
            if (target_b_joint_angle != target_b_joint_angle or target_m_joint_angle != target_m_joint_angle) or (target_b_joint_angle is None or target_m_joint_angle is None):  # NaN check
                MyLogger.logger.error(f"Target position {target_position} is out of the arm's reachable workspace for the given bulge direction {bulge_direction}.")
                raise ValueError(f"Target position {target_position} is out of the arm's reachable workspace for the given bulge direction {bulge_direction}.")
            
            # 换算关节控制角度
            target_b_joint_ctrl_angle = self.signs[0] * target_b_joint_angle + self.offsets[0]
            target_m_joint_ctrl_angle = self.signs[1] * target_m_joint_angle + self.offsets[1]

            # 控制关节运动至目标位置
            spd_val_base, spd_val_middle = int(b_joint_speed / 6), int(m_joint_speed / 6)
            self.base_joint.set_target_position(pos_deg=target_b_joint_ctrl_angle, spd_val=spd_val_base)
            self.middle_joint.set_target_position(pos_deg=target_m_joint_ctrl_angle, spd_val=spd_val_middle)

            # 计算行程
            delta_base_move = abs(start_b_joint_angle - target_b_joint_angle)
            delta_middle_move = abs(start_m_joint_angle - target_m_joint_angle)

            # 更新关节目标角度和折角
            self.bulge_direction = bulge_direction

            # 根据阻塞/非阻塞模式返回执行状态
            if blocking:
                # 计算运行等待时间
                ## 计算基座关节
                base_acc_angle = self.base_joint.acc_val * 180 / np.pi
                if b_joint_speed**2 >= base_acc_angle*delta_base_move:
                    waiting_time_b = np.sqrt(delta_base_move / base_acc_angle)
                else:
                    waiting_time_b = delta_base_move / b_joint_speed + b_joint_speed / base_acc_angle
                waiting_time_b_post = max(0.0, (delta_base_move - base_blocking_joint_angle) / delta_base_move) * waiting_time_b

                ## 计算中间关节
                middle_acc_angle = self.middle_joint.acc_val * 180 / np.pi
                if m_joint_speed**2 >= middle_acc_angle*delta_middle_move:
                    waiting_time_m = np.sqrt(delta_middle_move / middle_acc_angle)
                else:
                    waiting_time_m = delta_middle_move / m_joint_speed + m_joint_speed / middle_acc_angle
                waiting_time_m_post = max(0.0, (delta_middle_move - middle_blocking_joint_angle) / delta_middle_move) * waiting_time_m

                ## 执行运动等待
                time.sleep(max(waiting_time_b_post, waiting_time_m_post))

                # 尝试获取运动到位信息
                try_times = 0
                while try_times < 100:
                    # # 异步获取
                    # new_joints_angles = asyncio.run(self.async_get_current_pos(joint_only=True))
                    # 非异步获取
                    new_joints_angles = self.get_current_pos(joint_only=True)
                    new_base_pos, new_middle_pos = new_joints_angles.get("base_joint_pos"), new_joints_angles.get("middle_joint_pos")
                    if abs(target_b_joint_angle - new_base_pos) < base_blocking_joint_angle and abs(target_m_joint_angle - new_middle_pos) < middle_blocking_joint_angle:
                        return True
                    try_times += 1
                    time.sleep(0.01)
                return False
            else:
                return True
        
        except Exception as e:
            MyLogger.logger.error(f"Error occurred while moving scara arm to target position {target_position} on {self.com_port}: {e}")
            return False
    
    # 左臂移动到目标关节坐标系位置
    def reach_target_joints_pos(self, target_joints_pos=(0, 0), speed_ratio=(1.0,1.0), blocking: bool=False,base_blocking_joint_angle = 0.5,middle_blocking_joint_angle = 0.5):
        """
        Reach the target joints position (base_joint_angle, middle_joint_angle) uniformly with linear speed.
        The target joints position is in degrees.
        The speed_ratio is a list of [base_joint_speed_ratio, middle_joint_speed_ratio] in range [0.0, 1.0].
        """
        # 目标关节位置检查
        target_b_joint_angle, target_m_joint_angle = target_joints_pos
        if target_b_joint_angle < self.base_joint.movement_range["min"] or target_b_joint_angle > self.base_joint.movement_range["max"]:
            MyLogger.logger.error(f"Target base joint angle {target_b_joint_angle}° is out of the base joint's movement range.")
            raise ValueError(f"Target base joint angle {target_b_joint_angle}° is out of the base joint's movement range.")
        if target_m_joint_angle < self.middle_joint.movement_range["min"] or target_m_joint_angle > self.middle_joint.movement_range["max"]:
            MyLogger.logger.error(f"Target middle joint angle {target_m_joint_angle}° is out of the middle joint's movement range.")
            raise ValueError(f"Target middle joint angle {target_m_joint_angle}° is out of the middle joint's movement range.")

        # 速度检查
        b_speed_ratio, m_speed_ratio = speed_ratio
        if b_speed_ratio < 0 or m_speed_ratio < 0:
            MyLogger.logger.error(f"Speed ratio {speed_ratio} should be non-negative.")
            raise ValueError(f"Speed ratio {speed_ratio} should be non-negative.")
        if b_speed_ratio > 1.0 or m_speed_ratio > 1.0:
            MyLogger.logger.error(f"Speed ratio {speed_ratio} should be in the range of [0.0, 1.0].")
            raise ValueError(f"Speed ratio {speed_ratio} should be in the range of [0.0, 1.0].")
        b_joint_speed = self.base_joint.rated_speed * b_speed_ratio
        m_joint_speed = self.middle_joint.rated_speed * m_speed_ratio

        # 判断运动成功距离数值的合理性
        if blocking and (base_blocking_joint_angle <= 0 or middle_blocking_joint_angle <= 0):
            MyLogger.logger.error(f"The blocking joint angle {base_blocking_joint_angle} and {middle_blocking_joint_angle} should be positive.")
            raise ValueError(f"The blocking joint angle {base_blocking_joint_angle} and {middle_blocking_joint_angle} should be positive.")

        # 上使能左臂
        if not self.base_joint.ENABLE or not self.middle_joint.ENABLE:
            self.up_enable()


        # 获取初始位姿
        start_joint_angles = self.get_current_pos(joint_only=True)
        start_b_joint_angle, start_m_joint_angle = start_joint_angles.get("base_joint_pos"), start_joint_angles.get("middle_joint_pos")

        # 换算关节控制角度
        target_b_joint_ctrl_angle = self.signs[0] * target_b_joint_angle + self.offsets[0]
        target_m_joint_ctrl_angle = self.signs[1] * target_m_joint_angle + self.offsets[1]

        # 控制关节运动至目标位置
        spd_val_base, spd_val_middle = int(b_joint_speed / 6), int(m_joint_speed / 6)
        self.base_joint.set_target_position(pos_deg=target_b_joint_ctrl_angle, spd_val=spd_val_base)
        self.middle_joint.set_target_position(pos_deg=target_m_joint_ctrl_angle, spd_val=spd_val_middle)

        # 计算行程
        delta_base_move = abs(start_b_joint_angle - target_b_joint_angle)
        delta_middle_move = abs(start_m_joint_angle - target_m_joint_angle)

        # 更新关节目标角度和折角
        if self.arm_type == "left":
            theta1 = target_b_joint_angle + 270
        else:
            theta1 = target_b_joint_angle - 90
        theta2 = target_m_joint_angle + target_b_joint_angle + 90
        if theta1 <= theta2:
            self.bulge_direction = "CW"
        else:
            self.bulge_direction = "CCW"

        # 根据阻塞/非阻塞模式返回执行状态
        if blocking:
            # 计算运行等待时间
            ## 计算基座关节
            base_acc_angle = self.base_joint.acc_val * 180 / np.pi
            if b_joint_speed**2 >=base_acc_angle*delta_base_move:
                waiting_time_b = np.sqrt(delta_base_move / base_acc_angle)
            else:
                waiting_time_b = delta_base_move / b_joint_speed + b_joint_speed / base_acc_angle
            waiting_time_b_post = max(0.0, (delta_base_move - base_blocking_joint_angle) / delta_base_move) * waiting_time_b

            ## 计算中间关节
            middle_acc_angle = self.middle_joint.acc_val * 180 / np.pi
            if m_joint_speed**2 >= middle_acc_angle*delta_middle_move:
                waiting_time_m = np.sqrt(delta_middle_move / middle_acc_angle)
            else:
                waiting_time_m = delta_middle_move / m_joint_speed + m_joint_speed / middle_acc_angle
            # print(delta_middle_move,delta_middle_move,waiting_time_m)
            waiting_time_m_post = max(0.0, (delta_middle_move - middle_blocking_joint_angle) / delta_middle_move) * waiting_time_m

            ## 执行运动等待
            time.sleep(max(waiting_time_b_post, waiting_time_m_post))

            # 尝试获取运动到位信息
            try_times = 0
            while try_times < 100:
                # # 异步获取
                # new_joints_angles = asyncio.run(self.async_get_current_pos(joint_only=True))
                # 非异步获取
                new_joints_angles = self.get_current_pos(joint_only=True)
                new_base_pos, new_middle_pos = new_joints_angles.get("base_joint_pos"), new_joints_angles.get("middle_joint_pos")
                if abs(target_b_joint_angle - new_base_pos) < base_blocking_joint_angle and abs(target_m_joint_angle - new_middle_pos) < middle_blocking_joint_angle:
                    return True
                try_times += 1
                time.sleep(0.01)
            return False
        else:
            return True
        
    # 急停
    def stop(self):
        self.base_joint.down_enable_motor() # 掉使能代替急停
        self.middle_joint.down_enable_motor() # 掉使能代替急停
        return True

    # 启动平滑轨迹功能双线程
    def start_smooth_trajectory(self):
        time.sleep(1) # 等待机械臂稳定
        current_position = self.get_current_pos()
        self.smooth_trajectory_history_points = collections.deque(
            [(current_position.get("end_effector_x"), current_position.get("end_effector_y")+0.0, self.default_bulge_direction)],
            maxlen=6,
        )
        with self.smooth_trajectory_reference_state_lock:
            self.smooth_trajectory_reference_state = {
                "position": np.array(
                    [current_position.get("end_effector_x"), current_position.get("end_effector_y")],
                    dtype=float,
                ),
                "velocity": np.zeros(2, dtype=float),
                "acceleration": np.zeros(2, dtype=float),
            }
        print(self.smooth_trajectory_history_points)
        input("Press Enter to continue...")
        self.smooth_trajectory_planning_thread = threading.Thread(target=self.smooth_trajectory_planning, name="SmoothTrajectoryPlanningThread", daemon=True)
        self.smooth_trajectory_execution_thread = threading.Thread(target=self.smooth_trajectory_execution, name="SmoothTrajectoryExecutionThread", daemon=True)
        self.smooth_trajectory_planning_thread.start()
        self.smooth_trajectory_execution_thread.start()

    # 停止平滑轨迹功能双线程
    def stop_smooth_trajectory(self):
        self.smooth_trajectory_planning_thread_stop.set()
        self.smooth_trajectory_execution_thread_stop.set()
        self.smooth_trajectory_planning_thread.join()
        self.smooth_trajectory_execution_thread.join()
        self.smooth_trajectory_planning_thread = None
        self.smooth_trajectory_execution_thread = None
        with self.smooth_trajectory_reference_state_lock:
            self.smooth_trajectory_reference_state = None

    # 生成执行序列
    def generate_smooth_trajectory(self, target_position=None):
        # 目标位置检查
        if target_position is None:
            MyLogger.logger.error(f"Target position is None.")
            raise ValueError(f"Target position is None.")
        workspace_radius = self.distance_base_link + self.distance_middle_link
        if target_position[0]**2 + target_position[1]**2 > workspace_radius**2:
            MyLogger.logger.error(f"Target position {target_position} is out of the workspace.")
            raise ValueError(f"Target position {target_position} is out of the workspace.")

        target_position_np = np.array(target_position, dtype=float)
        dt = 1.0 / self.smooth_trajectory_execution_frequency
        max_speed = float(self.smooth_trajectory_execution_max_speed)
        max_step_distance = max_speed * dt
        max_acceleration = max(max_speed / dt, 1.0)

        def _clip_norm(vector, limit):
            vector = np.array(vector, dtype=float)
            norm = np.linalg.norm(vector)
            if norm < 1e-9 or limit <= 0.0 or norm <= limit:
                return vector
            return vector * (limit / norm)

        def _estimate_state_from_history(history_xy_points):
            if len(history_xy_points) == 0:
                return None

            dedup_points = [np.array(history_xy_points[0], dtype=float)]
            for point in history_xy_points[1:]:
                point_np = np.array(point, dtype=float)
                if np.linalg.norm(point_np - dedup_points[-1]) > 1e-6:
                    dedup_points.append(point_np)

            history_np = np.array(dedup_points, dtype=float)
            current_position = history_np[-1]
            current_velocity = np.zeros(2, dtype=float)
            current_acceleration = np.zeros(2, dtype=float)

            if len(history_np) >= 2:
                velocities = np.diff(history_np, axis=0) / dt
                recent_velocities = velocities[-3:]
                velocity_weights = np.arange(1.0, len(recent_velocities) + 1.0, dtype=float)
                current_velocity = np.average(recent_velocities, axis=0, weights=velocity_weights)

            if len(history_np) >= 3:
                velocities = np.diff(history_np, axis=0) / dt
                accelerations = np.diff(velocities, axis=0) / dt
                recent_accelerations = accelerations[-2:]
                acceleration_weights = np.arange(1.0, len(recent_accelerations) + 1.0, dtype=float)
                current_acceleration = np.average(recent_accelerations, axis=0, weights=acceleration_weights)

            return {
                "position": current_position,
                "velocity": _clip_norm(current_velocity, max_speed),
                "acceleration": _clip_norm(current_acceleration, max_acceleration),
            }

        def _solve_quintic(p0, v0, a0, p1, v1, a1, duration):
            duration_square = duration * duration
            duration_cube = duration_square * duration
            duration_quad = duration_cube * duration
            duration_quint = duration_quad * duration

            matrix = np.array(
                [
                    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
                    [1.0, duration, duration_square, duration_cube, duration_quad, duration_quint],
                    [0.0, 1.0, 2.0 * duration, 3.0 * duration_square, 4.0 * duration_cube, 5.0 * duration_quad],
                    [0.0, 0.0, 2.0, 6.0 * duration, 12.0 * duration_square, 20.0 * duration_cube],
                ],
                dtype=float,
            )
            boundary_values = np.array([p0, v0, a0, p1, v1, a1], dtype=float)
            return np.linalg.solve(matrix, boundary_values)

        def _evaluate_quintic(coefficients, sample_ts):
            return (
                coefficients[0]
                + coefficients[1] * sample_ts
                + coefficients[2] * sample_ts ** 2
                + coefficients[3] * sample_ts ** 3
                + coefficients[4] * sample_ts ** 4
                + coefficients[5] * sample_ts ** 5
            )

        def _evaluate_quintic_velocity(coefficients, sample_ts):
            return (
                coefficients[1]
                + 2.0 * coefficients[2] * sample_ts
                + 3.0 * coefficients[3] * sample_ts ** 2
                + 4.0 * coefficients[4] * sample_ts ** 3
                + 5.0 * coefficients[5] * sample_ts ** 4
            )

        def _evaluate_quintic_acceleration(coefficients, sample_ts):
            return (
                2.0 * coefficients[2]
                + 6.0 * coefficients[3] * sample_ts
                + 12.0 * coefficients[4] * sample_ts ** 2
                + 20.0 * coefficients[5] * sample_ts ** 3
            )

        with self.smooth_trajectory_history_lock:
            history_points = list(self.smooth_trajectory_history_points)
        with self.smooth_trajectory_reference_state_lock:
            reference_state = None if self.smooth_trajectory_reference_state is None else {
                "position": self.smooth_trajectory_reference_state["position"].copy(),
                "velocity": self.smooth_trajectory_reference_state["velocity"].copy(),
                "acceleration": self.smooth_trajectory_reference_state["acceleration"].copy(),
            }

        if len(history_points) == 0:
            MyLogger.logger.warning("Smooth trajectory history is empty, skip current planning.")
            return []

        history_xy_points = [history_point[:2] for history_point in history_points]
        history_state = _estimate_state_from_history(history_xy_points)
        if history_state is None:
            return []

        if reference_state is None:
            start_state = history_state
        else:
            history_position = history_state["position"]
            reference_position = reference_state["position"]
            if np.linalg.norm(history_position - reference_position) <= max(2.0 * max_step_distance, 1e-6):
                start_state = reference_state
                start_state["position"] = history_position
            else:
                start_state = history_state

        current_position = np.array(start_state["position"], dtype=float)
        chord_vector = target_position_np - current_position
        delta_distance = np.linalg.norm(chord_vector)
        if delta_distance < 1e-9:
            return []

        # 距离较近时直接跟踪新目标，避免引入不必要振荡。
        if delta_distance <= self.trajectory_tracking_threshold:
            return [(
                float(target_position_np[0]),
                float(target_position_np[1]),
                self.smooth_trajectory_execution_bulge_direction,
                0.0,
                0.0,
                0.0,
                0.0,
            )]

        current_velocity = np.array(start_state["velocity"], dtype=float)
        current_acceleration = np.array(start_state["acceleration"], dtype=float)
        chord_direction = chord_vector / delta_distance

        if delta_distance <= 4.0 * self.trajectory_tracking_threshold:
            end_velocity = np.zeros(2, dtype=float)
        else:
            end_velocity = chord_direction * min(0.25 * max_speed, delta_distance / max(6.0 * dt, 1e-6))
        end_acceleration = np.zeros(2, dtype=float)

        speed_time = delta_distance / max(max_speed, 1e-6)
        accel_time = np.sqrt(delta_distance / max(max_acceleration, 1e-6))
        duration = max(1.8 * speed_time, 2.5 * accel_time, 6.0 * dt, 0.12)

        coeff_x = _solve_quintic(
            current_position[0],
            current_velocity[0],
            current_acceleration[0],
            target_position_np[0],
            end_velocity[0],
            end_acceleration[0],
            duration,
        )
        coeff_y = _solve_quintic(
            current_position[1],
            current_velocity[1],
            current_acceleration[1],
            target_position_np[1],
            end_velocity[1],
            end_acceleration[1],
            duration,
        )

        sample_num = max(2, int(np.ceil(duration / dt)))
        sample_ts = np.linspace(dt, duration, sample_num)
        sample_xs = _evaluate_quintic(coeff_x, sample_ts)
        sample_ys = _evaluate_quintic(coeff_y, sample_ts)
        sample_vxs = _evaluate_quintic_velocity(coeff_x, sample_ts)
        sample_vys = _evaluate_quintic_velocity(coeff_y, sample_ts)
        sample_axs = _evaluate_quintic_acceleration(coeff_x, sample_ts)
        sample_ays = _evaluate_quintic_acceleration(coeff_y, sample_ts)

        sampled_points = np.column_stack((sample_xs, sample_ys)).astype(float)
        sampled_velocities = np.column_stack((sample_vxs, sample_vys)).astype(float)
        sampled_accelerations = np.column_stack((sample_axs, sample_ays)).astype(float)
        sampled_points[-1] = target_position_np

        planned_executions = []
        last_append_point = current_position
        for sampled_point, sampled_velocity, sampled_acceleration in zip(sampled_points, sampled_velocities, sampled_accelerations):
            if np.linalg.norm(sampled_point - last_append_point) < 1e-6:
                continue
            if sampled_point[0] ** 2 + sampled_point[1] ** 2 > workspace_radius ** 2 + 1e-6:
                MyLogger.logger.warning(
                    f"Quintic trajectory point {tuple(sampled_point.tolist())} is out of workspace, fallback to target only."
                )
                return [(
                    float(target_position_np[0]),
                    float(target_position_np[1]),
                    self.smooth_trajectory_execution_bulge_direction,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )]
            planned_executions.append(
                (
                    float(sampled_point[0]),
                    float(sampled_point[1]),
                    self.smooth_trajectory_execution_bulge_direction,
                    float(sampled_velocity[0]),
                    float(sampled_velocity[1]),
                    float(sampled_acceleration[0]),
                    float(sampled_acceleration[1]),
                )
            )
            last_append_point = sampled_point

        if len(planned_executions) == 0:
            return [(
                float(target_position_np[0]),
                float(target_position_np[1]),
                self.smooth_trajectory_execution_bulge_direction,
                0.0,
                0.0,
                0.0,
                0.0,
            )]
        return planned_executions


    # 平滑轨迹规划线程
    def smooth_trajectory_planning(self):
        planning_refresh_time = 0.02

        while not self.smooth_trajectory_planning_thread_stop.is_set():
            start_time = time.time()

            if self.smooth_trajectory_planning_signal is None or not self.smooth_trajectory_planning_signal.is_set():
                time.sleep(max(0.0, planning_refresh_time - time.time() + start_time))
                continue

            with self.smooth_trajectory_planning_target_position_lock:
                target_position = self.smooth_trajectory_planning_target_position
            if target_position is None:
                self.smooth_trajectory_planning_signal.clear()
                time.sleep(max(0.0, planning_refresh_time - time.time() + start_time))
                continue

            # 轨迹规划结果占位，后续在这里补充具体规划逻辑。
            planned_executions = self.generate_smooth_trajectory(target_position)

            with self.smooth_trajectory_execution_pool_lock:
                self.smooth_trajectory_execution_pool.clear()
                self.smooth_trajectory_execution_pool.extend(planned_executions)

            self.smooth_trajectory_planning_signal.clear()

            planning_time = time.time() - start_time
            if planning_time < planning_refresh_time:
                time.sleep(max(0.0, planning_refresh_time - planning_time))
    
    # 平滑轨迹执行线程
    def smooth_trajectory_execution(self):
        # 执行刷新时间
        execution_refresh_time = 1.0 / self.smooth_trajectory_execution_frequency

        # 重复执行
        while not self.smooth_trajectory_execution_thread_stop.is_set():

            start_time = time.time()
            execution = None

            # 锁读执行池
            with self.smooth_trajectory_execution_pool_lock:
                if len(self.smooth_trajectory_execution_pool) > 0:
                    execution = self.smooth_trajectory_execution_pool.popleft()

            if execution is None:
                time.sleep(max(0.0, execution_refresh_time - time.time() + start_time))
                continue

            try:
                res = self.reach_target_position(
                    target_position=(execution[0], execution[1]),
                    bulge_direction=execution[2],
                    speed_ratio=(0.5, 0.5),
                    blocking=False,
                )

                if not res:
                    MyLogger.logger.warning(f"Error occurred while executing smooth trajectory: {res}")
                else:
                    with self.smooth_trajectory_history_lock:
                        self.smooth_trajectory_history_points.append((execution[0], execution[1], execution[2]))
                    with self.smooth_trajectory_reference_state_lock:
                        self.smooth_trajectory_reference_state = {
                            "position": np.array([execution[0], execution[1]], dtype=float),
                            "velocity": np.array([execution[3], execution[4]], dtype=float),
                            "acceleration": np.array([execution[5], execution[6]], dtype=float),
                        }

            except Exception as e:
                MyLogger.logger.error(f"Error occurred while executing smooth trajectory: {e}")

            
            execution_time = time.time() - start_time
            if execution_time < execution_refresh_time:
                time.sleep(max(0.0, execution_refresh_time - execution_time))

    def set_smooth_trajectory_planning_target_position(self, target_position=(0, 0)):
        with self.smooth_trajectory_planning_target_position_lock:
            self.smooth_trajectory_planning_target_position = target_position
            self.smooth_trajectory_planning_signal.set()

def demonstration():
    pass

if __name__ == "__main__":
    import math
    import random
    import matplotlib.pyplot as plt

    def generate_circle_and_scatter_targets(
        circle_center,
        circle_radius,
        circle_turns,
        publish_frequency,
        line_speed,
        workspace_radius,
        scatter_num=3,
    ):
        if publish_frequency <= 0:
            raise ValueError("publish_frequency must be positive.")
        if circle_radius <= 0:
            raise ValueError("circle_radius must be positive.")
        if circle_turns <= 0:
            raise ValueError("circle_turns must be positive.")
        if line_speed <= 0:
            raise ValueError("line_speed must be positive.")

        circumference = 2.0 * math.pi * circle_radius
        total_length = circumference * circle_turns
        total_duration = total_length / line_speed
        sample_num = max(2, int(math.ceil(total_duration * publish_frequency)))

        continuous_targets = []
        for idx in range(sample_num):
            theta = 2.0 * math.pi * circle_turns * idx / (sample_num - 1)
            x = circle_center[0] + circle_radius * math.cos(theta)
            y = circle_center[1] + circle_radius * math.sin(theta)
            if x * x + y * y > workspace_radius * workspace_radius:
                raise ValueError(f"Circle point {(x, y)} is out of workspace.")
            continuous_targets.append((x, y))

        scatter_targets = []
        while len(scatter_targets) < scatter_num:
            random_radius = circle_radius * math.sqrt(random.random())
            random_theta = 2.0 * math.pi * random.random()
            x = circle_center[0] + random_radius * math.cos(random_theta)
            y = circle_center[1] + random_radius * math.sin(random_theta)
            if x * x + y * y <= workspace_radius * workspace_radius:
                scatter_targets.append((x, y))

        return continuous_targets, scatter_targets

    def wait_until_smooth_trajectory_idle(scara_arm, timeout=30.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            planning_busy = scara_arm.smooth_trajectory_planning_signal.is_set()
            with scara_arm.smooth_trajectory_execution_pool_lock:
                queue_empty = len(scara_arm.smooth_trajectory_execution_pool) == 0
            if not planning_busy and queue_empty:
                return True
            time.sleep(0.05)
        return False

    circle_center = (200.0, 200.0)
    circle_radius = 120.0
    circle_turns = 1.0
    continuous_publish_frequency = 60.0

    workspace_radius = 440

    configs = {
            "arm_type": "left",
            "controller_names": [
                "left_scara_base_can",
                "left_scara_middle_can"
            ],
            "link_lengths": [
                220.0,
                220.0
            ],
            "calibration": {
                "offsets": [
                    180.00925204809755,
                    179.978332310915
                ],
                "signs": [
                    1,
                    -1
                ]
            }
    }

    scara_arm = ScaraArm(
            robot_id=0,
            arm_com_port="ZLG_31F100058EA_0",
            arm_config_data=configs,
        )

    scara_arm.initialize()

    continuous_targets, scatter_targets = generate_circle_and_scatter_targets(
        circle_center=circle_center,
        circle_radius=circle_radius,
        circle_turns=circle_turns,
        publish_frequency=continuous_publish_frequency,
        line_speed=scara_arm.smooth_trajectory_execution_max_speed * 0.6,
        workspace_radius=workspace_radius,
        scatter_num=3,
    )

    try:
        scara_arm.reach_target_position((320, 250), "CCW", (0.1, 0.1), blocking= True)
        time.sleep(1)

        EXECUTION_TRAJECTORY.clear()
        
        scara_arm.start_smooth_trajectory()
        time.sleep(1)

        publish_period = 1.0 / continuous_publish_frequency
        next_publish_time = time.time()

        for target_position in continuous_targets:
            scara_arm.set_smooth_trajectory_planning_target_position(target_position=target_position)
            next_publish_time += publish_period
            time.sleep(max(0.0, next_publish_time - time.time()))

        if not wait_until_smooth_trajectory_idle(scara_arm, timeout=60.0):
            MyLogger.logger.warning("Continuous trajectory did not fully drain before scatter targets.")

        for target_position in scatter_targets:
            scara_arm.set_smooth_trajectory_planning_target_position(target_position=target_position)
            time.sleep(1.0)

        if not wait_until_smooth_trajectory_idle(scara_arm, timeout=60.0):
            MyLogger.logger.warning("Smooth trajectory executor still busy when stopping test.")

    finally:
        scara_arm.stop_smooth_trajectory()

    plt.figure(figsize=(8, 8))
    if len(EXECUTION_TRAJECTORY) > 0:
        trajectory_np = np.array([execution[:2] for execution in EXECUTION_TRAJECTORY], dtype=float)
        plt.plot(trajectory_np[:, 0], trajectory_np[:, 1], "-b", linewidth=1.5, label="execution trajectory")
        plt.scatter(trajectory_np[:, 0], trajectory_np[:, 1], s=10, c="b")

    continuous_targets_np = np.array(continuous_targets, dtype=float)
    scatter_targets_np = np.array(scatter_targets, dtype=float)
    plt.plot(
        continuous_targets_np[:, 0],
        continuous_targets_np[:, 1],
        "--g",
        linewidth=1.0,
        label="continuous target sequence",
    )
    plt.scatter(
        scatter_targets_np[:, 0],
        scatter_targets_np[:, 1],
        c="r",
        s=40,
        label="scatter targets",
    )
    plt.scatter([circle_center[0]], [circle_center[1]], c="k", s=50, label="circle center")
    plt.axis("equal")
    plt.xlabel("x / mm")
    plt.ylabel("y / mm")
    plt.title("Smooth Trajectory Planning Test")
    plt.grid(True)
    plt.legend()
    plt.show()