from pathlib import Path

import numpy as np
import json
import time

from multi_controller import MultiController

_BASE = Path(__file__).resolve().parent
SCARA_V1_BASIC_CONFIG = _BASE / "scara_v1_basic_config.json"

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
            print("arm_com_port and arm_config_data must be provided.")
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

    # 初始化
    def initialize(self):
        self.base_joint.initialize()
        self.middle_joint.initialize()

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
            cali_sign_base = 1 if self.arm_type == "left" else -1
            cali_sign_middle = -1 if self.arm_type == "left" else 1
            # 先确定基座关节的物理卡销位置
            self.base_joint.up_enable_motor()
            self.base_joint.set_target_speed(9 * cali_sign_base, 3)
            time.sleep(0.1) # 避开启动时的峰值电流
            while True:
                curr_curr = self.base_joint.get_instantaneous_current().get("content")
                if abs(curr_curr) > 2.95:
                    time.sleep(2.0) # 稳定时间
                    base_mark_pos = self.base_joint.get_instantaneous_position().get("content")
                    time.sleep(0.5)
                    break
                time.sleep(0.01)
            delta_base = base_mark_pos - self.signs[0] * (-75) * cali_sign_base # ctrl - set

            # 再确定中间关节的物理校准点位置
            self.middle_joint.up_enable_motor()
            self.middle_joint.set_target_speed(9 * cali_sign_middle, 3)
            time.sleep(0.1) # 避开启动时的峰值电流
            while True:
                curr_curr = self.middle_joint.get_instantaneous_current().get("content")
                if abs(curr_curr) > 2.95:
                    time.sleep(2.0) # 稳定时间
                    middle_mark_pos = self.middle_joint.get_instantaneous_position().get("content")
                    print(f"中间关节物理校准点位置: {middle_mark_pos}")
                    time.sleep(0.5)
                    break
                time.sleep(0.01)
            delta_middle = middle_mark_pos + self.signs[1] * 15 * cali_sign_middle # ctrl - set
            # print(f"delta_base: {delta_base}")
            # print(f"delta_middle: {delta_middle}")
            # 移动到伸直姿势
            # print(f"基座关节目标位置: {self.signs[0] * set_b_joint_angle + delta_base}")
            # print(f"中间关节目标位置: {self.signs[1] * set_m_joint_angle + delta_middle}")
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

    def position_inverse_kinematics(
        self,
        end_effector_x,
        end_effector_y,
        bulge_direction=None,
    ):
        """
        逆运动学：给定末端平面坐标 (x, y)（与 position_forward_kinematics 同一基座坐标系），
        求逻辑关节角 (base_joint_angle, middle_joint_angle)，单位为度。

        与正解一致：第一段世界角 b_temp 与第二段世界角 psi = m + b + pi/2，
        相对角 beta = psi - b_temp（左臂 beta = m - pi，右臂 beta = m + pi）。

        Args:
            end_effector_x, end_effector_y: 末端位置（与 FK 输出同单位，通常为 mm）。
            bulge_direction: "CW" / "CCW"，与 reach_target_joints_pos 中折角判定一致；为 None 时用 self.bulge_direction。

        Returns:
            (b_joint_angle_deg, m_joint_angle_deg) 或在工作空间外 / 无解时返回 None。
        """
        L1 = float(self.distance_base_link)
        L2 = float(self.distance_middle_link)
        x = float(end_effector_x)
        y = float(end_effector_y)
        r2 = x * x + y * y
        denom = 2.0 * L1 * L2
        if denom <= 0.0:
            return None
        cos_beta = (r2 - L1 * L1 - L2 * L2) / denom
        if cos_beta < -1.0 or cos_beta > 1.0:
            return None
        sin_mag = float(np.sqrt(max(0.0, 1.0 - cos_beta * cos_beta)))
        betas = (
            float(np.arctan2(sin_mag, cos_beta)),
            float(np.arctan2(-sin_mag, cos_beta)),
        )
        prefer = bulge_direction if bulge_direction is not None else self.bulge_direction

        def _bm_from_beta(beta_r):
            theta1_r = float(np.arctan2(y, x) - np.arctan2(L2 * np.sin(beta_r), L1 + L2 * np.cos(beta_r)))
            if self.arm_type == "left":
                b_r = theta1_r - 3.0 * np.pi / 2.0
                m_r = beta_r + np.pi
            else:
                b_r = theta1_r + np.pi / 2.0
                m_r = beta_r - np.pi
            b_deg = float(np.degrees(b_r))
            m_deg = float(np.degrees(m_r))
            return b_deg, m_deg

        def _bulge_for_solution(b_deg, m_deg):
            """基于归一化角度判定折角方向（嵌套函数勿写 self，通过闭包使用外层 self）。"""
            def norm(a):
                while a >= 360.0:
                    a -= 360.0
                while a < 0.0:
                    a += 360.0
                return a
            
            if self.arm_type == "left":
                theta1 = norm(b_deg + 270.0)
                theta2 = norm(m_deg + b_deg + 90.0)
            else:
                theta1 = norm(b_deg - 90.0)
                theta2 = norm(m_deg + b_deg + 90.0)
            return "CW" if theta1 <= theta2 else "CCW"

        candidates = []
        for beta_r in betas:
            b_deg, m_deg = _bm_from_beta(beta_r)
            if (
                self.base_joint.movement_range["min"] <= b_deg <= self.base_joint.movement_range["max"]
                and self.middle_joint.movement_range["min"] <= m_deg <= self.middle_joint.movement_range["max"]
            ):
                xe, ye = self.position_forward_kinematics(b_deg, m_deg)
                err = (xe - x) ** 2 + (ye - y) ** 2
                candidates.append((err, b_deg, m_deg, _bulge_for_solution(b_deg, m_deg)))

        if not candidates:
            for beta_r in betas:
                b_deg, m_deg = _bm_from_beta(beta_r)
                xe, ye = self.position_forward_kinematics(b_deg, m_deg)
                err = (xe - x) ** 2 + (ye - y) ** 2
                candidates.append((err, b_deg, m_deg, _bulge_for_solution(b_deg, m_deg)))

        candidates.sort(key=lambda t: t[0])
        matching = [c for c in candidates if c[3] == prefer]
        pick = matching[0] if matching else candidates[0]
        return pick[1], pick[2]
 
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
            print(f"Target base joint angle {target_b_joint_angle}° is out of the base joint's movement range.")
            raise ValueError(f"Target base joint angle {target_b_joint_angle}° is out of the base joint's movement range.")
        if target_m_joint_angle < self.middle_joint.movement_range["min"] or target_m_joint_angle > self.middle_joint.movement_range["max"]:
            print(f"Target middle joint angle {target_m_joint_angle}° is out of the middle joint's movement range.")
            raise ValueError(f"Target middle joint angle {target_m_joint_angle}° is out of the middle joint's movement range.")

        # 速度检查
        b_speed_ratio, m_speed_ratio = speed_ratio
        if b_speed_ratio < 0 or m_speed_ratio < 0:
            print(f"Speed ratio {speed_ratio} should be non-negative.")
            raise ValueError(f"Speed ratio {speed_ratio} should be non-negative.")
        if b_speed_ratio > 1.0 or m_speed_ratio > 1.0:
            print(f"Speed ratio {speed_ratio} should be in the range of [0.0, 1.0].")
            raise ValueError(f"Speed ratio {speed_ratio} should be in the range of [0.0, 1.0].")
        b_joint_speed = self.base_joint.rated_speed * b_speed_ratio
        m_joint_speed = self.middle_joint.rated_speed * m_speed_ratio

        # 判断运动成功距离数值的合理性
        if blocking and (base_blocking_joint_angle <= 0 or middle_blocking_joint_angle <= 0):
            print(f"The blocking joint angle {base_blocking_joint_angle} and {middle_blocking_joint_angle} should be positive.")
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

    def reach_target_position(
        self,
        target_position=(0.0, 0.0),
        bulge_direction=None,
        speed_ratio=(1.0, 1.0),
        blocking: bool = False,
        base_blocking_joint_angle=0.5,
        middle_blocking_joint_angle=0.5,
    ):
        """
        移动到目标末端平面坐标 (x, y)，与 get_current_pos / position_forward_kinematics 同一坐标系（通常为 mm）。
        先按折角方向做逆解，再调用 reach_target_joints_pos。

        Args:
            target_position: (end_effector_x, end_effector_y)
            bulge_direction: "CW" / "CCW"，与逆解选解一致；为 None 时使用 self.default_bulge_direction。
            speed_ratio, blocking, base_blocking_joint_angle, middle_blocking_joint_angle: 与 reach_target_joints_pos 相同。
        """
        if bulge_direction is None:
            bulge_direction = self.default_bulge_direction
        if bulge_direction not in ("CW", "CCW"):
            raise ValueError(
                f"Bulge direction {bulge_direction!r} is not supported. Use 'CW' or 'CCW'."
            )

        target_x, target_y = target_position
        r_max = float(self.distance_base_link) + float(self.distance_middle_link)
        if target_x * target_x + target_y * target_y > r_max * r_max:
            raise ValueError(
                f"Target position {target_position} is out of the arm's reachable workspace."
            )

        sol = self.position_inverse_kinematics(target_x, target_y, bulge_direction=bulge_direction)
        if sol is None:
            raise ValueError(
                f"Target position {target_position} has no inverse kinematics solution for "
                f"bulge_direction={bulge_direction!r} (e.g. inner workspace hole or limits)."
            )

        return self.reach_target_joints_pos(
            sol,
            speed_ratio,
            blocking,
            base_blocking_joint_angle,
            middle_blocking_joint_angle,
        )

    # 急停
    def stop(self):
        self.base_joint.down_enable_motor() # 掉使能代替急停
        self.middle_joint.down_enable_motor() # 掉使能代替急停
        return True


# 关节驱动型SCARA升降模块
class ScaraLiftingJoint(MultiController):
    def __init__(
        self, 
        robot_id=0,
        lifting_com_port=None,
        lifting_config_data=None,
    ):  
        self.robot_id = robot_id
        self.com_port = lifting_com_port
        self.lifting_config_data = lifting_config_data
        controller_name = self.lifting_config_data.get("controller_name")
        self.max_travel = self.lifting_config_data.get("max_travel") # mm
        self.max_speed_mm = self.lifting_config_data.get("max_speed_mm") # mm/s
        self.screw_lead = self.lifting_config_data.get("screw_lead") # mm
        calibration_config = self.lifting_config_data.get("calibration")


        controller_maps = {
            "lifting_joint": controller_name
        }
        super().__init__(controller_maps=controller_maps, com_port=lifting_com_port)

        # 读取校准配置数据
        self.zero_pos = calibration_config.get("zero_pos", None)
        self.sign = calibration_config.get("sign", 1)

    # 上使能
    def up_enable(self):
        self.lifting_joint.up_enable_motor()
        
    # 下使能
    def down_enable(self):
        self.lifting_joint.down_enable_motor()

    # 校准
    def calibration(self, auto=True):
        # 自动找物理零点
        if auto:
            # 启动回零
            self.lifting_joint.set_acceleration(3)
            # 先确定基座关节的物理卡销位置
            self.lifting_joint.up_enable_motor()
            self.lifting_joint.set_target_speed(-5, 3)
            time.sleep(0.1) # 避开启动时的峰值电流
            while True:
                curr_curr = self.lifting_joint.get_instantaneous_current().get("content")
                print(f"当前电流: {curr_curr}")
                if abs(curr_curr) > 2.95:
                    time.sleep(2.0) # 稳定时间
                    break
                time.sleep(0.01)
        # 手动找物理零点
        else:
            self.lifting_joint.down_enable_motor()
            input(f"确保升降滑块已摆放到校准所需最低点零点位置，并按下回车：")

        # 首先设置为零点
        self.lifting_joint.set_zero_point()
        # 上移到中间位置
        self.lifting_joint.up_enable_motor()
        self.lifting_joint.set_target_position(125, 10)
        time.sleep(4)
        # 再次设置为零点
        self.lifting_joint.set_zero_point()
        # 获取位置
        temp_pos = self.lifting_joint.get_instantaneous_position().get("content")

        # 写文件
        self.zero_pos = -125 + temp_pos
        self.lifting_config_data["calibration"]["zero_pos"] = self.zero_pos
        with open(SCARA_V1_BASIC_CONFIG, 'r') as f_config_r:
            config_data_full = json.load(f_config_r)
        config_data_full[str(self.robot_id)]["lifting"] = self.lifting_config_data
        with open(SCARA_V1_BASIC_CONFIG, 'w') as f_config_w:
            json.dump(config_data_full, f_config_w, indent=4)

    # 获取当前升降、电机位置
    def get_current_pos(self):
        current_pos_raw = self.lifting_joint.get_instantaneous_position().get("content")
        current_height = (self.sign * self.screw_lead * (current_pos_raw - self.zero_pos) * self.lifting_joint.scale_factor) / 360
        return {
            "lifting_motor_pos": current_pos_raw,
            "lifting_height": current_height
        }

    # 移动到目标高度，单位mm
    def move_to_height(self, height_mm: float, speed_ratio: float=0.5, blocking: bool=False, blocking_angle: float = 0.1):
        # 目标高度检查
        if height_mm < 0:
            print(f"Target height {height_mm}mm is below 0mm.")
            raise ValueError(f"Target height {height_mm}mm is below 0mm.")
        if height_mm > self.max_travel:
            print(f"Target height {height_mm}mm exceeds the max travel {self.max_travel}mm.")
            raise ValueError(f"Target height {height_mm}mm exceeds the max travel {self.max_travel}mm.")
        
        # 目标线速度检查
        if speed_ratio < 0:
            print(f"Speed ratio should be non-negative.")
            raise ValueError(f"Speed ratio should be non-negative.")
        if speed_ratio > 1.0:
            print(f"Speed ratio {speed_ratio} exceeds 1.0.")
            raise ValueError(f"Speed ratio {speed_ratio} exceeds 1.0.")
        speed_mm_s = speed_ratio*self.max_speed_mm

        # 判断运动成功距离数值的合理性
        if blocking and blocking_angle <= 0:
            print(f"The angle to judge moving success status {blocking_angle} should be positive.")
            raise ValueError(f"The angle to judge moving success status {blocking_angle} should be positive.")

        if not self.lifting_joint.ENABLE:
            self.lifting_joint.up_enable_motor()
        
        try:
            start_angle = self.lifting_joint.get_instantaneous_position().get("content")
            target_angle = (self.sign * 360 * height_mm) / (self.screw_lead * self.lifting_joint.scale_factor) + self.zero_pos

            speed_rpm = 60*speed_mm_s / self.screw_lead
            speed_rpm = speed_rpm / self.lifting_joint.scale_factor

            self.lifting_joint.set_target_position(target_angle, int(speed_rpm))

            # 计算行程
            delta_move = abs(start_angle - target_angle)

            # 根据阻塞/非阻塞模式返回执行状态
            if blocking:
                # 计算运行等待时间
                ## 计算基座关节
                speed_dps = speed_rpm * 6
                base_acc_angle = self.lifting_joint.acc_val * 180 / np.pi
                if speed_dps**2 >= base_acc_angle*delta_move:
                    waiting_time = np.sqrt(delta_move / base_acc_angle)
                else:
                    waiting_time = delta_move / speed_dps + speed_dps / base_acc_angle
                waiting_time = max(0.0, (delta_move - blocking_angle) / delta_move) * waiting_time

                ## 执行运动等待
                time.sleep(waiting_time)

                # 尝试获取运动到位信息
                try_times = 0
                while try_times < 100:
                    new_joint_angle = self.lifting_joint.get_instantaneous_position().get("content")
                    if abs(target_angle - new_joint_angle) < blocking_angle:
                        return True
                    try_times += 1
                    time.sleep(0.01)
                return False
            else:
                return True
        except Exception as e:
            print(f"Error occurred while moving lifting to height {height_mm} on {self.com_port}: {e}")
            return False

    # 停下
    def stop(self):
        self.lifting_joint.down_enable_motor()