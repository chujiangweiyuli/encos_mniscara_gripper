# arm_controller.py
"""
SCARA 基座 + 中间关节控制，对齐本仓库中：
- multi_controller.create_controller + controller_config.json 关节名与行程
- scara_v1.py：逻辑角 ↔ 电机控制角（offsets / signs）、正运动学左右臂分支
- encos_controller.initialize()：加速度与按配置使能（默认不再在 initialize 里强制 set_zero）
"""
from pathlib import Path
import json
import time
import numpy as np

from multi_controller import create_controller


_BASE = Path(__file__).resolve().parent
SCARA_V1_BASIC_CONFIG = _BASE / "scara_v1_basic_config.json"


class ScaraArmController:
    """
    SCARA 双臂关节控制器（单臂一条配置：基座 + 中间）。

    逻辑关节角与 scara_v1 一致；下发 CAN 前换算为控制角：
        ctrl = sign * logical + offset
    读回：logical = sign * (ctrl - offset)
    """

    def __init__(
        self,
        com_port="ZLG_31F10005727_1",
        robot_id=0,
        arm_config=None,
        basic_config_path=None,
    ):
        cfg_path = Path(basic_config_path) if basic_config_path else SCARA_V1_BASIC_CONFIG
        if arm_config is None:
            with open(cfg_path, "r", encoding="utf-8") as f:
                full = json.load(f)
            arm_config = full[str(robot_id)]["arm"]

        names = arm_config["controller_names"]
        self.base_joint = create_controller(names[0], com_port)
        self.middle_joint = create_controller(names[1], com_port)

        links = arm_config.get("link_lengths", [220.0, 220.0])
        self.distance_base_link = float(links[0])
        self.distance_middle_link = float(links[1])

        cal = arm_config.get("calibration") or {}
        self.offsets = list(cal.get("offsets", [0.0, 0.0]))
        self.signs = list(cal.get("signs", [1, 1]))
        self.arm_type = arm_config.get("arm_type", "left")
        self.default_bulge_direction = "CCW" if self.arm_type == "left" else "CW"

        self.speed_ratio = 0.01
        self.zero_set = False

    def _logical_to_ctrl(self, base_logical, middle_logical):
        b_ctrl = self.signs[0] * base_logical + self.offsets[0]
        m_ctrl = self.signs[1] * middle_logical + self.offsets[1]
        return b_ctrl, m_ctrl

    def _ctrl_to_logical(self, base_ctrl, middle_ctrl):
        b_log = self.signs[0] * (base_ctrl - self.offsets[0])
        m_log = self.signs[1] * (middle_ctrl - self.offsets[1])
        return b_log, m_log

    def initialize(self, rezero=False):
        """
        初始化两关节。
        rezero=False（默认）：各关节 motor.initialize()（加速度 + 按配置使能），不改写零点。
        rezero=True：下使能 → 当前位置设零 → 加速度 → 上使能（台架标定用）。
        """
        print("=" * 50)
        print("[Arm] Initializing SCARA arm...")

        ok_base = True
        ok_mid = True

        if rezero:
            print("[Arm] Re-zero mode: base joint...")
            self.base_joint.down_enable_motor()
            time.sleep(0.2)
            rb = self.base_joint.set_zero_point()
            ok_base = rb.get("success") and rb.get("content")
            if ok_base:
                print("[Arm] Base joint zero set")
            self.base_joint.set_acceleration(self.base_joint.acc_val)
            self.base_joint.up_enable_motor()
            time.sleep(0.5)

            print("[Arm] Re-zero mode: middle joint...")
            self.middle_joint.down_enable_motor()
            time.sleep(0.2)
            rm = self.middle_joint.set_zero_point()
            ok_mid = rm.get("success") and rm.get("content")
            if ok_mid:
                print("[Arm] Middle joint zero set")
            self.middle_joint.set_acceleration(self.middle_joint.acc_val)
            self.middle_joint.up_enable_motor()
            time.sleep(0.5)
        else:
            print("[Arm] Initializing base joint (Encos initialize)...")
            self.base_joint.initialize()
            time.sleep(0.2)
            print("[Arm] Initializing middle joint (Encos initialize)...")
            self.middle_joint.initialize()
            time.sleep(0.2)

        self.zero_set = ok_base and ok_mid if rezero else True
        print(f"[Arm] Initialization complete, ready={self.zero_set}")
        return self.zero_set

    def up_enable(self):
        """双臂上使能"""
        self.base_joint.up_enable_motor()
        time.sleep(0.2)
        self.middle_joint.up_enable_motor()
        time.sleep(0.2)

    def down_enable(self):
        """双臂下使能"""
        self.base_joint.down_enable_motor()
        time.sleep(0.2)
        self.middle_joint.down_enable_motor()
        time.sleep(0.2)

    def _get_speed_rpm(self, joint, speed_ratio):
        """速度比例 → 下发 set_target_position 的 RPM（与 gripper / encos 约定一致）。"""
        return max(1, int(joint.rated_speed * speed_ratio / 6))

    def get_joint_angles(self):
        """
        当前逻辑关节角（度），与 scara_v1.get_current_pos(joint_only=True) 一致。
        """
        base_pos = self.base_joint.get_instantaneous_position()
        middle_pos = self.middle_joint.get_instantaneous_position()

        if base_pos["success"] and middle_pos["success"]:
            return self._ctrl_to_logical(base_pos["content"], middle_pos["content"])
        return None, None

    def move_joint(self, joint, target_angle, speed_ratio=None, blocking=True):
        """
        单关节运动到目标逻辑角度（度）。行程以 controller_config 中 movement_range 为准。
        """
        if not self.zero_set:
            print("[Arm] Not ready, initializing...")
            self.initialize()

        if speed_ratio is None:
            speed_ratio = self.speed_ratio

        if joint == "base":
            controller = self.base_joint
            min_angle = self.base_joint.movement_range["min"]
            max_angle = self.base_joint.movement_range["max"]
            cur_b, cur_m = self.get_joint_angles()
            if cur_b is None:
                return False
            target_b, target_m = target_angle, cur_m
        elif joint == "middle":
            controller = self.middle_joint
            min_angle = self.middle_joint.movement_range["min"]
            max_angle = self.middle_joint.movement_range["max"]
            cur_b, cur_m = self.get_joint_angles()
            if cur_m is None:
                return False
            target_b, target_m = cur_b, target_angle
        else:
            print(f"[Arm] Invalid joint: {joint}")
            return False

        if target_angle < min_angle or target_angle > max_angle:
            print(f"[Arm] Target angle {target_angle}° out of range [{min_angle}, {max_angle}]")
            return False

        b_ctrl, m_ctrl = self._logical_to_ctrl(target_b, target_m)
        send_ctrl = b_ctrl if joint == "base" else m_ctrl

        speed = self._get_speed_rpm(controller, speed_ratio)
        print(f"[Arm] Moving {joint} joint (logical target {target_angle}°) ctrl={send_ctrl:.3f}° at {speed} RPM")

        result = controller.set_target_position(pos_deg=send_ctrl, spd_val=speed)

        if result["success"] and blocking:
            cur = controller.get_instantaneous_position()
            if cur["success"]:
                distance = abs(send_ctrl - cur["content"])
                # rated_speed 为 DPS（encos 内已 ×6）
                dps = max(controller.rated_speed * speed_ratio, 1e-6)
                wait_time = distance / dps
                time.sleep(max(wait_time, 0.5))

        return result["success"] if isinstance(result, dict) else result

    def move_joints(self, base_angle, middle_angle, speed_ratio=None, blocking=True):
        """双臂同时到目标逻辑角度（度）。"""
        if not self.zero_set:
            self.initialize()

        if speed_ratio is None:
            speed_ratio = self.speed_ratio

        b_min, b_max = self.base_joint.movement_range["min"], self.base_joint.movement_range["max"]
        m_min, m_max = self.middle_joint.movement_range["min"], self.middle_joint.movement_range["max"]
        if base_angle < b_min or base_angle > b_max:
            print(f"[Arm] Base angle {base_angle}° out of range [{b_min}, {b_max}]")
            return False
        if middle_angle < m_min or middle_angle > m_max:
            print(f"[Arm] Middle angle {middle_angle}° out of range [{m_min}, {m_max}]")
            return False

        b_ctrl, m_ctrl = self._logical_to_ctrl(base_angle, middle_angle)
        base_speed = self._get_speed_rpm(self.base_joint, speed_ratio)
        middle_speed = self._get_speed_rpm(self.middle_joint, speed_ratio)

        print(f"[Arm] Moving to base={base_angle}°, middle={middle_angle}° (ctrl base={b_ctrl:.3f}, mid={m_ctrl:.3f})")

        self.base_joint.set_target_position(pos_deg=b_ctrl, spd_val=base_speed)
        self.middle_joint.set_target_position(pos_deg=m_ctrl, spd_val=middle_speed)

        if blocking:
            max_wait = 10.0
            start_time = time.time()
            while time.time() - start_time < max_wait:
                bl, ml = self.get_joint_angles()
                if bl is not None and ml is not None:
                    if abs(bl - base_angle) < 1.0 and abs(ml - middle_angle) < 1.0:
                        print("[Arm] Both joints in position")
                        return True
                time.sleep(0.05)
            print("[Arm] Timeout waiting for joints")
            return False

        return True

    def go_home(self, speed_ratio=None, blocking=True):
        """逻辑零位 (0°, 0°)。"""
        if not self.zero_set:
            self.initialize()
        print("[Arm] Going home (logical 0, 0)...")
        return self.move_joints(0, 0, speed_ratio, blocking)

    def jog_joint(self, joint, delta_angle):
        """点动单关节（逻辑角）。"""
        if not self.zero_set:
            return False

        current = self.get_joint_angles()
        if current[0] is None:
            return False

        base_angle, middle_angle = current

        if joint == "base":
            target = base_angle + delta_angle
            return self.move_joint("base", target, speed_ratio=0.1, blocking=True)
        if joint == "middle":
            target = middle_angle + delta_angle
            return self.move_joint("middle", target, speed_ratio=0.1, blocking=True)

        return False

    def forward_kinematics(self, base_angle, middle_angle):
        """逻辑关节角（度）→ 末端 (x, y) mm，与 scara_v1.position_forward_kinematics 一致。"""
        b_joint_angle_r = np.radians(base_angle)
        m_joint_angle_r = np.radians(middle_angle)

        if self.arm_type == "left":
            b_temp_angle_r = b_joint_angle_r + 3 * np.pi / 2
        else:
            b_temp_angle_r = b_joint_angle_r - np.pi / 2

        x = self.distance_base_link * np.cos(b_temp_angle_r) + self.distance_middle_link * np.cos(
            m_joint_angle_r + b_joint_angle_r + np.pi / 2
        )
        y = self.distance_base_link * np.sin(b_temp_angle_r) + self.distance_middle_link * np.sin(
            m_joint_angle_r + b_joint_angle_r + np.pi / 2
        )
        return x, y

    def inverse_kinematics(self, x, y, bulge_direction="CCW"):
        """末端 (mm) → 逻辑关节角（度）。折角方向见 scara_v1 中 bulge 约定。"""
        r_sq = x**2 + y**2
        r = np.sqrt(r_sq)

        if r > self.distance_base_link + self.distance_middle_link:
            print(f"[Arm] Target ({x}, {y}) out of reach")
            return None, None

        cos_bulge = (
            self.distance_base_link**2 + self.distance_middle_link**2 - r_sq
        ) / (2 * self.distance_base_link * self.distance_middle_link)
        bulge_angle = np.arccos(np.clip(cos_bulge, -1, 1))

        cos_offset = (self.distance_base_link**2 + r_sq - self.distance_middle_link**2) / (
            2 * self.distance_base_link * r
        )
        offset_angle = np.arccos(np.clip(cos_offset, -1, 1))

        straight_angle = np.arctan2(y, x)
        if straight_angle < 0:
            straight_angle += 2 * np.pi

        if bulge_direction == "CCW":
            middle_angle = bulge_angle
            base_pre = straight_angle + offset_angle
            base_angle = base_pre - 3 * np.pi / 2
        else:
            middle_angle = 2 * np.pi - bulge_angle
            base_pre = straight_angle - offset_angle
            base_angle = base_pre - 3 * np.pi / 2

        base_deg = np.degrees(base_angle)
        middle_deg = np.degrees(middle_angle)

        return base_deg, middle_deg

    def move_to_position(self, x, y, bulge_direction=None, speed_ratio=None, blocking=True):
        """逆解后 move_joints。bulge_direction 默认随 arm_type（与 scara_v1 默认折向一致）。"""
        if bulge_direction is None:
            bulge_direction = self.default_bulge_direction

        base_angle, middle_angle = self.inverse_kinematics(x, y, bulge_direction)

        if base_angle is None:
            return False

        print(f"[Arm] IK solution: base={base_angle:.2f}°, middle={middle_angle:.2f}°")

        return self.move_joints(base_angle, middle_angle, speed_ratio, blocking)

    def get_status(self):
        """状态：逻辑角、是否在 movement_range 内、末端位置。"""
        base_pos = self.base_joint.get_instantaneous_position()
        middle_pos = self.middle_joint.get_instantaneous_position()

        status = {
            "zero_set": self.zero_set,
            "arm_type": self.arm_type,
            "base": {"enabled": self.base_joint.ENABLE},
            "middle": {"enabled": self.middle_joint.ENABLE},
        }

        if base_pos["success"] and middle_pos["success"]:
            bl, ml = self._ctrl_to_logical(base_pos["content"], middle_pos["content"])
            status["base"]["position_logical"] = bl
            status["base"]["position_ctrl"] = base_pos["content"]
            status["middle"]["position_logical"] = ml
            status["middle"]["position_ctrl"] = middle_pos["content"]
            b_min, b_max = self.base_joint.movement_range["min"], self.base_joint.movement_range["max"]
            m_min, m_max = self.middle_joint.movement_range["min"], self.middle_joint.movement_range["max"]
            status["base"]["in_range"] = b_min <= bl <= b_max
            status["middle"]["in_range"] = m_min <= ml <= m_max
            ex, ey = self.forward_kinematics(bl, ml)
            status["end_effector"] = {"x": ex, "y": ey}

        return status

    def emergency_stop(self):
        print("!!! ARM EMERGENCY STOP !!!")
        self.base_joint.down_enable_motor()
        self.middle_joint.down_enable_motor()


# === 测试脚本 ===
if __name__ == "__main__":
    arm = ScaraArmController("ZLG_31F10005727_1", robot_id=0)

    print("=" * 60)
    print("SCARA ARM TEST (logical angles, config from scara_v1_basic_config.json)")
    print("=" * 60)

    arm.initialize()
    time.sleep(1)

    status = arm.get_status()
    print(f"\nStatus: {status}")

    # print("\n[Test 1] Move base joint to -180° (logical)")
    # arm.move_joint("base", -180, speed_ratio=0.2)
    # time.sleep(2)

    # print("\n[Test 2] Move middle joint to 180° (logical)")
    # arm.move_joint("middle", 180, speed_ratio=0.2)
    # time.sleep(2)

    # print("\n[Test 3] Move both joints to home (logical 0,0)")
    # arm.go_home(speed_ratio=0.2)
    # time.sleep(2)

    # print("\n[Test 4] Jog base joint +5° (logical)")
    # arm.jog_joint("base", 5)
    # time.sleep(1)

    # print("\n[Test 5] Move to position (300, 200) mm")
    # arm.move_to_position(300, 200, bulge_direction=None, speed_ratio=0.2)
    # time.sleep(3)

    # print("\n[Test 6] Return home")
    # arm.go_home(speed_ratio=0.2)

    arm.down_enable()
    print("\nDone.")
