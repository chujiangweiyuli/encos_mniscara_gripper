# from scara_v1 import ScaraArm, ScaraLiftingJoint
# from multi_controller import  EncosCanController,MultiController
from encos_controller import EncosCanController
import time
import json
from pathlib import Path

_BASE = Path(__file__).resolve().parent
JOINT_MODULE_CTRL_CONTROLLER_CONFIG = _BASE / "controller_config.json"
with open(JOINT_MODULE_CTRL_CONTROLLER_CONFIG, 'r') as f_ctrl_conf:
    controller_data = json.load(f_ctrl_conf).get("left_scara_lifting_can", {})
    print(controller_data)
# arm = ScaraArm(robot_id=1, arm_com_port="ZLG_31F10005727_0", arm_config_data=None)
# lifting = ScaraLiftingJoint(robot_id=3, lifting_com_port="ZLG_31F10005727_1", lifting_config_data=controller_data)

# arm.initialize()
lifting = EncosCanController(com_port="ZLG_31F10005727_1", controller_name="left_scara_lifting_can")
lifting.initialize()

# arm.calibration()
# lifting.calibration()

nums = 15

pick_pos = [145, -120]
place_pos = [200, -40]

start_time = time.time()
while nums > 0:
    lifting.up_enable_motor()
    lifting.set_target_position(pos_deg=145, spd_val=100)
    time.sleep(10)
    # lifting.set_target_position(pos_deg=-120, spd_val=100)
    # time.sleep(5)
    # arm.move_to_joints_pos(target_joints_pos=pick_pos, speed_ratio=(1.0,1.0), blocking=True,base_blocking_joint_angle=0.5,middle_blocking_joint_angle=0.5)

    lifting.down_enable_motor()
    # arm.move_to_joints_pos(target_joints_pos=(0, 0), speed_ratio=(1.0,1.0), blocking=True,base_blocking_joint_angle=0.5,middle_blocking_joint_angle=0.5)
    nums -= 1

end_time = time.time()
print(f"Time taken: {end_time - start_time} seconds")