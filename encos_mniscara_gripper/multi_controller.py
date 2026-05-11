import json
from pathlib import Path

# from encos_controller import EncosCanController, EncosMCUController
from encos_controller import EncosCanController

_BASE = Path(__file__).resolve().parent
JOINT_MODULE_CTRL_CONTROLLER_CONFIG = _BASE / "controller_config.json"

def create_controller(controller_name, com_port, serial=None):
    with open(JOINT_MODULE_CTRL_CONTROLLER_CONFIG, 'r') as f_ctrl_conf:
        controller_data = json.load(f_ctrl_conf).get(controller_name, {})
        module_type = controller_data.get("module_type")
        com_protocol = controller_data.get("com_protocol")

        if module_type == "Encos":
            if com_protocol == "CAN":
                return EncosCanController(com_port, controller_name)
            # if com_protocol == "ESP-CAN":
            #     return EncosMCUController(com_port, controller_name, serial)

class MultiController:
    def __init__(self, controller_maps, com_port, serial=None):
        for controller_variable in controller_maps.keys():
            controller_name = controller_maps[controller_variable]
            setattr(self, controller_variable, create_controller(controller_name, com_port, serial))
            print(f"Controller '{controller_variable}' of type '{controller_name}' initialized on port '{com_port}'.")