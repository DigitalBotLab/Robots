from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.prims import XFormPrim

from .kinova import Kinova
from .rmpflow_controller import RMPFlowController
import numpy as np
from .numpy_utils import *

class CoffeeMakerController(BaseController):
    def __init__(self, name: str, kinova: Kinova) -> None: 
        BaseController.__init__(self, name=name)
        self._event = 0
        self.robot = kinova
        self._gripper = self.robot.gripper
        self._controller = RMPFlowController(name="cspace_controller", robot_articulation=self.robot)
        
        # TODO：find height
        self.ee_pos_target = np.array([0.0, 0.0, 1.0])
        self.ee_ori_target = np.array([1.0, 0.0, 0.0, 0.0])
        
    def update_ee_target(self, pos, ori):
        self.ee_pos_target = pos
        self.ee_ori_target = ori
        
    def forward(self):
        actions = self._controller.forward(
                target_end_effector_position=self.ee_pos_target,
                target_end_effector_orientation=self.ee_ori_target)
        
        self.robot.apply_action(actions)

        return actions
    
    