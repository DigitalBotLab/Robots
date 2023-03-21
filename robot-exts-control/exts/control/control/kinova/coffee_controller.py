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
        self._end_effector_initial_height = 0.4 / get_stage_units()
        self._pause = False
        
    def forward(self, target_end_effector_position, target_end_effector_orientation):
        actions = self._controller.forward(
                target_end_effector_position=target_end_effector_position,
                target_end_effector_orientation=target_end_effector_orientation)
        
        return actions
    
    