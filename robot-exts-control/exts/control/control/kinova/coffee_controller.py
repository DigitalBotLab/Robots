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
        self.event = "move" # action event
        self.event_t = 0 # event time
        self.robot = kinova
        self.gripper = self.robot.gripper
        self.cs_controller = RMPFlowController(name="cspace_controller", robot_articulation=self.robot)
        
        # TODO：find height
        self.ee_pos_target = np.array([0.0, 0.0, 1.0])
        self.ee_ori_target = np.array([1.0, 0.0, 0.0, 0.0])
        
    def update_ee_target(self, pos, ori):
        self.ee_pos_target = pos
        self.ee_ori_target = ori

    def update_event(self, event: str):
        if event != self.event:
            self.event = event
            self.event_t = 0
        
    def forward(self):
        if self.event == "move":
            actions = self.cs_controller.forward(
                    target_end_effector_position=self.ee_pos_target,
                    target_end_effector_orientation=self.ee_ori_target)    
        elif self.event == "close":
            actions = self.gripper.forward(action="close")
        elif self.event == "open":
            actions = self.gripper.forward(action="open")

        self.event_t += 1
        
        # print("actions: ", actions)
        self.robot.apply_action(actions)

        return actions
    
    