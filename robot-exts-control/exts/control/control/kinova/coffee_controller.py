from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.prims import XFormPrim

from .kinova import Kinova
from .rmpflow_controller import RMPFlowController
import numpy as np
from .numpy_utils import *
from .utils import regulate_degree

import asyncio
from .kinova_socket import KinovaClient

class CoffeeMakerController(BaseController):
    def __init__(self, name: str, kinova: Kinova, connect_server = False) -> None: 
        BaseController.__init__(self, name=name)
        self.event = "move" # action event
        self.total_event_count = 0 # event time
        self.event_elapsed = 0 # event elapsed time
        self.event_pool = [] # event pool
        self.robot = kinova
        self.gripper = self.robot.gripper
        self.cs_controller = RMPFlowController(name="cspace_controller", robot_articulation=self.robot)
        
        # TODO：find height
        self.ee_pos_target = np.array([0.0, 0.0, 1.0])
        self.ee_ori_target = np.array([1.0, 0.0, 0.0, 0.0])

        # connection
        self.connect_server = connect_server
        if connect_server:
            self.client = KinovaClient()

    def add_event_to_pool(self, event: str, elapsed: int, ee_pos: np.ndarray, ee_ori: np.ndarray):
        self.event_pool.append([event, elapsed, ee_pos, ee_ori])
        
    def update_ee_target(self, pos, ori):
        """
        Update End-Effector Target position and orientation
        """
        self.ee_pos_target = pos
        self.ee_ori_target = ori

    def update_event(self, event: str):
        """
        Update robot high-level event
        """
        if event != self.event:
            self.event = event
            self.total_event_count = 0

    async def synchronize_robot(self):
        """
        Send message to the Server to 
        """
        # get joint positions and gripper degree
        all_positions = self.robot.get_joint_positions()
        gripper_degree = regulate_degree(all_positions[7], indegree=False)
        joint_positions = [regulate_degree(e, indegree=False) for e in all_positions[:7]]
        joint_positions = joint_positions + [gripper_degree]

        assert len(joint_positions) == 8, "Invalid number of joint positions"

        # send message
        message = " ".join([str(e) for e in joint_positions])
        self.client.send_message(message)
        
    def forward(self):
        """
        Main function to update the robot
        """
        # update event
        if len(self.event_pool) > 0:
            if self.event_elapsed <= 0:
                event, elapsed, ee_pos, ee_ori = self.event_pool.pop(0)
                self.update_event(event)
                self.update_ee_target(ee_pos, ee_ori)
                self.event_elapsed = elapsed

        if self.event == "move":
            actions = self.cs_controller.forward(
                    target_end_effector_position=self.ee_pos_target,
                    target_end_effector_orientation=self.ee_ori_target)    
        elif self.event == "close":
            actions = self.gripper.forward(action="close")
        elif self.event == "open":
            actions = self.gripper.forward(action="open")

        self.robot.apply_action(actions)

        # from omni.isaac.core.utils.types import ArticulationAction
        # joint_actions = ArticulationAction()
    
        # joint_actions.joint_positions = [0, 15, 180, -130, 0, 55, 90] + [0.8] * 6
        # for i in range(13):
        #     joint_actions.joint_positions[i] = np.deg2rad(joint_actions.joint_positions[i])

        # print("joint_actions", joint_actions)

        
        # self.robot.apply_action(joint_actions)
        
        self.total_event_count += 1 # update event time
        self.event_elapsed -= 1 # update event elapsed time
        # synchronize
        if self.connect_server:
            if self.total_event_count % 60 == 0:
                asyncio.ensure_future(self.synchronize_robot())


        # return actions
    
    