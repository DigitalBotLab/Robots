import omni.usd

from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.prims import XFormPrim

from .kinova import Kinova
from .rmpflow_controller import RMPFlowController
import numpy as np
from .numpy_utils import *
from .utils import regulate_degree, get_transform_mat_from_pos_rot, generate_slerp_action_sequence

import asyncio
from .kinova_socket import KinovaClient
from .coffee_config import kinova_action_config

class CoffeeMakerController(BaseController):
    def __init__(self, name: str, kinova: Kinova, connect_server = False) -> None: 
        BaseController.__init__(self, name=name)

        # env
        self.stage = omni.usd.get_context().get_stage()

        # event
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
            self.sending_message = False
       
        # add go home default action
        # self.apply_high_level_action()
        # self.sending_message = False

    def add_event_to_pool(self, event: str, elapsed: int, 
                          ee_pos: np.ndarray, ee_ori: np.ndarray, gripper_ratio: float = 1.0):
        self.event_pool.append([event, elapsed, ee_pos, ee_ori, gripper_ratio])
        
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

################################## sync robot ##################################
    def synchronize_robot(self):
        """
        Send message to the Server to 
        """
        if not self.sending_message:
            # get joint positions and gripper degree
            all_positions = self.robot.get_joint_positions()
            gripper_degree = all_positions[7] / 0.8757
            joint_positions = [regulate_degree(e, indegree=False) for e in all_positions[:7]]
            joint_positions = joint_positions + [gripper_degree]

            assert len(joint_positions) == 8, "Invalid number of joint positions"

            # send message
            message = " ".join([str(e) for e in joint_positions])
        
            self.sending_message = True
            self.client.send_message("Control", message)
            self.sending_message = False

    def obtain_robot_state(self):
        """
        Get robot state from the Server
        """
        if not self.sending_message:
            self.sending_message = True
            answer_message = self.client.send_message("GetJoints", "NA")
            self.sending_message = False
            return [float(e) for e in answer_message.split(" ")]

    def apply_high_level_action(self, action_name: str = "go_home"):
        """
        Apply high-level action to the robot
        """
        action = kinova_action_config[action_name]
        if action['base_prim'] is None:
            base_world_pos, base_world_rot = self.robot.get_world_pose()
        else:
            base_prim = XFormPrim(action['base_prim'])
            base_world_pos, base_world_rot = base_prim.get_world_pose()
        
        base_mat = get_transform_mat_from_pos_rot(base_world_pos, base_world_rot)
        print("base_mat", base_mat)
        
        for action_step in action['steps']:

            step_type = action_step['action_type']
            duration = action_step['duration']

            if step_type == "move":
                offset_mat = get_transform_mat_from_pos_rot(action_step['position'], action_step['orientation'])
                print("offset_mat", offset_mat)

                target_mat = offset_mat * base_mat 
                print("target_mat", target_mat.ExtractTranslation(), target_mat.ExtractRotationQuat())

                target_pos = target_mat.ExtractTranslation()
                target_rot = target_mat.ExtractRotationQuat()

                pos_array = np.array([target_pos[0], target_pos[1], target_pos[2]])
                rot_array = np.array([target_rot.GetReal(), target_rot.GetImaginary()[0], target_rot.GetImaginary()[1], target_rot.GetImaginary()[2]])

                self.add_event_to_pool(step_type, duration, pos_array, rot_array)
            elif step_type in ["close", "open"]: 
                gripper_ratio = action_step['ratio']
                self.add_event_to_pool(step_type, duration, None, None, gripper_ratio)
            elif step_type == "slerp":
                slerp_action_sequence = generate_slerp_action_sequence(
                    action_step['position'], 
                    action_step['orientation'],
                    action_step['relative_rotation'], 
                    sub_steps=action_step['sub_steps'],
                    sub_duration=action_step['duration'] // action_step['sub_steps'],
                    slerp_last=action_step['slerp_last'],
                    slerp_offset=action_step['slerp_offset']
                    )

                print("action_sequence", slerp_action_sequence)
                for sub_action in slerp_action_sequence:
                    offset_mat = get_transform_mat_from_pos_rot(sub_action['position'], sub_action['orientation'])
                    target_mat = offset_mat * base_mat 
                    target_pos = target_mat.ExtractTranslation()
                    target_rot = target_mat.ExtractRotationQuat()

                    pos_array = np.array([target_pos[0], target_pos[1], target_pos[2]])
                    rot_array = np.array([target_rot.GetReal(), target_rot.GetImaginary()[0], target_rot.GetImaginary()[1], target_rot.GetImaginary()[2]])

                    self.add_event_to_pool(sub_action['action_type'], sub_action['duration'], pos_array, rot_array)


    def forward(self):
        """
        Main function to update the robot
        """
        # update event
        if len(self.event_pool) > 0:
            if self.event_elapsed <= 0:
                event, elapsed, ee_pos, ee_ori, gripper_ratio = self.event_pool.pop(0)
                print("event, elapsed, ee_pos, ee_ori ", event, elapsed, ee_pos, ee_ori, gripper_ratio)
                self.update_event(event)
                if self.event == "move":
                    self.update_ee_target(ee_pos, ee_ori)
                elif self.event == "close":
                    self.gripper.set_close_ratio(gripper_ratio)
                

                if self.connect_server:
                    self.synchronize_robot()

                self.event_elapsed = elapsed
        else:
            if self.connect_server:
                if self.total_event_count % (60 * 3) == 0:
                    self.synchronize_robot()

        # print("coffee control event", self.event, self.event_elapsed)
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
        # if self.connect_server:
        #     if self.total_event_count % 60 == 0:
        #         self.synchronize_robot()


        # return actions
    
    