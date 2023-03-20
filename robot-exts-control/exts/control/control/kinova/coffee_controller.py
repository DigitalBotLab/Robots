from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.prims import XFormPrim

from .kinova import Kinova
import numpy as np
from .numpy_utils import *

class CoffeeMakerController(BaseController):
    def __init__(self, name: str, kinova: Kinova) -> None: 
        BaseController.__init__(self, name=name)
        self._event = 0
        self.robot = kinova
        self._gripper = self.kinova.gripper
        
        # TODO：find height
        self._end_effector_initial_height = 0.4 / get_stage_units()
        self._pause = False

        self.xforms = XFormPrim("/World/kinova")
        
    def forward(self, target_end_effector_position, target_end_effector_orientation):
        actions = self._controller.forward(
                target_end_effector_position=target_end_effector_position,
                target_end_effector_orientation=target_end_effector_orientation)
        
        return actions
    
    def move_to_target(self, goal_pos, goal_rot):
        """
        Move hand to target points
        """
        # get end effector transforms
        hand_pos, hand_rot = self.xforms.get_world_poses()
        hand_rot = hand_rot[:,[1,2,3,0]] # WXYZ

        # get franka DOF states
        dof_pos = self.robots.get_joint_positions()

        # compute position and orientation error
        pos_err = goal_pos - hand_pos
        orn_err = orientation_error(goal_rot, hand_rot)
        dpose = np.concatenate([pos_err, orn_err], -1)[:, None].transpose(0, 2, 1)

        jacobians = self.robots._physics_view.get_jacobians()

        # jacobian entries corresponding to franka hand
        franka_hand_index = 8  # !!!
        j_eef = jacobians[:, franka_hand_index - 1, :]

        # solve damped least squares
        j_eef_T = np.transpose(j_eef, (0, 2, 1))
        d = 0.05  # damping term
        lmbda = np.eye(6) * (d ** 2)
        u = (j_eef_T @ np.linalg.inv(j_eef @ j_eef_T + lmbda) @ dpose).reshape(self.num_envs, 9)

        # update position targets
        pos_targets = dof_pos + u  # * 0.3

        return pos_targets