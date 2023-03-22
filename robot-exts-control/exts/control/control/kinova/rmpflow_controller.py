# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import omni.isaac.motion_generation as mg
from omni.isaac.core.articulations import Articulation

from .utils import process_policy_config, EXTENSION_FOLDER_PATH


class RMPFlowController(mg.MotionPolicyController):
    """[summary]

        Args:
            name (str): [description]
            robot_articulation (Articulation): [description]
            physics_dt (float, optional): [description]. Defaults to 1.0/60.0.
        """

    def __init__(self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0) -> None:
        # print("EXTENSION_FOLDER_PATH: ", EXTENSION_FOLDER_PATH)
        self.rmp_flow_config = process_policy_config(EXTENSION_FOLDER_PATH + "/control/kinova/rmpflow/config7.json")
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
