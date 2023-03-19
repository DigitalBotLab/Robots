from typing import Optional, List
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
import carb
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper

class Kinova(Robot):
    def __init__(
        self,
        prim_path: str = "/World/kinova",
        name: str = "kinova_robot",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        end_effector_prim_name: Optional[str] = None,
        gripper_dof_names: Optional[List[str]] = None,
        gripper_open_position: Optional[np.ndarray] = None,
        gripper_closed_position: Optional[np.ndarray] = None,
        deltas: Optional[np.ndarray] = None,
    ) -> None:
        prim = get_prim_at_path(prim_path)
        assert prim.IsValid(), "Please load Kinova into the environment first"
        self._end_effector = None
        self._gripper = None
        self._end_effector_prim_name = end_effector_prim_name
        self._end_effector_prim_path = prim_path + "/panda_rightfinger"

        super().__init__(
            prim_path=prim_path, name=name, position=position, orientation=orientation, articulation_controller=None
        )

        self._end_effector_prim_path = prim_path + "/right_inner_finger"
        gripper_dof_names = [
                            "left_outer_knuckle_joint", "right_outer_knuckle_joint",
                            "left_inner_knuckle_joint", "right_inner_knuckle_joint",
                            "left_outer_finger_joint", "right_outer_finger_joint", 
                            "left_inner_finger_joint", "right_inner_finger_joint",
                            ]
        
        gripper_open_position = np.zeros(8)
        gripper_closed_position = np.ones(8)
        deltas = None

        self.gripper = ParallelGripper(
                end_effector_prim_path=self._end_effector_prim_path,
                joint_prim_names=gripper_dof_names,
                joint_opened_positions=gripper_open_position,
                joint_closed_positions=gripper_closed_position,
                action_deltas=deltas,
            )

        return