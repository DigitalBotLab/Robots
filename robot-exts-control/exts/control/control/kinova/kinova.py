from typing import Optional, List
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units

import carb
from pxr import UsdPhysics
from .kinova_gripper import KinovaGripper

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
        # self.prim_path = prim_path
        prim = get_prim_at_path(prim_path)
        assert prim.IsValid(), "Please load Kinova into the environment first"
        self._end_effector = None
        self._gripper = None
        self._end_effector_prim_name = end_effector_prim_name

        super().__init__(
            prim_path=prim_path, name=name, position=position, orientation=orientation, articulation_controller=None
        )

        self._end_effector_prim_path = prim_path + "/robotiq_85_base_link"
        gripper_dof_names = [
                            "finger_joint", "right_outer_knuckle_joint",
                            "left_inner_knuckle_joint", "right_inner_knuckle_joint",
                            #"left_outer_finger_joint", "right_outer_finger_joint", 
                            "left_inner_finger_joint", "right_inner_finger_joint",
                            ]
        
        gripper_open_position = np.zeros(6)
        gripper_closed_position = np.array([0.8757, -0.8757, 0.8757, -0.8757, -0.8757, 0.8757])
        deltas = None # -gripper_closed_position / 5.0

        self._gripper = KinovaGripper(
                end_effector_prim_path=self._end_effector_prim_path,
                joint_prim_names=gripper_dof_names,
                joint_opened_positions=gripper_open_position,
                joint_closed_positions=gripper_closed_position,
                action_deltas=deltas,
            )

        return
    
    @property
    def end_effector(self) -> RigidPrim:
        """[summary]

        Returns:
            RigidPrim: [description]
        """
        return self._end_effector

    @property
    def gripper(self) -> KinovaGripper:
        """[summary]

        Returns:
            ParallelGripper: [description]
        """
        return self._gripper
    
    def initialize(self, physics_sim_view=None) -> None:
        """[summary]
        """
        super().initialize(physics_sim_view)
        self._end_effector = RigidPrim(prim_path=self._end_effector_prim_path, name=self.name + "_end_effector")
        self._end_effector.initialize(physics_sim_view)
        self._gripper.initialize(
            physics_sim_view=physics_sim_view,
            articulation_apply_action_func=self.apply_action,
            get_joint_positions_func=self.get_joint_positions,
            set_joint_positions_func=self.set_joint_positions,
            dof_names=self.dof_names,
        )
        return
    
    def post_reset(self) -> None:
        """[summary]
        """
        super().post_reset()

        self._gripper.post_reset()

        for i in range(self.gripper._gripper_joint_num):
            self._articulation_controller.switch_dof_control_mode(
                dof_index=self.gripper.joint_dof_indicies[i], mode="position"
            )

        return
    
    def fix_damping_and_stiffness(prim_path = "/World/kinova_gen3_7_hand/kinova", stiffness = 1e3, damping = 1e6):
            print("fixing damping and stiffness")
            # stiffness_name = "drive:angular:physics:stiffness"
            # damping_name = "drive:angular:physics:damping"

            joint_prim_paths = [
                "/base_link/Actuator1",
                "/shoulder_link/Actuator2",
                "/half_arm_1_link/Actuator3",
                "/half_arm_2_link/Actuator4",
                "/forearm_link/Actuator5",
                "/spherical_wrist_1_link/Actuator6",
                "/spherical_wrist_2_link/Actuator7",
            ]

            for joint_prim_path in joint_prim_paths:
                joint_prim = get_prim_at_path(prim_path + joint_prim_path)
                joint_driver = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
                joint_driver.GetStiffnessAttr().Set(stiffness)
                joint_driver.GetDampingAttr().Set(damping)

                