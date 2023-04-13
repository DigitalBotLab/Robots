from typing import List, Callable
import numpy as np


from omni.isaac.manipulators.grippers.gripper import Gripper
from omni.isaac.core.utils.types import ArticulationAction
import omni.kit.app
 

class KinovaGripper(Gripper):
    def __init__(
        self,
        end_effector_prim_path: str,
        joint_prim_names: List[str],
        joint_opened_positions: np.ndarray,
        joint_closed_positions: np.ndarray,
        action_deltas: np.ndarray = None,
    ) -> None:
        Gripper.__init__(self, end_effector_prim_path=end_effector_prim_path)
        self._joint_prim_names = joint_prim_names
        self._gripper_joint_num = 6
        self._joint_dof_indicies = np.array([None] * self._gripper_joint_num)
        self._joint_opened_positions = joint_opened_positions
        self._joint_closed_positions = joint_closed_positions
        self._get_joint_positions_func = None
        self._set_joint_positions_func = None
        self._action_deltas = action_deltas
        self._articulation_num_dofs = None
        self._close_ratio = 1.0
        return
     
    @property
    def joint_opened_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively when opened.
        """
        return self._joint_opened_positions

    @property
    def joint_closed_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively when closed.
        """
        return self._joint_closed_positions

    @property
    def joint_dof_indicies(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint dof indices in the articulation of the left finger joint and the right finger joint respectively.
        """
        return self._joint_dof_indicies

    @property
    def joint_prim_names(self) -> List[str]:
        """
        Returns:
            List[str]: the left finger joint prim name and the right finger joint prim name respectively.
        """
        return self._joint_prim_names

    def initialize(
        self,
        articulation_apply_action_func: Callable,
        get_joint_positions_func: Callable,
        set_joint_positions_func: Callable,
        dof_names: List,
        physics_sim_view: omni.physics.tensors.SimulationView = None,
    ) -> None:
        """Create a physics simulation view if not passed and creates a rigid prim view using physX tensor api.
            This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
            of the functions of this class.

        Args:
            articulation_apply_action_func (Callable): apply_action function from the Articulation class.
            get_joint_positions_func (Callable): get_joint_positions function from the Articulation class.
            set_joint_positions_func (Callable): set_joint_positions function from the Articulation class.
            dof_names (List): dof names from the Articulation class.
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None

        Raises:
            Exception: _description_
        """
        Gripper.initialize(self, physics_sim_view=physics_sim_view)
        self._get_joint_positions_func = get_joint_positions_func
        self._articulation_num_dofs = len(dof_names)
        for index in range(len(dof_names)):
            if dof_names[index] in self._joint_prim_names:
                which_index = self._joint_prim_names.index(dof_names[index])
                self._joint_dof_indicies[which_index] = index

        # make sure that all gripper dof names were resolved
        if None in self._joint_dof_indicies:
            raise Exception("Not all gripper dof names were resolved to dof handles and dof indices.")
        self._articulation_apply_action_func = articulation_apply_action_func
        current_joint_positions = get_joint_positions_func()
        if self._default_state is None:
            self._default_state = np.array(
                [0.0] * self._gripper_joint_num
            )
        self._set_joint_positions_func = set_joint_positions_func
        return
    
    def open(self) -> None:
        """Applies actions to the articulation that opens the gripper (ex: to release an object held).
        """
        self._articulation_apply_action_func(self.forward(action="open"))
        return

    def close(self) -> None:
        """Applies actions to the articulation that closes the gripper (ex: to hold an object).
        """
        self._articulation_apply_action_func(self.forward(action="close"))
        return

    def set_action_deltas(self, value: np.ndarray) -> None:
        """
        Args:
            value (np.ndarray): deltas to apply for finger joint positions when openning or closing the gripper. 
                               [left, right]. Defaults to None.
        """
        self._action_deltas = value
        return
    
    def get_action_deltas(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: deltas that will be applied for finger joint positions when openning or closing the gripper. 
                        [left, right]. Defaults to None.
        """
        return self._action_deltas

    def set_default_state(self, joint_positions: np.ndarray) -> None:
        """Sets the default state of the gripper

        Args:
            joint_positions (np.ndarray): joint positions of the left finger joint and the right finger joint respectively.
        """
        self._default_state = joint_positions
        return
    
    def get_default_state(self) -> np.ndarray:
        """Gets the default state of the gripper

        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively.
        """
        return self._default_state
    
    def post_reset(self):
        Gripper.post_reset(self)
        self._set_joint_positions_func(
            positions=self._default_state, joint_indices=list(self._joint_dof_indicies)
        )
        return
    
    def set_joint_positions(self, positions: np.ndarray) -> None:
        """
        Args:
            positions (np.ndarray): joint positions of the left finger joint and the right finger joint respectively.
        """
        self._set_joint_positions_func(
            positions=positions, joint_indices=list(self._joint_dof_indicies)
        )
        return

    def get_joint_positions(self) -> np.ndarray:
        """
        Returns:
            np.ndarray: joint positions of the left finger joint and the right finger joint respectively.
        """
        return self._get_joint_positions_func(joint_indices=list(self._joint_dof_indicies))

    def forward(self, action: str) -> ArticulationAction:
        """calculates the ArticulationAction for all of the articulation joints that corresponds to "open"
           or "close" actions.

        Args:
            action (str): "open" or "close" as an abstract action.

        Raises:
            Exception: _description_

        Returns:
            ArticulationAction: articulation action to be passed to the articulation itself
                                (includes all joints of the articulation).
        """
        if action == "open":
            target_joint_positions = [None] * self._articulation_num_dofs
            if self._action_deltas is None:
                for i in range(self._gripper_joint_num):
                    target_joint_positions[self._joint_dof_indicies[i]] = self._joint_opened_positions[i]
            else:
                current_joint_positions = self._get_joint_positions_func()
                for i in range(self._gripper_joint_num):
                    current_finger_position = current_joint_positions[self._joint_dof_indicies[i]]
                    next_position = self.regulate_joint_position(
                        current_finger_position + self._action_deltas[i],
                        self._joint_opened_positions[i],
                        self._joint_closed_positions[i]
                    )
                    target_joint_positions[self._joint_dof_indicies[i]] = (
                        next_position
                    )

        elif action == "close":
            target_joint_positions = [None] * self._articulation_num_dofs
            if self._action_deltas is None:
                for i in range(self._gripper_joint_num):
                    target_joint_positions[self._joint_dof_indicies[i]] = self._joint_closed_positions[i] * self._close_ratio
            else:
                current_joint_positions = self._get_joint_positions_func()
                for i in range(self._gripper_joint_num):
                    current_finger_position = current_joint_positions[self._joint_dof_indicies[i]]
                    next_position = self.regulate_joint_position(
                        current_finger_position - self._action_deltas[i],
                        self._joint_opened_positions[i],
                        self._joint_closed_positions[i]
                    )
                    target_joint_positions[self._joint_dof_indicies[i]] = (
                        next_position
                    )
        else:
            raise Exception("action {} is not defined for ParallelGripper".format(action))
        
        # print("target_joint_positions", target_joint_positions)
        return ArticulationAction(joint_positions=target_joint_positions)
    
    def regulate_joint_position(self, joint_pos, open_pos, close_pos):
        """
        Regulates the joint position to be within the range of the open and close positions.
        """
        if open_pos > close_pos:
            open_pos, close_pos = close_pos, open_pos
        if joint_pos < open_pos:
            joint_pos = open_pos
        elif joint_pos > close_pos:
            joint_pos = close_pos
        return joint_pos
    
    def apply_action(self, control_actions: ArticulationAction) -> None:
        """Applies actions to all the joints of an articulation that corresponds to the ArticulationAction of the finger joints only.

        Args:
            control_actions (ArticulationAction): ArticulationAction for the left finger joint and the right finger joint respectively.
        """
        joint_actions = ArticulationAction()
        if control_actions.joint_positions is not None:
            joint_actions.joint_positions = [None] * self._articulation_num_dofs
            for i in range(self._gripper_joint_num):
                joint_actions.joint_positions[self._joint_dof_indicies[i]] = control_actions.joint_positions[i]

        # if control_actions.joint_velocities is not None:
        #     joint_actions.joint_velocities = [None] * self._articulation_num_dofs
        #     joint_actions.joint_velocities[self._joint_dof_indicies[0]] = control_actions.joint_velocities[0]
        #     joint_actions.joint_velocities[self._joint_dof_indicies[1]] = control_actions.joint_velocities[1]
        # if control_actions.joint_efforts is not None:
        #     joint_actions.joint_efforts = [None] * self._articulation_num_dofs
        #     joint_actions.joint_efforts[self._joint_dof_indicies[0]] = control_actions.joint_efforts[0]
        #     joint_actions.joint_efforts[self._joint_dof_indicies[1]] = control_actions.joint_efforts[1]

        self._articulation_apply_action_func(control_actions=joint_actions)
        return
    
    def set_close_ratio(self, ratio):
        """
        Sets the ratio of the closed position of the gripper.
        """
        self._close_ratio = ratio