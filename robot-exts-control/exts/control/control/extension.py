import omni.ext
import omni.ui as ui
import omni.timeline

from typing import Optional, List
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
import carb
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper

from .kinova.kinova import Kinova
from .kinova.coffee_controller import CoffeeMakerController

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.

class ControlExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[control] control startup")

        # ui
        self._window = ui.Window("Robot control", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                # ui.Button("Set Robot", height = 20, clicked_fn=self.set_robot)
                ui.Button("Register Physics Event", height = 20, clicked_fn=self.register_physics_event)

        # robot
        self.kinova = None
        self.controller = None  

        # stream
        self._is_stopped = True
        self._tensor_started = False
    
    def set_robot(self):
        print("set_robot")

        # set robot
        prim_path = "/World/kinova"
        self.kinova = Kinova(prim_path = prim_path, name = "kinova_robot")
        self.kinova.initialize()
        print("kinova_info", self.kinova.num_dof)
        print("kinova_gripper", self.kinova.gripper._gripper_joint_num)

        # set controller
        self.controller = CoffeeMakerController("task_controller", self.kinova)
    
    def register_physics_event(self):
        print("register_physics_event")
        
        # timeline
        stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(self._on_timeline_event)

    def _on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._physics_update_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_physics_step)
            self._is_stopped = False

        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physics_update_sub = None
            self._timeline_sub = None

            self._is_stopped = True
            self._tensor_started = False
        

    def _can_callback_physics_step(self) -> bool:
        if self._is_stopped:
            return False

        if self._tensor_started:
            return True

        self._tensor_started = True
        self.set_robot()
        return True
    
    def _on_physics_step(self, dt):
        if not self._can_callback_physics_step():
            return

        # pass
        position_target = np.zeros(3)
        end_effector_orientation = np.array([1, 0, 0, 0])
        
        if self.controller:
            print("_on_physics_step")
            actions = self.controller.forward(
                target_end_effector_position=position_target,
                target_end_effector_orientation=end_effector_orientation
            )

            self.kinova.apply(actions)

    def on_shutdown(self):
        print("[control] control shutdown")
