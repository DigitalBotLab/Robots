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
from .kinova.numpy_utils import euler_angles_to_quat

# UI
from .ui.custom_multifield_widget import CustomMultifieldWidget 

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

                ui.Spacer(height = 9)
                ui.Label("End Effector", height = 20)
                with ui.HStack(height = 20):
                    self.ee_pos_widget = CustomMultifieldWidget(
                        label="Transform",
                        default_vals=[0.0, -0.02486, 1.18738],
                        height = 20,
                    )
                ui.Spacer(height = 9)
                with ui.HStack(height = 20):
                    self.ee_ori_widget = CustomMultifieldWidget(
                        label="Orient (Euler)",
                        default_vals=[0.0, 0.0, 0.0],
                        height = 20,
                    )
                ui.Button("Update EE Target", height = 20, clicked_fn=self.update_ee_target)
                ui.Button("Open/Close Gripper", height = 20, clicked_fn=self.toggle_gripper)
                ui.Button("Debug", height = 20, clicked_fn = self.debug)

        # robot
        self.robot = None
        self.controller = None  

        # stream
        self._is_stopped = True
        self._tensor_started = False
    
    def on_shutdown(self):
        print("[control] control shutdown")
    
    ########################## events #######################################################
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

            self.robot = None
            self.controller = None  
        

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

        if self.controller:
            # print("_on_physics_step")
            self.controller.forward()
        
        # if self.robot:
        #     actions = self.robot.gripper.forward(action="close")
        #     # print("close actions:", actions)
        #     self.robot.apply_action(actions)

    ############################################# Robot #######################################
    def update_ee_target(self):
        print("update_ee_target")
        if self.controller:
            self.controller.update_event("move")
            pos = [self.ee_pos_widget.multifields[i].model.as_float for i in range(3)]
            rot = [self.ee_ori_widget.multifields[i].model.as_float for i in range(3)]
            
            pos = np.array(pos)
            rot = euler_angles_to_quat(rot, degrees=True)

            print("updating controller ee target:", pos, rot)
            self.controller.update_ee_target(pos, rot)

    def set_robot(self):
        print("set_robot")

        # set robot
        prim_path = "/World/kinova"
        self.robot = Kinova(prim_path = prim_path, name = "kinova_robot")
        self.robot.initialize()
        print("kinova_info", self.robot.num_dof)
        print("kinova_gripper", self.robot.gripper._gripper_joint_num)

        # set controller
        self.controller = CoffeeMakerController("task_controller", self.robot)
            
    def toggle_gripper(self):
        print("Toggle Gripper")
        if self.controller:
            event = "open" if self.controller.event == "close" else "close"
            self.controller.update_event(event)

    def debug(self):
        print("debug")
        if self.robot:
            print("robot get pos: ", self.robot.get_joint_positions())
