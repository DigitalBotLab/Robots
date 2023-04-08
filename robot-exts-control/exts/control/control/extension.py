import omni.ext
import omni.ui as ui
import omni.timeline
import omni.kit.app
import carb

from typing import Optional, List
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units

from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper

from .kinova.kinova import Kinova
from .kinova.coffee_controller import CoffeeMakerController
from .kinova.numpy_utils import euler_angles_to_quat, quat_to_euler_angles

# UI
from .ui.style import julia_modeler_style
from .ui.custom_multifield_widget import CustomMultifieldWidget
from .ui.custom_bool_widget import CustomBoolWidget

class ControlExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[control] control startup")

        # set up fps limit
        carb.settings.get_settings().set_float("/app/runLoops/main/rateLimitFrequency", 30) 
        carb.settings.get_settings().set_float("/app/runLoops/present/rateLimitFrequency", 30) 
        carb.settings.get_settings().set_bool("/rtx/ecoMode/enabled", True)
       

        # ui
        self._window = ui.Window("Robot control", width=300, height=300)
        self._window.frame.style = julia_modeler_style
        with self._window.frame:
            with ui.VStack():
                # ui.Button("Set Robot", height = 20, clicked_fn=self.set_robot)
                ui.Line(height = 2)
                ui.Button("Register Physics Event", height = 50, clicked_fn=self.register_physics_event)
                with ui.HStack(height = 20): 
                    ui.Label("Robot Prim Path:", width = 200)
                    self.robot_path_widget = ui.StringField(width = 300)
                    self.robot_path_widget.model.set_value("/World/kinova_gen3_7_hand/kinova")
                with ui.HStack(height = 20): 
                    self.server_widget = CustomBoolWidget(label="Connect to Server", default_value=False)
                ui.Spacer(height = 9)
                ui.Label("End Effector", height = 20)
                with ui.HStack(height = 20):
                    self.ee_pos_widget = CustomMultifieldWidget(
                        label="Transform",
                        default_vals=[0.45666, 0, 0.43371],
                        height = 20,
                    )
                ui.Spacer(height = 9)
                with ui.HStack(height = 20):
                    self.ee_ori_widget = CustomMultifieldWidget(
                        label="Orient (Euler)",
                        default_vals=[90, 0.0, 90],
                        height = 20,
                    )
                ui.Spacer(height = 9)
                ui.Button("Update EE Target", height = 20, clicked_fn=self.update_ee_target)
                ui.Button("Open/Close Gripper", height = 20, clicked_fn=self.toggle_gripper)

                ui.Spacer(height = 9)
                ui.Line(height = 2)
                with ui.HStack(height = 20):
                    self.joint_read_widget = CustomMultifieldWidget(
                        label="Joint Angle (read only):",
                        sublabels=["j1", "j2", "j3", "j4", "j5", "j6", "j7"],
                        default_vals=[0.0] * 7,
                        read_only= True
                    )
                    
                with ui.HStack(height = 20):
                    self.ee_pos_read_widget = CustomMultifieldWidget(
                        label="EE Position(read only):",
                        sublabels=["x", "y", "z"],
                        default_vals=[0, 0, 0],
                        read_only= True
                    )

                with ui.HStack(height = 20):
                    self.ee_ori_quat_read_widget = CustomMultifieldWidget(
                        label="EE Quaternion(read only):",
                        sublabels=[ "w", "x", "y", "z"],
                        default_vals=[1, 0, 0, 0],
                        read_only= True
                    )

                # with ui.HStack(height = 20):
                #     self.ee_ori_euler_read_widget = CustomMultifieldWidget(
                #         label="EE Euler Rot(read only):",
                #         sublabels=["x", "y", "z"],
                #         default_vals=[0, 0, 0],
                #         read_only= True
                #     )
                

                ui.Spacer(height = 9)
                ui.Line(height = 2)
                ui.Button("Debug", height = 20, clicked_fn = self.debug)
                ui.Button("yh Debug", height = 20, clicked_fn = self.yuanhong_debug)
                ui.Button("Debug2", height = 20, clicked_fn = self.debug2)
                


        # robot
        self.robot = None
        self.controller = None  
        self.event_t = 0.0

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
        self.event_t += dt # update time

        if not self._can_callback_physics_step():
            return

        if self.controller:
            # print("_on_physics_step")
            self.controller.forward()


            if self.event_t >= 1.0:
                # update joint info 
                self.update_robot_ui()
                self.event_t = 0.0

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
        prim_path = self.robot_path_widget.model.as_string
        self.robot = Kinova(prim_path = prim_path, name = "kinova_robot")
        self.robot.initialize()
        print("kinova_info", self.robot.num_dof)
        print("kinova_gripper", self.robot.gripper._gripper_joint_num)

        # set controller
        self.controller = CoffeeMakerController("task_controller", self.robot, connect_server=self.server_widget.value)

    def toggle_gripper(self):
        print("Toggle Gripper")
        if self.controller:
            event = "open" if self.controller.event == "close" else "close"
            self.controller.update_event(event)
        
    ######################### ui #############################################################
    def update_robot_ui(self):
        """
        read robot joint angles and update ui
        """
        assert self.robot, "robot is not initialized"
        joint_angles = self.robot.get_joint_positions()
        joint_angles = [np.rad2deg(joint_angles[i]) for i in range(7)]
        self.joint_read_widget.update(joint_angles)
        self.ee_pos_read_widget.update(self.robot.end_effector.get_world_pose()[0])
        rot_quat = self.robot.end_effector.get_world_pose()[1]
        self.ee_ori_quat_read_widget.update(rot_quat)
        # rot_euler = quat_to_euler_angles(rot_quat, degrees=True)
        # print("rot_euler:", rot_euler)
        # self.ee_ori_euler_read_widget.update(rot_euler[0])


    def debug(self):
        print("debug")
        if self.robot:
            self.controller.apply_high_level_action("close_coffee_machine_handle")
            # self.controller.apply_high_level_action("move_capsule_to_coffee_machine")
    
    def debug2(self):
        print("debug2")
        if self.robot:
            # self.controller.apply_high_level_action("pick_up_capsule")
            # self.controller.apply_high_level_action("move_capsule_to_coffee_machine")
            # self.controller.apply_high_level_action("pick_up_papercup")
            # self.controller.apply_high_level_action("open_coffee_machine_handle")
            self.controller.apply_high_level_action("press_coffee_machine_button")


        # from omni.isaac.core.prims import XFormPrim
        # from .kinova.utils import get_transform_mat_from_pos_rot

        # stage = omni.usd.get_context().get_stage()
        # base_prim = XFormPrim("/World/capsule")
        # base_world_pos, base_world_rot = base_prim.get_world_pose()
        # base_mat = get_transform_mat_from_pos_rot(base_world_pos, base_world_rot)
            
    def yuanhong_debug(self):
        # target_mat = get_transform_mat_from_pos_rot([-0.083, 0.43895, 0], [0.5] * 4)  
        
        # rel_mat = target_mat * base_mat.GetInverse()
        # print("base_mat:", base_mat)
        # print("target_mat:", target_mat)
        # print("rel_mat:", rel_mat.ExtractTranslation(), rel_mat.ExtractRotationQuat())

        pass

        
