import omni.ext
import omni.ui as ui
import omni.timeline
import omni.kit.app
import carb

from typing import Optional, List
import numpy as np
from pxr import Gf

from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units

from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper

from .kinova.kinova import Kinova
from .kinova.coffee_controller import CoffeeMakerController
from .kinova.numpy_utils import euler_angles_to_quat, quat_mul

# UI
from .ui.style import julia_modeler_style
from .ui.custom_multifield_widget import CustomMultifieldWidget
from .ui.custom_bool_widget import CustomBoolWidget

class ControlExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[control] control startup")
        self.ext_id = ext_id
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
                        default_vals=[0, 0, 0],
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
                
                # vision part

                ui.Spacer(height = 9)
                ui.Line(height = 2)
                ui.Button("Test vision", height = 20, clicked_fn = self.test_vision)
                ui.Button("Draw vision", height = 20, clicked_fn = self.draw_vision)
                ui.Button("Draw vision 2", height = 20, clicked_fn = self.draw_vision2)
                
                

                ui.Spacer(height = 9)
                ui.Line(height = 2)
                ui.Button("Debug", height = 20, clicked_fn = self.debug)
                ui.Button("Debug2", height = 20, clicked_fn = self.debug2)
                ui.Button("yh Debug", height = 20, clicked_fn = self.yuanhong_debug)
                ui.Spacer(height = 9)
                ui.Line(height = 2)


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
            current_pos, current_rot = self.robot.end_effector.get_world_pose()
            pos = [self.ee_pos_widget.multifields[i].model.as_float for i in range(3)]
            rot = [self.ee_ori_widget.multifields[i].model.as_float for i in range(3)]
            
            pos = np.array(current_pos) + np.array(pos)
            rot = euler_angles_to_quat(rot, degrees=True)
            # current_rot = np.array([current_rot[1], current_rot[2], current_rot[3], current_rot[0]])
            # rot = quat_mul(current_rot, rot)
            rot = np.array([rot[3], rot[0], rot[1], rot[2]])

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
        # if self.robot:
        #     self.controller.apply_high_level_action("pick_up_capsule")
        #     self.controller.apply_high_level_action("move_capsule_to_coffee_machine")
       
    
    def debug2(self):
        print("debug2")
        if self.robot:
            # self.controller.apply_high_level_action("pick_up_capsule")
            # self.controller.apply_high_level_action("move_capsule_to_coffee_machine")
            # self.controller.apply_high_level_action("pick_up_papercup")
            # self.controller.apply_high_level_action("open_coffee_machine_handle")
            self.controller.apply_high_level_action("close_coffee_machine_handle")
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
        print("yuanhong_debug")
        if self.robot:
            self.controller.apply_high_level_action("pick_up_papercup")
            self.controller.apply_high_level_action("move_papercup_to_coffee_machine")
            #obtain_robot_state = self.controller.obtain_robot_state()
            # print("obtain_robot_state:", obtain_robot_state)

        # from pxr import UsdGeom, Usd
        # stage = omni.usd.get_context().get_stage()
        # cup_prim = stage.GetPrimAtPath("/World/Simple_Paper_Cup")
        # xformable = UsdGeom.Xformable(cup_prim)
        # mat0 = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        # pos = mat0.ExtractTranslation()
        # print("cup pos:", pos)
        pass

    
    ########################## vision ########################################################
    def test_vision(self):
        print("test_vision")
        from .vision.vision_helper import VisionHelper
        self.vision_helper = VisionHelper(vision_url=None, vision_folder="I:\\Temp")
        # self.vision_helper.get_image_from_webcam()

        self.vision_helper.obtain_camera_transform(camara_path="/World/Camera")
        t = self.vision_helper.camera_mat.ExtractTranslation()
        print("camera offset", t)
        foc = 1000
        world_d = self.vision_helper.get_world_direction_from_camera_point(0, 0, foc, foc)
        world_d= world_d.GetNormalized()
        print("world_d:", world_d)

        self.vision_helper.draw_debug_line(t, world_d)
        self.vision_helper.get_hit_position(t, world_d, target_prim_path="/World/Desk")
    #     from omni.physx import get_physx_scene_query_interface
    #     t = carb.Float3(t[0], t[1], t[2])
    #     d = carb.Float3(world_d[0], world_d[1], world_d[2])
    #     get_physx_scene_query_interface().raycast_all(t, d, 100.0, self.report_all_hits)

    # def report_all_hits(self, hit):
    #     stage = omni.usd.get_context().get_stage()
    #     from pxr import UsdGeom
    #     usdGeom = UsdGeom.Mesh.Get(stage, hit.rigid_body)
    #     print("hit:", hit.rigid_body, usdGeom.GetPrim().GetPath(), hit.position, hit.normal, hit.distance, hit.face_index)
    #     return True


    def draw_vision(self):
        print("draw_vision")
        from omni.ui import scene as sc
        from omni.ui import color as cl

        from omni.kit.viewport.utility import get_active_viewport_window
        self._viewport_window = get_active_viewport_window()

        if hasattr(self, "scene_view"):
            self.scene_view.scene.clear()
            if self._viewport_window:
                self._viewport_window.viewport_api.remove_scene_view(self.scene_view)
            self.scene_view = None

        with self._viewport_window.get_frame(0):
            self.scene_view = sc.SceneView()
            self.scene_view.scene.clear()

            points_b = [[12500.0, 0, 0.0], [0.0, 0, 12500.0], [-12500.0, 0, 0.0], [-0.0, 0, -12500.0], [12500.0, 0, -0.0]]
            with self.scene_view.scene:
                transform = sc.Transform()
                # move_ges = MoveGesture(transform)
                with transform:
                    for pt in points_b:
                        sc.Curve([pt, [0, 0, 0]], thicknesses=[1.0], colors=[cl.green], curve_type=sc.Curve.CurveType.LINEAR)
      

            self._viewport_window.viewport_api.add_scene_view(self.scene_view)

    def draw_vision2(self):
        # print("draw_vision2")

        from .vision.vision_helper import VisionHelper
        self.vision_helper = VisionHelper(vision_url="http://127.0.0.1:7860/run/predict", 
                                          vision_folder="I:\\Temp",
                                          camera_prim_path="/World/Camera",
                                          vision_model="fastsam")

        # self.vision_helper.capture_image(folder_path="I:\\Temp\\VisionTest", image_name="test") 
    
        import cv2
        import os
        import numpy as np

        img_path = None
        print("os.listdir", os.listdir("I:\\Temp\\VisionTest"))
        for item in os.listdir("I:\\Temp\\VisionTest"):
            print("item:", item)
            if item.endswith(".png") and item.startswith("test"):
                img_path = os.path.join("I:\\Temp\\VisionTest", item)
                break
        
        assert img_path, "image not found"
        print("img_path:", img_path)
        image = cv2.imread(img_path)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([90, 50, 50])   
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contour = contours[0]
        arclen = cv2.arcLength(contour, True)
        # WARNING: 0.005 is a magic number
        contour = cv2.approxPolyDP(contour, arclen*0.005, True)
        cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)  # Green color, thickness 2

        print("contour:", contour, len(contour))
        

        # response_data = self.vision_helper.get_prediction_data("I:\\Temp\\0.jpg", "grey tea tower")
        # print(response_data)

        # response_data =  {'data': ['[[[[736, 113]], [[608, 133]], [[591, 151]], [[590, 373]], [[620, 419]], [[646, 419]], [[741, 392]], [[790, 162]]]]'], 'is_generating': False, 'duration': 11.769976139068604, 'average_duration': 11.769976139068604}
        # import json
        # import numpy as np
        # countour = json.loads(response_data["data"][0])
        print("countour", contour)
        points = np.array([p[0] for p in contour])
        print("p0", points)

        from .vision.utils import find_bottom_point, find_left_point
        bottom_point = find_bottom_point(points)
        left_point = find_left_point(points)
        print("bottom_point", bottom_point)

        image = cv2.circle(image, bottom_point, radius=10, color=(255, 0, 255), thickness=-1)
        image = cv2.circle(image, left_point, radius=10, color=(255, 255, 0), thickness=-1)
        
        cv2.imshow('Blue Contours', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        #REFERENCE: Camera Calibration and 3D Reconstruction from Single Images Using Parallelepipeds

        self.vision_helper.obtain_camera_transform(camara_path="/World/Camera")
        t = self.vision_helper.camera_mat.ExtractTranslation()
        print("camera offset", t)
        foc = 900
        world_d = self.vision_helper.get_world_direction_from_camera_point(left_point[0], 1080 - left_point[1], foc, foc)
        world_d= world_d.GetNormalized()
        print("world_d:", world_d)

        self.vision_helper.draw_debug_line(t, world_d, length=10)
        # self.vision_helper.get_hit_position(t, world_d, target_prim_path="/World/Desk")


             