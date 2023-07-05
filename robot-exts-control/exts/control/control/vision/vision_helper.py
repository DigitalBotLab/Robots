# send message to Kinova Server to control the real robot
try:
    import cv2
except:
# omni.kit.pipapi extension is required
    import omni.kit.pipapi
    # It wraps `pip install` calls and reroutes package installation into user specified environment folder.
    # That folder is added to sys.path.
    # Note: This call is blocking and slow. It is meant to be used for debugging, development. For final product packages
    # should be installed at build-time and packaged inside extensions.
    omni.kit.pipapi.install(
        package="opencv-python",
    )
    import cv2
 
from PIL import Image
import requests
import base64
import os

import omni.usd
import carb 
from pxr import Gf, UsdGeom

import omni.timeline
import omni.graph.core as og
from omni.physx import get_physx_scene_query_interface
from omni.debugdraw import get_debug_draw_interface

CX = 1920/2 # principal point x
CY = 1080/2 # principal point y

class VisionHelper():
    def __init__(self, 
                 vision_url: str, 
                 vision_folder:str, 
                 camera_prim_path = "/OmniverseKit_Persp",
                 vision_model = "dino") -> None:
        # vision
        self.vision_url = vision_url
        self.vision_folder = vision_folder
        self.vision_model = vision_model
        self.camera_prim_path = camera_prim_path
        
        # stage
        self.stage = omni.usd.get_context().get_stage()

    def get_prediction_data(self, image_file: str, object_name: str):
        """
        Get bounding box data from the Gradio server
        """

        # Set the request payload
        with open(image_file, "rb") as f:
            encoded_string = base64.b64encode(f.read())

        data_url = "data:image/png;base64," + encoded_string.decode("utf-8")
        payload = {
            "data": [
                data_url, object_name
            ]
        }

        # Send the request to the Gradio server
        response = requests.post(self.vision_url, json=payload)

        # Get the response data as a Python object
        response_data = response.json()

        # Print the response data
        # print(response_data)
        return response_data
    
    def get_image_from_webcam(self, image_name = "0.jpg"):
        """
        Get image from webcam
        """
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame, 'RGB')
        image.save(self.vision_folder + f"/{image_name}")
        print("Image saved at path: " + self.vision_folder + f"/{image_name}")
        cap.release()
        
    def obtain_camera_transform(self, camara_path: str):
        """
        Obtain camera transform
        """
        camera_prim = omni.usd.get_context().get_stage().GetPrimAtPath(camara_path)
        xformable = UsdGeom.Xformable(camera_prim)
        self.camera_mat = xformable.ComputeLocalToWorldTransform(0)
        
    def get_world_direction_from_camera_point(self, x, y, fx, fy):
        """
        Get world direction from camera point
        """
        # camera_point = Gf.Vec3d(x, y, 1)
        # K = Gf.Matrix3d(fx, 0, 0, 0, fy, 0, CX, CY, 1)
        # K_inverse = K.GetInverse()
        Z = -1
        R = self.camera_mat.ExtractRotationMatrix()
        R_inverse = R.GetInverse()
        # world_point = (camera_point * K_inverse - t) * R_inverse
        D = Gf.Vec3d((CX - x) * Z / fx, (CY - y) * Z / fy, Z)
        world_direction = R_inverse * D 

        return world_direction 
    
    def draw_debug_line(self, origin, direction, length = 1, node_path = "/World/PushGraph/make_array"):
        """
        Draw debug line
        """
        make_array_node = og.Controller.node(node_path)
        if make_array_node.is_valid():
            # print("draw debug line")
            origin_attribute = make_array_node.get_attribute("inputs:input0")
            target_attribute = make_array_node.get_attribute("inputs:input1")
            size_attribute = make_array_node.get_attribute("inputs:arraySize")
            # attr_value = og.Controller.get(attribute)
            og.Controller.set(size_attribute, 2)
            og.Controller.set(origin_attribute, [origin[0], origin[1], origin[2]])
            og.Controller.set(target_attribute, [direction[0] * length + origin[0], direction[1] * length + origin[1], direction[2] * length + origin[2]])
            
            # print("attr:", attr_value)

    def get_hit_position(self, origin, direction, target_prim_path = "/World/Desk"):
        """
        Get hit position
        note: should be call while timeline is playing
        """
        timeline = omni.timeline.get_timeline_interface()
        assert timeline.is_playing(), "timeline is not playing"
        def report_all_hits(hit):
            usdGeom = UsdGeom.Mesh.Get(self.stage, hit.rigid_body)
            print("hit:", hit.rigid_body, usdGeom.GetPrim().GetPath(), hit.position, hit.normal, hit.distance, hit.face_index)
            if usdGeom.GetPrim().GetPath().pathString == target_prim_path:
                hit_position = hit.position

        hit_position = None
        t = carb.Float3(origin[0], origin[1], origin[2])
        d = carb.Float3(direction[0], direction[1], direction[2])
        # print("t:", t, "d:", d)
        get_physx_scene_query_interface().raycast_all(t, d, 100.0, report_all_hits)

        return hit_position
        
    ############################################# action #############################################
    def capture_image(self, folder_path = "I:\\Temp\\VisionTest", image_name = "test"):

        from omni.kit.capture.viewport import CaptureOptions, CaptureExtension

        options = CaptureOptions()
        options.file_name = image_name
        options.file_type = ".png"
        options.output_folder = str(folder_path)

        options.camera = self.camera_prim_path
        
        if not os.path.exists(options.output_folder):
            pass
        images = os.listdir(options.output_folder)
        for item in images:
            if item.endswith(options.file_type) and item.startswith(options.file_name):
                os.remove(os.path.join(options.output_folder, item))

        capture_instance = CaptureExtension().get_instance()
        capture_instance.options = options
        capture_instance.start() 