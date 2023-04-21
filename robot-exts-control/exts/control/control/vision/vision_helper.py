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

import omni.usd
import carb 
from pxr import Gf, UsdGeom

from omni.physx import get_physx_scene_query_interface
from omni.debugdraw import get_debug_draw_interface

CX = 1280/2 # principal point x
CY = 720/2 # principal point y

class VisionHelper():
    def __init__(self, vision_url: str, vision_folder:str, vision_model = "owl_vit") -> None:
        self.vision_url = vision_url
        self.vision_folder = vision_folder
        self.vision_model = vision_model

    def get_bounding_box_data(self, image_file: str, object_name: str, threshold: float):
        """
        Get bounding box data from the Gradio server
        """

        # Set the request payload
        with open(image_file, "rb") as f:
            encoded_string = base64.b64encode(f.read())

        data_url = "data:image/png;base64," + encoded_string.decode("utf-8")
        payload = {
            "data": [
                data_url, object_name, threshold
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

        t = self.camera_mat.ExtractTranslation()
        # debug
        self._debugDraw = get_debug_draw_interface()
        COLOR_YELLOW = COLOR_YELLOW = 0xffffff00
        self._debugDraw.draw_line(t, COLOR_YELLOW, t + world_direction * 10, COLOR_YELLOW)

        return world_direction 
