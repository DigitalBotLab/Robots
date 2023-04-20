# send message to Kinova Server to control the real robot
import cv2 
import requests
import base64
import json

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
    
    def get_image_from_webcam(self):
        """
        Get image from webcam
        """
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cv2.imwrite(self.vision_folder + "/0.jpg", frame)
        cap.release()
        