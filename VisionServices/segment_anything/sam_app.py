import numpy as np
import torch
import matplotlib.pyplot as plt
import cv2
import os
import gradio as gr
import json

from segment_anything import sam_model_registry, SamPredictor

SEGMENT_ANYTHING_FOLDER = "C:\\Users\\zhaoy\\Downloads"#"I:/Research/semgent-anything"
MODEL_TYPE = "vit_b" #"vit_b"
SAM_CHECKPOINT = os.path.join(SEGMENT_ANYTHING_FOLDER, "sam_vit_b_01ec64.pth") # sam_vit_h_4b8939 # sam_vit_b_01ec64
device = "cuda"

sam = sam_model_registry[MODEL_TYPE](checkpoint=SAM_CHECKPOINT)
sam.to(device=device)

predictor = SamPredictor(sam)

def segment_with_points(
    image,
    input_point_x,
    input_point_y,
    shape = "cuboid",
    input_label = np.array([1]),
    shape_contour_count = 6, 
    debug_plot = True,
):
    predictor.set_image(image)
    input_points = np.array([[input_point_x, input_point_y]])

    masks, scores, logits = predictor.predict(
        point_coords=input_points,
        point_labels=input_label,
        multimask_output=True,
    )

    print("mask", masks.shape, "scores", scores.shape, "logits", logits.shape)

    # only return the first mask
    target_mask = masks[0].astype(np.uint8)
    target_contours, _ = cv2.findContours(target_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    target_contour_count = len(target_contours)

    for mask in masks:
        # get contours
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0 and len(contours[0]) < target_contour_count:
            target_mask = mask
            target_contours = contours
            target_contour_count = len(contours)

    if debug_plot:
        cv2.drawContours(image, target_contours, -1, (255, 255, 255), 2)
        cv2.imshow('target_contours', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    mask = target_mask
    contour = max(target_contours, key=cv2.contourArea)
    arclen = cv2.arcLength(contour, True)
    
    if shape == "cuboid":
        for ratio in [0.01, 0.02, 0.005, 0.01, 0.02, 0.05, 0.1]:
            epsilon = ratio * arclen
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == shape_contour_count:
                
                break
    else: # bounding box
        x, y, w, h = cv2.boundingRect(contour)
        approx = np.array([[[x, y]], [[x+w, y]], [[x+w, y+h]], [[x, y+h]]])
    
    print("approx", approx, approx.shape)

    if debug_plot:
        temp = cv2.drawContours(image, [approx], -1, (255, 0, 0), 1)

        temp = cv2.resize(temp, (960, 540)) 
        cv2.imshow('Final Contours', temp)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return json.dumps(approx.tolist())

cond_img_e = gr.Image(label="Input image", type='numpy', image_mode = "RGB")
input_point_x = gr.Number(label="input h", value = 0)
input_point_y = gr.Number(label="input w", value = 0)


if __name__ == "__main__":
    demo = gr.Interface(
        segment_with_points, 
        inputs=[cond_img_e, 
                input_point_x, 
                input_point_y,
                "text"
        ],
        outputs="text",
        title="Segment Anything",
    )
    demo.launch(share = False)
