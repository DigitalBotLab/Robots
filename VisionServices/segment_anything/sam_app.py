import numpy as np
import torch
import matplotlib.pyplot as plt
import cv2
import os
import gradio as gr
import json

from segment_anything import sam_model_registry, SamPredictor

SEGMENT_ANYTHING_FOLDER = "I:/Research/semgent-anything"
MODEL_TYPE = "vit_b"
SAM_CHECKPOINT = os.path.join(SEGMENT_ANYTHING_FOLDER, "sam_vit_b_01ec64.pth")
device = "cuda"

sam = sam_model_registry[MODEL_TYPE](checkpoint=SAM_CHECKPOINT)
sam.to(device=device)

predictor = SamPredictor(sam)

def segment_with_points(
    image,
    input_point_x,
    input_point_y,
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

    # only return the first mask
    mask = masks[0].astype(np.uint8)

    # get contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # only return the first contour
    contour = contours[0] 
    arclen = cv2.arcLength(contour, True)

    for ratio in [0.005, 0.01, 0.02, 0.05, 0.1]:
        epsilon = ratio * arclen
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == shape_contour_count:
            break
    
    print("approx", approx, approx.shape)

    if debug_plot:
        temp = np.zeros((mask.shape[0], mask.shape[1], 1))
        temp = cv2.drawContours(temp, [approx], -1, (255, 255, 255), 1)
        color = np.array([0 / 255, 0 / 255, 255 / 255, 0.9])
        contour_mask = temp / 255 * color.reshape(1, 1, -1)

        temp = cv2.resize(temp, (960, 540)) 
        cv2.imshow('Blue Contours', temp)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return json.dumps(approx.tolist())

cond_img_e = gr.Image(label="Input image", type='numpy')
input_point_x = gr.Number(label="input h", value = 0)
input_point_y = gr.Number(label="input w", value = 0)


if __name__ == "__main__":
    demo = gr.Interface(
        segment_with_points, 
        inputs=[cond_img_e, 
                input_point_x, 
                input_point_y,
        ],
        outputs="text",
        title="Segment Anything",
    )
    demo.launch(share = False)
