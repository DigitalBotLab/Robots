from ultralytics import YOLO
import gradio as gr
import torch
from utils.my_tools import fast_process
from utils.tools import format_results, box_prompt, point_prompt, text_prompt
from PIL import ImageDraw
import numpy as np
import os
import json

FASTSAM_FOLDER  = "I:/Research/FastSAM/"

# Load the pre-trained model
model = YOLO(os.path.join(FASTSAM_FOLDER,'weights/FastSAM.pt'))

device = torch.device(
    "cuda"
    if torch.cuda.is_available()
    else "mps"
    if torch.backends.mps.is_available()
    else "cpu"
)


def segment_everything(
    input,
    text="",
    input_size=1024, 
    iou_threshold=0.7,
    conf_threshold=0.25,
    better_quality=False,
    withContours=True,
    use_retina=True,
    mask_random_color=True,
):
    input_size = int(input_size)  # 确保 imgsz 是整数
    # Thanks for the suggestion by hysts in HuggingFace.
    w, h = input.size
    scale = input_size / max(w, h)
    new_w = int(w * scale)
    new_h = int(h * scale)
    input = input.resize((new_w, new_h))

    results = model(input,
                    device=device,
                    retina_masks=True,
                    iou=iou_threshold,
                    conf=conf_threshold,
                    imgsz=input_size,)

    if len(text) > 0:
        results = format_results(results[0], 0)
        annotations, _ = text_prompt(results, text, input, device=device, 
                                    clip_model_path=os.path.join(FASTSAM_FOLDER,'weights/CLIP_ViT_B_32.pt')
                                    )
        
        annotations = np.array([annotations])
    else:
        annotations = results[0].masks.data
    
    contour_str = fast_process(annotations=annotations,
                       image=input,
                       device=device,
                       scale=(1024 // input_size),
                       better_quality=better_quality,
                       mask_random_color=mask_random_color,
                       bbox=None,
                       use_retina=use_retina,
                       )
    
    return json.dumps(contour_str.tolist())

cond_img_e = gr.Image(label="Input", type='pil')
cond_img_p = gr.Image(label="Input with points", type='pil')
cond_img_t = gr.Image(label="Input with text", type='pil', 
                      value = os.path.join(FASTSAM_FOLDER,"examples/0.jpg"))

segm_img_e = gr.Image(label="Segmented Image", interactive=False, type='pil')
segm_img_p = gr.Image(label="Segmented Image with points", interactive=False, type='pil')
segm_img_t = gr.Image(label="Segmented Image with text", interactive=False, type='pil')

global_points = []
global_point_label = []

if __name__ == "__main__":
    demo = gr.Interface(
        segment_everything, 
        inputs=[cond_img_t, 
                gr.Textbox(label="text prompt", value="grey tea tower"),
                # 1024, 
                # 0.7, 
                # 0.25,
                # False,
                # True,
                # True,
                ], 
        outputs="text",
        title="FastSAM",
    )
    demo.launch(share = False)


    
