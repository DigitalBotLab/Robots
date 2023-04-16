import argparse
import numpy as np
import gradio as gr
import torch

# Grounding DINO
import GroundingDINO.groundingdino.datasets.transforms as T
from GroundingDINO.groundingdino.models import build_model
from GroundingDINO.groundingdino.util.slconfig import SLConfig
from GroundingDINO.groundingdino.util.utils import clean_state_dict
from GroundingDINO.groundingdino.util.inference import predict



# segment anything
# from segment_anything import build_sam, SamPredictor 
# import cv2
import numpy as np


from huggingface_hub import hf_hub_download


ckpt_repo_id = "ShilongLiu/GroundingDINO"
ckpt_filenmae = "groundingdino_swinb_cogcoor.pth"
ckpt_config_filename = "GroundingDINO_SwinB.cfg.py"


def load_model_hf(repo_id, filename, ckpt_config_filename, device='cpu'):
    cache_config_file = hf_hub_download(repo_id=repo_id, filename=ckpt_config_filename)

    args = SLConfig.fromfile(cache_config_file) 
    model = build_model(args)
    args.device = device
    try:
        checkpoint = torch.load(cache_file, map_location='cpu')
        log = model.load_state_dict(clean_state_dict(checkpoint['model']), strict=False)
    except:
        cache_file = hf_hub_download(repo_id=repo_id, filename=filename)
        checkpoint = torch.load(cache_file, map_location='cpu')
        log = model.load_state_dict(clean_state_dict(checkpoint['model']), strict=False)

    # cache_file = hf_hub_download(repo_id=repo_id, filename=filename)
    # checkpoint = torch.load(cache_file, map_location='cpu')
    # log = model.load_state_dict(clean_state_dict(checkpoint['model']), strict=False)
    print("Model loaded from {} \n => {}".format(cache_file, log))
    _ = model.eval()
    return model


def transform_image(image_pil):

    transform = T.Compose(
        [
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ]
    )
    image, _ = transform(image_pil, None)  # 3, h, w
    return image


def run_gdino(image, text_prompt, box_threshold, text_threshold):
    w, h = image.size
    print(image.size)
    image_pil = image.convert("RGB")
    image = transform_image(image_pil)


    groundingdino_model = load_model_hf(ckpt_repo_id, ckpt_filenmae, ckpt_config_filename)

    boxes, scores, labels = predict(
        model=groundingdino_model, 
        image=image, 
        caption=text_prompt, 
        box_threshold=box_threshold, 
        text_threshold=text_threshold
    )

    def to_center(x):
        x *= np.array([w, h, w, h])
        a = x[2] / 2
        b = x[3] / 2
        return np.array([x[0]-a, x[1]-b, x[2]+x[0], x[3]+x[1]])
    
    if boxes.shape[0] == 0:
        return ""

    boxes = boxes.cpu().detach().numpy()
    pixel_coord = np.apply_along_axis(to_center, 1, boxes)
    scores = scores.cpu().detach().numpy()


    print(list(pixel_coord), list(scores))

    record = []
    for box, score, label in zip(list(np.around(pixel_coord).astype("int")), list(scores), labels):
        # print("box", box)
        # print("score", score)
        record.append(str(list(box)) + "_" + "{:.3f}".format(score) + "_" + str(label))

    return str(record)


if __name__ == "__main__":

    demo = gr.Interface(
    run_gdino, 
    inputs=[gr.Image(source='upload', type="pil"), "text", gr.Slider(0, 1, value=0.3), gr.Slider(0, 1, value=0.25)], 
    outputs="text",
    title="Grounded Dino",
    examples=[

        ],
    )
    demo.launch(share = True)
