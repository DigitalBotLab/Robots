import torch
import cv2
import gradio as gr
import numpy as np
from transformers import OwlViTProcessor, OwlViTForObjectDetection


# Use GPU if available
if torch.cuda.is_available():
    device = torch.device("cuda")
else:
    device = torch.device("cpu")

model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32").to(device)
model.eval()
processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")


def query_image(img, text_queries, score_threshold):
    text_queries = text_queries
    text_queries = text_queries.split(",")

    target_sizes = torch.Tensor([img.shape[:2]])
    inputs = processor(text=text_queries, images=img, return_tensors="pt").to(device)

    with torch.no_grad():
        outputs = model(**inputs)
    
    outputs.logits = outputs.logits.cpu()
    outputs.pred_boxes = outputs.pred_boxes.cpu() 
    results = processor.post_process(outputs=outputs, target_sizes=target_sizes)
    boxes, scores, labels = results[0]["boxes"], results[0]["scores"], results[0]["labels"]

    # font = cv2.FONT_HERSHEY_SIMPLEX

    # for box, score, label in zip(boxes, scores, labels):
    #     box = [int(i) for i in box.tolist()]

    #     if score >= score_threshold:
    #         img = cv2.rectangle(img, pt1 = (box[0], box[1]), pt2 = (box[2], box[3]), color = (255,0,0), thickness = 5)
    #         if box[3] + 25 > 768:
    #             y = box[3] - 10
    #         else:
    #             y = box[3] + 25
                
    #         img = cv2.putText(
    #             img, text_queries[label], (box[0], y), font, 1, (255,0,0), 2, cv2.LINE_AA
    #         )

    records = []
    for box, score, label in zip(boxes, scores, labels):
        # print(box, score, label)
        if score >= score_threshold:
            records.append(str(box.long().tolist()) + "_" + "{:.3f}".format(score.item()) + "_" + str(label.item()))

    return str(records)


description = """
Gradio demo for <a href="https://huggingface.co/docs/transformers/main/en/model_doc/owlvit">OWL-ViT</a>, 
introduced in <a href="https://arxiv.org/abs/2205.06230">Simple Open-Vocabulary Object Detection
with Vision Transformers</a>. 
\n\nYou can use OWL-ViT to query images with text descriptions of any object. 
To use it, simply upload an image and enter comma separated text descriptions of objects you want to query the image for. You
can also use the score threshold slider to set a threshold to filter out low probability predictions. 

\n\nOWL-ViT is trained on text templates,
hence you can get better predictions by querying the image with text templates used in training the original model: *"photo of a star-spangled banner"*, 
*"image of a shoe"*. Refer to the <a href="https://arxiv.org/abs/2103.00020">CLIP</a> paper to see the full list of text templates used to augment the training data.
\n\n<a href="https://colab.research.google.com/github/huggingface/notebooks/blob/main/examples/zeroshot_object_detection_with_owlvit.ipynb">Colab demo</a>
"""
demo = gr.Interface(
    query_image, 
    inputs=[gr.Image(), "text", gr.Slider(0, 1, value=0.1)], 
    outputs="text",
    title="Zero-Shot Object Detection with OWL-ViT",
    description=description,
    examples=[

    ],
)
demo.launch(share = True)