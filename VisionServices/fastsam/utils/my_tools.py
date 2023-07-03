import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import cv2
import torch


def fast_process(
    annotations,
    image,
    device,
    scale,
    better_quality=False,
    mask_random_color=True,
    bbox=None,
    use_retina=True,
    # withContours=True, # must true
):
    if isinstance(annotations[0], dict):
        annotations = [annotation['segmentation'] for annotation in annotations]

    original_h = image.height
    original_w = image.width
    if better_quality:
        if isinstance(annotations[0], torch.Tensor):
            annotations = np.array(annotations.cpu())
        for i, mask in enumerate(annotations):
            mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            annotations[i] = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((8, 8), np.uint8))
    if device == 'cpu':
        annotations = np.array(annotations)
    else:
        if isinstance(annotations[0], np.ndarray):
            annotations = torch.from_numpy(annotations)
    if isinstance(annotations, torch.Tensor):
        annotations = annotations.cpu().numpy()


    contour_all = []
    temp = np.zeros((original_h, original_w, 1))
    for i, mask in enumerate(annotations):
        if type(mask) == dict:
            mask = mask['segmentation']
        annotation = mask.astype(np.uint8)
        if use_retina == False:
            annotation = cv2.resize(
                annotation,
                (original_w, original_h),
                interpolation=cv2.INTER_NEAREST,
            )
        contours, _ = cv2.findContours(annotation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            arclen = cv2.arcLength(contour, True)
            # WARNING: 0.005 is a magic number
            approx = cv2.approxPolyDP(contour, arclen*0.005, True)
            print("approx!!", approx.shape)
            contour_all.append(approx)
            
        
        print("contour_all!!!", contour_all)


    return np.array(contour_all)

    cv2.drawContours(temp, contour_all, -1, (255, 255, 255), 2 // scale)
    color = np.array([0 / 255, 0 / 255, 255 / 255, 0.9])
    contour_mask = temp / 255 * color.reshape(1, 1, -1)

    image = image.convert('RGBA')
    overlay_contour = Image.fromarray((contour_mask * 255).astype(np.uint8), 'RGBA')
    image.paste(overlay_contour, (0, 0), overlay_contour)

    return image
