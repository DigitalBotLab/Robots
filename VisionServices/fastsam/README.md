---
title: FastSAM
emoji: 🐠
colorFrom: pink
colorTo: indigo
sdk: gradio
sdk_version: 3.35.2
app_file: app_gradio.py
pinned: false
license: apache-2.0
---

# Fast Segment Anything

Official PyTorch Implementation of the <a href="https://github.com/CASIA-IVA-Lab/FastSAM">.

The **Fast Segment Anything Model(FastSAM)** is a CNN Segment Anything Model trained by only 2% of the SA-1B dataset published by SAM authors. The FastSAM achieve a comparable performance with
the SAM method at **50× higher run-time speed**.


## License

The model is licensed under the [Apache 2.0 license](LICENSE).


## Acknowledgement

- [Segment Anything](https://segment-anything.com/) provides the SA-1B dataset and the base codes.
- [YOLOv8](https://github.com/ultralytics/ultralytics) provides codes and pre-trained models.
- [YOLACT](https://arxiv.org/abs/2112.10003) provides powerful instance segmentation method.
- [Grounded-Segment-Anything](https://huggingface.co/spaces/yizhangliu/Grounded-Segment-Anything) provides a useful web demo template.

## Citing FastSAM

If you find this project useful for your research, please consider citing the following BibTeX entry.

```
@misc{zhao2023fast,
      title={Fast Segment Anything}, 
      author={Xu Zhao and Wenchao Ding and Yongqi An and Yinglong Du and Tao Yu and Min Li and Ming Tang and Jinqiao Wang},
      year={2023},
      eprint={2306.12156},
      archivePrefix={arXiv},
      primaryClass={cs.CV}
}
```