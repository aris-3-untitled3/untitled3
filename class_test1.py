import argparse
import csv
import os
import platform
import sys
from pathlib import Path

import torch
from ultralytics.utils.plotting import Annotator, colors, save_one_box
from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (
    LOGGER,
    Profile,
    check_file,
    check_img_size,
    check_imshow,
    check_requirements,
    colorstr,
    cv2,
    increment_path,
    non_max_suppression,
    print_args,
    scale_boxes,
    strip_optimizer,
    xyxy2xywh,
)
from utils.torch_utils import select_device, smart_inference_mode

class HandDetector:
    def __init__(self, weights=Path("yolov5s.pt"), source=Path("data/images"), data=Path("data/coco128.yaml"),
                 imgsz=(640, 640), conf_thres=0.25, iou_thres=0.45, max_det=1000, device="", view_img=False,
                 save_txt=False, save_csv=False, save_conf=False, save_crop=False, nosave=False, classes=None,
                 agnostic_nms=False, augment=False, visualize=False, update=False, project=Path("runs/detect"),
                 name="exp", exist_ok=False, line_thickness=3, hide_labels=False, hide_conf=False, half=False,
                 dnn=False, vid_stride=1):
        self.weights = weights
        self.source = source
        self.data = data
        self.imgsz = imgsz
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.device = device
        self.view_img = view_img
        self.save_txt = save_txt
        self.save_csv = save_csv
        self.save_conf = save_conf
        self.save_crop = save_crop
        self.nosave = nosave
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.augment = augment
        self.visualize = visualize
        self.update = update
        self.project = project
        self.name = name
        self.exist_ok = exist_ok
        self.line_thickness = line_thickness
        self.hide_labels = hide_labels
        self.hide_conf = hide_conf
        self.half = half
        self.dnn = dnn
        self.vid_stride = vid_stride

        self.model = None
        self.device = None
        self.save_dir = None

    @smart_inference_mode()
    def run(self):
        self.setup()
        self.load_model()
        self.process_images()

    def setup(self):
        self.save_dir = increment_path(Path(self.project) / self.name, exist_ok=self.exist_ok)
        (self.save_dir / "labels" if self.save_txt else self.save_dir).mkdir(parents=True, exist_ok=True)
        self.device = select_device(self.device)

    def load_model(self):
        self.model = DetectMultiBackend(self.weights, device=self.device, dnn=self.dnn, data=self.data, fp16=self.half)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(self.imgsz, s=self.stride)

    def process_images(self):
        is_file = Path(self.source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
        is_url = self.source.lower().startswith(("rtsp://", "rtmp://", "http://", "https://"))
        webcam = self.source.isnumeric() or self.source.endswith(".streams") or (is_url and not is_file)
        screenshot = self.source.lower().startswith("screen")

        bs = 1
        if webcam:
            self.view_img = check_imshow(warn=True)
            dataset = LoadStreams(self.source, img_size=self.imgsz, stride=self.stride, auto=self.pt, vid_stride=self.vid_stride)
            bs = len(dataset)
        elif screenshot:
            dataset = LoadScreenshots(self.source, img_size=self.imgsz, stride=self.stride, auto=self.pt)
        else:
            dataset = LoadImages(self.source, img_size=self.imgsz, stride=self.stride, auto=self.pt, vid_stride=self.vid_stride)

        vid_path, vid_writer = [None] * bs, [None] * bs
        self.model.warmup(imgsz=(1 if self.pt or self.model.triton else bs, 3, *self.imgsz))

        seen, windows, dt = 0, [], (Profile(device=self.device), Profile(device=self.device), Profile(device=self.device))
        for path, im, im0s, vid_cap, s in dataset:
            with dt[0]:
                im = torch.from_numpy(im).to(self.model.device)
                im = im.half() if self.model.fp16 else im.float()
                im /= 255
                if len(im.shape) == 3:
                    im = im[None]
                if self.model.xml and im.shape[0] > 1:
                    ims = torch.chunk(im, im.shape[0], 0)

            with dt[1]:
                visualize = increment_path(self.save_dir / Path(path).stem, mkdir=True) if self.visualize else False
                if self.model.xml and im.shape[0] > 1:
                    pred = None
                    for image in ims:
                        if pred is None:
                            pred = self.model(image, augment=self.augment, visualize=visualize).unsqueeze(0)
                        else:
                            pred = torch.cat((pred, self.model(image, augment=self.augment, visualize=visualize).unsqueeze(0)), dim=0)
                    pred = [pred, None]
                else:
                    pred = self.model(im, augment=self.augment, visualize=visualize)

            with dt[2]:
                pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

            csv_path = self.save_dir / "predictions.csv"

            def write_to_csv(image_name, prediction, confidence):
                data = {"Image Name": image_name, "Prediction": prediction, "Confidence": confidence}
                with open(csv_path, mode="a", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=data.keys())
                    if not csv_path.is_file():
                        writer.writeheader()
                    writer.writerow(data)

            for i, det in enumerate(pred):
                seen += 1
                if webcam:
                    p, im0, frame = path[i], im0s[i].copy(), dataset.count
                    s += f"{i}: "
                else:
                    p, im0, frame = path, im0s.copy(), getattr(dataset, "frame", 0)

                detected_hands = False
                p = Path(p)
                save_path = str(self.save_dir / p.name)
                txt_path = str(self.save_dir / "labels" / p.stem) + ("" if dataset.mode == "image" else f"_{frame}")
                s += "%gx%g " % im.shape[2:]
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
                imc = im0.copy() if self.save_crop else im0
                annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
                if len(det):
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                    for c in det[:, 5].unique():
                        n = (det[:, 5] == c).sum()
                        s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "

                    for *xyxy, conf, cls in reversed(det):
                        c = int(cls)
                        if c == 0:
                            detected_hands = True

                        label = self.names[c] if self.hide_conf else f"{self.names[c]}"
                        confidence = float(conf)
                        confidence_str = f"{confidence:.2f}"

                        if self.save_csv:
                            write_to_csv(p.name, label, confidence_str)

                        if self.save_txt:
                            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
                            line = (cls, *xywh, conf) if self.save_conf else (cls, *xywh)
                            with open(f"{txt_path}.txt", "a") as f:
                                f.write(("%g " * len(line)).rstrip() % line + "\n")

                        if self.save_img or self.save_crop or self.view_img:
                            c = int(cls)
                            label = None if self.hide_labels else (self.names[c] if self.hide_conf else f"{self.names[c]} {conf:.2f}")
                            annotator.box_label(xyxy, label, color=colors(c, True))
                        if self.save_crop:
                            save_one_box(xyxy, imc, file=self.save_dir / "crops" / self.names[c] / f"{p.stem}.jpg", BGR=True)

                    im0 = annotator.result()

                if self.save_img:
                    if dataset.mode == "image":
                        cv2.imwrite(save_path, im0)
                    else:
                        if vid_path[i] != save_path:
                            vid_path[i] = save_path
                            if isinstance(vid_writer[i], cv2.VideoWriter):
                                vid_writer[i].release()
                            vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*"mp4v"), 30, im0.shape[::-1])
                        vid_writer[i].write(im0)

                if self.view_img:
                    cv2.imshow(p.name, im0)
                    if cv2.waitKey(1) == ord("q"):
                        raise StopIteration

        print(f"\n{seen} images processed in {dt[0].elapsed() / 1.0:.3f}s, {dt[1].elapsed() / seen:.3f}s/image, "
              f"{dt[2].elapsed() / seen:.3f}s/inference, {dt[2].elapsed() / seen / dt[1].elapsed():.3f}x speedup")

        if self.update:
            strip_optimizer(self.weights)

if __name__ == "__main__":
    # Example arguments
    args = {
        'weights': 'yolov5s.pt',
        'source': 'data/images',
        'data': 'data/coco128.yaml',
        'imgsz': (640, 640),
        'conf_thres': 0.25,
        'iou_thres': 0.45,
        'max_det': 1000,
        'device': '',
        'view_img': False,
        'save_txt': False,
        'save_csv': True,
        'save_conf': False,
        'save_crop': False,
        'nosave': False,
        'classes': None,
        'agnostic_nms': False,
        'augment': False,
        'visualize': False,
        'update': False,
        'project': 'runs/detect',
        'name': 'exp',
        'exist_ok': False,
        'line_thickness': 3,
        'hide_labels': False,
        'hide_conf': False,
        'half': False,
        'dnn': False,
        'vid_stride': 1
    }

    detector = HandDetector(**args)
    detector.run()
