#!/usr/bin/env python3
"""Export a YOLO model to ONNX for bear_yolo_node (offline utility, not used at runtime).

Tested with ultralytics 8.4.183 + onnx 1.23 + onnxruntime 1.22 (C++).

  pip install "ultralytics>=8.4,<9" onnx onnxslim
  python3 export_yolo.py                                  # yolo26n.pt -> ../models/yolo26n.onnx
  python3 export_yolo.py --weights runs/detect/train/weights/best.pt --name bear_custom.onnx
  python3 export_yolo.py --nms-free                       # YOLO26 end2end [1,300,6] output

Default export is the raw [1, 4+nc, 8400] output (node applies NMS); both layouts are supported
and auto-detected by the node from the model metadata.
"""
import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
MODELS = HERE.parent / 'models'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default='yolo26n.pt',
                    help='Ultralytics weights; official names download automatically')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--opset', type=int, default=17)
    ap.add_argument('--nms-free', action='store_true',
                    help='export the YOLO26 end-to-end (NMS-free) head')
    ap.add_argument('--name', default=None, help='output file name in models/')
    a = ap.parse_args()

    model = YOLO(a.weights)
    out = Path(model.export(format='onnx', imgsz=a.imgsz, opset=a.opset, simplify=True,
                            dynamic=False, end2end=a.nms_free))

    MODELS.mkdir(exist_ok=True)
    dest = MODELS / (a.name or out.name)
    shutil.copy2(out, dest)

    import onnxruntime as ort  # noqa: E402  (only needed for the summary)
    sess = ort.InferenceSession(str(dest), providers=['CPUExecutionProvider'])
    meta = sess.get_modelmeta().custom_metadata_map
    print(f'\nWrote {dest}')
    print(f'  input  {sess.get_inputs()[0].shape}')
    print(f'  output {sess.get_outputs()[0].shape}  end2end={meta.get("end2end")}')
    names = meta.get('names', '')
    print(f'  "bear" in class names: {"bear" in names}')


if __name__ == '__main__':
    main()