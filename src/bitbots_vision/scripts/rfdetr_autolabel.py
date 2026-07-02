#!/usr/bin/env python3

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

try:
    from tqdm import tqdm
except ImportError:
    tqdm = None


def has_image_extension(path):
    return path.suffix.lower() in {".png", ".jpg", ".jpeg"}


def jsonl_write(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        for record in records:
            f.write(json.dumps(record, sort_keys=True) + "\n")


def progress(iterable, **kwargs):
    if tqdm is None:
        return iterable
    return tqdm(iterable, **kwargs)


def load_model(repo, model_class, checkpoint):
    repo = Path(repo)
    sys.path.insert(0, str(repo / "src"))
    sys.path.insert(0, str(repo))
    import rfdetr

    cls = getattr(rfdetr, model_class)
    kwargs = {}
    if checkpoint:
        kwargs["pretrain_weights"] = checkpoint
    return cls(**kwargs)


def class_matches(model, class_id, class_name, selected_class_id):
    if selected_class_id is not None:
        return int(class_id) == selected_class_id
    names = getattr(model, "class_names", None)
    if names is None:
        return False
    try:
        return names[int(class_id)] == class_name
    except (IndexError, TypeError):
        return False


def main():
    parser = argparse.ArgumentParser(description="Run RF-DETR over an image directory and emit autolabel JSONL.")
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--repo", default="/home/jan/git/bitbots/rf-detr")
    parser.add_argument("--checkpoint", default="/home/jan/git/bitbots/rf-detr/output/checkpoint_best_regular.pth")
    parser.add_argument("--model-class", default="RFDETRNano")
    parser.add_argument("--class-name", default="ball")
    parser.add_argument("--class-id", type=int)
    parser.add_argument("--threshold", type=float, default=0.5)
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    masks_dir = output_dir / "masks"
    masks_dir.mkdir(parents=True, exist_ok=True)

    model = load_model(args.repo, args.model_class, args.checkpoint)
    records = []
    image_paths = sorted(path for path in input_dir.rglob("*") if path.is_file() and has_image_extension(path))
    image_progress = progress(image_paths, desc="RF-DETR", unit="img")
    for image_path in image_progress:
        image = Image.open(image_path).convert("RGB")
        detections = model.predict(image, threshold=args.threshold)
        xyxy = np.asarray(detections.xyxy)
        confidences = np.asarray(detections.confidence)
        class_ids = np.asarray(detections.class_id)
        masks = getattr(detections, "mask", None)
        kept = []
        mask_records = {}
        for i, class_id in enumerate(class_ids):
            if not class_matches(model, class_id, args.class_name, args.class_id):
                continue
            x1, y1, x2, y2 = xyxy[i].tolist()
            kept.append(
                {
                    "class": args.class_name,
                    "confidence": float(confidences[i]),
                    "bbox": [float(x1), float(y1), float(x2 - x1), float(y2 - y1)],
                }
            )
            if masks is not None and len(masks) > i and masks[i] is not None:
                mask_name = f"{image_path.stem}_rfdetr_{i}.png"
                cv2.imwrite(str(masks_dir / mask_name), (np.asarray(masks[i]) > 0).astype("uint8") * 255)
                mask_records[f"{args.class_name}_{i}"] = f"masks/{mask_name}"
        if not kept:
            continue
        records.append(
            {
                "image": image_path.relative_to(input_dir).as_posix(),
                "width": image.width,
                "height": image.height,
                "backend": "rfdetr",
                "detections": kept,
                "masks": mask_records,
            }
        )
        if tqdm is not None:
            image_progress.set_postfix(selected=len(records))

    jsonl_write(output_dir / "detections.jsonl", records)
    print(f"Selected {len(records)} images with RF-DETR detections")


if __name__ == "__main__":
    main()
