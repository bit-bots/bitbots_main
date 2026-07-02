#!/usr/bin/env python3

import argparse
import json
import random
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import yaml

try:
    from tqdm import tqdm
except ImportError:
    tqdm = None

DEFAULT_CONFIG = {
    "paths": {
        "source": "/home/jan/vision/rosbags/RC2026",
        "out": "/home/jan/vision/vision_data",
        "torso21_reference": "/home/jan/git/bitbots/TORSO_21_dataset",
    },
    "topics": {
        "image": "/zed/zed_node/rgb/image_rect_color",
    },
    "sampling": {
        "stride": 1,
        "sample_rate": 1.0,
        "seed": 23,
        "limit": None,
    },
    "detection": {
        "backend": "yoeo",
        "confidence_threshold": 0.5,
        "nms_threshold": 0.4,
        "ball_threshold": 0.5,
        "max_balls": 1,
        "yoeo_command": "bitbots_vision_yoeo_autolabel",
        "yoeo_model_path": "2022_10_07_flo_torso21_yoeox",
        "rfdetr_repo": "/home/jan/git/bitbots/rf-detr",
        "rfdetr_checkpoint": "/home/jan/git/bitbots/rf-detr/output/checkpoint_best_regular.pth",
        "rfdetr_model_class": "RFDETRNano",
        "rfdetr_class_name": "ball",
    },
    "export": {
        "test_fraction": 0.3,
        "seed": 23,
        "include_unreviewed": False,
    },
}


def deep_update(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_update(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path):
    config = json.loads(json.dumps(DEFAULT_CONFIG))
    if path:
        with open(path) as f:
            loaded = yaml.safe_load(f) or {}
        deep_update(config, loaded)
    return config


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def sanitize_name(value):
    return "".join(c if c.isalnum() or c in "._-" else "_" for c in value)


def jsonl_read(path):
    if not path.exists():
        return []
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def jsonl_write(path, records):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        for record in records:
            f.write(json.dumps(record, sort_keys=True) + "\n")


def progress(iterable, **kwargs):
    if tqdm is None:
        return iterable
    return tqdm(iterable, **kwargs)


def progress_bar(**kwargs):
    if tqdm is None:
        return None
    return tqdm(**kwargs)


def progress_message(message):
    if tqdm is None:
        print(message)
    else:
        tqdm.write(message)


def output_paths(out):
    out = Path(out)
    return {
        "out": out,
        "intermediate": out / "intermediate",
        "images": out / "intermediate" / "images",
        "source_manifest": out / "intermediate" / "source_manifest.jsonl",
        "detection_dir": out / "intermediate" / "detections",
        "detections": out / "intermediate" / "detections" / "detections.jsonl",
        "index": out / "intermediate" / "index.json",
        "review_tags": out / "intermediate" / "review_tags.json",
    }


def normalize_topic(topic):
    return topic if topic.startswith("/") else "/" + topic


def cmd_extract_images(args):
    config = load_config(args.config)
    source = Path(args.source or config["paths"]["source"])
    paths = output_paths(args.out or config["paths"]["out"])
    image_topic = normalize_topic(args.image_topic or config["topics"]["image"])
    stride = args.stride if args.stride is not None else int(config["sampling"]["stride"])
    sample_rate = args.sample_rate if args.sample_rate is not None else float(config["sampling"]["sample_rate"])
    seed = args.seed if args.seed is not None else int(config["sampling"]["seed"])
    limit = args.limit if args.limit is not None else config["sampling"]["limit"]

    from cv_bridge import CvBridge
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    paths["images"].mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)
    bridge = CvBridge()
    manifest = []
    written = 0
    mcaps = sorted(source.rglob("*.mcap"))

    for mcap in progress(mcaps, desc="Bags", unit="bag"):
        reader = SequentialReader()
        reader.open(StorageOptions(uri=str(mcap), storage_id="mcap"), ConverterOptions("", ""))
        topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        if image_topic not in topic_types:
            progress_message(f"Skipping {mcap}: topic {image_topic} not present")
            continue
        image_type = get_message(topic_types[image_topic])
        seen_on_topic = 0
        bag_progress = progress_bar(desc=f"Extract {mcap.parent.name}", unit="img", leave=False)
        try:
            while reader.has_next():
                topic, data, timestamp = reader.read_next()
                if topic != image_topic:
                    continue
                seen_on_topic += 1
                if stride > 1 and (seen_on_topic - 1) % stride != 0:
                    continue
                if sample_rate < 1.0 and rng.random() > sample_rate:
                    continue
                msg = deserialize_message(data, image_type)
                image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                bag_id = sanitize_name(mcap.parent.name or mcap.stem)
                image_name = f"{bag_id}_{seen_on_topic:08d}.png"
                image_path = paths["images"] / image_name
                cv2.imwrite(str(image_path), image)
                manifest.append(
                    {
                        "image": image_name,
                        "source_bag": str(mcap),
                        "source_topic": image_topic,
                        "source_timestamp": int(timestamp),
                        "frame_index": seen_on_topic,
                        "width": int(image.shape[1]),
                        "height": int(image.shape[0]),
                    }
                )
                written += 1
                if bag_progress is None:
                    if written % 100 == 0:
                        print(f"Extracted {written} images so far")
                else:
                    bag_progress.update(1)
                    bag_progress.set_postfix(total=written, seen=seen_on_topic)
                if limit is not None and written >= int(limit):
                    jsonl_write(paths["source_manifest"], manifest)
                    print(f"Wrote {written} images to {paths['images']}")
                    return
        finally:
            if bag_progress is not None:
                bag_progress.close()

    jsonl_write(paths["source_manifest"], manifest)
    print(f"Wrote {written} images to {paths['images']}")


def find_yoeo_command(command):
    if "/" in command:
        return command
    resolved = shutil.which(command)
    if resolved:
        return resolved
    candidates = [
        Path("install/bitbots_vision/lib/bitbots_vision") / command,
        Path("build/bitbots_vision") / command,
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return command


def merge_detection_index(paths):
    source = {record["image"]: record for record in jsonl_read(paths["source_manifest"])}
    records = []
    detection_records = jsonl_read(paths["detections"])
    for record in progress(detection_records, desc="Index detections", unit="record"):
        image_rel = record["image"]
        detections = record.get("detections", [])
        max_confidence = max((det.get("confidence", 0.0) for det in detections), default=0.0)
        record["max_confidence"] = max_confidence
        if image_rel in source:
            record["source"] = source[image_rel]
        records.append(record)
    paths["index"].parent.mkdir(parents=True, exist_ok=True)
    with open(paths["index"], "w") as f:
        json.dump({"records": records, "updated_at": utc_now()}, f, indent=2, sort_keys=True)
    print(f"Indexed {len(records)} detected images at {paths['index']}")


def cmd_detect(args):
    config = load_config(args.config)
    paths = output_paths(args.out or config["paths"]["out"])
    backend = args.backend or config["detection"]["backend"]
    paths["detection_dir"].mkdir(parents=True, exist_ok=True)

    def arg_or_config(name):
        value = getattr(args, name)
        return value if value is not None else config["detection"][name]

    if backend == "yoeo":
        command = find_yoeo_command(args.yoeo_command or config["detection"]["yoeo_command"])
        run_args = [
            command,
            "--input-dir",
            str(paths["images"]),
            "--output-dir",
            str(paths["detection_dir"]),
            "--model-path",
            args.yoeo_model_path or config["detection"]["yoeo_model_path"],
            "--conf-threshold",
            str(arg_or_config("confidence_threshold")),
            "--nms-threshold",
            str(arg_or_config("nms_threshold")),
            "--ball-threshold",
            str(arg_or_config("ball_threshold")),
            "--max-balls",
            str(arg_or_config("max_balls")),
        ]
        subprocess.run(run_args, check=True)
    elif backend == "rfdetr":
        script_path = Path(__file__).with_name("rfdetr_autolabel.py")
        run_args = [
            sys.executable,
            str(script_path),
            "--input-dir",
            str(paths["images"]),
            "--output-dir",
            str(paths["detection_dir"]),
            "--repo",
            args.rfdetr_repo or config["detection"]["rfdetr_repo"],
            "--checkpoint",
            args.rfdetr_checkpoint or config["detection"]["rfdetr_checkpoint"],
            "--model-class",
            args.rfdetr_model_class or config["detection"]["rfdetr_model_class"],
            "--class-name",
            args.rfdetr_class_name or config["detection"]["rfdetr_class_name"],
            "--threshold",
            str(arg_or_config("ball_threshold")),
        ]
        subprocess.run(run_args, check=True)
    elif backend == "external-jsonl":
        if not args.external_jsonl:
            raise SystemExit("--external-jsonl is required for backend external-jsonl")
        shutil.copy2(args.external_jsonl, paths["detections"])
    else:
        raise SystemExit(f"Unknown backend: {backend}")

    merge_detection_index(paths)


def load_index(paths):
    with open(paths["index"]) as f:
        return json.load(f)["records"]


def load_tags(paths):
    if not paths["review_tags"].exists():
        return {}
    with open(paths["review_tags"]) as f:
        return json.load(f)


def save_tags(paths, tags):
    paths["review_tags"].parent.mkdir(parents=True, exist_ok=True)
    with open(paths["review_tags"], "w") as f:
        json.dump(tags, f, indent=2, sort_keys=True)


def cmd_gui(args):
    config = load_config(args.config)
    paths = output_paths(args.out or config["paths"]["out"])

    from flask import Flask, redirect, render_template_string, request, send_from_directory, url_for

    app = Flask(__name__)

    def filtered_records():
        threshold = float(request.args.get("threshold", args.threshold))
        records = [record for record in load_index(paths) if record.get("max_confidence", 0.0) >= threshold]
        records.sort(key=lambda item: item.get("max_confidence", 0.0), reverse=True)
        return records, threshold

    @app.route("/")
    def index():
        records, threshold = filtered_records()
        tags = load_tags(paths)
        idx = int(request.args.get("idx", 0))
        if records:
            idx = max(0, min(idx, len(records) - 1))
            record = records[idx]
            tag = tags.get(record["image"], {}).get("tag", "unreviewed")
        else:
            record = None
            tag = "unreviewed"
        return render_template_string(
            GUI_TEMPLATE,
            record=record,
            idx=idx,
            total=len(records),
            threshold=threshold,
            tag=tag,
            tags=tags,
        )

    @app.route("/tag", methods=["POST"])
    def tag():
        image = request.form["image"]
        selected_tag = request.form["tag"]
        idx = int(request.form["idx"])
        threshold = request.form["threshold"]
        tags = load_tags(paths)
        tags[image] = {"tag": selected_tag, "updated_at": utc_now()}
        save_tags(paths, tags)
        return redirect(url_for("index", idx=idx + 1, threshold=threshold))

    @app.route("/preview/<path:image>")
    def preview(image):
        records = {record["image"]: record for record in load_index(paths)}
        record = records[image]
        img = cv2.imread(str(paths["images"] / image), cv2.IMREAD_COLOR)
        overlay = img.copy()
        for mask_key, color in [("field", (0, 90, 0)), ("lines", (255, 255, 255))]:
            mask_rel = record.get("masks", {}).get(mask_key)
            if not mask_rel:
                continue
            mask = cv2.imread(str(paths["detection_dir"] / mask_rel), cv2.IMREAD_GRAYSCALE)
            if mask is None:
                continue
            overlay[mask > 0] = color
        img = cv2.addWeighted(overlay, 0.35, img, 0.65, 0.0)
        for det in record.get("detections", []):
            x, y, w, h = [int(v) for v in det["bbox"]]
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(
                img,
                f"{det.get('confidence', 0.0):.3f}",
                (x, max(20, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )
        ok, encoded = cv2.imencode(".jpg", img)
        if not ok:
            raise RuntimeError("Could not encode preview image")
        return app.response_class(encoded.tobytes(), mimetype="image/jpeg")

    @app.route("/image/<path:image>")
    def image(image):
        return send_from_directory(paths["images"], image)

    app.run(host=args.host, port=args.port, debug=False)


GUI_TEMPLATE = """
<!doctype html>
<html>
<head>
  <title>False ball review</title>
  <style>
    body { font-family: sans-serif; margin: 0; background: #202124; color: #f1f3f4; }
    header { display: flex; gap: 16px; align-items: center; padding: 10px 16px; background: #111; }
    main { display: grid; grid-template-columns: minmax(0, 1fr) 320px; min-height: calc(100vh - 50px); }
    .preview { display: flex; align-items: center; justify-content: center; overflow: hidden; }
    .preview img { max-width: 100%; max-height: calc(100vh - 58px); object-fit: contain; }
    aside { padding: 16px; background: #2b2c30; border-left: 1px solid #444; }
    button, input, a.nav { font-size: 16px; padding: 8px 10px; margin: 4px 0; }
    button { width: 100%; cursor: pointer; border: 0; color: #111; }
    .keep { background: #8fd694; }
    .discard { background: #ffb3a7; }
    .unsure { background: #ffe28a; }
    a { color: #8ab4f8; }
    code { word-break: break-all; }
    .meta { line-height: 1.5; }
  </style>
</head>
<body>
<header>
  <form method="get">
    Confidence threshold
    <input name="threshold" type="number" min="0" max="1" step="0.01" value="{{ threshold }}">
    <input name="idx" type="hidden" value="0">
    <button type="submit">Apply</button>
  </form>
  {% if record %}
    <a class="nav" href="/?idx={{ idx - 1 }}&threshold={{ threshold }}">Previous</a>
    <a class="nav" href="/?idx={{ idx + 1 }}&threshold={{ threshold }}">Next</a>
    <span>{{ idx + 1 }} / {{ total }}</span>
  {% else %}
    <span>No records match the filter.</span>
  {% endif %}
</header>
{% if record %}
<main>
  <section class="preview"><img src="/preview/{{ record.image }}"></section>
  <aside>
    <h2>{{ tag }}</h2>
    <form method="post" action="/tag">
      <input type="hidden" name="image" value="{{ record.image }}">
      <input type="hidden" name="idx" value="{{ idx }}">
      <input type="hidden" name="threshold" value="{{ threshold }}">
      <button class="keep" name="tag" value="false_positive">False positive, keep</button>
      <button class="discard" name="tag" value="real_ball">Real ball, discard</button>
      <button class="unsure" name="tag" value="unsure">Unsure</button>
    </form>
    <div class="meta">
      <p><strong>Image</strong><br><code>{{ record.image }}</code></p>
      <p><strong>Backend</strong> {{ record.backend }}</p>
      <p><strong>Max confidence</strong> {{ "%.3f"|format(record.max_confidence) }}</p>
      <p><strong>Detections</strong></p>
      <pre>{{ record.detections|tojson(indent=2) }}</pre>
      {% if record.source %}
      <p><strong>Source bag</strong><br><code>{{ record.source.source_bag }}</code></p>
      <p><strong>Frame</strong> {{ record.source.frame_index }}</p>
      {% endif %}
    </div>
  </aside>
</main>
{% endif %}
</body>
</html>
"""


def selected_records(paths, include_unreviewed):
    tags = load_tags(paths)
    records = []
    for record in load_index(paths):
        tag = tags.get(record["image"], {}).get("tag", "unreviewed")
        if tag == "false_positive" or (include_unreviewed and tag == "unreviewed"):
            record = dict(record)
            record["review_tag"] = tag
            records.append(record)
    return records


def deterministic_split(records, test_fraction, seed):
    rng = random.Random(seed)
    shuffled = list(records)
    rng.shuffle(shuffled)
    test_count = int(round(len(shuffled) * test_fraction))
    return shuffled[test_count:], shuffled[:test_count]


def copy_image(record, image_dir, target_dir):
    source = image_dir / record["image"]
    name = sanitize_name(record["image"].replace("/", "_"))
    target_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target_dir / name)
    return name


def combined_segmentation(record, detection_dir, shape):
    seg = np.zeros((shape[0], shape[1]), dtype=np.uint8)
    masks = record.get("masks", {})
    field = masks.get("field")
    lines = masks.get("lines")
    if field:
        mask = cv2.imread(str(detection_dir / field), cv2.IMREAD_GRAYSCALE)
        if mask is not None:
            seg[mask > 0] = 1
    if lines:
        mask = cv2.imread(str(detection_dir / lines), cv2.IMREAD_GRAYSCALE)
        if mask is not None:
            seg[mask > 0] = 2
    return seg


def write_torso_split(records, split_dir, image_dir, detection_dir, start_id, split_name):
    images_dir = split_dir / "images"
    seg_dir = split_dir / "segmentations"
    images_dir.mkdir(parents=True, exist_ok=True)
    seg_dir.mkdir(parents=True, exist_ok=True)
    annotations = {"images": {}}
    next_id = start_id
    for record in progress(records, desc=f"TORSO-21 {split_name}", unit="img"):
        image_name = copy_image(record, image_dir, images_dir)
        seg = combined_segmentation(record, detection_dir, (record["height"], record["width"]))
        cv2.imwrite(str(seg_dir / image_name), seg)
        annotations["images"][image_name] = {
            "width": int(record["width"]),
            "height": int(record["height"]),
            "id": next_id,
            "annotations": [{"type": "ball", "in_image": False}],
            "metadata": {
                "tags": ["autolabeled_false_positive_ball", record["review_tag"]],
                "detector_backend": record.get("backend"),
                "ball_candidates": record.get("detections", []),
                "source": record.get("source", {}),
            },
        }
        next_id += 1
    with open(split_dir / "annotations.yaml", "w") as f:
        yaml.safe_dump(annotations, f, sort_keys=True)
    return next_id


def mask_to_rle(mask):
    try:
        from pycocotools import mask as mask_utils
    except ImportError as exc:
        raise SystemExit("pycocotools is required for COCO RLE export") from exc
    binary = np.asfortranarray((mask > 0).astype("uint8"))
    rle = mask_utils.encode(binary)
    rle["counts"] = rle["counts"].decode("ascii")
    area = float(mask_utils.area(rle))
    return rle, area


def bbox_to_mask(bbox, height, width):
    x, y, w, h = [int(v) for v in bbox]
    mask = np.zeros((height, width), dtype="uint8")
    mask[max(0, y) : min(height, y + h), max(0, x) : min(width, x + w)] = 1
    return mask


def write_coco_split(records, split_dir, image_dir, detection_dir, split_name):
    images_dir = split_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    coco = {
        "images": [],
        "annotations": [],
        "categories": [
            {"id": 1, "name": "false_ball_candidate", "supercategory": "detector_metadata"},
            {"id": 2, "name": "field", "supercategory": "segmentation"},
            {"id": 3, "name": "lines", "supercategory": "segmentation"},
        ],
    }
    annotation_id = 1
    for image_id, record in enumerate(progress(records, desc=f"COCO {split_name}", unit="img"), start=1):
        image_name = copy_image(record, image_dir, images_dir)
        coco["images"].append(
            {
                "id": image_id,
                "file_name": image_name,
                "width": int(record["width"]),
                "height": int(record["height"]),
            }
        )
        for det in record.get("detections", []):
            mask = bbox_to_mask(det["bbox"], record["height"], record["width"])
            rle, area = mask_to_rle(mask)
            coco["annotations"].append(
                {
                    "id": annotation_id,
                    "image_id": image_id,
                    "category_id": 1,
                    "bbox": det["bbox"],
                    "segmentation": rle,
                    "area": area,
                    "iscrowd": 0,
                    "attributes": {"confidence": det.get("confidence"), "false_positive": True},
                }
            )
            annotation_id += 1
        for mask_name, category_id in [("field", 2), ("lines", 3)]:
            mask_rel = record.get("masks", {}).get(mask_name)
            if not mask_rel:
                continue
            mask = cv2.imread(str(detection_dir / mask_rel), cv2.IMREAD_GRAYSCALE)
            if mask is None:
                continue
            rle, area = mask_to_rle(mask)
            if area == 0:
                continue
            coco["annotations"].append(
                {
                    "id": annotation_id,
                    "image_id": image_id,
                    "category_id": category_id,
                    "bbox": [0, 0, int(record["width"]), int(record["height"])],
                    "segmentation": rle,
                    "area": area,
                    "iscrowd": 1,
                }
            )
            annotation_id += 1
    annotations_dir = split_dir / "annotations"
    annotations_dir.mkdir(parents=True, exist_ok=True)
    with open(annotations_dir / "instances_default.json", "w") as f:
        json.dump(coco, f)


def cmd_export(args):
    config = load_config(args.config)
    paths = output_paths(args.out or config["paths"]["out"])
    test_fraction = args.test_fraction if args.test_fraction is not None else float(config["export"]["test_fraction"])
    seed = args.seed if args.seed is not None else int(config["export"]["seed"])
    include_unreviewed = args.include_unreviewed or bool(config["export"]["include_unreviewed"])
    records = selected_records(paths, include_unreviewed)
    train, test = deterministic_split(records, test_fraction, seed)

    dataset_dir = paths["out"] / "dataset"
    torso_dir = dataset_dir / "torso21" / "data" / "reality"
    coco_dir = dataset_dir / "coco"
    if args.clean and dataset_dir.exists():
        shutil.rmtree(dataset_dir)

    next_id = write_torso_split(train, torso_dir / "train", paths["images"], paths["detection_dir"], 1, "train")
    write_torso_split(test, torso_dir / "test", paths["images"], paths["detection_dir"], next_id, "test")
    write_coco_split(train, coco_dir / "train", paths["images"], paths["detection_dir"], "train")
    write_coco_split(test, coco_dir / "test", paths["images"], paths["detection_dir"], "test")
    print(f"Exported {len(train)} train and {len(test)} test images to {dataset_dir}")


def build_parser():
    parser = argparse.ArgumentParser(description="Autolabel false positive ball detections from RC rosbags.")
    parser.add_argument("--config", help="YAML config file")
    subparsers = parser.add_subparsers(required=True)

    extract = subparsers.add_parser("extract-images")
    extract.add_argument("--source")
    extract.add_argument("--out")
    extract.add_argument("--image-topic")
    extract.add_argument("--stride", type=int)
    extract.add_argument("--sample-rate", type=float)
    extract.add_argument("--seed", type=int)
    extract.add_argument("--limit", type=int)
    extract.set_defaults(func=cmd_extract_images)

    detect = subparsers.add_parser("detect")
    detect.add_argument("--out")
    detect.add_argument("--backend", choices=["yoeo", "rfdetr", "external-jsonl"])
    detect.add_argument("--yoeo-command")
    detect.add_argument("--yoeo-model-path")
    detect.add_argument("--confidence-threshold", type=float)
    detect.add_argument("--nms-threshold", type=float)
    detect.add_argument("--ball-threshold", type=float)
    detect.add_argument("--max-balls", type=int)
    detect.add_argument("--external-jsonl")
    detect.add_argument("--rfdetr-repo")
    detect.add_argument("--rfdetr-checkpoint")
    detect.add_argument("--rfdetr-model-class")
    detect.add_argument("--rfdetr-class-name")
    detect.set_defaults(func=cmd_detect)

    gui = subparsers.add_parser("gui")
    gui.add_argument("--out")
    gui.add_argument("--threshold", type=float, default=0.0)
    gui.add_argument("--host", default="127.0.0.1")
    gui.add_argument("--port", type=int, default=5055)
    gui.set_defaults(func=cmd_gui)

    export = subparsers.add_parser("export")
    export.add_argument("--out")
    export.add_argument("--test-fraction", type=float)
    export.add_argument("--seed", type=int)
    export.add_argument("--include-unreviewed", action="store_true")
    export.add_argument("--clean", action="store_true")
    export.set_defaults(func=cmd_export)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
