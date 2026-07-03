# False Positive Ball Autolabeling

This is a quick data-mining workflow for finding images where the detector reports a ball, but the image does not contain a real ball. The resulting reviewed images can be exported as negative ball examples in the TORSO-21 layout and as COCO annotations for CVAT cleanup.

The tool is intentionally split into stages so expensive work is not repeated:

- Extract camera frames from recorded rosbags into an intermediate image directory.
- Run a detector backend over those images and store ball candidates, confidences, and masks.
- Review candidates in a small browser UI and tag each image without deleting data.
- Export only the selected false positives into train/test datasets.

Long-running stages print progress while they run. Python stages use progress bars when `tqdm` is available and fall back to periodic text output otherwise. The YOEO batch executable prints its own image-level progress bar.

## Files

- `autolabel_false_balls.py` is the main command-line entry point.
- `rfdetr_autolabel.py` adapts a local RF-DETR checkout to the shared detection JSONL format.
- `bitbots_vision_yoeo_autolabel` is built from `src/bitbots_vision/src/yoeo_autolabel.cpp` and runs the existing YOEO ONNX pipeline in batch mode.
- `../config/autolabel_false_balls.yaml` contains the default paths, topics, sampling, detector, and export settings.

## Data Model

Intermediate data is stored below the configured output directory:

- `intermediate/images/` contains extracted frames.
- `intermediate/source_manifest.jsonl` maps each extracted image back to its source bag, topic, timestamp, and frame index.
- `intermediate/detections/detections.jsonl` contains detector output.
- `intermediate/detections/masks/` contains detector masks, when the backend provides them.
- `intermediate/index.json` joins source metadata and detections for review.
- `intermediate/review_tags.json` stores manual review tags.

Review tags are metadata only. The GUI never deletes images or detections.

The intended tags are:

- `false_positive`: no real ball is visible, keep this image for export.
- `real_ball`: a real ball is visible, exclude this image from false-positive export.
- `unsure`: exclude by default until reviewed again.
- `unreviewed`: exclude by default unless explicitly requested during export.

In TORSO-21 export, reviewed false positives are written with `ball` marked as `in_image: false`. The detector's mistaken candidate boxes and confidences are stored in image metadata.

## Build

Build the package before using the YOEO backend:

```bash
pixi run -e default build --packages-select bitbots_vision
```

Use the repository Pixi environment for all commands so ROS bag readers, message bindings, OpenCV, and the installed YOEO model are resolved consistently.

COCO export writes compressed run-length encoded masks through `pycocotools`. Make sure it is available in the environment before exporting non-empty COCO datasets.

## Flow

Set a shell variable to the config file used for the run:

```bash
CONFIG=src/bitbots_vision/config/autolabel_false_balls.yaml
```

Extract frames from all bags below the configured source path:

```bash
pixi run -e default python src/bitbots_vision/scripts/autolabel_false_balls.py \
  --config "$CONFIG" extract-images
```

Extraction shows one progress bar over bag files and one progress indicator for images extracted from the current bag.

For a quick first pass, use `--stride`, `--sample-rate`, or `--limit` to reduce the number of extracted frames:

```bash
pixi run -e default python src/bitbots_vision/scripts/autolabel_false_balls.py \
  --config "$CONFIG" extract-images --stride <N> --sample-rate <FRACTION>
```

Run YOEO over the extracted images:

```bash
pixi run -e default python src/bitbots_vision/scripts/autolabel_false_balls.py \
  --config "$CONFIG" detect --backend yoeo
```

YOEO shows model-loading logs followed by an image-level progress bar with the current number of selected candidate images.

Run RF-DETR instead:

```bash
pixi run -e default python src/bitbots_vision/scripts/autolabel_false_balls.py \
  --config "$CONFIG" detect --backend rfdetr
```

RF-DETR shows an image-level progress bar and the current selected-image count.

The RF-DETR backend uses the repo, checkpoint, model class, and target class from the config. If class names are not embedded in the checkpoint, pass a class id explicitly:

```bash
pixi run -e default python src/bitbots_vision/scripts/rfdetr_autolabel.py \
  --input-dir <IMAGES> --output-dir <DETECTIONS> --class-id <ID>
```

Open the review UI:

```bash
pixi run -e default python src/bitbots_vision/scripts/autolabel_false_balls.py \
  --config "$CONFIG" gui
```

The UI shows the image, candidate boxes, confidences, and available mask overlays. Use the threshold filter to narrow review to higher-confidence candidates first.

### Review UI

The review screen is for deciding whether the detector candidate is actually a real ball. Only images tagged as false positives are exported by default.

Buttons:

- `False positive, keep`: no real ball is visible. The image is kept for export as negative ball data.
- `Real ball, discard`: a real ball is visible. The image remains in the intermediate data, but is excluded from false-positive export.
- `Unsure, skip`: use this when the image is ambiguous. It is excluded from export by default.
- `Overlay`: toggles candidate boxes, confidence labels, and available mask overlays on the preview image.
- `Previous` / `Next`: navigate within the current confidence-filtered list.

Shortcuts:

- `F`: tag as false positive and move to the next image.
- `R`: tag as real ball and move to the next image.
- `U`: tag as unsure and move to the next image.
- `O`: toggle overlays.
- Left/right arrow: previous/next image.

The confidence input filters the review list by maximum detector confidence. The tag dropdown filters by current review tag, which is useful for verifying a first pass. For example, select `false positives` to re-check images marked for export, or `unsure` to resolve ambiguous cases. Changing either filter resets the review position to the beginning of the filtered list.

Export reviewed false positives:

```bash
pixi run -e default python src/bitbots_vision/scripts/autolabel_false_balls.py \
  --config "$CONFIG" export
```

Include images tagged as unsure as well:

```bash
pixi run -e default python src/bitbots_vision/scripts/autolabel_false_balls.py \
  --config "$CONFIG" export --tag false_positive --tag unsure
```

Export only one specific tag:

```bash
pixi run -e default python src/bitbots_vision/scripts/autolabel_false_balls.py \
  --config "$CONFIG" export --tag unsure
```

Export shows progress for the TORSO-21 train/test writers and the combined COCO writer.

The export writes both:

- `dataset/torso21/data/reality/{train,test}/`
- `dataset/coco/`

Use `--clean` to replace a previous export directory. Use `--tag` to choose which review tag to export; pass it multiple times to combine tags. Without `--tag`, export uses `false_positive`. Use `--include-unreviewed` only for quick experiments; it can add unverified images to the exported dataset.

CVAT requires the COCO category names to match labels registered in the task or project. The default COCO labels are configured for the TORSO CVAT project:

- field masks: `football field`
- line masks: `field lines`

If a CVAT task uses different label names, change `export.coco_labels` in the config or pass `--coco-field-label` and `--coco-lines-label` during export. Use `--coco-skip-masks` when the export should contain images only and no mask annotations.

False-positive ball candidates are not exported as `soccer ball` annotations or categories. They are detector mistakes, so creating a ball label for them would be wrong. Their boxes and confidences remain in the intermediate detector metadata and in the TORSO-21 image metadata for traceability.

If true ball or robot annotations are added to this exporter later, they should be exported as bounding boxes, not masks or polygons.

## Detector Backends

### YOEO

The YOEO backend runs the same `YoeoHandler` used by the ROS node, but without ROS replay. It reads extracted images, writes ball candidates above the configured threshold, and stores field and line masks if the model produces them.

### RF-DETR

The RF-DETR backend is a lightweight adapter around the local RF-DETR Python API. It emits the same `detections.jsonl` schema as YOEO so review and export do not change when comparing detectors.

RF-DETR segmentation masks are preserved as detector metadata when provided. Field and line masks are not produced by RF-DETR unless a future model/backend adds them.

### External JSONL

Use `--backend external-jsonl --external-jsonl <FILE>` to review and export detections produced by another script. The JSONL records must use the same schema as the YOEO/RF-DETR outputs.

## Output Semantics

The exported TORSO-21 data is negative ball data. A selected image means:

- YOEO or RF-DETR detected a candidate ball.
- A human reviewed the image and tagged it as a false positive.
- The exported annotation says no real ball is in the image.
- Detector boxes and confidences remain available as metadata.

The COCO export contains the selected images plus field and line masks when available. It does not label false-positive detector candidates as balls. These annotations are intended as CVAT preparation data, not as final cleaned labels.

## Notes

- Re-running detection replaces `intermediate/detections/detections.jsonl` and `intermediate/index.json`, but does not delete review tags.
- Re-extracting images can invalidate review tags if image names change.
- Keep the config file with the generated dataset so the sampling and detector settings remain reproducible.
