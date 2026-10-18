#!/usr/bin/env python3
"""Prepare a balanced ball-detection subset using official TORSO annotations."""

import argparse
import hashlib
import io
import json
import math
import time
import zipfile
from collections import Counter
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

import cv2
import numpy as np
import yaml

BASE_URL = "https://data.bit-bots.de/TORSO-21/"
REPOSITORY = "https://github.com/bit-bots/TORSO_21_dataset"
RAW_REPOSITORY = "https://raw.githubusercontent.com/bit-bots/TORSO_21_dataset/master/"
SEED = "torso-balanced-v1"
SPLIT_COUNTS = {"train": 80, "test": 20}
MAX_RANGE_BYTES = 64 * 1024 * 1024


def fetch(url, headers=None):
    for attempt in range(3):
        try:
            with urlopen(Request(url, headers=headers or {}), timeout=90) as response:
                if headers and "Range" in headers and response.status != 206:
                    raise ValueError(f"Server ignored Range; refusing full archive download: {url}")
                limit = MAX_RANGE_BYTES if headers else 128 * 1024 * 1024
                if int(response.headers.get("Content-Length", 0)) > limit:
                    raise ValueError(f"Response exceeds safety limit: {url}")
                content = response.read(limit + 1)
                if len(content) > limit:
                    raise ValueError(f"Response exceeds safety limit: {url}")
                return content, dict(response.headers)
        except HTTPError as error:
            if error.code < 500 or attempt == 2:
                raise
            time.sleep(attempt + 1)
        except (URLError, TimeoutError):
            if attempt == 2:
                raise
            time.sleep(attempt + 1)
    raise RuntimeError("Download retries exhausted")


class RangeReader(io.RawIOBase):
    """Seek over an HTTP ZIP without ever requesting the complete archive."""

    def __init__(self, url):
        super().__init__()
        self.url = url
        _, headers = fetch(url, {"Range": "bytes=0-0"})
        self.size = int(headers["Content-Range"].rsplit("/", 1)[1])
        self.position = 0

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.position

    def seek(self, offset, whence=io.SEEK_SET):
        if whence == io.SEEK_SET:
            self.position = offset
        elif whence == io.SEEK_CUR:
            self.position += offset
        elif whence == io.SEEK_END:
            self.position = self.size + offset
        else:
            raise ValueError("Invalid seek mode")
        if self.position < 0:
            raise ValueError("Negative seek position")
        return self.position

    def read(self, size=-1):
        size = self.size - self.position if size < 0 else min(size, self.size - self.position)
        if size <= 0:
            return b""
        if size > MAX_RANGE_BYTES:
            raise ValueError("Refusing oversized archive range")
        end = self.position + size - 1
        content, headers = fetch(self.url, {"Range": f"bytes={self.position}-{end}"})
        expected = f"bytes {self.position}-{end}/{self.size}"
        if headers.get("Content-Range") != expected or len(content) != size:
            raise ValueError(f"Invalid HTTP range response: {self.url}")
        self.position += len(content)
        return content


def digest(content):
    return hashlib.sha256(content).hexdigest()


def ball_boxes(record):
    width, height = record["width"], record["height"]
    balls = [annotation for annotation in record["annotations"] if annotation["type"] == "ball"]
    if not balls:
        raise ValueError("Missing explicit ball annotation")
    boxes = []
    for ball in balls:
        if ball["in_image"] is False:
            continue
        if ball["in_image"] is not True:
            raise ValueError("Ambiguous ball visibility")
        vector = ball["vector"]
        if len(vector) != 2 or any(len(point) != 2 for point in vector):
            raise ValueError("Ball annotation is not a bounding box")
        xmin, ymin = vector[0]
        xmax, ymax = vector[1]
        if not all(math.isfinite(value) for value in (xmin, ymin, xmax, ymax)):
            raise ValueError("Non-finite ball annotation")
        if xmin >= xmax or ymin >= ymax:
            raise ValueError("Degenerate ball annotation")
        clipped = [max(0, xmin), max(0, ymin), min(width, xmax), min(height, ymax)]
        if clipped[0] >= clipped[2] or clipped[1] >= clipped[3]:
            raise ValueError("Visible ball lies outside image")
        boxes.append(clipped)
    return boxes


def archive_url(domain, split):
    if domain == "simulation" and split == "train":
        return BASE_URL + "simulation.zip"
    return BASE_URL + f"{domain}/{split}.zip"


def validate(output, manifest):
    counts = Counter()
    identifiers, images, hashes = set(), set(), set()
    for item in manifest:
        if item["id"] in identifiers or item["image"] in images:
            raise ValueError("Duplicate manifest entry")
        identifiers.add(item["id"])
        images.add(item["image"])
        content = (output / item["image"]).read_bytes()
        if digest(content) != item["provenance"]["image_sha256"]:
            raise ValueError(f"Image checksum mismatch: {item['image']}")
        if item["provenance"]["original_sha256"] in hashes:
            raise ValueError("Duplicate original image content")
        hashes.add(item["provenance"]["original_sha256"])
        image = cv2.imdecode(np.frombuffer(content, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Unreadable image: {item['image']}")
        height, width = image.shape[:2]
        if [width, height] != item["provenance"]["output_size"]:
            raise ValueError("Output dimensions mismatch")
        original_width, original_height = item["provenance"]["original_size"]
        expected = [
            [
                box[0] * width / original_width,
                box[1] * height / original_height,
                box[2] * width / original_width,
                box[3] * height / original_height,
            ]
            for box in ball_boxes(item["provenance"]["original_annotation"])
        ]
        if item["boxes"] != expected:
            raise ValueError("Boxes do not match scaled original annotations")
        for xmin, ymin, xmax, ymax in item["boxes"]:
            if not (0 <= xmin < xmax <= width and 0 <= ymin < ymax <= height):
                raise ValueError("Invalid output box")
        counts[(item["source"], bool(item["boxes"]), item["split"])] += 1
    expected_counts = Counter(
        {
            (source, positive, split): count
            for source in ("real", "simulation")
            for positive in (False, True)
            for split, count in SPLIT_COUNTS.items()
        }
    )
    if counts != expected_counts:
        raise ValueError(f"Unbalanced dataset: {counts}")
    print(
        json.dumps(
            {
                f"{source}/{'ball' if positive else 'no_ball'}/{split}": count
                for (source, positive, split), count in sorted(counts.items())
            },
            indent=2,
        )
    )


def prepare(output, max_side):
    provenance_dir = output / "provenance"
    provenance_dir.mkdir(parents=True, exist_ok=True)
    (output / "images").mkdir(exist_ok=True)
    for filename in ("README.md", "LICENSE", "CITATION.cff"):
        content, _ = fetch(RAW_REPOSITORY + filename)
        (provenance_dir / ("UPSTREAM_" + filename)).write_bytes(content)
    manifest = []
    selection_summary = {}
    for domain in ("reality", "simulation"):
        source = "real" if domain == "reality" else "simulation"
        for split, count in SPLIT_COUNTS.items():
            annotations_url = BASE_URL + f"{domain}/{split}/annotations.yaml"
            cache = provenance_dir / f"{domain}_{split}_annotations.yaml"
            if not cache.exists():
                cache.write_bytes(fetch(annotations_url)[0])
            raw_annotations = cache.read_bytes()
            annotation_hash = digest(raw_annotations)
            print(f"Reading {domain}/{split} annotations", flush=True)
            records = yaml.load(raw_annotations, Loader=yaml.CSafeLoader)["images"]
            del raw_annotations
            candidates = {False: [], True: []}
            rejected = 0
            for name, record in records.items():
                try:
                    boxes = ball_boxes(record)
                except (KeyError, TypeError, ValueError):
                    rejected += 1
                    continue
                candidates[bool(boxes)].append(name)
            selected = []
            for positive, names in candidates.items():
                names.sort(key=lambda name: digest(f"{SEED}/{domain}/{split}/{name}".encode()))
                if len(names) < count:
                    raise ValueError(f"Insufficient candidates: {domain}/{split}/{positive}")
                selected.extend(names[:count])
            selection_summary[f"{domain}/{split}"] = {
                "annotations_url": annotations_url,
                "annotations_sha256": annotation_hash,
                "eligible_ball": len(candidates[True]),
                "eligible_no_ball": len(candidates[False]),
                "rejected_invalid_annotations": rejected,
            }
            archive = None
            archive_members = None
            try:
                for name in selected:
                    original = records[name]
                    safe_name = Path(name).name.rsplit(".", 1)[0]
                    identifier = f"torso_{domain}_{split}_{safe_name}"
                    destination = output / "images" / f"{identifier}.jpg"
                    image_url = BASE_URL + f"{domain}/{split}/images/" + quote(name, safe="")
                    transport = {"method": "direct_http"}
                    try:
                        content, _ = fetch(image_url)
                    except HTTPError as error:
                        if error.code != 404:
                            raise
                        if archive is None:
                            url = archive_url(domain, split)
                            print(f"Using selective HTTP ranges: {url}", flush=True)
                            archive = zipfile.ZipFile(RangeReader(url))
                            archive_members = archive.namelist()
                        matches = [
                            member
                            for member in archive_members
                            if member.endswith(f"/{split}/images/{name}") or member == f"{split}/images/{name}"
                        ]
                        if len(matches) != 1:
                            raise ValueError(f"Cannot uniquely locate archive image {name}: {matches}") from error
                        member = matches[0]
                        info = archive.getinfo(member)
                        if info.file_size > MAX_RANGE_BYTES:
                            raise ValueError(f"Oversized image: {member}") from error
                        content = archive.read(member)
                        transport = {
                            "method": "zip_http_range",
                            "archive_url": archive_url(domain, split),
                            "archive_member": member,
                            "archive_crc32": f"{info.CRC:08x}",
                        }
                    image = cv2.imdecode(np.frombuffer(content, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        raise ValueError(f"Unreadable original: {name}")
                    height, width = image.shape[:2]
                    if (width, height) != (original["width"], original["height"]):
                        raise ValueError(f"Original annotation/image size mismatch: {name}")
                    factor = min(1.0, max_side / max(width, height))
                    output_width, output_height = round(width * factor), round(height * factor)
                    if (output_width, output_height) != (width, height):
                        image = cv2.resize(image, (output_width, output_height), interpolation=cv2.INTER_AREA)
                    boxes = [
                        [
                            box[0] * output_width / width,
                            box[1] * output_height / height,
                            box[2] * output_width / width,
                            box[3] * output_height / height,
                        ]
                        for box in ball_boxes(original)
                    ]
                    success, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    if not success:
                        raise ValueError(f"Cannot encode {name}")
                    destination.write_bytes(encoded.tobytes())
                    ball_annotations = [
                        {
                            key: annotation[key]
                            for key in ("type", "in_image", "vector", "blurred", "concealed")
                            if key in annotation
                        }
                        for annotation in original["annotations"]
                        if annotation["type"] == "ball"
                    ]
                    manifest.append(
                        {
                            "id": identifier,
                            "image": f"images/{destination.name}",
                            "boxes": boxes,
                            "source": source,
                            "split": split,
                            "provenance": {
                                "dataset": "TORSO-21",
                                "repository": REPOSITORY,
                                "original_filename": name,
                                "original_image_url": image_url,
                                "annotations_url": annotations_url,
                                "annotations_sha256": annotation_hash,
                                "original_sha256": digest(content),
                                "image_sha256": digest(encoded.tobytes()),
                                "original_size": [width, height],
                                "output_size": [output_width, output_height],
                                "original_annotation": {
                                    "width": width,
                                    "height": height,
                                    "annotations": ball_annotations,
                                },
                                "transport": transport,
                            },
                        }
                    )
                    if len(manifest) % 20 == 0:
                        print(f"Downloaded and labeled {len(manifest)} images", flush=True)
            finally:
                if archive is not None:
                    archive.close()
            del records
            cache.unlink()
    manifest.sort(key=lambda item: item["id"])
    validate(output, manifest)
    preparation = output.parent.parent / "preparation" / output.name
    preparation.mkdir(parents=True, exist_ok=True)
    (preparation / "full_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    simple = [{key: record[key] for key in ("image", "boxes", "source", "split")} for record in manifest]
    (output / "manifest.json").write_text(json.dumps(simple, indent=2) + "\n")
    (preparation / "selection.json").write_text(
        json.dumps(
            {
                "seed": SEED,
                "max_side": max_side,
                "split_counts_per_group": SPLIT_COUNTS,
                "selection": "SHA-256 rank of seed/domain/official_split/original_filename",
                "groups": selection_summary,
            },
            indent=2,
        )
        + "\n"
    )
    write_documentation(output)


def write_documentation(output):
    (output / "TORSO_DATASET.md").write_text(
        "# TORSO subset\n\n"
        "Balanced real/simulation and ball/no-ball subset using the official train/test partitions. "
        "The manifest is the authoritative file list; unrelated images or manifests are not part of it.\n\n"
        "Run `pixi run -e default python tutorials/schnupperstudium/prepare_torso_dataset.py` "
        "from the workspace root to reproduce, or add `--verify-only` to validate offline. "
        "Sampling uses a stable hash ranking, without detector-generated labels. "
        "Only explicit ball annotations qualify; invalid annotations are excluded before sampling. "
        "Visible original boxes are clipped to image bounds, then scaled independently on each axis "
        "to the rounded resized dimensions. Negative examples have explicit absent-ball labels.\n\n"
        "Source: " + BASE_URL + "\n\nRepository: " + REPOSITORY + "\n\n"
        "Individual image endpoints may be unavailable; in that case the downloader requests only "
        "ZIP directory metadata and selected image entries through HTTP byte ranges, never entire archives. "
        "Preparation records outside the student dataset retain original annotations and checksums. "
        "Private paths and unrelated upstream image metadata are not copied.\n\n"
        "## License and attribution\n\n"
        "The upstream dataset's CITATION.cff declares the MIT license. The corresponding license is "
        "copied verbatim in `provenance/UPSTREAM_LICENSE`. "
        "The upstream README and citation are included unchanged in `provenance/`. "
        "Attribute the dataset authors using `provenance/UPSTREAM_CITATION.cff`.\n"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "datasets" / "realistic")
    parser.add_argument("--max-side", type=int, default=640)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    if args.max_side <= 0:
        parser.error("--max-side must be positive")
    if args.verify_only:
        validate(
            args.output,
            json.loads(
                (args.output.parent.parent / "preparation" / args.output.name / "full_manifest.json").read_text()
            ),
        )
    else:
        prepare(args.output, args.max_side)


if __name__ == "__main__":
    main()
