"""Validate and combine independently annotated real and simulation samples."""

import argparse
import json
from collections import Counter
from pathlib import Path

import cv2

DIRECTORY = Path(__file__).resolve().parent / "datasets/orange"


def assemble(directory, allow_incomplete=False):
    preparation = directory.parent.parent / "preparation" / directory.name
    records = []
    for name in ("real_manifest.json", "sim_manifest.json"):
        records.extend(json.loads((preparation / name).read_text()))
    identifiers, images, counts = set(), set(), Counter()
    for record in records:
        if record["id"] in identifiers or record["image"] in images:
            raise ValueError(f"Doppelter Eintrag: {record['id']}")
        identifiers.add(record["id"])
        images.add(record["image"])
        image = cv2.imread(str(directory / record["image"]))
        if image is None:
            raise ValueError(f"Bild fehlt: {record['image']}")
        height, width = image.shape[:2]
        for box in record["boxes"]:
            if len(box) != 4 or not (0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height):
                raise ValueError(f"Ungültige Box: {record['id']} {box}")
        if record["split"] not in ("train", "test") or record["source"] not in ("real", "simulation"):
            raise ValueError(f"Ungültige Gruppe: {record['id']}")
        counts[(record["source"], "positive" if record["boxes"] else "negative")] += 1
    groups = [
        {"source": source, "class": category, "actual": counts[source, category], "expected": 100}
        for source in ("real", "simulation")
        for category in ("positive", "negative")
    ]
    complete = all(group["actual"] == group["expected"] for group in groups)
    status = {"complete": complete, "groups": groups}
    (preparation / "status.json").write_text(json.dumps(status, indent=2) + "\n")
    if not complete and not allow_incomplete:
        raise ValueError(
            "Datensatz unvollständig; siehe Vorbereitungsstatus. Explizite Freigabe mit --allow-incomplete."
        )
    (directory / "manifest.json").write_text(
        json.dumps(
            [{key: record[key] for key in ("image", "boxes", "source", "split")} for record in records], indent=2
        )
        + "\n"
    )
    print(json.dumps(status, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=DIRECTORY)
    parser.add_argument("--allow-incomplete", action="store_true")
    arguments = parser.parse_args()
    assemble(arguments.directory, arguments.allow_incomplete)
