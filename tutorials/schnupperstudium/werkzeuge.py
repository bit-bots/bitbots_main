"""Laden und Anzeigen für die Ballsuche, nach der Vorlage aus ballsuche/werkzeuge.py."""

import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from download_datasets import ensure_datasets
from matplotlib.patches import Rectangle

DATASET_URLS = {
    "orange": "https://data.bit-bots.de/Schnupperstudium2026/orange.tar.gz",
    "realistic": "https://data.bit-bots.de/Schnupperstudium2026/realistic.tar.gz",
}


def bilder_laden(limit=12):
    """Lade eine gemischte Auswahl unserer Trainingsbilder in RGB-Reihenfolge."""
    ensure_datasets(DATASET_URLS)
    records = load_dataset("orange", split="train", limit=limit)
    return [cv2.cvtColor(record["image_bgr"], cv2.COLOR_BGR2RGB) for record in records]


def zu_hsv(bild: np.ndarray, *, bgr: bool = False) -> np.ndarray:
    """Wandle ein RGB- oder ausdrücklich als BGR markiertes Bild nach HSV um."""
    conversion = cv2.COLOR_BGR2HSV_FULL if bgr else cv2.COLOR_RGB2HSV_FULL
    return cv2.cvtColor(np.asarray(bild, dtype=np.uint8), conversion)


def bild_anzeigen(bild, titel="", x=None, y=None, box=None, groesse=(8, 4), achsen=True):
    """Zeige RGB-Bild, Kanal oder Maske; optional mit Kreuz und einer Bounding Box."""
    bild = np.asarray(bild)
    if bild.ndim not in (2, 3) or bild.size == 0:
        print("Noch kein darstellbares Bild. Prüfe die Aufgabe für diesen Ausschnitt oder Kanal.")
        return
    figure, axis = plt.subplots(figsize=groesse, layout="constrained")
    vmax = 1 if bild.dtype == bool else 255
    axis.imshow(bild, cmap="gray", vmin=0, vmax=vmax)
    if x is not None and y is not None and np.isfinite([x, y]).all():
        axis.plot(x, y, "w+", markersize=30, markeredgewidth=3)
    if box is not None:
        for left, top, right, bottom in box_als_array(box):
            axis.add_patch(Rectangle((left, top), right - left, bottom - top, fill=False, edgecolor="magenta"))
    axis.set(title=titel, xlabel="x → nach rechts", ylabel="y → nach unten")
    if not achsen:
        axis.axis("off")
    plt.show()
    plt.close(figure)


def histogramm_anzeigen(histogramm, titel=""):
    """Zeige eine bereits berechnete Zählliste."""
    figure, axis = plt.subplots(figsize=(8, 3), layout="constrained")
    axis.bar(range(len(histogramm)), histogramm, width=1)
    axis.set(title=titel, ylabel="Anzahl Pixel")
    plt.show()
    plt.close(figure)


def ball_verdoppeln(bild, maske, dx, dy):
    """Kopiere markierte Pixel verschoben ins Bild, ohne das Original zu verändern."""
    neues_bild = bild.copy()
    hoehe, breite = maske.shape
    zeilen, spalten = np.nonzero(maske)
    for y, x in zip(zeilen, spalten, strict=True):
        if 0 <= y + dy < hoehe and 0 <= x + dx < breite:
            neues_bild[y + dy, x + dx] = bild[y, x]
    return neues_bild


def detektor_anzeigen(bild, detector):
    """Passe das RGB-Übungsbild an den BGR-Vertrag des Roboterdetektors an."""
    box = detector(cv2.cvtColor(bild, cv2.COLOR_RGB2BGR))
    bild_anzeigen(bild, "Deine Bounding Box", box=box)
    return box


def box_als_array(box):
    """Passe genau ein Box-Tupel oder None an die interne Mehrbox-Auswertung an."""
    if box is None:
        return np.empty((0, 4), dtype=float)
    if not isinstance(box, tuple) or len(box) != 4:
        raise ValueError("Erwartet wird eine Bounding Box als Tupel (links, oben, rechts, unten) oder None.")
    return as_boxes([box])


def auswerten(detector, datensatz="orange", split="train"):
    """Prüfe unverändert auf dem gesamten Split, getrennt nach Bildquelle."""
    records = load_dataset(datensatz, split=split, limit=None)
    result = evaluate(records, lambda image: box_als_array(detector(image)), threshold=0.5)
    print_evaluation(result, f"{datensatz}/{split}")
    failures = [item for item in result["items"] if item["fp"] or item["fn"]]
    if failures:
        figure = plot_gallery(failures, limit=6)
        plt.show()
        plt.close(figure)
    else:
        print("In diesen Bildern sind keine Fehler aufgetreten.")
    return result


def find_dataset_root(start=None, name=None):
    """Suche vom Arbeitsverzeichnis und vom Helper aus nach den Datensätzen."""
    starts = [Path(start or Path.cwd()).resolve(), Path(__file__).resolve().parent]
    for origin in starts:
        for candidate in (origin, *origin.parents):
            names = (name,) if name else ("orange", "realistic")
            if any((candidate / "datasets" / dataset / "manifest.json").is_file() for dataset in names):
                return candidate
    raise FileNotFoundError(
        "Workshop-Daten fehlen: erwartet datasets/orange/manifest.json oder datasets/realistic/manifest.json. "
        "Bitte die Datensätze bereitstellen und das Notebook erneut ausführen."
    )


def dataset_summary(name="orange", root=None):
    """Zähle Bilder mit und ohne Ball nach Quelle."""
    root = Path(root) if root is not None else find_dataset_root(name=name)
    directory = root / "datasets" / name
    manifest = directory / "manifest.json"
    records = json.loads(manifest.read_text(encoding="utf-8"))
    counts = {
        source: {
            "positive": sum(record["source"] == source and bool(record["boxes"]) for record in records),
            "negative": sum(record["source"] == source and not record["boxes"] for record in records),
        }
        for source in ("real", "simulation")
    }
    warnings = []
    if any(count == 0 for group in counts.values() for count in group.values()):
        warnings.append("WARNUNG: Der Datensatz ist unvollständig. Die Kennzahlen gelten nur für vorhandene Bilder.")
    for source, group in counts.items():
        if group["positive"] == 0:
            warnings.append(
                f"{source}: Positive Bilder fehlen. Recall und Ball-Erkennung dieser Quelle sind nicht messbar; "
                "negative Bilder erlauben nur Aussagen über Fehlalarme."
            )
    return {
        "manifest": str(manifest),
        "complete": all(count > 0 for group in counts.values() for count in group.values()),
        "counts": counts,
        "warnings": warnings,
    }


def as_boxes(boxes):
    """Prüfe xyxy-Pixelkoordinaten; leere Listen bleiben echte leere Boxmengen."""
    array = np.asarray(boxes, dtype=float)
    if array.shape == (0,):
        array = array.reshape(0, 4)
    if array.ndim != 2 or array.shape[1] != 4:
        raise ValueError("Boxen müssen die Form (N, 4) im Format xmin, ymin, xmax, ymax haben.")
    if not np.isfinite(array).all() or np.any(array[:, 2:] <= array[:, :2]):
        raise ValueError("Boxen brauchen endliche Koordinaten und eine positive Breite und Höhe.")
    return array


def load_dataset(name="orange", split="train", limit=32, root=None):
    """Lade eine deterministische, nach Quelle und Ballpräsenz gemischte Teilmenge."""
    if name not in {"orange", "realistic"} or split not in {"train", "test"}:
        raise ValueError("Datensatz: orange/realistic; Split: train/test.")
    if limit is not None and (not isinstance(limit, int) or limit <= 0):
        raise ValueError("limit muss eine positive ganze Zahl oder None sein.")
    dataset_dir = (Path(root) if root is not None else find_dataset_root(name=name)) / "datasets" / name
    manifest = dataset_dir / "manifest.json"
    if not manifest.is_file():
        raise FileNotFoundError(f"Datensatz fehlt: {manifest}. Bitte Workshop-Daten bereitstellen.")
    try:
        records = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ValueError(f"Manifest kann nicht gelesen werden: {manifest}: {error}") from error
    if not isinstance(records, list):
        raise ValueError(f"{manifest}: erwartet eine JSON-Liste von Bild-Einträgen.")
    groups = {(source, positive): [] for source in ("real", "simulation") for positive in (True, False)}
    seen_ids = set()
    for record in records:
        if not isinstance(record, dict) or not {"image", "boxes", "source", "split"} <= record.keys():
            raise ValueError(f"{manifest}: Eintrag braucht image, boxes, source und split.")
        record = {"id": record.get("id", record["image"]), **record}
        if not isinstance(record["id"], (str, int)) or record["id"] in seen_ids:
            raise ValueError(f"{manifest}: Bild-IDs müssen eindeutig und Zeichenketten oder ganze Zahlen sein.")
        seen_ids.add(record["id"])
        if record["source"] not in {"real", "simulation"} or record["split"] not in {"train", "test"}:
            raise ValueError(f"{manifest}: unbekannte Quelle oder unbekannter Split.")
        boxes = as_boxes(record["boxes"])
        if not isinstance(record["image"], str):
            raise ValueError(f"{manifest}: image muss ein relativer Bildpfad sein.")
        relative_path = Path(record["image"])
        image_path = (dataset_dir / relative_path).resolve()
        if relative_path.is_absolute() or not image_path.is_relative_to(dataset_dir.resolve()):
            raise ValueError(f"{manifest}: Bildpfad muss innerhalb des Datensatzes liegen.")
        if record["split"] == split:
            groups[record["source"], bool(len(boxes))].append({**record, "boxes": boxes, "image_path": image_path})
    selected = []
    while any(groups.values()) and (limit is None or len(selected) < limit):
        for group in groups.values():
            if group and (limit is None or len(selected) < limit):
                selected.append(group.pop(0))
    if not selected:
        raise ValueError(f"{manifest}: keine Bilder im Split {split!r} gefunden.")
    for record in selected:
        image = cv2.imread(str(record["image_path"]))
        if image is None:
            raise FileNotFoundError(f"Bild fehlt oder ist nicht lesbar: {record['image_path']}")
        height, width = image.shape[:2]
        if np.any(record["boxes"] < 0) or np.any(record["boxes"][:, 2:] > [width, height]):
            raise ValueError(f"Box außerhalb des Bildes: {record['id']}")
        record["image_bgr"] = image
    return selected


def box_iou(first, second):
    """Paarweise Intersection over Union mit exklusiver rechter/unterer Grenze."""
    first, second = as_boxes(first), as_boxes(second)
    top_left = np.maximum(first[:, None, :2], second[None, :, :2])
    bottom_right = np.minimum(first[:, None, 2:], second[None, :, 2:])
    intersection = np.prod(np.maximum(bottom_right - top_left, 0), axis=2)
    first_area = np.prod(first[:, 2:] - first[:, :2], axis=1)
    second_area = np.prod(second[:, 2:] - second[:, :2], axis=1)
    union = first_area[:, None] + second_area[None, :] - intersection
    return np.divide(intersection, union, out=np.zeros_like(intersection), where=union > 0)


def match_boxes(predicted, ground_truth, threshold=0.5):
    """Maximiere die Trefferzahl durch eindeutiges Matching oberhalb der IoU-Schwelle."""
    if not np.isfinite(threshold) or not 0 < threshold <= 1:
        raise ValueError("Die IoU-Schwelle muss positiv und höchstens eins sein.")
    overlaps = box_iou(predicted, ground_truth)
    candidates = [
        [int(index) for index in np.argsort(-row, kind="stable") if row[index] >= threshold] for row in overlaps
    ]
    owners = {}

    def assign(prediction_index, visited):
        for truth_index in candidates[prediction_index]:
            if truth_index in visited:
                continue
            visited.add(truth_index)
            if truth_index not in owners or assign(owners[truth_index], visited):
                owners[truth_index] = prediction_index
                return True
        return False

    for prediction_index in range(len(overlaps)):
        assign(prediction_index, set())
    matches = sorted((prediction_index, truth_index) for truth_index, prediction_index in owners.items())
    return matches


def detection_metrics(true_positive, false_positive, false_negative):
    """Berechne Kennzahlen aus Zählern; ohne Nenner ist die Kennzahl null."""
    precision_denominator = true_positive + false_positive
    recall_denominator = true_positive + false_negative
    f1_denominator = 2 * true_positive + false_positive + false_negative
    return {
        "tp": true_positive,
        "fp": false_positive,
        "fn": false_negative,
        "precision": true_positive / precision_denominator if precision_denominator else 0.0,
        "recall": true_positive / recall_denominator if recall_denominator else 0.0,
        "f1": 2 * true_positive / f1_denominator if f1_denominator else 0.0,
    }


def evaluate(records, detector, threshold=0.5):
    """Zähle bildweise und aggregiere mikro-gemittelt, insgesamt und nach Quelle."""
    match_boxes([], [], threshold)
    counts = {source: np.zeros(3, dtype=int) for source in ("real", "simulation")}
    items = []
    for record in records:
        if record["source"] not in counts:
            raise ValueError(f"Unbekannte Quelle: {record['source']}")
        predicted = as_boxes(detector(record["image_bgr"].copy()))
        truth = as_boxes(record["boxes"])
        matches = match_boxes(predicted, truth, threshold)
        true_positive = len(matches)
        values = (true_positive, len(predicted) - true_positive, len(truth) - true_positive)
        counts[record["source"]] += values
        items.append({"record": record, "predicted": predicted, "matches": matches, **detection_metrics(*values)})
    total = np.sum(list(counts.values()), axis=0)
    support = {}
    for source in ("overall", "real", "simulation"):
        selected = [item for item in items if source == "overall" or item["record"]["source"] == source]
        ground_truth_count = sum(len(item["record"]["boxes"]) for item in selected)
        support[source] = {
            "images": len(selected),
            "positive_images": sum(bool(len(item["record"]["boxes"])) for item in selected),
            "ground_truth_boxes": ground_truth_count,
            "recall_defined": ground_truth_count > 0,
        }
    return {
        "overall": detection_metrics(*(int(value) for value in total)),
        "by_source": {
            source: detection_metrics(*(int(value) for value in values)) for source, values in counts.items()
        },
        "items": items,
        "support": support,
    }


def print_evaluation(result, label="Auswertung"):
    """Zeige Kennzahlen mit Datenbasis und expliziter Recall-Einschränkung."""
    for source, metrics in {"overall": result["overall"], **result["by_source"]}.items():
        support = result["support"][source]
        print(f"{label} · {source}: {metrics}")
        print(f"  Bilder: {support['images']}; positive Bilder: {support['positive_images']}")
        if not support["recall_defined"]:
            print("  Recall/F1 nicht als Erkennungsleistung interpretierbar: keine Groundtruth-Bälle vorhanden.")


def draw_overlay(record, predicted):
    """Zeichne Groundtruth und Vorhersage ohne Änderung des Originalbildes."""
    image = record["image_bgr"].copy()
    for label, boxes, color in (
        ("Groundtruth", record["boxes"], (0, 220, 0)),
        ("Vorhersage", predicted, (255, 0, 255)),
    ):
        for box in as_boxes(boxes):
            xmin, ymin, xmax, ymax = np.rint(box).astype(int)
            thickness = 4 if label == "Groundtruth" else 1
            label_y = max(ymin - 5, 12) if label == "Groundtruth" else min(ymax + 14, image.shape[0] - 3)
            cv2.rectangle(image, (xmin, ymin), (xmax - 1, ymax - 1), color, thickness)
            cv2.putText(image, label, (xmin, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    return image


def plot_gallery(items, limit=9):
    """Zeige eine Galerie vorhandener Datensatzbilder mit Groundtruth und Vorhersagen."""
    selected = items[:limit]
    if not selected:
        raise ValueError("Für die Galerie werden ausgewertete Datensatzbilder benötigt.")
    rows = (len(selected) + 2) // 3
    figure, axes = plt.subplots(rows, 3, figsize=(12, rows * 3), squeeze=False, layout="constrained")
    for axis in axes.flat:
        axis.axis("off")
    for axis, item in zip(axes.flat, selected, strict=False):
        record = item["record"]
        image = draw_overlay(record, item["predicted"])
        axis.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        presence = "mit Ball" if len(record["boxes"]) else "ohne Ball"
        axis.set_title(f"{record['source']} · {presence}\nTP {item['tp']} / FP {item['fp']} / FN {item['fn']}")
    figure.suptitle("Datensatzgalerie: Groundtruth grün / Vorhersage magenta")
    return figure


def plot_overview(records):
    """Zeige die Zusammensetzung der tatsächlich geladenen Teilmenge."""
    labels = [f"{source}\n{presence}" for source in ("real", "simulation") for presence in ("mit Ball", "ohne Ball")]
    counts = [
        sum(record["source"] == source and bool(len(record["boxes"])) == positive for record in records)
        for source in ("real", "simulation")
        for positive in (True, False)
    ]
    figure, axis = plt.subplots(figsize=(8, 3), layout="constrained")
    bars = axis.bar(labels, counts)
    axis.bar_label(bars)
    axis.set_ylabel("Bilder")
    axis.set_title("Geladene Auswahl – nicht der gesamte Datensatz")
    return figure


def plot_metrics(result):
    """Zeige Kennzahlen und Fehlerzahlen ohne negative Bilder als Treffer umzudeuten."""
    groups = {"gesamt": result["overall"], **result["by_source"]}
    figure, axes = plt.subplots(1, 2, figsize=(11, 4), layout="constrained")
    positions = np.arange(len(groups))
    for offset, metric in enumerate(("precision", "recall", "f1")):
        values = [
            np.nan
            if metric in {"recall", "f1"}
            and not result["support"]["overall" if name == "gesamt" else name]["recall_defined"]
            else group[metric]
            for name, group in groups.items()
        ]
        axes[0].bar(positions + (offset - 1) * 0.25, values, 0.25, label=metric)
    for position, name in enumerate(groups):
        if not result["support"]["overall" if name == "gesamt" else name]["recall_defined"]:
            axes[0].text(position, 0.04, "Recall/F1\nnicht messbar", ha="center", fontsize="small")
    axes[0].set_ylim(0, 1.05)
    axes[0].set_title("Qualität der erkannten Boxen")
    for offset, metric in enumerate(("tp", "fp", "fn")):
        axes[1].bar(
            positions + (offset - 1) * 0.25, [group[metric] for group in groups.values()], 0.25, label=metric.upper()
        )
    axes[1].set_title("Treffer und Fehler")
    for axis in axes:
        axis.set_xticks(positions, groups)
        axis.legend()
    return figure
