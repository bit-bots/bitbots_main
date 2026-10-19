"""Workshop-Datensätze offline wiederverwenden oder sicher mit wget laden."""

import json
import shutil
import subprocess
import tarfile
import tempfile
from pathlib import Path, PurePosixPath


class DatasetDownloadError(RuntimeError):
    """Verständlicher Fehler beim Bereitstellen eines Datensatzes."""


def dataset_root():
    """Nutze die Workshop-Suche, auch vor dem ersten Download."""
    from werkzeuge import find_dataset_root

    try:
        return find_dataset_root() / "datasets"
    except FileNotFoundError:
        return Path(__file__).resolve().parent / "datasets"


def _relative_path(value):
    if not isinstance(value, str) or not value or "\\" in value:
        raise DatasetDownloadError(f"Ungültiger relativer Dateipfad: {value!r}")
    relative = PurePosixPath(value)
    if relative.is_absolute() or ".." in relative.parts or ":" in value:
        raise DatasetDownloadError(f"Unsicherer Dateipfad: {value!r}")
    return relative


def _validate(directory):
    manifest = directory / "manifest.json"
    if directory.is_symlink() or manifest.is_symlink():
        raise DatasetDownloadError(f"Datensatz darf keine Verknüpfung sein: {directory}")
    try:
        records = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise DatasetDownloadError(f"Datensatz nicht lesbar: {manifest}: {error}") from error
    if not isinstance(records, list) or not records:
        raise DatasetDownloadError(f"{manifest}: eine nicht leere Bildliste wird erwartet.")
    for record in records:
        if not isinstance(record, dict) or "image" not in record:
            raise DatasetDownloadError(f"{manifest}: Eintrag ohne Bildpfad.")
        relative = _relative_path(record["image"])
        image = directory / relative
        current = directory
        for part in relative.parts:
            current = current / part
            if current.is_symlink():
                raise DatasetDownloadError(f"Bildpfad enthält eine Verknüpfung: {image}")
        if not image.is_file():
            raise DatasetDownloadError(f"Referenziertes Bild fehlt: {image}")


def _extract(archive_path, destination, name):
    with tarfile.open(archive_path, "r:gz") as archive:
        members = archive.getmembers()
        seen = set()
        for member in members:
            relative = _relative_path(member.name)
            if not member.isfile() and not member.isdir():
                raise DatasetDownloadError(f"Archiv enthält Verknüpfung oder Spezialdatei: {member.name}")
            if relative in seen:
                raise DatasetDownloadError(f"Archiv enthält einen doppelten Pfad: {member.name}")
            seen.add(relative)
        for member in members:
            target = destination / _relative_path(member.name)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as source, target.open("xb") as output:
                    shutil.copyfileobj(source, output)
    directory = destination if (destination / "manifest.json").is_file() else destination / name
    _validate(directory)
    return directory


def ensure_datasets(urls, root=None, wget="wget"):
    """Lade nur fehlende Datensätze; veröffentliche erst nach vollständiger Prüfung."""
    root = Path(root) if root is not None else dataset_root()
    results = {}
    for name in ("orange", "realistic"):
        destination = root / name
        if destination.exists() or destination.is_symlink():
            _validate(destination)
            print(f"{name}: lokale Daten geprüft, kein Download nötig.")
            results[name] = destination
            continue
        url = urls.get(name, "")
        if not isinstance(url, str) or not url.startswith(("https://", "http://")) or "REPLACE_ME" in url:
            raise DatasetDownloadError(
                f"{name}: Daten fehlen. Bitte zuerst die Download-URL in DATASET_URLS eintragen."
            )
        root.mkdir(parents=True, exist_ok=True)
        print(f"{name}: Download mit wget …")
        try:
            with tempfile.TemporaryDirectory(prefix=f".{name}-", dir=root) as temporary:
                staging = Path(temporary)
                archive_path = staging / "download.tar.gz"
                completed = subprocess.run(
                    [str(wget), "--no-verbose", "--output-document", str(archive_path), "--", url],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                if completed.returncode:
                    detail = completed.stderr.strip() or completed.stdout.strip()
                    raise DatasetDownloadError(f"{name}: wget fehlgeschlagen ({completed.returncode}): {detail}")
                extracted = staging / "extracted"
                extracted.mkdir()
                directory = _extract(archive_path, extracted, name)
                if destination.exists() or destination.is_symlink():
                    raise DatasetDownloadError(f"{name}: Ziel ist inzwischen vorhanden; nichts überschrieben.")
                directory.rename(destination)
        except (OSError, tarfile.TarError) as error:
            raise DatasetDownloadError(f"{name}: Download/Entpacken fehlgeschlagen: {error}") from error
        print(f"{name}: Daten vollständig geprüft und bereit.")
        results[name] = destination
    return results
