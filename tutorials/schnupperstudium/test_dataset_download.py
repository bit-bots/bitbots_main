"""Offline-Tests für die sichere Bereitstellung der Workshop-Datensätze."""

import io
import json
import sys
import tarfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import download_datasets as downloads


def local_dataset(root, name):
    directory = root / name
    (directory / "images").mkdir(parents=True)
    (directory / "images" / "ball.png").write_bytes(b"image")
    (directory / "manifest.json").write_text(json.dumps([{"image": "images/ball.png"}]))
    return directory


def make_archive(path, prefix="", extra=None, image="images/ball.png", records=None):
    with tarfile.open(path, "w:gz") as archive:
        files = {
            "manifest.json": json.dumps(records if records is not None else [{"image": image}]).encode(),
            "images/ball.png": b"image",
        }
        for name, content in files.items():
            member = tarfile.TarInfo(prefix + name)
            member.size = len(content)
            archive.addfile(member, io.BytesIO(content))
        if extra is not None:
            archive.addfile(extra, io.BytesIO(b""))


@pytest.fixture
def fake_wget(tmp_path, monkeypatch):
    executable = tmp_path / "fake-wget"
    executable.write_text(
        f"#!{sys.executable}\n"
        "import os, pathlib, shutil, sys\n"
        "pathlib.Path(os.environ['WGET_LOG']).write_text(sys.argv[-1])\n"
        "output = pathlib.Path(sys.argv[sys.argv.index('--output-document') + 1])\n"
        "if os.environ.get('WGET_FAIL'):\n"
        "    output.write_bytes(b'partial')\n"
        "    print('Verbindung abgebrochen', file=sys.stderr)\n"
        "    sys.exit(4)\n"
        "shutil.copyfile(os.environ['WGET_ARCHIVE'], output)\n"
    )
    executable.chmod(0o755)
    monkeypatch.setenv("WGET_LOG", str(tmp_path / "wget.log"))
    return executable


def test_offline_reuse(tmp_path, monkeypatch, capsys):
    for name in ("orange", "realistic"):
        local_dataset(tmp_path, name)

    def no_network(*args, **kwargs):
        pytest.fail("Vollständige lokale Daten dürfen keinen Download auslösen")

    monkeypatch.setattr(downloads.subprocess, "run", no_network)
    assert set(downloads.ensure_datasets({}, root=tmp_path)) == {"orange", "realistic"}
    assert "kein Download nötig" in capsys.readouterr().out


def test_missing_placeholder_never_requested(tmp_path, monkeypatch):
    monkeypatch.setattr(downloads.subprocess, "run", lambda *args, **kwargs: pytest.fail("Netzwerkaufruf"))
    with pytest.raises(downloads.DatasetDownloadError, match="Download-URL"):
        downloads.ensure_datasets({"orange": "https://data.bit-bots.de/REPLACE_ME/orange.tar.gz"}, root=tmp_path)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("name", ["orange", "realistic"])
@pytest.mark.parametrize("wrapped", [False, True])
def test_valid_download_only_missing_set(tmp_path, fake_wget, monkeypatch, name, wrapped):
    root = tmp_path / "datasets"
    other = "realistic" if name == "orange" else "orange"
    local_dataset(root, other)
    archive = tmp_path / "input.tar.gz"
    make_archive(archive, prefix=f"{name}/" if wrapped else "")
    monkeypatch.setenv("WGET_ARCHIVE", str(archive))
    url = f"https://example.invalid/{name}.tar.gz"
    downloads.ensure_datasets({name: url}, root=root, wget=fake_wget)
    assert (root / name / "images/ball.png").read_bytes() == b"image"
    assert (tmp_path / "wget.log").read_text() == url
    assert sorted(item.name for item in root.iterdir()) == ["orange", "realistic"]


@pytest.mark.parametrize("kind", ["traversal", "absolute", "symlink", "hardlink", "block", "fifo", "duplicate"])
def test_unsafe_tar_rejected(tmp_path, fake_wget, monkeypatch, kind):
    root = tmp_path / "datasets"
    local_dataset(root, "realistic")
    outside = tmp_path / "untouched"
    outside.write_text("original")
    member = tarfile.TarInfo("unexpected")
    if kind == "traversal":
        member.name = "../untouched"
    elif kind == "absolute":
        member.name = str(outside)
    elif kind == "duplicate":
        member.name = "manifest.json"
    else:
        member.type = {
            "symlink": tarfile.SYMTYPE,
            "hardlink": tarfile.LNKTYPE,
            "block": tarfile.BLKTYPE,
            "fifo": tarfile.FIFOTYPE,
        }[kind]
        member.linkname = str(outside)
    archive = tmp_path / "unsafe.tar.gz"
    make_archive(archive, extra=member)
    monkeypatch.setenv("WGET_ARCHIVE", str(archive))
    with pytest.raises(downloads.DatasetDownloadError):
        downloads.ensure_datasets({"orange": "https://example.invalid/data.tar.gz"}, root=root, wget=fake_wget)
    assert outside.read_text() == "original"
    assert [item.name for item in root.iterdir()] == ["realistic"]


@pytest.mark.parametrize("image", ["../outside.png", "/absolute.png", "missing.png", "images\\ball.png"])
def test_invalid_image_reference(tmp_path, fake_wget, monkeypatch, image):
    archive = tmp_path / "input.tar.gz"
    make_archive(archive, image=image)
    monkeypatch.setenv("WGET_ARCHIVE", str(archive))
    root = tmp_path / "datasets"
    with pytest.raises(downloads.DatasetDownloadError):
        downloads.ensure_datasets({"orange": "https://example.invalid/data.tar.gz"}, root=root, wget=fake_wget)
    assert not list(root.iterdir())


@pytest.mark.parametrize("records", [{"image": "images/ball.png"}, [], [{}]])
def test_invalid_manifest(tmp_path, fake_wget, monkeypatch, records):
    archive = tmp_path / "input.tar.gz"
    make_archive(archive, records=records)
    monkeypatch.setenv("WGET_ARCHIVE", str(archive))
    root = tmp_path / "datasets"
    with pytest.raises(downloads.DatasetDownloadError):
        downloads.ensure_datasets({"orange": "https://example.invalid/data.tar.gz"}, root=root, wget=fake_wget)
    assert not list(root.iterdir())


def test_download_failure_no_partial_destination(tmp_path, fake_wget, monkeypatch):
    monkeypatch.setenv("WGET_FAIL", "1")
    root = tmp_path / "datasets"
    with pytest.raises(downloads.DatasetDownloadError, match="wget fehlgeschlagen.*Verbindung abgebrochen"):
        downloads.ensure_datasets({"orange": "https://example.invalid/data.tar.gz"}, root=root, wget=fake_wget)
    assert not list(root.iterdir())


def test_incomplete_existing_set_not_overwritten(tmp_path, monkeypatch):
    directory = local_dataset(tmp_path, "orange")
    (directory / "images/ball.png").unlink()
    monkeypatch.setattr(downloads.subprocess, "run", lambda *args, **kwargs: pytest.fail("Netzwerkaufruf"))
    with pytest.raises(downloads.DatasetDownloadError, match="Bild fehlt"):
        downloads.ensure_datasets({}, root=tmp_path)
    assert (directory / "manifest.json").is_file()


def test_local_symlink_not_reused(tmp_path):
    directory = local_dataset(tmp_path, "orange")
    (directory / "images/ball.png").unlink()
    (directory / "images/ball.png").symlink_to(directory / "manifest.json")
    with pytest.raises(downloads.DatasetDownloadError, match="Verknüpfung"):
        downloads.ensure_datasets({}, root=tmp_path)


def test_root_falls_back_to_helper_directory(monkeypatch):
    import vision_workshop

    def no_manifest():
        raise FileNotFoundError

    monkeypatch.setattr(vision_workshop, "find_dataset_root", no_manifest)
    assert downloads.dataset_root() == Path(downloads.__file__).resolve().parent / "datasets"
