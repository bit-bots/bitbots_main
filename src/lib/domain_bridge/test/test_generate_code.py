"""Regression tests for interface discovery across incremental builds."""

import importlib.util
import os
import sys
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location("generate_code", Path(__file__).parents[1] / "bin" / "generate_code.py")
generator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generator)


@pytest.fixture
def installed_interfaces(tmp_path, monkeypatch):
    packages = {"test_msgs": str(tmp_path)}
    services = tmp_path / "share" / "test_msgs" / "srv"
    services.mkdir(parents=True)
    (services / "Empty.srv").touch()
    monkeypatch.setattr(generator, "get_packages_with_prefixes", lambda: packages)
    return packages


def generate(output, monkeypatch, *arguments):
    monkeypatch.setattr(sys, "argv", ["generate_code.py", "--output", str(output), *arguments])
    generator.main()


def test_newly_installed_service_regenerates_dispatcher(tmp_path, monkeypatch, installed_interfaces):
    output = tmp_path / "generated_code.cpp"
    generate(output, monkeypatch)
    assert "bitbots_localization/srv/ResetFilter" not in output.read_text()

    installed_interfaces["bitbots_localization"] = str(tmp_path)
    services = tmp_path / "share" / "bitbots_localization" / "srv"
    services.mkdir(parents=True)
    (services / "ResetFilter.srv").touch()
    generate(output, monkeypatch, "--require-service", "bitbots_localization/srv/ResetFilter")
    assert 'service_type == "bitbots_localization/srv/ResetFilter"' in output.read_text()
    assert "bridge_service<bitbots_localization::srv::ResetFilter>" in output.read_text()


def test_unchanged_interfaces_preserve_output(tmp_path, monkeypatch, installed_interfaces):
    services = tmp_path / "share" / "test_msgs" / "srv"
    (services / "Another.srv").touch()
    output = tmp_path / "generated_code.cpp"
    generate(output, monkeypatch)
    os.utime(output, ns=(1_000_000_000, 1_000_000_000))
    timestamp = output.stat().st_mtime_ns
    # Discovery order must not cause unnecessary recompilation either.
    listdir = os.listdir
    monkeypatch.setattr(generator.os, "listdir", lambda path: sorted(listdir(path), reverse=True))
    generate(output, monkeypatch)
    assert output.stat().st_mtime_ns == timestamp


def test_missing_required_service_fails_without_replacing_output(tmp_path, monkeypatch, installed_interfaces, capsys):
    output = tmp_path / "generated_code.cpp"
    generate(output, monkeypatch)
    previous = output.read_bytes()
    with pytest.raises(SystemExit) as error:
        generate(output, monkeypatch, "--require-service", "bitbots_localization/srv/ResetFilter")
    assert error.value.code != 0
    assert "bitbots_localization/srv/ResetFilter" in capsys.readouterr().err
    assert output.read_bytes() == previous
