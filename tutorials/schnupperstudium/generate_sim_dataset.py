"""Render the orange tutorial dataset offline; see SIM_DATASET.md for provenance."""

import argparse
import hashlib
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

REPOSITORY = Path(__file__).resolve().parents[2]
SCENE_DIRECTORY = REPOSITORY / "src/bitbots_simulation/bitbots_mujoco_sim/xml"
SCENE = SCENE_DIRECTORY / "dataset_orange.xml"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "datasets/orange"


def expand_scene(path: Path) -> ET.Element:
    """Resolve MJCF includes while keeping the package's original assets."""
    root = ET.parse(path).getroot()
    for include in list(root.findall("include")):
        included = expand_scene(path.parent / include.attrib["file"])
        position = list(root).index(include)
        root.remove(include)
        for child in included:
            root.insert(position, child)
            position += 1
    return root


def scene_xml(positive: bool, width: int, height: int) -> str:
    root = expand_scene(SCENE)
    root.find("compiler").set("assetdir", str(SCENE_DIRECTORY / "assets"))
    visual = root.find("visual/global")
    visual.set("offwidth", str(width))
    visual.set("offheight", str(height))
    if not positive:
        for worldbody in root.findall("worldbody"):
            for body in list(worldbody.findall("body")):
                if body.get("name") == "ball":
                    worldbody.remove(body)
    return ET.tostring(root, encoding="unicode")


def mask_box(mask: np.ndarray) -> list[int] | None:
    """Return a tight visible-pixel box with exclusive right and bottom edges."""
    rows, columns = np.nonzero(mask)
    if not len(rows):
        return None
    return [int(columns.min()), int(rows.min()), int(columns.max()) + 1, int(rows.max()) + 1]


def split_counts(count: int, test_fraction: float) -> dict[str, int]:
    test_count = round(count * test_fraction)
    return {"train": count - test_count, "test": test_count}


def scene_digest() -> str:
    digest = hashlib.sha256()
    for path in sorted(SCENE_DIRECTORY.rglob("*")):
        if path.is_file() and (
            path.name in {"dataset_field.xml", "dataset_orange.xml", "goal_kid.xml", "kid_field.png"}
            or path.parent.name == "backgrounds"
        ):
            digest.update(str(path.relative_to(SCENE_DIRECTORY)).encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


def generate(args: argparse.Namespace) -> list[dict]:
    os.environ["MUJOCO_GL"] = args.backend
    import mujoco

    output = args.output.resolve()
    preparation = output.parent.parent / "preparation" / output.name
    preparation.mkdir(parents=True, exist_ok=True)
    manifest_path = preparation / "sim_manifest.json"
    if manifest_path.exists() and not args.overwrite:
        raise FileExistsError(f"{manifest_path} exists; use --overwrite to regenerate simulation records")
    (output / "images").mkdir(parents=True, exist_ok=True)
    records = []
    digest = scene_digest()
    for positive, count in ((True, args.positives), (False, args.negatives)):
        model = mujoco.MjModel.from_xml_string(scene_xml(positive, args.width, args.height))
        data = mujoco.MjData(model)
        ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ball")
        if (ball_id >= 0) != positive:
            raise RuntimeError("Ball geometry does not match the requested scene class")
        pitch_id = model.material("pitch").id
        light_id = model.light("dataset_light").id
        joint_address = model.joint("ball-root").qposadr[0] if positive else None
        with mujoco.Renderer(model, height=args.height, width=args.width) as renderer:
            for split_index, (split, split_count) in enumerate(split_counts(count, args.test_fraction).items()):
                seed = np.random.SeedSequence([args.seed, split_index, int(positive)])
                rng = np.random.default_rng(seed)
                for index in range(split_count):
                    identifier = f"sim_orange_{split}_{'positive' if positive else 'negative'}_{index:04d}"
                    image_path = output / "images" / f"{identifier}.png"
                    if not args.overwrite and image_path.exists():
                        raise FileExistsError(f"Refusing to replace existing sample {identifier}")
                    for _attempt in range(args.max_attempts):
                        mujoco.mj_resetData(model, data)
                        anchor = np.array([rng.uniform(-3.5, 3.5), rng.uniform(-2.3, 2.3), 0.0])
                        radius = float(rng.uniform(0.055, 0.095))
                        ball_rgba = [1.0, float(rng.uniform(0.22, 0.42)), float(rng.uniform(0.0, 0.035)), 1.0]
                        if positive:
                            model.geom_size[ball_id, 0] = radius
                            model.geom_rgba[ball_id] = ball_rgba
                            data.qpos[joint_address : joint_address + 3] = [*anchor[:2], radius]
                            quaternion = np.empty(4)
                            axis = rng.normal(size=3)
                            axis /= np.linalg.norm(axis)
                            mujoco.mju_axisAngle2Quat(quaternion, axis, rng.uniform(-np.pi, np.pi))
                            data.qpos[joint_address + 3 : joint_address + 7] = quaternion
                        pitch_rgba = [float(rng.uniform(0.65, 1.0)) for _ in range(3)] + [1.0]
                        light_diffuse = [float(rng.uniform(0.45, 1.0)) for _ in range(3)]
                        light_direction = [float(rng.uniform(-0.8, 0.8)), float(rng.uniform(-0.8, 0.8)), -1.0]
                        model.mat_rgba[pitch_id] = pitch_rgba
                        model.light_diffuse[light_id] = light_diffuse
                        model.light_dir[light_id] = light_direction
                        mujoco.mj_forward(model, data)
                        for _ in range(25):
                            mujoco.mj_step(model, data)
                        camera = mujoco.MjvCamera()
                        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
                        camera.lookat[:] = anchor + [rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), 0.0]
                        camera.distance = rng.uniform(1.2, 4.0)
                        camera.azimuth = rng.uniform(-180, 180)
                        camera.elevation = rng.uniform(-40, -12)
                        renderer.disable_segmentation_rendering()
                        renderer.update_scene(data, camera=camera)
                        rgb = renderer.render().copy()
                        renderer.enable_segmentation_rendering()
                        segmentation = renderer.render()
                        mask = (segmentation[:, :, 0] == ball_id) & (
                            segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)
                        )
                        if not positive:
                            mask[:] = False
                        box = mask_box(mask)
                        if positive and (box is None or int(mask.sum()) < 30):
                            continue
                        break
                    else:
                        raise RuntimeError(f"Could not produce a visible ball for {identifier}")
                    if not cv2.imwrite(str(image_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                        raise OSError(f"Could not write {image_path}")
                    records.append(
                        {
                            "id": identifier,
                            "image": f"images/{identifier}.png",
                            "boxes": [box] if positive else [],
                            "source": "simulation",
                            "split": split,
                            "provenance": {
                                "generator": "tutorials/schnupperstudium/generate_sim_dataset.py",
                                "generator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                                "scene": str(SCENE.relative_to(REPOSITORY)),
                                "scene_assets_sha256": digest,
                                "mujoco_version": mujoco.__version__,
                                "backend": args.backend,
                                "seed_entropy": seed.entropy,
                                "sample_index": index,
                                "attempt": _attempt,
                                "image_size": [args.width, args.height],
                                "ball_present": positive,
                                "ball_radius": radius if positive else None,
                                "ball_rgba": ball_rgba if positive else None,
                                "ball_position": data.qpos[joint_address : joint_address + 3].tolist()
                                if positive
                                else None,
                                "camera": {
                                    "lookat": camera.lookat.tolist(),
                                    "distance": float(camera.distance),
                                    "azimuth": float(camera.azimuth),
                                    "elevation": float(camera.elevation),
                                },
                                "pitch_rgba": pitch_rgba,
                                "light_diffuse": light_diffuse,
                                "light_direction": light_direction,
                                "annotation": "mujoco_visible_geom_segmentation_xyxy_exclusive",
                                "visible_pixels": int(mask.sum()),
                                "negative_method": None if positive else "ball_body_removed_before_compile",
                            },
                        }
                    )
                print(f"Rendered {split_count} {split} {'positive' if positive else 'negative'} samples", flush=True)
    temporary = manifest_path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(records, indent=2) + "\n")
    temporary.replace(manifest_path)
    return records


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--positives", type=int, default=100)
    parser.add_argument("--negatives", type=int, default=100)
    parser.add_argument("--test-fraction", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=20261016)
    parser.add_argument("--width", type=int, default=672)
    parser.add_argument("--height", type=int, default=376)
    parser.add_argument("--backend", choices=("egl", "osmesa"), default="egl")
    parser.add_argument("--max-attempts", type=int, default=100)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if min(args.positives, args.negatives) < 0 or args.positives + args.negatives == 0:
        parser.error("Sample counts must be nonnegative and their sum must be positive")
    if not 0 < args.test_fraction < 1:
        parser.error("--test-fraction must be between zero and one")
    if min(args.width, args.height, args.max_attempts) <= 0 or args.seed < 0:
        parser.error("Dimensions and attempts must be positive; seed must be nonnegative")
    return args


if __name__ == "__main__":
    generate(parse_args())
