from pathlib import Path

import numpy as np
import onnx
import onnxruntime as rt

MODELS_DIR = Path(__file__).parent.parent / "models"


# Walking
def test_walk_model_loadable():
    model_path = MODELS_DIR / "wolfgang_walk_ppo.onnx"
    model = onnx.load(model_path)
    onnx.checker.check_model(model)


def test_walk_model_input_output_shape():
    model_path = MODELS_DIR / "wolfgang_walk_ppo.onnx"
    model = onnx.load(model_path)

    inputs = model.graph.input
    outputs = model.graph.output

    assert inputs[0].type.tensor_type.shape.dim[1] == 49
    assert outputs[0].type.tensor_type.shape.dim[1] == 18


def test_walk_model_runnable():
    model_path = MODELS_DIR / "wolfgang_walk_ppo.onnx"
    session = rt.InferenceSession(str(model_path))

    dummy_input = np.zeros((1, 49), dtype=np.float32)
    result = session.run(None, {"in_0": dummy_input})

    assert result[0].shape == (1, 18)


# Kicking
def test_kick_model_loadable():
    model_path = MODELS_DIR / "wolfgang_dribbling_ppo.onnx"
    model = onnx.load(model_path)
    onnx.checker.check_model(model)


def test_kick_model_input_output_shape():
    model_path = MODELS_DIR / "wolfgang_dribbling_ppo.onnx"
    model = onnx.load(model_path)

    inputs = model.graph.input
    outputs = model.graph.output

    assert inputs[0].type.tensor_type.shape.dim[1] == 67
    assert outputs[0].type.tensor_type.shape.dim[1] == 18


def test_kick_model_runnable():
    model_path = MODELS_DIR / "wolfgang_dribbling_ppo.onnx"
    session = rt.InferenceSession(str(model_path))

    dummy_input = np.zeros((1, 67), dtype=np.float32)
    result = session.run(None, {"in_0": dummy_input})

    assert result[0].shape == (1, 18)
