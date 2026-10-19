import ast
import importlib.util
import inspect
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock, patch
from xml.etree import ElementTree

import cv2
import numpy as np
from cv_bridge import CvBridge
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters
from schnupper_vision.detector import detect_balls, zu_hsv
from schnupper_vision.node import SchnupperVision, make_ball_array, validate_boxes
from std_msgs.msg import Header

PACKAGE = Path(__file__).resolve().parents[1]
BRINGUP = PACKAGE.parent / "bitbots_misc" / "bitbots_bringup"


def load_launch(path):
    spec = importlib.util.spec_from_file_location("vision_launch_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestDetector(unittest.TestCase):
    def test_empty_image(self):
        self.assertIsNone(detect_balls(np.zeros((80, 120, 3), dtype=np.uint8)))

    def test_orange_ball_and_no_input_mutation(self):
        image = np.zeros((120, 180, 3), dtype=np.uint8)
        cv2.circle(image, (40, 50), 15, (0, 140, 255), -1)
        cv2.circle(image, (90, 25), 10, (255, 0, 0), -1)
        before = image.copy()
        box = detect_balls(image)
        self.assertIsInstance(box, tuple)
        left, top, right, bottom = box
        self.assertTrue(left < 40 < right and top < 50 < bottom)
        np.testing.assert_array_equal(before, image)

    def test_notebook_body_needs_only_image_zu_hsv_and_np(self):
        function = ast.parse(inspect.getsource(detect_balls)).body[0]
        function.args.args[0].annotation = None
        function.returns = None
        namespace = {"zu_hsv": zu_hsv, "np": np}
        exec(
            compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])), "notebook", "exec"),
            namespace,
        )
        self.assertIsNone(namespace["detect_balls"](np.zeros((40, 40, 3), dtype=np.uint8)))


class TestRosConversion(unittest.TestCase):
    def test_box_contract_and_clipping(self):
        box = (-5, -2, 30, 20)
        np.testing.assert_array_equal(validate_boxes(box, (80, 120, 3)), [[0, 0, 30, 20]])
        self.assertEqual(box[0], -5)
        self.assertEqual(validate_boxes((200, 200, 250, 250), (80, 120, 3)).shape, (0, 4))
        self.assertEqual(validate_boxes(None, (80, 120, 3)).shape, (0, 4))
        for invalid in (
            [],
            np.empty((0, 4)),
            (0, 0, 1),
            (0, 0, np.nan, 4),
            (4, 0, 1, 3),
            (0, 0, 0, 3),
            ("a",) * 4,
            (False,) * 4,
        ):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                validate_boxes(invalid, (80, 120, 3))

    def test_message_fields_and_header(self):
        header = Header(frame_id="camera_optical_frame")
        header.stamp.sec = 42
        header.stamp.nanosec = 123
        message = make_ball_array(np.array([[10, 20, 30, 60]]), header)
        self.assertEqual(message.header, header)
        ball = message.balls[0]
        self.assertEqual((ball.center.x, ball.center.y), (20.0, 40.0))
        self.assertEqual((ball.bb.center.position.x, ball.bb.center.position.y), (20.0, 40.0))
        self.assertEqual((ball.bb.size_x, ball.bb.size_y), (20.0, 40.0))
        self.assertEqual(ball.confidence.confidence, 1.0)
        self.assertEqual(make_ball_array(np.empty((0, 4)), header).balls, [])

    def test_callback_debug_and_detector_errors(self):
        bridge = CvBridge()
        image = bridge.cv2_to_imgmsg(np.zeros((80, 120, 3), dtype=np.uint8), encoding="bgr8")
        image.header.frame_id = "camera"
        image.header.stamp.sec = 17
        node = SimpleNamespace(
            bridge=bridge,
            ball_publisher=Mock(),
            debug_publisher=Mock(),
            get_parameter=Mock(return_value=SimpleNamespace(value=True)),
            get_logger=Mock(return_value=Mock()),
        )
        with patch("schnupper_vision.node.detect_balls", return_value=(10, 20, 30, 40)):
            SchnupperVision.image_callback(node, image)
        self.assertEqual(node.ball_publisher.publish.call_args.args[0].header, image.header)
        debug = node.debug_publisher.publish.call_args.args[0]
        self.assertEqual(debug.header, image.header)
        self.assertEqual(debug.encoding, "bgr8")
        self.assertTrue(bridge.imgmsg_to_cv2(debug).any())
        self.assertFalse(bridge.imgmsg_to_cv2(image).any())
        node.debug_publisher.reset_mock()
        node.get_parameter.return_value.value = False
        for result in (None, np.array([1, 2]), RuntimeError("Schülerfehler")):
            with self.subTest(result=result):
                with patch("schnupper_vision.node.detect_balls") as detector:
                    if isinstance(result, Exception):
                        detector.side_effect = result
                    else:
                        detector.return_value = result
                    SchnupperVision.image_callback(node, image)
                self.assertEqual(node.ball_publisher.publish.call_args.args[0].balls, [])
        node.debug_publisher.publish.assert_not_called()


class TestLaunchContract(unittest.TestCase):
    def test_backend_selection_and_parameters(self):
        module = load_launch(PACKAGE / "launch" / "vision_backend.launch.py")
        declarations = {
            action.name: action for action in module.generate_launch_description().entities if hasattr(action, "name")
        }
        context = LaunchContext()
        self.assertEqual(declarations["vision_backend"].default_value[0].perform(context), "original")
        for backend, executable in (("original", "vision"), ("schnupper", "schnupper_vision_node")):
            context.launch_configurations.update(vision_backend=backend, sim="true", debug="true")
            with patch.object(module, "get_package_share_directory", return_value=str(PACKAGE)):
                nodes = module.launch_vision(context)
            self.assertEqual(len(nodes), 1)
            self.assertEqual(nodes[0].node_executable, executable)
            params = evaluate_parameters(context, nodes[0]._Node__parameters)
            self.assertEqual(params[-1], {"components.debug_active": True, "use_sim_time": True})
        context.launch_configurations["vision_backend"] = "typo"
        with self.assertRaises(ValueError):
            module.launch_vision(context)

    def test_xml_forwarding(self):
        edges = {
            "vision.launch": "bitbots_vision)/launch/vision.launch",
            "vision_standalone.launch": "bitbots_bringup)/launch/vision.launch",
            "highlevel.launch": "bitbots_bringup)/launch/vision.launch",
            "teamplayer.launch": "bitbots_bringup)/launch/highlevel.launch",
            "simulator_teamplayer.launch": "bitbots_bringup)/launch/teamplayer.launch",
            "demo.launch": "bitbots_bringup)/launch/teamplayer.launch",
        }
        for filename, target in edges.items():
            with self.subTest(filename=filename):
                root = ElementTree.parse(BRINGUP / "launch" / filename).getroot()
                expected = (
                    "schnupper" if filename in {"teamplayer.launch", "simulator_teamplayer.launch"} else "original"
                )
                self.assertEqual(root.find("arg[@name='vision_backend']").get("default"), expected)
                include = next(entry for entry in root.iter("include") if target in entry.get("file"))
                self.assertEqual(include.find("arg[@name='vision_backend']").get("value"), "$(var vision_backend)")
        root = ElementTree.parse(PACKAGE / "launch" / "vision.launch").getroot()
        self.assertEqual(root.find("include/arg[@name='vision_backend']").get("value"), "$(var vision_backend)")

    def test_mujoco_exposes_teamplayer_arguments(self):
        module = load_launch(BRINGUP / "launch" / "mujoco_simulation.launch.py")
        names = {name for name, _ in module.TEAMPLAYER_ARGS}
        self.assertTrue({"vision_backend", "vision_debug"}.issubset(names))
        self.assertEqual(module.PRAKTIKUM_DEFAULTS["vision_backend"], "schnupper")

    def test_mujoco_ball_style_templates(self):
        module = load_launch(BRINGUP / "launch" / "mujoco_simulation.launch.py")
        declarations = {
            action.name: action for action in module.generate_launch_description().entities if hasattr(action, "name")
        }
        self.assertEqual(declarations["ball_style"].default_value[0].perform(LaunchContext()), "orange")
        self.assertEqual(declarations["teamplayer"].default_value[0].perform(LaunchContext()), "true")
        with TemporaryDirectory() as directory:
            xml_dir = Path(directory) / "xml"
            xml_dir.mkdir()
            for style, filename in (("original", "kid_field.xml"), ("orange", "kid_field_orange.xml")):
                (xml_dir / filename).write_text(style + " {{NUM_ROBOTS}} {{OFFSET}} {{START_X}} {{ROBOT_TYPE}}")
                output = module.generate_world_xml(2, directory, "piplus", False, style).read_text()
                self.assertTrue(output.startswith(style + " 2 "))
                self.assertTrue(output.endswith(" piplus"))
                self.assertNotIn("{{", output)
            self.assertTrue(module.generate_world_xml(1, directory, "piplus", False).read_text().startswith("orange"))
            with self.assertRaises(ValueError):
                module.generate_world_xml(1, directory, "piplus", False, "typo")

    def test_mujoco_setup_passes_ball_style(self):
        module = load_launch(BRINGUP / "launch" / "mujoco_simulation.launch.py")
        context = LaunchContext()
        context.launch_configurations.update({name: "" for name, _ in module.TEAMPLAYER_ARGS})
        context.launch_configurations.update(
            num_robots="1",
            one_versus_one="false",
            robot_type="piplus",
            ball_style="orange",
            web="true",
            teamplayer="true",
            fake_localization="false",
            vision_backend="schnupper",
            vision_debug="true",
        )
        with (
            patch.object(module, "get_package_share_directory", return_value="/offline/sim"),
            patch.object(module, "generate_world_xml", return_value=Path("/offline/world.xml")) as generate_world,
            patch.object(module, "generate_domain_bridge_config", return_value=Path("/offline/bridge.yaml")),
        ):
            module.launch_setup(context)
        generate_world.assert_called_once_with(1, "/offline/sim", "piplus", False, "orange")


if __name__ == "__main__":
    unittest.main()
