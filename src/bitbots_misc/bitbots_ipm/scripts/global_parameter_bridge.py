#!/usr/bin/env python3

import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python


class GlobalParameterBridge(Node):
    def __init__(self) -> None:
        super().__init__("ipm_global_parameter_bridge")
        self._source = self.create_client(GetParameters, "/parameter_blackboard/get_parameters")
        self._targets = {
            "/soccer_ipm": (
                self.create_client(SetParameters, "/soccer_ipm/set_parameters"),
                "balls.ball_diameter",
            ),
            "/soccer_vision_3d_rviz_marker_visualizer": (
                self.create_client(
                    SetParameters,
                    "/soccer_vision_3d_rviz_marker_visualizer/set_parameters",
                ),
                "ball_diameter",
            ),
        }
        self._ball_diameter: float | None = None
        self._pending = False
        self._completed: set[str] = set()
        self._timer = self.create_timer(0.25, self._tick)

    def _tick(self) -> None:
        if self._ball_diameter is None:
            if self._pending or not self._source.service_is_ready():
                return
            self._pending = True
            request = GetParameters.Request()
            request.names = ["ball.diameter"]
            self._source.call_async(request).add_done_callback(self._received_global_parameter)
            return

        for node_name, (client, parameter_name) in self._targets.items():
            if node_name in self._completed or not client.service_is_ready():
                continue
            request = SetParameters.Request()
            request.parameters = [Parameter(parameter_name, value=self._ball_diameter).to_parameter_msg()]
            client.call_async(request).add_done_callback(
                lambda future, name=node_name: self._parameter_set(name, future)
            )
            self._completed.add(node_name)

        if len(self._completed) == len(self._targets):
            self.get_logger().info(f"Applied global ball diameter {self._ball_diameter:.3f} m to imported IPM nodes")
            self._timer.cancel()

    def _received_global_parameter(self, future) -> None:
        self._pending = False
        try:
            response = future.result()
            value = parameter_value_to_python(response.values[0])
            if not isinstance(value, float) or value <= 0:
                raise ValueError(f"invalid ball.diameter value {value!r}")
            self._ball_diameter = value
        except Exception as exc:
            self.get_logger().warning(f"Could not read global ball.diameter yet: {exc}")

    def _parameter_set(self, node_name: str, future) -> None:
        try:
            response = future.result()
            if not response.results[0].successful:
                self._completed.discard(node_name)
                self.get_logger().warning(f"{node_name} rejected ball diameter: {response.results[0].reason}")
        except Exception as exc:
            self._completed.discard(node_name)
            self.get_logger().warning(f"Could not update {node_name}: {exc}")


def main() -> None:
    rclpy.init()
    node = GlobalParameterBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
