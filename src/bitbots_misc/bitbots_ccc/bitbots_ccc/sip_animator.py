import os
import time
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from rclpy.action import ActionClient

from bitbots_msgs.action import PlayAnimation

from pyVoIP.VoIP import VoIPPhone, InvalidStateError, CallState


class SIPAnimator(Node):
    def __init__(self):
        super().__init__('sip_animator_node')

        # Action client
        self.action_client = ActionClient(self, PlayAnimation, 'animation')

        # Get environment variables for SIP configuration
        sip_server = os.getenv("SIP_SERVER_IP")
        sip_port = int(os.getenv("SIP_SERVER_PORT", "5060"))
        username = os.getenv("SIP_USERNAME")
        password = os.getenv("SIP_PASSWORD")
        local_ip = os.getenv("LOCAL_IP")

        if not all([sip_server, username, password, local_ip]):
            msg = "SIP configuration environment variables are not fully set."
            self.get_logger().fatal(msg)
            raise RuntimeError(msg)

        # SIP phone setup
        self.phone = VoIPPhone(
            sip_server,
            sip_port,
            username,
            password,
            myIP=local_ip,
            callCallback=self.answer_call
        )

        self.phone.start()
        self.get_logger().info("Phone action node started")

    def answer_call(self, call):
        try:
            self.get_logger().info("Incoming call")
            call.answer()

            while call.state == CallState.ANSWERED:
                dtmf = call.get_dtmf()

                if dtmf == "1":
                    self.get_logger().info("DTMF 1 pressed")
                    self.run_animation("cheering_only_arms")

                elif dtmf == "2":
                    self.get_logger().info("DTMF 2 pressed")

                time.sleep(0.1)

        except InvalidStateError:
            pass
        except Exception as e:
            self.get_logger().error(str(e))
            call.hangup()

    def run_animation(self, animation_name: str):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available")
            return

        goal_msg = PlayAnimation.Goal()
        goal_msg.animation = animation_name
        goal_msg.hcm = True

        def result_cb(future: Future):
            result = future.result().result
            if not result.successful:
                self.get_logger().error(f"Animation '{animation_name}' failed")
            self.get_logger().info(f"Animation '{animation_name}' completed")

        def accept_cb(future: Future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info(f"Animation '{animation_name}' rejected")
                return

            result_future: Future = goal_handle.get_result_async()
            result_future.add_done_callback(result_cb)

        self.get_logger().info("Sending action goal")
        self.action_client.send_goal_async(goal_msg).add_done_callback(accept_cb)


def main(args=None):
    rclpy.init(args=args)
    node = SIPAnimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.phone.stop()
    node.destroy_node()


if __name__ == '__main__':
    main()
