def clock_publisher(queue, clock_topic, model_topic, domain_id):
    import os
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)

    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from gazebo_msgs.msg import ModelStates
    from rclpy.time import Time

    class ClockPublisher(Node):
        def __init__(self):
            super().__init__("clock_publisher")

            self.pub_clock = self.create_publisher(
                Clock,
                clock_topic,
                10
            )
            self.pub_model_states = self.create_publisher(
                ModelStates,
                model_topic,
                10
            )

        # def publish_time(self, t):
        #     msg = Clock()

        #     msg.clock.sec = int(t)
        #     msg.clock.nanosec = int(
        #         (t % 1.0) * 1e9
        #     )

        #     self.pub_clock.publish(msg)


    rclpy.init()

    node = ClockPublisher()

    while rclpy.ok():
        # blocks until supervisor advances simulation
        msg = queue.get()

        if type(msg) is Clock:
            node.pub_clock.publish(msg)
        elif type(msg) is ModelStates:
            node.pub_model_states.publish(msg)
        else:
            raise ValueError(f"Unknown message type: {type(msg)}")
    rclpy.shutdown()
