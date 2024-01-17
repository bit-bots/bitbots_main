#!/usr/bin/env python3
import argparse
import time
from collections import deque, defaultdict

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_msgs.msg import TFMessage


class TFMonitor:
    def __init__(self, use_chain, frame_a=None, frame_b=None):
        """
        The TFMonitor class, used to monitor and display the currently broadcasted transforms

        :param use_chain: Whether a specific chain is monitored. If True, specify frame_a and frame_b
        :param frame_a: The start frame of the monitored chain
        :param frame_b: The end frame of the monitored chain
        """
        def deque_1000():
            """Construct a deque that stores at most 1000 items"""
            return deque(maxlen=1000)

        self.use_chain = use_chain
        self.frame_a = frame_a
        self.frame_b = frame_b

        # A dict mapping frames to their broadcasters
        self.frame_broadcaster_dict = defaultdict(str)
        # A dict mapping broadcasters to a list of the last average delays
        self.broadcaster_delay_dict = defaultdict(deque_1000)
        # A dict mapping broadcasters to a list of the last time they broadcasted a transform
        self.broadcaster_frequency_dict = defaultdict(deque_1000)
        # A dict mapping frames to a list of the last delays
        self.frame_delay_dict = defaultdict(deque_1000)
        # A list with the delays of the chain
        self.chain_delay = deque_1000()
        self.chain = []

        if use_chain:
            # Wait for the chain
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            print(f'Waiting for transform chain to become available between {frame_a} and {frame_b}...')
            while rclpy.ok():
                if self.tf_buffer.can_transform(frame_a, frame_b, rospy.Time()):
                    try:
                        self.chain = self.tf_buffer._chain(frame_b, rospy.Time(), frame_a, rospy.Time(), frame_b)
                    except tf2_ros.TransformException as e:
                        rospy.logwarn("Transform Exception", e)
                    break
                else:
                    time.sleep(0.1)

    def callback(self, msg: TFMessage, is_static):
        """
        Callback for new tf messages. Processes the message and updates
        the stored dictionaries for the current publishers and delays.

        :param msg: The TF message
        :param is_static: Whether the transform is static (delay = 0)
        """
        delay_sum = 0
        update_chain = False
        broadcaster = msg._connection_header['callerid']
        if is_static:
            broadcaster += ' (static)'

        for transform in msg.transforms:
            # Update the information about this transform
            if transform.child_frame_id in self.chain:
                # When a frame of the chain is in this message, the chain delay has to be updated
                update_chain = True
            self.frame_broadcaster_dict[transform.child_frame_id] = broadcaster
            if is_static:
                delay = 0
            else:
                delay = (self.get_clock().now() - transform.header.stamp).to_sec()
            delay_sum += delay
            self.frame_delay_dict[transform.child_frame_id].append(delay)

        # Update the information about the broadcaster of this message
        average_delay = delay_sum / max(1, len(msg.transforms))
        self.broadcaster_delay_dict[broadcaster].append(average_delay)
        self.broadcaster_frequency_dict[broadcaster].append(self.get_clock().now().to_sec())

        if update_chain and self.use_chain:
            # Update the chain delay
            tmp = self.tf_buffer.lookup_transform(self.frame_a, self.frame_b, rospy.Time())
            delay = (self.get_clock().now() - tmp.header.stamp).to_sec()
            self.chain_delay.append(delay)

    def display_frames(self, event=None):
        """
        Display the frames that are monitored. This function is usually called from a ros timer.
        """
        if not self.frame_delay_dict:
            # This can happen when display_frames is called before the first tf callback
            return

        print('\n\n')
        if event.last_real and event.current_real:
            monitor_freq = round(1/(event.current_real - event.last_real).to_sec(), 3)
            print(f'The tf_monitor is running with a display rate of {monitor_freq:.2f} Hz\n')

        # For pretty-printing
        round_len = 10
        num_len = round_len + 5
        if self.use_chain:
            frames = set(frame for frame in self.frame_delay_dict if frame in self.chain)
            broadcasters = set(self.frame_broadcaster_dict[frame] for frame in frames)
        else:
            frames = self.frame_delay_dict.keys()
            broadcasters = self.broadcaster_delay_dict.keys()
        frame_len = max(len(frame) for frame in frames)
        broadcaster_len = max(len(broadcaster) for broadcaster in broadcasters)

        if self.use_chain:
            # Print general information about the chain
            print('Chain is:', ' -> '.join(self.chain))
            average_delay = round(sum(self.chain_delay) / len(self.chain_delay), round_len)
            max_delay = round(max(self.chain_delay), round_len)
            print(f'Average Net Delay: {average_delay:.{num_len}f}    Max Net Delay: {max_delay:.{num_len}f}')
            print('\nFrames in chain:')
        else:
            print('All Frames:')

        # Print details about each frame
        for frame in sorted(frames):
            delay_list = self.frame_delay_dict[frame]
            average_delay = round(sum(delay_list) / len(delay_list), round_len)
            max_delay = round(max(delay_list), round_len)
            print(f'Frame: {frame:{frame_len}}     Published by {self.frame_broadcaster_dict[frame]:<{broadcaster_len}}   '
                  f'Average Delay: {average_delay:.{num_len}f}    Max Delay: {max_delay:.{num_len}f}')

        # Print details about each broadcaster
        if self.use_chain:
            print('\nBroadcasters in chain:')
        else:
            print('\nAll Broadcasters:')
        for node in sorted(broadcasters):
            average_delay = round(sum(self.broadcaster_delay_dict[node]) / len(self.broadcaster_delay_dict[node]), round_len)
            max_delay = round(max(self.broadcaster_delay_dict[node]), round_len)
            frequency_list = self.broadcaster_frequency_dict[node]
            frequency_out = round(len(frequency_list) / max(1e-8, (frequency_list[-1] - frequency_list[0])), round_len)
            print(f'Node: {node:{broadcaster_len}} {frequency_out:.{num_len}f} Hz    '
                  f'Average Delay: {average_delay:.{num_len}f}    Max Delay: {max_delay:.{num_len}f}')


if __name__ == '__main__':
    rclpy.init(args=None)

    parser = argparse.ArgumentParser(usage='%(prog)s [-h] [--tf-topic TOPIC] [--display-rate HZ] [frame_a frame_b]\n',
                                     description='Monitor the published tf messages. Without command line arguments, '
                                                 'monitor all frames and their publishers, with two arguments '
                                                 'monitor the chain between them.')
    parser.add_argument('frame_a', nargs='?', help='The start frame of the monitored chain')
    parser.add_argument('frame_b', nargs='?', help='The end frame of the monitored chain')
    parser.add_argument('--tf-topic', required=False, default='tf', metavar='TOPIC', help='tf topic to listen to')
    parser.add_argument('--display-rate', required=False, type=float, default=5.0, metavar='HZ',
                        help='display update rate')
    args = parser.parse_args()

    if args.frame_a and not args.frame_b:
        parser.error("Specify either two frames or none")

    use_chain = args.frame_a and args.frame_b

    if args.tf_topic != 'tf' and use_chain:
        # This is not supported because it is not possible to change the tf topic for the ros tf listener
        parser.error('Monitoring a specific chain is not supported with a custom tf topic')

    while self.get_clock().now() == rospy.Time() and rclpy.ok():
        rospy.loginfo_throttle(10, 'tf_monitor waiting for time to be published')
        time.sleep(0.1)

    monitor = TFMonitor(use_chain, args.frame_a, args.frame_b)
    rospy.Subscriber(args.tf_topic, TFMessage, lambda msg: monitor.callback(msg, False))
    rospy.Subscriber(args.tf_topic + '_static', TFMessage, lambda msg: monitor.callback(msg, True))
    rospy.Timer(rospy.Duration(1/args.display_rate), monitor.display_frames)
    rclpy.spin(self)
