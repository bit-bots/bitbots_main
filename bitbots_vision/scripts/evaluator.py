#!/usr/bin/env python3
import rospy
from humanoid_league_msgs.msg import \
    ObstacleInImageArray, BallInImageArray, \
    GoalPostInImageArray
from geometry_msgs.msg import Point, PolygonStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math
import cv2
import yaml
import os
import sys
import signal
import time
import pickle


class Evaluation(object):
    def __init__(self):
        self.received_message = False  # boolean signaling whether a message of the type was received
        self.pixel_mask_rates = None
        self.duration = None


class ImageMeasurement(object):
    def __init__(self, image_data, eval_classes):
        self.evaluations = dict()
        self.image_data = image_data
        for eval_class in eval_classes:
            self.evaluations[eval_class] = Evaluation()

    def serialize(self):
        return {
            'evaluations': {
                'classes': {eval_class: vars(self.evaluations[eval_class]) for eval_class in self.evaluations.keys() if self.evaluations[eval_class].received_message},
                'max_latency': self.get_max_duration(),
            },
            'image_data': self.image_data,
        }

    def get_max_duration(self):
        # returns the maximal duration a measurement in the image took
        max_duration = 0
        for evaluation in self.evaluations.values():
            if evaluation.duration is not None and evaluation.duration > max_duration:
                max_duration = evaluation.duration
        return max_duration


class Evaluator(object):
    def __init__(self):
        self._set_sim_time_param()
        rospy.init_node("bitbots_vision_evaluator")

        self._set_sim_time_param()
        self._evaluated_classes = list()

        self._ball_sub = None
        if rospy.get_param("bitbots_vision_evaluator/listen_balls", False):
            rospy.loginfo('listening for balls in image...')
            self._evaluated_classes.append('ball')
            self._ball_sub = rospy.Subscriber(rospy.get_param("bitbots_vision_evaluator/balls_topic", "balls_in_image"),
                 BallInImageArray,
                 self._balls_callback,
                 queue_size=1,
                 tcp_nodelay=True)

        self._line_sub = None
        if rospy.get_param("bitbots_vision_evaluator/listen_lines", False):
            rospy.loginfo('listening for lines in image...')
            self._evaluated_classes.append('line')
            self._line_sub = rospy.Subscriber(rospy.get_param("bitbots_vision_evaluator/lines_topic", "line_mask_in_image"),
                 Image,
                 self._lines_callback,
                 queue_size=1,
                 tcp_nodelay=True)

        self._obstacle_sub = None
        if rospy.get_param("bitbots_vision_evaluator/listen_obstacles", False):
            rospy.loginfo('listening for obstacles in image...')
            self._evaluated_classes.append('robot_red')
            self._evaluated_classes.append('robot_blue')
            self._evaluated_classes.append('obstacle')
            self._obstacle_sub = rospy.Subscriber(rospy.get_param("bitbots_vision_evaluator/obstacles_topic", "obstacles_in_image"),
                 ObstacleInImageArray,
                 self._obstacles_callback,
                 queue_size=1,
                 tcp_nodelay=True)

        self._goalpost_sub = None
        if rospy.get_param("bitbots_vision_evaluator/listen_goalposts", False):
            rospy.loginfo('listening for goalposts in image...')
            self._evaluated_classes.append('goalpost')
            self._goalpost_sub = rospy.Subscriber(rospy.get_param("bitbots_vision_evaluator/goalpost_topic", "goal_posts_in_image"),
                 GoalPostInImageArray,
                 self._goalpost_callback,
                 queue_size=1,
                 tcp_nodelay=True)

        self._field_boundary_sub = None
        if rospy.get_param("bitbots_vision_evaluator/listen_field_boundary", False):
            rospy.loginfo('listening for field_boundary in image...')
            self._evaluated_classes.append('field edge')
            self._field_boundary_sub = rospy.Subscriber(rospy.get_param("bitbots_vision_evaluator/field_boundary_topic", "field_boundary_in_image"),
                 PolygonStamped,
                 self._field_boundary_callback,
                 queue_size=1,
                 tcp_nodelay=True)

        self._image_pub = rospy.Publisher('image_raw', Image, queue_size=1, latch=True)

        self._loop_images = rospy.get_param("bitbots_vision_evaluator/loop_images", False)
        self._image_path = rospy.get_param("bitbots_vision_evaluator/folder_path")
        self._line_thickness = rospy.get_param("bitbots_vision_evaluator/line_thickness")

        self._set_sim_time_param()

        self.bridge = CvBridge()

        self._measurements = dict()

        # Stop-Stuff
        self._stop = False  # stop flag to handle kills
        signal.signal(signal.SIGINT, self._kill_callback)
        signal.signal(signal.SIGTERM, self._kill_callback)

        # read label YAML file
        self._label_filename = rospy.get_param('bitbots_vision_evaluator/label_file_name')
        rospy.loginfo('Reading label-file \"{}\"...'.format(self._label_filename))
        self._images = self._read_labels(self._label_filename)
        rospy.loginfo('Done reading label-file.')
        self._classify_robots()  # to differ between robot colors based on blurred/concealed
        rospy.loginfo('Validating labels of {} images...'.format(len(self._images)))
        self._images = self._analyze_labels(self._images)
        rospy.loginfo('Labels of {} images are valid'.format(len(self._images)))
        rospy.loginfo('Filling field_boundary vectors...')
        for image in self._images:
            for label in image['annotations']:
                if label['type'] == 'field_boundary':
                    label['vector'] = self._fill_field_boundary_vector(label['vector'])
        rospy.loginfo('Done filling field_boundary vectors.')
        self._set_sim_time_param()

        self._current_image_counter = 0  # represents the current image index in the list defined by the label yaml file
        
        self._image_send_time = rospy.Time.now()

        self._image_count = len(self._images)  # number of images (important for loop stuff)
        self._image_shape = None  # tuple (height, width)

        self._lock = 0
        self._react_timer = rospy.Timer(rospy.Duration(0.2), self._react_callback)  # 2 second timer TODO: make this a variable
        self._send_image()
        rospy.spin()

    def _kill_callback(self, a, b):
        # the rest of the process is handled in the send_image method
        self._stop = True

    def _react_callback(self, event):
        while self._lock:
            #print('waiting...')
            time.sleep(.05)
        timeout = (rospy.Time.now() - self._image_send_time).to_sec() > 2.0
        if timeout: rospy.logwarn("Stoped waiting for responses. Maybe some detections are lost")
        if (self._recieved_all_messages_for_image(self._current_image_counter) or timeout) and not self._lock:
            self._current_image_counter += 1
            self._image_send_time = rospy.Time.now()
            self._send_image()

    def _get_send_image_name(self):
        return self._images[self._current_image_counter]['name']

    def _get_current_labels(self):
        return self._images[self._current_image_counter]['annotations']

    def _send_image(self, name=None):
        # handle stop at end of image list
        if self._current_image_counter >= self._image_count:  # iterated through all images
            rospy.loginfo('iterated through all images.')
            self._stop = True
        # executing any kind of stop
        if self._stop:
            rospy.loginfo('Stopping the evaluator.')
            # stop timer
            # self._resend_timer.shutdown()
            # write measurements to file
            self._write_measurements_to_file()
            # stop the spinner
            rospy.signal_shutdown('killed.')
            sys.exit(0)
            return  # this is just to show that nothing happens after this

        # handling unknown image name
        if name is None:
            name = self._get_send_image_name()

        print("Process image " + name)

        # reading image file
        imgpath = os.path.join(self._image_path, name)
        image = cv2.imread(imgpath)
        if image is None:
            rospy.logwarn('Could not open image {} at path {}'.format(name, self._image_path))
            return

        # setting image size in the first run
        if self._image_shape is None:
            self._image_shape = image.shape[:-1]

        # building and sending message
        rospy.loginfo('sending image {} of {} (starting by 0).'.format(self._current_image_counter, self._image_count))
        msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = str(self._current_image_counter)
        self._image_pub.publish(msg)

        # set up evaluation element in measurements list
        self._measurements[self._current_image_counter] = ImageMeasurement(self._images[self._current_image_counter], self._evaluated_classes)

    def _read_labels(self, filename):
        # reads the labels YAML file and returns a list of image names with their labels
        # this is set up to work with the "evaluation" export format of the Bit-Bots Team in the ImageTagger.
        filepath = os.path.join(self._image_path, filename)
        images = None
        if not os.path.isfile(filepath):
            rospy.logerr('File at path \'{}\' not found!'.format(filepath))
            return
        pickle_filepath = filepath + '.pickle'
        if not os.path.isfile(pickle_filepath):
            rospy.loginfo('Creating pickle file from yaml...')
            with open(filepath, 'r') as stream:
                try:
                    images = yaml.load(stream)['labels']
                except yaml.YAMLError as exc:
                    rospy.logerr(exc)
            with open(pickle_filepath, 'wb') as f:
                pickle.dump(images, f)
            rospy.loginfo('Done.')
        else:
            rospy.loginfo('Reading pickle file...')
            try:
                with open(pickle_filepath, 'rb') as f:
                    images = pickle.load(f)
            except pickle.PickleError as exc:
                rospy.logerr(exc)
        return images

    def _get_image_measurement(self, image_sequence):
        if image_sequence not in self._measurements.keys():
            rospy.logerr('got an unknown image with seq {}! Is there a ROS-bag running? Stop it please!'.format(image_sequence))
            return
        return self._measurements[image_sequence]

    def _balls_callback(self, msg):
        if 'ball' not in self._evaluated_classes:
            return
        self._lock += 1
        measurement = self._get_image_measurement(int(msg.header.frame_id)).evaluations['ball']
        # mark as received
        measurement.received_message = True
        # measure duration of processing
        measurement.duration = self._measure_timing(msg.header)
        # match masks
        measurement.pixel_mask_rates = self._match_masks(
            self._generate_circle_mask_from_vectors(
                Evaluator._extract_vectors_from_annotations(
                    self._images[int(msg.header.frame_id)]['annotations'],
                    typename='ball'
                )),
            self._generate_ball_mask_from_msg(msg))
        self._lock -= 1

    def _obstacles_callback(self, msg):
        class_colors = [('obstacle', 1), ('robot_red', 2), ('robot_blue', 3)]
        for class_color in class_colors:
            if class_color[0] not in self._evaluated_classes:
                continue
            self._lock += 1
            # getting the measurement which is set here
            measurement = self._get_image_measurement(int(msg.header.frame_id)).evaluations[class_color[0]]
            # mark as received
            measurement.received_message = True
            # measure duration of processing
            measurement.duration = self._measure_timing(msg.header)
            # match masks
            measurement.pixel_mask_rates = self._match_masks(
                self._generate_rectangle_mask_from_vectors(
                    Evaluator._extract_vectors_from_annotations(
                        self._images[int(msg.header.frame_id)]['annotations'],
                        typename=class_color[0]
                    )),
                self._generate_obstacle_mask_from_msg(msg, color=class_color[1]))

            self._lock -= 1

    def _goalpost_callback(self, msg):
        if 'goalpost' not in self._evaluated_classes:
            return
        self._lock += 1
        # getting the measurement which is set here
        measurement = self._get_image_measurement(int(msg.header.frame_id)).evaluations['goalpost']
        # mark as received
        measurement.received_message = True
        # measure duration of processing
        measurement.duration = self._measure_timing(msg.header)
        # match masks
        measurement.pixel_mask_rates = self._match_masks(
            self._generate_rectangle_mask_from_vectors(
                Evaluator._extract_vectors_from_annotations(
                    self._images[int(msg.header.frame_id)]['annotations'],
                    typename='goalpost'
                )),
            self._generate_goal_post_mask_from_msg(msg))
        self._lock -= 1

    def _lines_callback(self, msg):
        if 'line' not in self._evaluated_classes:
            return
        self._lock += 1
        # getting the measurement which is set here
        measurement = self._get_image_measurement(int(msg.header.frame_id)).evaluations['line']
        # mark as received
        measurement.received_message = True
        # measure duration of processing
        measurement.duration = self._measure_timing(msg.header)

        # generating and matching masks
        measurement.pixel_mask_rates = self._match_masks(
            self._generate_line_mask_from_vectors(
                Evaluator._extract_vectors_from_annotations(
                    self._images[int(msg.header.frame_id)]['annotations'],
                    typename='line'
                )),
            self.bridge.imgmsg_to_cv2(msg, '8UC1'))
        self._lock -= 1

    def _field_boundary_callback(self, msg):
        if 'field edge' not in self._evaluated_classes:
            return
        self._lock += 1
        # getting the measurement which is set here
        measurement = self._get_image_measurement(int(msg.header.frame_id)).evaluations['field edge']
        # mark as received
        measurement.received_message = True
        # measure duration of processing
        measurement.duration = self._measure_timing(msg.header)
        # generating and matching masks
        measurement.pixel_mask_rates = self._match_masks(
            self._generate_field_boundary_mask_from_vector(
                Evaluator._extract_vectors_from_annotations(
                    self._images[int(msg.header.frame_id)]['annotations'],
                    typename='field edge'
                )),
            self._generate_field_boundary_mask_from_msg(msg))
        self._lock -= 1

    def _measure_timing(self, header):
        # calculating the time the processing took
        return (rospy.get_rostime() - header.stamp).to_sec()

    def _generate_polygon_mask_from_vectors(self, vectors):
        mask = np.zeros(self._image_shape, dtype=np.uint8)

        for vector in vectors:
            if not vector:
                continue
            cv2.fillConvexPoly(mask, np.array(vector), 1.0)
        return mask

    def _generate_field_boundary_mask_from_vector(self, vector):
        mask = np.zeros(self._image_shape, dtype=np.uint8)
        vector = [list(pts) for pts in vector][0] #TODO wtf

        vector.append([self._image_shape[1] - 1, self._image_shape[0] - 0])
        vector.append([0, self._image_shape[0] - 1])  # extending the points to fill the space below the field_boundary
        points = np.array(vector, dtype=np.int32)
        points = points.reshape((1, -1, 2))
        cv2.fillPoly(mask, points, 1.0)
        return mask

    def _generate_rectangle_mask_from_vectors(self, vectors):
        mask = np.zeros(self._image_shape, dtype=np.uint8)
        for vector in vectors:
            if not vector:
                continue
            cv2.rectangle(mask, tuple(vector[0]), tuple(vector[1]), 1.0, thickness=-1)
        return mask

    def _generate_circle_mask_from_vectors(self, vectors):
        mask = np.zeros(self._image_shape, dtype=np.uint8)

        for vector in vectors:
            if not vector:
                continue
            center = (
                int(math.floor(vector[0][0] + (vector[1][0] - vector[0][0]) // 2)),
                int(math.floor(vector[0][1] + (vector[1][1] - vector[0][1]) // 2)))
            radius = int(math.floor((vector[1][0] - vector[0][0]) / 2 + (vector[1][1] - vector[0][1]) / 2) / 2)
            cv2.circle(mask, center, radius, 1.0, thickness=-1)
        return mask

    def _generate_line_mask_from_vectors(self, vectors):
        mask = np.zeros(self._image_shape, dtype=np.uint8)
        for vector in vectors:
            if not vector:
                continue
            cv2.line(mask, tuple(vector[0]), tuple(vector[1]), 1.0, thickness=self._line_thickness)
        return mask

    def _generate_ball_mask_from_msg(self, msg):
        mask = np.zeros(self._image_shape, dtype=np.uint8)
        for ball in msg.candidates:
            cv2.circle(mask, (int(round(ball.center.x)), int(round(ball.center.y))), int(round(ball.diameter/2)), 1.0, thickness=-1)
        return mask

    def _generate_field_boundary_mask_from_msg(self, msg):
        mask = np.zeros(self._image_shape, dtype=np.uint8)
        points = [[int(point.x), int(point.y)] for point in msg.polygon.points]
        points = points + [(self._image_shape[1] - 1, self._image_shape[0] - 0), (0, self._image_shape[0] - 1)]  # extending the points to fill the space below the field_boundary
        points = np.array(points, dtype=np.int32)
        points = points.reshape((1, -1, 2))
        cv2.fillPoly(mask, points, 1.0)
        return mask

    def _generate_obstacle_mask_from_msg(self, msg, obstacle_type=-1):
        vectors = list()
        for obstacle in msg.obstacles:
            if obstacle_type == -1 or obstacle.type == obstacle_type:
                vector = ((int(obstacle.top_left.x), int(obstacle.top_left.y)), (int(obstacle.top_left.x) + obstacle.width, int(obstacle.top_left.y) + obstacle.height))
                vectors.append(vector)
        return self._generate_rectangle_mask_from_vectors(vectors)

    def _generate_goal_post_mask_from_msg(self, msg):
        vectors = list()
        for post in msg.posts:
                vector = ((int(post.foot_point.x - post.width // 2), int(post.top_point.y)), (int(post.foot_point.x + post.width // 2), int(post.foot_point.y)))
                vectors.append(vector)
        return self._generate_rectangle_mask_from_vectors(vectors)

    @staticmethod
    def _match_masks(label_mask, detected_mask):
        # matches the masks onto each other to determine multiple measurements.
        label_mask = label_mask.astype(bool)
        detected_mask = detected_mask.astype(bool)
        rates = dict()
        rates['tp'] = float(np.mean(np.bitwise_and(label_mask, detected_mask)))
        rates['tn'] = float(np.mean(np.bitwise_not(np.bitwise_or(label_mask, detected_mask))))
        rates['fp'] = float(np.mean(np.bitwise_and(detected_mask, np.bitwise_not(label_mask))))
        rates['fn'] = float(np.mean(np.bitwise_and(np.bitwise_not(detected_mask), label_mask)))
        # https://en.wikipedia.org/wiki/Jaccard_index
        numerator = float(np.sum(np.bitwise_and(label_mask, detected_mask)))
        denominator = float(np.sum(np.bitwise_or(label_mask, detected_mask)))
        rates['iou'] = numerator / denominator if denominator > 0 else 1
        rates['lp'] = float(np.mean(label_mask))
        rates['ln'] = 1 - rates['lp']  # because all the other pixels have to be negative
        rates['dp'] = float(np.mean(detected_mask))
        rates['dn'] = 1 - rates['dp']  # because all the other pixels have to be negative
        return rates

    def _recieved_all_messages_for_image(self, image_seq):
        while self._lock:
            time.sleep(.05)
        measurement = self._measurements[image_seq]
        for eval_class in self._evaluated_classes:
            if not measurement.evaluations[eval_class].received_message:
                return False
        return True

    @staticmethod
    def _filter_type(annotations, typename):
        # returns the annotations of type TYPE
        return [annotation for annotation in annotations if annotation['type'] == typename]

    @staticmethod
    def _extract_vectors_from_annotations(annotations, typename=None):
        # returns the vectors of annotations of type TYPE
        if typename:
            return [annotation['vector'] for annotation in annotations if annotation['type'] == typename and annotation['in']]
        return [annotation['vector'] for annotation in annotations]

    def _analyze_labels(self, images):
        # analyzes the label file for stuff

        not_in_image_count = dict()
        for eval_class in self._evaluated_classes:
            not_in_image_count[eval_class] = 0

        filtered_images = list()
        for image in images:
            add_image = True  # whether the image is used in the evaluation or not
            in_image = dict()
            found_label = dict()
            for eval_class in self._evaluated_classes:
                found_label[eval_class] = False
                in_image[eval_class] = None

            for annotation in image['annotations']:
                # determine whether the class is evaluated or not
                if annotation['type'] not in self._evaluated_classes:
                    continue  # ignore other classes annotations
                # annotation type is in evaluated classes
                found_label[annotation['type']] = True
                # handle multi-class... classes.
                if annotation['type'] == 'obstacle' or 'robot' in annotation['type']:
                    found_label['obstacle'] = True
                    found_label['robot_red'] = True
                    found_label['robot_blue'] = True
                if in_image[annotation['type']] == True:
                    if not annotation['in']:  # contradiction!
                        rospy.logwarn('Found contradicting labels of type {} in image \"{}\"! The image will be removed!'.format(annotation['type'], image['name']))
                        add_image = False
                        break
                elif in_image[annotation['type']]  == False:
                    if annotation['in']:  # contradiction!
                        rospy.logwarn('Found contradicting labels of type {} in image \"{}\"! The image will be removed!'.format(annotation['type'], image['name']))
                        add_image = False
                        break
                else:  # it is None and therefor not set yet
                    in_image[annotation['type']] = annotation['in']

            # increase the counters when no label was found for a type
            for eval_class in self._evaluated_classes:
                if not found_label[eval_class]:
                    not_in_image_count[eval_class] += 1
                    rospy.logwarn('Image without label found')
                    add_image = False  # remove images when not all labels are defined somehow.
            if add_image:
                filtered_images.append(image)
        return filtered_images

    def _fill_field_boundary_vector(self, vector):
        # extend the borders of the field_boundary to the borders of the image
        if len(vector) < 2:
            rospy.logwarn('found field_boundary label shorter than 2 elements')
            return
        begin = vector[0]
        new_begin = vector[0]
        end = vector[-1]
        new_end = vector[-1]
        if -10 < begin[0] < 10:
            new_begin = (0, begin[1])
        if self._image_shape[1]-10 < end[0] < self._image_shape[1]+10:
            new_end = (self._image_shape[1] - 1, begin[1])
        vector[0] = new_begin
        vector[-1] = new_end
        return vector

    def _write_measurements_to_file(self):
        serialized_measurements = [measurement.serialize() for measurement in self._measurements.values()]
        rospy.loginfo('Writing {} measurements to file...'.format(len(serialized_measurements)))
        filepath = 'data.yaml'  # TODO: this properly
        with open(filepath, 'w') as outfile:
            yaml.dump(serialized_measurements, outfile)  # , default_flow_style=False)
        rospy.loginfo('Done writing to file.')
        rospy.loginfo('Mean IoUs (by class):')
        evaluations = [k['evaluations'] for k in serialized_measurements]
        for iou_class in self._evaluated_classes:
            class_evals = [ev[iou_class] for ev in evaluations if ev and iou_class in ev.keys()]
            ious = [measurement['pixel_mask_rates']['iou'] for measurement in class_evals if measurement]
            rospy.loginfo('{}: {}'.format(iou_class, sum(ious) / float(len(ious))))

    def _classify_robots(self):
        # special handling of robot classes
        # this is an ugly workaround!!!
        for image in self._images:
            for annotation in image['annotations']:
                if annotation['type'] == 'robot':
                    if annotation['concealed']:
                        annotation['type'] = 'robot_red'
                    elif annotation['blurred']:
                        annotation['type'] = 'robot_blue'
                    else:
                        annotation['type'] = 'obstacle'

    def _set_sim_time_param(self):
        return
        if rospy.get_param('/use_sim_time'):
            print('setting /use_sim_time to false...')
            rospy.set_param('/use_sim_time', False)

if __name__ == "__main__":
    Evaluator()
    rospy.spin()
