# bitbots_apriltag_localization

Localizes the robot from AprilTags (and rigid AprilTag bundles) placed at
known poses in the environment, e.g. on the walls of the lab. Useful as a
drift-free pose source for experiments, such as evaluating path following.

## How it works

1. A tag map YAML (see `config/tag_map.yaml`) defines which tags are placed
   where in the map frame, including the size of every tag. Tags can be
   standalone or grouped into rigid *bundles* (e.g. several tags printed on
   one board). Internally a standalone tag is just a single-tag bundle.
2. Every camera image is searched for the mapped tags with `pupil_apriltags`.
   All detected member tags of a bundle contribute their four corners to one
   joint `cv2.solvePnP`, so a partially visible bundle still yields a full 6D
   pose and extra visible tags improve accuracy (ported from the gripper
   tracking of ma_flo).
3. Each detected bundle gives a camera pose in the map frame via the bundle's
   known map pose. Multiple detections are fused by an average weighted with
   the number of detected corners.
4. The robot pose (`frames.base` in `frames.map`) is derived through tf from
   the camera pose and published as:
   * `apriltag_pose` (`geometry_msgs/PoseWithCovarianceStamped`), published
     per detection, and
   * the `map -> odom` transform (if `publish_tf` is set), re-broadcast
     continuously at `tf_publish_rate` based on the most recent detection, so
     tf consumers get a drift-free pose that stays available and smooth while
     no tag is visible.

## Conventions

* **Tag frame**: looking at the printed tag, x points right, y up and z out
  of the tag face towards the observer. A tag hanging upright on a wall has
  `rpy_deg: [90, 0, yaw]` where yaw turns the face into the room
  (`[90, 0, -90]` faces -x, `[90, 0, 90]` faces +x, `[90, 0, 0]` faces -y,
  `[90, 0, 180]` faces +y).
* **Tag size**: edge length of the (outer) black square in meters, without
  the white margin. Print tags from the canonical
  [AprilRobotics tag images](https://github.com/AprilRobotics/apriltag-imgs)
  (family tag36h11 by default) and keep a white margin around the black
  square.
* The corner order convention was verified against the canonical tag
  renderings with synthetic images, see `test/test_detector.py`.

## Usage

```bash
ros2 launch bitbots_apriltag_localization apriltag_localization.launch
```

Node parameters live in `config/apriltag_localization.yaml` (camera topics,
frames, detector tuning, reprojection error gate, covariance, debug outputs).
The `tag_map` parameter selects the map file; relative paths are resolved in
this package's `config/` directory.

To verify a freshly measured tag map, enable `debug.publish_detection_tf`
(each detected bundle appears as tf frame `apriltag_<name>` under the camera
frame) and `debug.publish_debug_image` (detections drawn into the image on
`apriltag_debug_image`, green = used, red = dropped by the reprojection error
gate).
