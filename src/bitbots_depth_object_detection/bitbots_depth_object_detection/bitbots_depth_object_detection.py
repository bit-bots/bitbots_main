#!/usr/bin/env python3
"""
Depth image -> obstacle occupancy grid (ROS 2).
 
Pipeline
  1. Deproject depth image to 3D points (camera optical frame).
  2. Transform points into the robot base frame via TF.
  3. Estimate the ground plane (RANSAC on near-ground points, or fixed z=0).
  4. Split points into ground / obstacle by height above the plane.
  5. Project onto the plane and accumulate per-cell *observed surface area*
     (each point is weighted by the area its pixel covers at that depth, which
     compensates for the density drop with distance).
  6. Gaussian-smooth and threshold -> OccupancyGrid in the base frame
     (100 = occupied, 0 = observed ground/free, -1 = unknown).
 
Run:
  python3 depth_obstacle_grid_node.py --ros-args \
      --params-file depth_obstacle_grid.yaml -p use_sim_time:=true
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from scipy.ndimage import gaussian_filter
 
 
def quat_to_matrix(x, y, z, w):
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])
 
 
class DepthObstacleGrid(Node):
    def __init__(self):
        super().__init__('depth_obstacle_grid')
 
        d = self.declare_parameter
        d('topics.depth_image', '/zed/zed_node/depth/depth_registered')
        d('topics.camera_info', '/zed/zed_node/depth/camera_info')
        d('topics.occupancy_grid', '/obstacle_grid')
        d('frames.base_frame', 'base_footprint')
        d('tf_timeout', 0.2)
 
        d('depth.min_range', 0.0)
        d('depth.max_range', 8.0)
        d('depth.pixel_stride', 4)
 
        d('ground.mode', 'ransac')  # 'ransac' or 'tf' (plane z=0 in base frame)
        d('ground.candidate_band', 0.20)
        d('ground.ransac_iterations', 100)
        d('ground.ransac_inlier_threshold', 0.03)
        d('ground.ransac_min_inliers', 300)
        d('ground.ransac_max_points', 5000)
        d('ground.max_tilt_deg', 10.0)
        d('ground.tolerance', 0.03)
 
        d('obstacle.min_height', 0.03)
        d('obstacle.max_height', 1.5)
        d('obstacle.min_area', 0.0002)
        d('ground.min_area', 0.02)
 
        d('grid.resolution', 0.05)
        d('grid.x_min', -1.0)
        d('grid.x_max', 8.0)
        d('grid.y_min', -4.0)
        d('grid.y_max', 4.0)
        d('grid.smoothing_sigma', 0.025)
 
        d('debug.publish_cloud', True)
        d('debug.cloud_topic', '/obstacle_grid_debug/points')
        d('debug.project_to_plane', False)  # True: show points as projected onto ground plane
 
        g = lambda n: self.get_parameter(n).value
        self.g = g
        self.base_frame = g('frames.base_frame')
        self.res = g('grid.resolution')
        self.x_min, self.y_min = g('grid.x_min'), g('grid.y_min')
        self.nx = int(round((g('grid.x_max') - self.x_min) / self.res))
        self.ny = int(round((g('grid.y_max') - self.y_min) / self.res))
        self.rng = np.random.default_rng(0)
 
        self.cam_info = None
        self._uv_cache = {}
 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
 
        self.create_subscription(CameraInfo, g('topics.camera_info'),
                                 self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, g('topics.depth_image'),
                                 self.depth_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(OccupancyGrid,
                                         g('topics.occupancy_grid'), 1)
        self.cloud_pub = None
        if g('debug.publish_cloud'):
            self.cloud_pub = self.create_publisher(
                PointCloud2, g('debug.cloud_topic'), 1)
        self.get_logger().info(
            f'Grid {self.nx}x{self.ny} cells @ {self.res} m in "{self.base_frame}"')
 
    # ------------------------------------------------------------------ input
    def info_cb(self, msg):
        self.cam_info = msg
 
    @staticmethod
    def decode_depth(msg):
        if msg.encoding == '32FC1':
            arr = np.frombuffer(msg.data, np.float32).reshape(
                msg.height, msg.step // 4)[:, :msg.width]
            return arr
        if msg.encoding == '16UC1':  # millimetres
            arr = np.frombuffer(msg.data, np.uint16).reshape(
                msg.height, msg.step // 2)[:, :msg.width]
            return arr.astype(np.float32) * 1e-3
        return None
 
    # ------------------------------------------------------------- processing
    def deproject(self, depth, info):
        """Return (points_cam Nx3, pixel_area_weights N)."""
        g = self.g
        s = int(g('depth.pixel_stride'))
        h, w = depth.shape
        sx, sy = w / info.width, h / info.height  # handle resolution mismatch
        fx, fy = info.k[0] * sx, info.k[4] * sy
        cx, cy = info.k[2] * sx, info.k[5] * sy
 
        key = (h, w, s)
        if key not in self._uv_cache:
            vs, us = np.mgrid[0:h:s, 0:w:s]
            self._uv_cache[key] = (us.ravel().astype(np.float32),
                                   vs.ravel().astype(np.float32))
        u, v = self._uv_cache[key]
        z = depth[::s, ::s].ravel()
 
        ok = np.isfinite(z) & (z > g('depth.min_range')) & (z < g('depth.max_range'))
        u, v, z = u[ok], v[ok], z[ok]
        pts = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)
        weights = (z / fx) * (z / fy) * (s * s)  # m^2 covered by each sample
        return pts, weights
 
    def fit_ground(self, pts):
        """Return plane (n, d) with n.p + d = 0, n pointing up."""
        g = self.g
        fallback = (np.array([0.0, 0.0, 1.0]), 0.0)
        if g('ground.mode') != 'ransac':
            return fallback
 
        cand = pts[np.abs(pts[:, 2]) < g('ground.candidate_band')]
        max_pts = int(g('ground.ransac_max_points'))
        if len(cand) > max_pts:
            cand = cand[self.rng.choice(len(cand), max_pts, replace=False)]
        if len(cand) < 3 * 10:
            return fallback
 
        iters = int(g('ground.ransac_iterations'))
        thr = g('ground.ransac_inlier_threshold')
        cos_tilt = np.cos(np.deg2rad(g('ground.max_tilt_deg')))
 
        idx = self.rng.integers(0, len(cand), (iters, 3))
        a, b, c = cand[idx[:, 0]], cand[idx[:, 1]], cand[idx[:, 2]]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n, axis=1)
        good = norm > 1e-6
        n[good] /= norm[good, None]
        n[n[:, 2] < 0] *= -1
        good &= n[:, 2] > cos_tilt  # near-horizontal planes only
        if not good.any():
            return fallback
        dd = -np.einsum('ij,ij->i', n, a)
 
        dist = np.abs(cand @ n.T + dd)  # (N, iters)
        counts = (dist < thr).sum(axis=0)
        counts[~good] = -1
        best = int(np.argmax(counts))
        if counts[best] < g('ground.ransac_min_inliers'):
            return fallback
 
        inl = cand[dist[:, best] < thr]
        centroid = inl.mean(axis=0)
        _, _, vt = np.linalg.svd(inl - centroid, full_matrices=False)
        nb = vt[-1]
        if nb[2] < 0:
            nb = -nb
        if nb[2] < cos_tilt:
            return fallback
        return nb, -float(nb @ centroid)
 
    def accumulate(self, xy, w):
        ix = np.floor((xy[:, 0] - self.x_min) / self.res).astype(np.int64)
        iy = np.floor((xy[:, 1] - self.y_min) / self.res).astype(np.int64)
        inside = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        flat = iy[inside] * self.nx + ix[inside]
        acc = np.bincount(flat, weights=w[inside], minlength=self.nx * self.ny)
        return acc.reshape(self.ny, self.nx)
 
    def publish_debug_cloud(self, stamp, pts, h, n, is_ground, is_obst):
        """PointCloud2 in base frame: green=ground, red=obstacle, gray=ignored.
        Extra field 'height' = height above fitted ground plane [m]."""
        if self.cloud_pub is None or self.cloud_pub.get_subscription_count() == 0:
            return
        out = pts - h[:, None] * n[None, :] if self.g('debug.project_to_plane') else pts
 
        rgb = np.full(len(pts), 0x808080, dtype=np.uint32)
        rgb[is_ground] = 0x00FF00
        rgb[is_obst] = 0xFF0000
 
        cloud = np.empty(len(pts), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                          ('rgb', '<f4'), ('height', '<f4')])
        cloud['x'], cloud['y'], cloud['z'] = out[:, 0], out[:, 1], out[:, 2]
        cloud['rgb'] = rgb.view(np.float32)
        cloud['height'] = h
 
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='height', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 20
        msg.row_step = 20 * len(pts)
        msg.is_dense = True
        msg.data = cloud.tobytes()
        self.cloud_pub.publish(msg)
 
    def depth_cb(self, msg):
        if self.cam_info is None:
            self.get_logger().warn('No camera_info yet', throttle_duration_sec=5.0)
            return
        depth = self.decode_depth(msg)
        if depth is None:
            self.get_logger().error(f'Unsupported depth encoding {msg.encoding}',
                                    throttle_duration_sec=5.0)
            return
 
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.g('tf_timeout')))
        except Exception as e:  # tf2 exceptions
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return
        q = tf.transform.rotation
        t = tf.transform.translation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])
 
        pts_cam, w = self.deproject(depth, self.cam_info)
        if len(pts_cam) == 0:
            return
        pts = pts_cam @ R.T + T
 
        n, dist0 = self.fit_ground(pts)
        h = pts @ n + dist0  # signed height above ground plane
 
        # project onto the ground plane (along its normal), keep x/y
        xy = pts[:, :2] - h[:, None] * n[None, :2]
 
        tol = self.g('ground.tolerance')
        is_ground = np.abs(h) < tol
        is_obst = (h > self.g('obstacle.min_height')) & \
                  (h < self.g('obstacle.max_height'))
 
        self.publish_debug_cloud(msg.header.stamp, pts, h, n, is_ground, is_obst)
 
        obst_area = self.accumulate(xy[is_obst], w[is_obst])
        gnd_area = self.accumulate(xy[is_ground], w[is_ground])
 
        sigma = self.g('grid.smoothing_sigma') / self.res
        if sigma > 0:
            obst_area = gaussian_filter(obst_area, sigma, mode='constant')
            gnd_area = gaussian_filter(gnd_area, sigma, mode='constant')
 
        grid = np.full((self.ny, self.nx), -1, dtype=np.int8)
        grid[gnd_area >= self.g('ground.min_area')] = 0
        grid[obst_area >= self.g('obstacle.min_area')] = 100
 
        og = OccupancyGrid()
        og.header.stamp = msg.header.stamp
        og.header.frame_id = self.base_frame
        og.info.resolution = float(self.res)
        og.info.width = self.nx
        og.info.height = self.ny
        og.info.origin.position.x = float(self.x_min)
        og.info.origin.position.y = float(self.y_min)
        og.info.origin.orientation.w = 1.0
        og.data = grid.ravel().tolist()
        self.pub.publish(og)
 
        self.get_logger().debug(
            f'plane n={np.round(n, 3)} d={dist0:.3f}, obstacle pts={int(is_obst.sum())}')
 
 
def main():
    rclpy.init()
    node = DepthObstacleGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
 
 
if __name__ == '__main__':
    main()
