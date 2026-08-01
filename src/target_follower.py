"""
Part 4: Visual Target Following Controller
===========================================
基于视觉标签（ArUco）的双机器人目标跟踪控制器。

角色分配：
- agent0 = 领航者（leader），背后贴有 ArUco 标签（默认 ID=1），由键盘手动控制
- agent1 = 跟随者（follower），搭载相机，自动检测标签并跟踪领航者

核心流程：
1. 订阅 follower 相机图像和 CameraInfo（SensorDataQoS）
2. ArUco 检测得到标签角点
3. 使用相机内参 + solvePnP 估计标签相对相机的平移向量 tvec
4. 从 tvec 提取深度、左右偏移和偏角
5. 双回路 PD 控制输出 /agent1/cmd_vel
6. 标签丢失时按“短暂遮挡 / 自旋搜索 / 安全停车”降级

设计要点：
- 默认 aruco_dict=auto，会优先尝试 DICT_4X4_250，再回退到常见字典。
- 不依赖 cv2.aruco.estimatePoseSingleMarkers，兼容 OpenCV 4.5、4.7+、4.13+。
- 相机图像若 Webots 默认不发布，可配合 camera_plugin.py 发布 /agent1/camera/image_raw。
"""

import math
import os

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


ARUCO_CANDIDATES = [
    'DICT_4X4_250',
    'DICT_4X4_50',
    'DICT_4X4_100',
    'DICT_5X5_250',
    'DICT_6X6_250',
    'DICT_6X6_50',
    'DICT_6X6_100',
    'DICT_ARUCO_ORIGINAL',
]


class PDController:
    """PD 控制器，带输出限幅。"""

    def __init__(self, kp=0.8, kd=0.05, out_min=-1.0, out_max=1.0):
        self.kp = kp
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._prev_error = 0.0
        self._first_run = True

    def update(self, error, dt):
        p = self.kp * error
        if self._first_run:
            d = 0.0
            self._first_run = False
        else:
            d = self.kd * (error - self._prev_error) / dt if dt > 1e-6 else 0.0
        self._prev_error = error
        return clamp(p + d, self.out_min, self.out_max)

    def reset(self):
        self._prev_error = 0.0
        self._first_run = True


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


class TargetFollower(Node):
    def __init__(self):
        super().__init__('target_follower')

        self.declare_parameter('robot_name', 'agent1')
        self.declare_parameter('marker_id', 1)
        self.declare_parameter('marker_size', 0.1)
        self.declare_parameter('aruco_dict', 'auto')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('camera_info_topic', '')
        self.declare_parameter('cmd_topic', '')
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('lateral_tolerance', 0.03)
        self.declare_parameter('kp_dist', 0.8)
        self.declare_parameter('kd_dist', 0.2)
        self.declare_parameter('kp_yaw', 1.5)
        self.declare_parameter('kd_yaw', 0.1)
        self.declare_parameter('kp_lateral', 0.5)
        self.declare_parameter('use_lateral', False)
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('w_max', 2.0)
        self.declare_parameter('loss_timeout', 0.3)
        self.declare_parameter('search_timeout', 2.0)
        self.declare_parameter('image_timeout', 1.5)
        self.declare_parameter('search_angular_speed', 0.3)
        self.declare_parameter('search_when_never_seen', False)
        self.declare_parameter('occluded_cmd_decay', 0.5)
        self.declare_parameter('filter_alpha', 0.7)
        self.declare_parameter('debug', False)
        self.declare_parameter('log_csv', '')

        robot_name = self.get_parameter('robot_name').value
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.target_dist = float(self.get_parameter('target_distance').value)
        self.dist_tol = float(self.get_parameter('distance_tolerance').value)
        self.angle_tol = float(self.get_parameter('angle_tolerance').value)
        self.lateral_tol = float(self.get_parameter('lateral_tolerance').value)
        self.kp_lateral = float(self.get_parameter('kp_lateral').value)
        self.use_lateral = bool(self.get_parameter('use_lateral').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.loss_timeout = float(self.get_parameter('loss_timeout').value)
        self.search_timeout = float(self.get_parameter('search_timeout').value)
        self.image_timeout = float(self.get_parameter('image_timeout').value)
        self.search_w = float(self.get_parameter('search_angular_speed').value)
        self.search_when_never_seen = bool(self.get_parameter('search_when_never_seen').value)
        self.occluded_cmd_decay = float(self.get_parameter('occluded_cmd_decay').value)
        self.filter_alpha = float(self.get_parameter('filter_alpha').value)
        self.debug = bool(self.get_parameter('debug').value)
        self.debug_window_enabled = self.debug
        if self.debug and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            self.debug_window_enabled = False
            self.get_logger().warn('debug:=true but no GUI display is available; cv2.imshow will be skipped.')
        csv_path = self.get_parameter('log_csv').value

        image_topic = self.get_parameter('image_topic').value or f'/{robot_name}/camera/image_raw'
        info_topic = self.get_parameter('camera_info_topic').value or f'/{robot_name}/camera/camera_info'
        cmd_topic = self.get_parameter('cmd_topic').value or f'/{robot_name}/cmd_vel'

        requested_dict = self.get_parameter('aruco_dict').value
        self.detectors = {}
        self.dict_order = self._build_dict_order(requested_dict)
        for dict_name in self.dict_order:
            backend = self._create_aruco_backend(dict_name)
            if backend is not None:
                self.detectors[dict_name] = backend
        if not self.detectors:
            raise RuntimeError('当前 OpenCV 没有可用的 ArUco 字典。请安装 opencv-contrib-python。')
        self.active_dict_name = None

        self.get_logger().info(
            f'ArUco 字典: {requested_dict}; 候选={list(self.detectors.keys())}; '
            f'跟踪标签 ID={self.marker_id}, 尺寸={self.marker_size}m')

        self.camera_matrix = None
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        self.bridge = CvBridge()

        self.pd_dist = PDController(
            kp=float(self.get_parameter('kp_dist').value),
            kd=float(self.get_parameter('kd_dist').value),
            out_min=-self.v_max,
            out_max=self.v_max)
        self.pd_yaw = PDController(
            kp=float(self.get_parameter('kp_yaw').value),
            kd=float(self.get_parameter('kd_yaw').value),
            out_min=-self.w_max,
            out_max=self.w_max)

        self.last_depth = None
        self.last_yaw = None
        self.last_lateral = None
        self.filtered_depth = None
        self.filtered_yaw = None
        self.filtered_lateral = None
        self.last_seen_time = None
        self.last_image_time = None
        self.last_control_time = None
        self.frame_count = 0
        self.detect_count = 0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.state = 'INIT'

        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self.get_logger().info(f'订阅图像: {image_topic}')
        self.get_logger().info(f'订阅相机内参: {info_topic}')
        self.get_logger().info(f'发布速度: {cmd_topic}')
        self.get_logger().info(
            f'目标距离: {self.target_dist}m | 距离死区: {self.dist_tol}m | '
            f'角度死区: {math.degrees(self.angle_tol):.1f}deg | 横向修正: {self.use_lateral}')
        self.get_logger().info(
            f'PD 距离: kp={self.pd_dist.kp} kd={self.pd_dist.kd} | '
            f'PD 偏角: kp={self.pd_yaw.kp} kd={self.pd_yaw.kd}')

        self.csv_file = None
        if csv_path:
            self.csv_file = open(csv_path, 'w', encoding='utf-8')
            self.csv_file.write(
                't,detected,depth,lateral,yaw,dist_err,yaw_err,vx,vy,wz,state,dict\n')
            self.log_start = self.get_clock().now()
            self.get_logger().info(f'数据记录到: {csv_path}')

        self.safety_timer = self.create_timer(0.1, self.safety_check)

    def _build_dict_order(self, requested):
        if requested and str(requested).lower() != 'auto':
            order = [requested]
            order.extend(name for name in ARUCO_CANDIDATES if name != requested)
            return order
        return list(ARUCO_CANDIDATES)

    def _get_aruco_dict_id(self, name):
        dict_map = {
            'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_100': aruco.DICT_4X4_100,
            'DICT_4X4_250': aruco.DICT_4X4_250,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_5X5_100': aruco.DICT_5X5_100,
            'DICT_5X5_250': aruco.DICT_5X5_250,
            'DICT_6X6_50': aruco.DICT_6X6_50,
            'DICT_6X6_100': aruco.DICT_6X6_100,
            'DICT_6X6_250': aruco.DICT_6X6_250,
            'DICT_7X7_50': aruco.DICT_7X7_50,
            'DICT_7X7_100': aruco.DICT_7X7_100,
            'DICT_7X7_250': aruco.DICT_7X7_250,
            'DICT_ARUCO_ORIGINAL': aruco.DICT_ARUCO_ORIGINAL,
        }
        for name_april in ['DICT_APRILTAG_16h5', 'DICT_APRILTAG_25h9', 'DICT_APRILTAG_36h11']:
            if hasattr(aruco, name_april):
                dict_map[name_april] = getattr(aruco, name_april)
        return dict_map.get(name)

    def _create_aruco_backend(self, dict_name):
        dict_id = self._get_aruco_dict_id(dict_name)
        if dict_id is None:
            self.get_logger().warn(f'当前 OpenCV 不支持字典 {dict_name}，已跳过。')
            return None

        if hasattr(aruco, 'getPredefinedDictionary'):
            dictionary = aruco.getPredefinedDictionary(dict_id)
        else:
            dictionary = aruco.Dictionary_get(dict_id)

        if hasattr(aruco, 'DetectorParameters'):
            params = aruco.DetectorParameters()
        else:
            params = aruco.DetectorParameters_create()

        detector = aruco.ArucoDetector(dictionary, params) if hasattr(aruco, 'ArucoDetector') else None
        return dictionary, params, detector

    def _detect_markers(self, gray, dict_name):
        dictionary, params, detector = self.detectors[dict_name]
        if detector is not None:
            return detector.detectMarkers(gray)
        return aruco.detectMarkers(gray, dictionary, parameters=params)

    def _detect_target(self, gray):
        ordered_names = []
        if self.active_dict_name:
            ordered_names.append(self.active_dict_name)
        ordered_names.extend(name for name in self.detectors if name not in ordered_names)

        last_corners, last_ids = None, None
        for dict_name in ordered_names:
            corners, ids, _ = self._detect_markers(gray, dict_name)
            last_corners, last_ids = corners, ids
            if ids is None:
                continue

            for idx, marker_id in enumerate(ids.flatten()):
                if int(marker_id) != self.marker_id:
                    continue
                ok, rvec, tvec = self._estimate_marker_pose(corners[idx])
                if ok:
                    if self.active_dict_name != dict_name:
                        self.get_logger().info(f'锁定 ArUco 字典: {dict_name}')
                    self.active_dict_name = dict_name
                    return True, corners, ids, idx, rvec, tvec, dict_name

        return False, last_corners, last_ids, None, None, None, ''

    def _estimate_marker_pose(self, marker_corners):
        if self.camera_matrix is None:
            return False, None, None

        s = self.marker_size
        object_points = np.array([
            [-s / 2.0,  s / 2.0, 0.0],
            [ s / 2.0,  s / 2.0, 0.0],
            [ s / 2.0, -s / 2.0, 0.0],
            [-s / 2.0, -s / 2.0, 0.0],
        ], dtype=np.float64)
        image_points = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)

        flags = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=flags)
        if (not ok) or float(tvec[2]) <= 0:
            ok, rvec, tvec = cv2.solvePnP(
                object_points, image_points, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE)
        return bool(ok), rvec, tvec

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            if msg.d:
                self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            else:
                self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
            self.get_logger().info(
                f'相机内参已获取: fx={self.camera_matrix[0, 0]:.1f}, '
                f'fy={self.camera_matrix[1, 1]:.1f}, '
                f'cx={self.camera_matrix[0, 2]:.1f}, cy={self.camera_matrix[1, 2]:.1f}')

    def image_callback(self, msg):
        now_sec = self._now_sec()
        self.last_image_time = now_sec
        self.frame_count += 1

        if self.camera_matrix is None:
            if self.frame_count % 30 == 0:
                self.get_logger().info('等待 CameraInfo 内参...', throttle_duration_sec=2.0)
            return

        dt = 0.1 if self.last_control_time is None else max(now_sec - self.last_control_time, 0.001)
        self.last_control_time = now_sec

        cv_image = self._image_msg_to_bgr(msg)
        if cv_image is None:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        detected, corners, ids, marker_idx, rvec, tvec, dict_name = self._detect_target(gray)

        if detected:
            self.detect_count += 1
            self.last_seen_time = now_sec

            tvec_flat = np.asarray(tvec, dtype=np.float64).reshape(3)
            depth = float(tvec_flat[2])
            lateral = float(tvec_flat[0])
            yaw = math.atan2(lateral, depth)

            self._update_filtered_measurement(depth, lateral, yaw)
            self._control_tracking(dt, dict_name)
        else:
            self._handle_loss(dt)

        if self.debug:
            self._draw_debug(cv_image, corners, ids, marker_idx, rvec, tvec)

    def _image_msg_to_bgr(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge 转换失败，尝试手动转换: {exc}', throttle_duration_sec=2.0)

        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            encoding = msg.encoding.lower()
            if encoding == 'bgra8':
                bgra = data.reshape((msg.height, msg.width, 4))
                return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            if encoding == 'rgba8':
                rgba = data.reshape((msg.height, msg.width, 4))
                return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
            if encoding == 'bgr8':
                return data.reshape((msg.height, msg.width, 3))
            if encoding == 'rgb8':
                rgb = data.reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        except Exception as exc:
            self.get_logger().error(f'图像转换失败: encoding={msg.encoding}, error={exc}')
        return None

    def _update_filtered_measurement(self, depth, lateral, yaw):
        if self.filtered_depth is None:
            self.filtered_depth = depth
            self.filtered_lateral = lateral
            self.filtered_yaw = yaw
        else:
            a = self.filter_alpha
            self.filtered_depth = a * self.filtered_depth + (1.0 - a) * depth
            self.filtered_lateral = a * self.filtered_lateral + (1.0 - a) * lateral
            self.filtered_yaw = a * self.filtered_yaw + (1.0 - a) * yaw

        self.last_depth = self.filtered_depth
        self.last_lateral = self.filtered_lateral
        self.last_yaw = self.filtered_yaw

    def _control_tracking(self, dt, dict_name):
        depth = self.filtered_depth
        lateral = self.filtered_lateral
        yaw = self.filtered_yaw

        dist_err = depth - self.target_dist
        yaw_err = -yaw

        vx = 0.0 if abs(dist_err) < self.dist_tol else self.pd_dist.update(dist_err, dt)
        wz = 0.0 if abs(yaw_err) < self.angle_tol else self.pd_yaw.update(yaw_err, dt)

        if self.use_lateral and abs(lateral) > self.lateral_tol:
            # OpenCV 相机坐标 x 向右；ROS base_link 的 linear.y 通常左正，因此取负号。
            vy = clamp(-self.kp_lateral * lateral, -self.v_max, self.v_max)
        else:
            vy = 0.0

        vx, vy = self._limit_planar_velocity(vx, vy)
        self._publish_cmd(vx, vy, wz)
        self.state = 'TRACKING'
        self._log_data(True, depth, lateral, yaw, dist_err, yaw_err, vx, vy, wz, self.state, dict_name)

        self.get_logger().info(
            f'[TRACK] dict={dict_name} depth={depth:.2f}m lateral={lateral:.2f}m '
            f'yaw={math.degrees(yaw):.1f}deg cmd=({vx:.3f},{vy:.3f},{wz:.3f})',
            throttle_duration_sec=0.5)

    def _handle_loss(self, dt):
        now_sec = self._now_sec()

        if self.last_seen_time is None:
            if self.search_when_never_seen:
                self._publish_cmd(0.0, 0.0, self.search_w)
                self.state = 'SEARCHING_INIT'
                self._log_data(
                    False, 0, 0, 0, 0, 0, 0, 0, self.search_w,
                    self.state, self.active_dict_name or '')
                self.get_logger().info(
                    f'尚未检测到目标标签，原地自旋搜索 wz={self.search_w:.2f}',
                    throttle_duration_sec=1.0)
            else:
                self._publish_cmd(0.0, 0.0, 0.0)
                self.state = 'WAITING'
                self._log_data(False, 0, 0, 0, 0, 0, 0, 0, 0, self.state, self.active_dict_name or '')
                self.get_logger().info('等待检测到目标标签...', throttle_duration_sec=2.0)
            return

        elapsed = now_sec - self.last_seen_time
        if elapsed < self.loss_timeout:
            vx, vy, wz = self.last_cmd
            decay = self.occluded_cmd_decay
            vx, vy, wz = vx * decay, vy * decay, wz * decay
            self._publish_cmd(vx, vy, wz)
            self.state = 'OCCLUDED'
            self._log_data(
                False, self.last_depth or 0.0, self.last_lateral or 0.0, self.last_yaw or 0.0,
                0.0, 0.0, vx, vy, wz, self.state, self.active_dict_name or '')
            self.get_logger().info(
                f'[OCC] 标签短暂丢失 {elapsed:.2f}s，按上一指令衰减继续',
                throttle_duration_sec=0.5)
            return

        if elapsed < self.search_timeout:
            self._publish_cmd(0.0, 0.0, self.search_w)
            self.state = 'SEARCHING'
            self._log_data(False, 0, 0, 0, 0, 0, 0, 0, self.search_w, self.state, self.active_dict_name or '')
            self.get_logger().info(
                f'[SRCH] 标签丢失 {elapsed:.2f}s，原地自旋搜索 wz={self.search_w:.2f}',
                throttle_duration_sec=0.5)
            return

        self._publish_cmd(0.0, 0.0, 0.0)
        self.state = 'LOST'
        self._log_data(False, 0, 0, 0, 0, 0, 0, 0, 0, self.state, self.active_dict_name or '')
        self.get_logger().info(
            f'[LOST] 标签丢失 {elapsed:.2f}s，安全停车',
            throttle_duration_sec=1.0)

    def safety_check(self):
        now_sec = self._now_sec()
        if self.last_image_time is None:
            self._publish_cmd(0.0, 0.0, 0.0)
            self.get_logger().info('等待相机图像...', throttle_duration_sec=2.0)
            return

        if now_sec - self.last_image_time > self.image_timeout:
            self.get_logger().warn(
                f'相机图像超过 {self.image_timeout:.1f}s 未更新，进入丢失处理',
                throttle_duration_sec=1.0)
            self._handle_loss(0.1)

    def _limit_planar_velocity(self, vx, vy):
        v_total = math.hypot(vx, vy)
        if v_total > self.v_max:
            scale = self.v_max / v_total
            vx *= scale
            vy *= scale
        return vx, vy

    def _publish_cmd(self, vx, vy, wz):
        vx = float(clamp(vx, -self.v_max, self.v_max))
        vy = float(clamp(vy, -self.v_max, self.v_max))
        wz = float(clamp(wz, -self.w_max, self.w_max))
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.cmd_pub.publish(msg)
        self.last_cmd = (vx, vy, wz)

    def _draw_debug(self, image, corners, ids, marker_idx, rvec, tvec):
        if corners is not None and ids is not None:
            aruco.drawDetectedMarkers(image, corners, ids)

        if marker_idx is not None and rvec is not None and tvec is not None:
            if hasattr(cv2, 'drawFrameAxes'):
                cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
            elif hasattr(aruco, 'drawAxis'):
                aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

        if self.last_depth is not None and self.last_yaw is not None:
            info = (
                f'ID={self.marker_id} depth={self.last_depth:.2f}m '
                f'yaw={math.degrees(self.last_yaw):.1f}deg state={self.state}')
            cv2.putText(image, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        if not self.debug_window_enabled:
            return

        try:
            cv2.imshow('Target Follower Debug', image)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.debug_window_enabled = False
            self.get_logger().warn(f'Debug window disabled automatically: {exc}')

    def _log_data(self, detected, depth, lateral, yaw, dist_err, yaw_err, vx, vy, wz, state, dict_name):
        if self.csv_file:
            t = (self.get_clock().now() - self.log_start).nanoseconds / 1e9
            self.csv_file.write(
                f'{t:.3f},{1 if detected else 0},{depth:.4f},{lateral:.4f},{yaw:.4f},'
                f'{dist_err:.4f},{yaw_err:.4f},{vx:.4f},{vy:.4f},{wz:.4f},{state},{dict_name}\n')
            self.csv_file.flush()

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def destroy_node(self):
        self._publish_cmd(0.0, 0.0, 0.0)
        if self.csv_file:
            self.csv_file.close()
        if self.debug_window_enabled:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TargetFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('检测到手动中断，停车。')
        node._publish_cmd(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ============================================================
# 运行方法
# ============================================================
# 终端 1：启动仿真
# source /opt/ros/humble/setup.bash
# source ~/ros2_ws/install/setup.bash
# ros2 launch webots_ros2_robomaster robot_launch.py
#
# 终端 2：键盘控制领航者 agent0
# source /opt/ros/humble/setup.bash
# source ~/ros2_ws/install/setup.bash
# python3 ~/ros2_ws/src/webots_ros2_robomaster/webots_ros2_robomaster/keyboard_twist.py --topic /agent0/cmd_vel
#
# 终端 3：启动跟随者 agent1
# source /opt/ros/humble/setup.bash
# source ~/ros2_ws/install/setup.bash
# ros2 run webots_ros2_robomaster target_follower --ros-args \
#   -p use_sim_time:=True \
#   -p aruco_dict:=auto \
#   -p log_csv:=/home/chengxiaoyu/ros2_ws/follower_log.csv
#
# 如果实际相机话题不是默认值，可覆盖：
# ros2 run webots_ros2_robomaster target_follower --ros-args \
#   -p use_sim_time:=True \
#   -p image_topic:=/robomaster_1/camera_0/image_proc \
#   -p camera_info_topic:=/robomaster_1/camera_0/camera_info
#
# 可选：开启横向修正（利用全向底盘）
# ros2 run webots_ros2_robomaster target_follower --ros-args \
#   -p use_sim_time:=True -p use_lateral:=True
#
# 编译说明：
# 1. 复制本文件到 webots_ros2_robomaster/webots_ros2_robomaster/
# 2. setup.py 的 console_scripts 中加入：
#    'target_follower = webots_ros2_robomaster.target_follower:main',
# 3. cd ~/ros2_ws && colcon build --packages-select webots_ros2_robomaster
# 4. source install/setup.bash

