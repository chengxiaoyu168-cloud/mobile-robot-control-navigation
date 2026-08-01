"""
Camera Plugin for webots_ros2_driver
=====================================
在 Webots driver 进程内读取 Camera 设备图像，发布为 ROS 2 Image/CameraInfo 话题。
"""

import math
import rclpy
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraPublisher:
    """Webots Camera -> ROS 2 Image/CameraInfo 发布插件。"""

    def init(self, webots_node, properties):
        # 先创建 ROS 节点，用 logger 输出（和 PluginExample 一致）
        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node(f'camera_plugin_{id(self)}')
        self.__node.get_logger().info('CameraPlugin init() called - plugin is loading!')

        self.__robot = webots_node.robot
        self.__robot_name = self.__robot.getName()
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__last_publish_time = -1.0
        self.__camera = None

        self.__node.get_logger().info(f'Robot: {self.__robot_name}, timestep: {self.__timestep}')

        # 列出所有设备
        n_devices = self.__robot.getNumberOfDevices()
        self.__node.get_logger().info(f'Number of devices: {n_devices}')
        camera_names = []
        for i in range(n_devices):
            dev = self.__robot.getDeviceByIndex(i)
            dev_name = dev.getName()
            dev_model = dev.getModel()
            self.__node.get_logger().info(f'  Device [{i}]: name="{dev_name}", model="{dev_model}"')
            if 'camera' in dev_name.lower() or 'camera' in dev_model.lower():
                camera_names.append(dev_name)

        publish_hz = float(properties.get('publishHz', properties.get('updateRate', '10')))
        self.__publish_period = 0.0 if publish_hz <= 0.0 else 1.0 / publish_hz

        # 尝试获取相机
        camera_name = properties.get('cameraName', 'camera')
        self.__node.get_logger().info(f'Trying camera name: "{camera_name}"')
        self.__camera = self.__robot.getDevice(camera_name)

        if self.__camera is None and camera_names:
            self.__node.get_logger().info(f'Trying alternatives: {camera_names}')
            for name in camera_names:
                self.__camera = self.__robot.getDevice(name)
                if self.__camera is not None:
                    camera_name = name
                    break

        if self.__camera is None:
            self.__node.get_logger().error('No camera device found!')
            return

        self.__camera.enable(self.__timestep)
        self.__width = self.__camera.getWidth()
        self.__height = self.__camera.getHeight()
        self.__fov = self.__camera.getFov()
        self.__near = self.__camera.getNear()

        self.__node.get_logger().info(
            f'Camera "{camera_name}" enabled: {self.__width}x{self.__height}, '
            f'FOV={math.degrees(self.__fov):.1f}deg, near={self.__near}')

        # 话题
        topic_prefix = properties.get('topicName', f'/{self.__robot_name}/camera')
        image_topic = f'{topic_prefix}/image_raw'
        info_topic = f'{topic_prefix}/camera_info'

        self.__image_pub = self.__node.create_publisher(Image, image_topic, qos_profile_sensor_data)
        self.__info_pub = self.__node.create_publisher(CameraInfo, info_topic, qos_profile_sensor_data)
        self.__node.get_logger().info(f'Publishing: {image_topic} and {info_topic}')

        # 内参
        fx = (self.__width / 2.0) / math.tan(self.__fov / 2.0)
        fy = fx
        cx = self.__width / 2.0
        cy = self.__height / 2.0

        self.__camera_info = CameraInfo()
        self.__camera_info.header.frame_id = f'{self.__robot_name}/camera_link'
        self.__camera_info.height = self.__height
        self.__camera_info.width = self.__width
        self.__camera_info.distortion_model = 'plumb_bob'
        self.__camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.__camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.__camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.__camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.__image_msg = Image()
        self.__image_msg.header.frame_id = f'{self.__robot_name}/camera_link'
        self.__image_msg.height = self.__height
        self.__image_msg.width = self.__width
        self.__image_msg.encoding = 'bgra8'
        self.__image_msg.is_bigendian = 0
        self.__image_msg.step = self.__width * 4

        self.__node.get_logger().info(
            f'Camera K: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}')
        self.__node.get_logger().info('Camera plugin initialized successfully!')

    def step(self):
        if self.__camera is None:
            return
        rclpy.spin_once(self.__node, timeout_sec=0)

        sim_time = self.__robot.getTime()
        if self.__publish_period > 0.0:
            if self.__last_publish_time >= 0.0 and sim_time - self.__last_publish_time < self.__publish_period:
                return
        self.__last_publish_time = sim_time

        image_data = self.__camera.getImage()
        if image_data is None:
            return

        stamp = Time()
        stamp.sec = int(sim_time)
        stamp.nanosec = int((sim_time - stamp.sec) * 1e9)

        self.__image_msg.header.stamp = stamp
        self.__image_msg.data = bytes(image_data)
        self.__image_pub.publish(self.__image_msg)

        self.__camera_info.header.stamp = stamp
        self.__info_pub.publish(self.__camera_info)
