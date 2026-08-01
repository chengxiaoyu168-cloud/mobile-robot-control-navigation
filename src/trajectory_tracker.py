"""
Part 3: 轨迹跟踪控制器
========================
基于 Part 2 的 PID 控制器，实现多航点轨迹跟踪。

核心策略：
- 前瞻点（Pure Pursuit）：不瞄航点本身，瞄航点前方一段距离，使轨迹平滑
- 航点切换：接近当前航点（< switch_threshold）时自动切换下一个，不停顿
- 巡航速度：航点间保持恒定速度，只在最后一个航点用 P 控制减速停车
- 理想轨迹记录：同步记录理想轨迹点，用于可视化对比

支持三种轨迹类型：
  line    : 直线（2 个航点）
  square  : 正方形（4 个航点）
  circle  : 圆形（N 个航点，参数方程生成）

使用方法见文件末尾注释。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PointStamped
from webots_ros2_msgs.msg import FloatStamped


# ============================================================
# PID 控制器类（复用 Part 2）
# ============================================================
class PID:
    """标准 PID 控制器，带积分限幅与输出限幅。"""

    def __init__(self, kp=0.8, ki=0.0, kd=0.05,
                 out_min=-1.0, out_max=1.0,
                 i_min=-0.5, i_max=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_min = i_min
        self.i_max = i_max
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_run = True

    def update(self, error, dt):
        p = self.kp * error
        self._integral += error * dt
        self._integral = clamp(self._integral, self.i_min, self.i_max)
        i = self.ki * self._integral
        if self._first_run:
            d = 0.0
            self._first_run = False
        else:
            d = self.kd * (error - self._prev_error) / dt if dt > 1e-6 else 0.0
        self._prev_error = error
        return clamp(p + i + d, self.out_min, self.out_max)

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_run = True


# ============================================================
# 工具函数（复用 Part 2）
# ============================================================
def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ============================================================
# 航点生成器
# ============================================================
def generate_waypoints(traj_type, **kwargs):
    """
    根据轨迹类型生成航点列表。
    返回: [(x0, y0), (x1, y1), ...]
    """
    if traj_type == 'line':
        # 直线：从起点到终点
        start = kwargs.get('start', (0.0, 0.0))
        end = kwargs.get('end', (1.0, 0.0))
        return [start, end]

    elif traj_type == 'square':
        # 正方形：4 个角点（逆时针）
        center = kwargs.get('center', (0.0, 0.0))
        size = kwargs.get('size', 1.0)
        cx, cy = center
        half = size / 2.0
        return [
            (cx - half, cy - half),  # 左下
            (cx + half, cy - half),  # 右下
            (cx + half, cy + half),  # 右上
            (cx - half, cy + half),  # 左上
            (cx - half, cy - half),  # 回到起点（闭合）
        ]

    elif traj_type == 'circle':
        # 圆形：参数方程生成 N 个点
        center = kwargs.get('center', (0.0, 0.0))
        radius = kwargs.get('radius', 1.0)
        num_points = kwargs.get('num_points', 36)
        cx, cy = center
        waypoints = []
        for i in range(num_points + 1):  # +1 闭合
            angle = 2.0 * math.pi * i / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            waypoints.append((x, y))
        return waypoints

    else:
        raise ValueError(f"未知轨迹类型: {traj_type}")


# ============================================================
# 轨迹跟踪节点
# ============================================================
class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        # ---------- 声明参数 ----------
        # 轨迹参数
        self.declare_parameter('trajectory_type', 'circle')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('end_x', 1.0)
        self.declare_parameter('end_y', 0.0)
        self.declare_parameter('square_center_x', 0.0)
        self.declare_parameter('square_center_y', 0.0)
        self.declare_parameter('square_size', 1.5)
        self.declare_parameter('circle_center_x', 0.0)
        self.declare_parameter('circle_center_y', 0.0)
        self.declare_parameter('circle_radius', 0.8)
        self.declare_parameter('circle_points', 36)
        # 跟踪参数
        self.declare_parameter('cruise_speed', 0.2)        # 巡航速度 m/s
        self.declare_parameter('lookahead', 0.3)           # 前瞻距离 m
        self.declare_parameter('switch_threshold', 0.15)   # 航点切换阈值 m
        self.declare_parameter('goal_tolerance', 0.05)     # 终点停车容差 m
        self.declare_parameter('v_max', 0.25)              # 速度限幅 m/s
        self.declare_parameter('w_max', 2.0)               # 角速度限幅
        self.declare_parameter('control_hz', 50.0)
        # PID 参数（用于最终减速停车）
        self.declare_parameter('kp_end', 0.8)
        self.declare_parameter('kd_end', 0.05)
        self.declare_parameter('kp_w', 1.5)
        self.declare_parameter('kd_w', 0.1)
        # 话题
        self.declare_parameter('gps_topic', '/agent0/gps')
        self.declare_parameter('compass_topic', '/agent0/compass/bearing')
        self.declare_parameter('cmd_topic', '/agent0/cmd_vel')
        self.declare_parameter('log_csv', '')

        # ---------- 读取参数 ----------
        traj_type = self.get_parameter('trajectory_type').value
        cruise_speed = self.get_parameter('cruise_speed').value
        self.lookahead = self.get_parameter('lookahead').value
        self.switch_threshold = self.get_parameter('switch_threshold').value
        self.tol = self.get_parameter('goal_tolerance').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        control_hz = self.get_parameter('control_hz').value
        gps_topic = self.get_parameter('gps_topic').value
        compass_topic = self.get_parameter('compass_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        csv_path = self.get_parameter('log_csv').value

        # ---------- 生成航点 ----------
        if traj_type == 'line':
            self.waypoints = generate_waypoints('line',
                start=(self.get_parameter('start_x').value, self.get_parameter('start_y').value),
                end=(self.get_parameter('end_x').value, self.get_parameter('end_y').value))
        elif traj_type == 'square':
            self.waypoints = generate_waypoints('square',
                center=(self.get_parameter('square_center_x').value, self.get_parameter('square_center_y').value),
                size=self.get_parameter('square_size').value)
        elif traj_type == 'circle':
            self.waypoints = generate_waypoints('circle',
                center=(self.get_parameter('circle_center_x').value, self.get_parameter('circle_center_y').value),
                radius=self.get_parameter('circle_radius').value,
                num_points=self.get_parameter('circle_points').value)
        else:
            self.get_logger().error(f"未知轨迹类型: {traj_type}")
            raise ValueError(f"未知轨迹类型: {traj_type}")

        self.cruise_speed = cruise_speed
        self.current_wp_idx = 0
        self.dt = 1.0 / control_hz
        self.finished = False

        # 用于最终减速的 PID（x 和 y 各一个独立实例，避免微分项串扰）
        self.pid_end_x = PID(
            kp=self.get_parameter('kp_end').value,
            kd=self.get_parameter('kd_end').value,
            out_min=-self.v_max, out_max=self.v_max)
        self.pid_end_y = PID(
            kp=self.get_parameter('kp_end').value,
            kd=self.get_parameter('kd_end').value,
            out_min=-self.v_max, out_max=self.v_max)
        self.pid_w = PID(
            kp=self.get_parameter('kp_w').value,
            kd=self.get_parameter('kd_w').value,
            out_min=-self.w_max, out_max=self.w_max)

        # ---------- 当前位姿 ----------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.gps_received = False
        self.compass_received = False
        self._gps_logged = False       # 调试：首次GPS日志
        self._compass_logged = False   # 调试：首次Compass日志
        self._loop_count = 0           # 调试：控制循环计数

        # ---------- ROS 通信 ----------
        self.create_subscription(PointStamped, gps_topic, self.gps_callback, qos_profile_sensor_data)
        self.create_subscription(FloatStamped, compass_topic, self.compass_callback, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.get_logger().info(f'[DEBUG] 订阅创建完成: GPS={gps_topic}, Compass={compass_topic}, CMD={cmd_topic}')

        # ---------- 数据记录 ----------
        self.csv_file = None
        if csv_path:
            self.csv_file = open(csv_path, 'w', encoding='utf-8')
            self.csv_file.write('t,x,y,theta,rho,vx,vy,w,wp_idx,ideal_x,ideal_y,reached\n')
            self.log_start = self.get_clock().now()
            self.get_logger().info(f'数据记录到: {csv_path}')

        # ---------- 启动日志 ----------
        self.get_logger().info(f'轨迹类型: {traj_type} | 航点数: {len(self.waypoints)}')
        self.get_logger().info(f'巡航速度: {cruise_speed} m/s | 前瞻距离: {self.lookahead} m | '
                               f'切换阈值: {self.switch_threshold} m')
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  航点 {i}: ({wp[0]:.3f}, {wp[1]:.3f})')

        # ---------- 控制定时器 ----------
        self.timer = self.create_timer(self.dt, self.control_loop)

    # ---------- GPS 回调 ----------
    def gps_callback(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y
        self.gps_received = True
        if not self._gps_logged:
            self.get_logger().info(f'[DEBUG] 首次收到GPS数据: ({self.x:.3f}, {self.y:.3f})')
            self._gps_logged = True

    # ---------- 罗盘回调 ----------
    def compass_callback(self, msg):
        self.theta = wrap_angle(math.radians(msg.data))
        self.compass_received = True
        if not self._compass_logged:
            self.get_logger().info(f'[DEBUG] 首次收到Compass数据: {math.degrees(self.theta):.1f}°')
            self._compass_logged = True

    # ---------- 计算前瞻点 ----------
    def compute_lookahead_point(self, current_pos, wp_current, wp_next):
        """
        计算前瞻点：从当前航点朝下一个航点方向延伸 lookahead 距离。
        如果是最后一个航点（无下一个），返回航点本身。
        """
        dx = wp_next[0] - wp_current[0]
        dy = wp_next[1] - wp_current[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return wp_current
        # 单位方向向量
        ux = dx / dist
        uy = dy / dist
        # 前瞻点 = 当前航点 + 方向 × 前瞻距离
        lx = wp_current[0] + ux * self.lookahead
        ly = wp_current[1] + uy * self.lookahead
        return (lx, ly)

    # ---------- 主控制循环 ----------
    def control_loop(self):
        self._loop_count += 1
        if not (self.gps_received and self.compass_received):
            # 调试：每2秒打印一次等待状态
            if self._loop_count % 100 == 0:
                self.get_logger().info(
                    f'[DEBUG] 控制循环已运行 {self._loop_count} 次，等待数据: '
                    f'gps={self.gps_received}, compass={self.compass_received}')
            return

        if self.finished:
            return

        x, y, theta = self.x, self.y, self.theta

        # 当前航点
        wp_idx = self.current_wp_idx
        wp_current = self.waypoints[wp_idx]

        # 距离当前航点
        rho = math.hypot(wp_current[0] - x, wp_current[1] - y)

        # ---------- 检查是否到达当前航点 → 切换 ----------
        if rho < self.switch_threshold and wp_idx < len(self.waypoints) - 1:
            self.current_wp_idx += 1
            wp_idx = self.current_wp_idx
            wp_current = self.waypoints[wp_idx]
            rho = math.hypot(wp_current[0] - x, wp_current[1] - y)
            self.get_logger().info(
                f'>>> 切换到航点 {wp_idx}/{len(self.waypoints)-1} '
                f'({wp_current[0]:.2f},{wp_current[1]:.2f}) 当前({x:.2f},{y:.2f})')

        # ---------- 判断是否是最后一个航点 ----------
        is_last_waypoint = (wp_idx == len(self.waypoints) - 1)

        if is_last_waypoint:
            # 最后一个航点：用 P 控制减速停车（同 Part 2）
            ex_w = wp_current[0] - x
            ey_w = wp_current[1] - y
            if rho < self.tol:
                self.publish_cmd(0.0, 0.0, 0.0)
                self.log_data(rho, 0.0, 0.0, 0.0, wp_idx, wp_current)
                self.finished = True
                self.timer.cancel()
                self.get_logger().info(
                    f'>>> 轨迹跟踪完成! 终点({x:.3f},{y:.3f}) 误差 {rho*100:.1f}cm')
                return
            # 坐标变换 + PID 减速
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            ex_r = ex_w * cos_t + ey_w * sin_t
            ey_r = -ex_w * sin_t + ey_w * cos_t
            vx = clamp(self.pid_end_x.update(ex_r, self.dt), -self.v_max, self.v_max)
            vy = clamp(self.pid_end_y.update(ey_r, self.dt), -self.v_max, self.v_max)
            # 合速度限幅
            v_total = math.hypot(vx, vy)
            if v_total > self.v_max:
                scale = self.v_max / v_total
                vx *= scale
                vy *= scale
            self.publish_cmd(vx, vy, 0.0)
            self.log_data(rho, vx, vy, 0.0, wp_idx, wp_current)
            self.get_logger().info(
                f'[END] wp={wp_idx} pos=({x:.2f},{y:.2f}) ρ={rho:.3f} cmd=({vx:.3f},{vy:.3f})',
                throttle_duration_sec=1.0)

        else:
            # 非最后航点：直接朝当前航点走，巡航速度
            # 全向底盘无需转向，直接朝目标方向平移
            dx = wp_current[0] - x
            dy = wp_current[1] - y
            dist = math.hypot(dx, dy)

            if dist < 1e-6:
                vx_world = 0.0
                vy_world = 0.0
            else:
                # 巡航速度沿目标方向
                vx_world = self.cruise_speed * dx / dist
                vy_world = self.cruise_speed * dy / dist

            # 世界系速度 → 机器人系
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            vx_robot = vx_world * cos_t + vy_world * sin_t
            vy_robot = -vx_world * sin_t + vy_world * cos_t

            # 限幅
            v_total = math.hypot(vx_robot, vy_robot)
            if v_total > self.v_max:
                scale = self.v_max / v_total
                vx_robot *= scale
                vy_robot *= scale

            self.publish_cmd(vx_robot, vy_robot, 0.0)
            self.log_data(rho, vx_robot, vy_robot, 0.0, wp_idx, wp_current)
            self.get_logger().info(
                f'[CRUISE] wp={wp_idx}/{len(self.waypoints)-1} '
                f'pos=({x:.2f},{y:.2f}) θ={math.degrees(theta):.0f}° '
                f'ρ={rho:.3f} cmd=({vx_robot:.3f},{vy_robot:.3f})',
                throttle_duration_sec=1.0)

    # ---------- 发布速度命令 ----------
    def publish_cmd(self, vx, vy, w):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w
        self.cmd_pub.publish(msg)

    # ---------- 记录数据 ----------
    def log_data(self, rho, vx, vy, w, wp_idx, ideal_pt):
        if self.csv_file:
            t = (self.get_clock().now() - self.log_start).nanoseconds / 1e9
            self.csv_file.write(
                f'{t:.3f},{self.x:.4f},{self.y:.4f},{self.theta:.4f},'
                f'{rho:.4f},{vx:.4f},{vy:.4f},{w:.4f},{wp_idx},'
                f'{ideal_pt[0]:.4f},{ideal_pt[1]:.4f},{1 if self.finished else 0}\n')
            self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


# ============================================================
# main
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('检测到手动中断，停车。')
        node.publish_cmd(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# ============================================================
# 运行方法
# ============================================================
# 【前置】把本文件复制到包目录，并在 setup.py 注册入口点：
#   'trajectory_tracker = webots_ros2_robomaster.trajectory_tracker:main',
# 然后重新编译：cd ~/ros2_ws && colcon build --packages-select webots_ros2_robomaster
#
# --- 终端 1：启动仿真 ---
# ros2 launch webots_ros2_robomaster robot_launch.py
#
# --- 终端 2：运行轨迹跟踪 ---
# source /opt/ros/humble/setup.bash
# source ~/ros2_ws/install/setup.bash
#
# # 测试 1：圆形轨迹（主测试）
# ros2 run webots_ros2_robomaster trajectory_tracker --ros-args \
#   -p trajectory_type:=circle -p circle_radius:=0.8 -p circle_points:=36 \
#   -p cruise_speed:=0.2 -p lookahead:=0.3
#
# # 测试 2：正方形轨迹
# ros2 run webots_ros2_robomaster trajectory_tracker --ros-args \
#   -p trajectory_type:=square -p square_size:=1.5 \
#   -p cruise_speed:=0.2 -p lookahead:=0.3
#
# # 测试 3：直线轨迹
# ros2 run webots_ros2_robomaster trajectory_tracker --ros-args \
#   -p trajectory_type:=line -p start_x:=0.0 -p start_y:=0.0 \
#   -p end_x:=1.0 -p end_y:=0.0 -p cruise_speed:=0.2
#
# # 带数据记录
# ros2 run webots_ros2_robomaster trajectory_tracker --ros-args \
#   -p trajectory_type:=circle -p circle_radius:=0.8 \
#   -p log_csv:=/home/chengxiaoyu/ros2_ws/traj_log_circle.csv

