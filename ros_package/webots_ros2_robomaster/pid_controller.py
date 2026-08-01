"""
Part 2: PID Point-to-Point Navigation Controller
=================================================
全向底盘（麦克纳姆轮）点到点导航 PID 控制器。

控制策略：位置-朝向解耦控制
- 前向 (vx): PID_x(前方误差)
- 横向 (vy): PID_y(左侧误差)
- 旋转 (w) : PID_w(朝向误差)  [可选，未指定目标朝向则为 0]

工作流程：
1. 订阅位姿源获取当前 (x, y, theta)
   - 默认 GPS + compass（/agent0/gps + /agent0/compass/bearing）
   - 可用 -p pose_source:=odom 切换为 /agent0/odom（需 diffdrive_controller 已运行）
2. 计算世界系误差 -> 坐标变换到机器人系
3. 三个独立 PID 分别算出 (vx, vy, w)
4. 限幅 + 死区后发布到 /agent0/cmd_vel
5. 到达目标点（距离 < tolerance）后停车

注意：compass bearing 单位为度，0°=东(+x)，逆时针为正，内部转为弧度。
      GPS 直接给出世界坐标系真值 (point.x, point.y)，无累积误差。

使用方法见文件末尾注释。
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
from webots_ros2_msgs.msg import FloatStamped


# ============================================================
# PID 控制器类（含 anti-windup 积分限幅 + 死区）
# ============================================================
class PID:
    """标准 PID 控制器，带积分限幅与输出限幅。"""

    def __init__(self, kp=0.8, ki=0.0, kd=0.05,
                 out_min=-1.0, out_max=1.0,
                 i_min=-0.5, i_max=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # 输出限幅
        self.out_min = out_min
        self.out_max = out_max
        # 积分项限幅（anti-windup）
        self.i_min = i_min
        self.i_max = i_max
        # 内部状态
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_run = True

    def update(self, error, dt):
        """根据误差和 dt 计算控制量。"""
        # P 比例项
        p = self.kp * error
        # I 积分项（累加并限幅）
        self._integral += error * dt
        self._integral = clamp(self._integral, self.i_min, self.i_max)
        i = self.ki * self._integral
        # D 微分项（首次运行无前值，置 0）
        if self._first_run:
            d = 0.0
            self._first_run = False
        else:
            d = self.kd * (error - self._prev_error) / dt if dt > 1e-6 else 0.0
        self._prev_error = error
        # 合成并输出限幅
        return clamp(p + i + d, self.out_min, self.out_max)

    def reset(self):
        """重置内部状态（到达目标后调用，避免积分残留）。"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_run = True


# ============================================================
# 工具函数
# ============================================================
def clamp(value, low, high):
    """把 value 夹在 [low, high] 区间。"""
    if value < low:
        return low
    if value > high:
        return high
    return value


def wrap_angle(angle):
    """把任意角度折到 [-pi, pi]，取最短转向方向。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(qx, qy, qz, qw):
    """从四元数提取偏航角 theta（绕 z 轴）。"""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


# ============================================================
# PID 导航节点
# ============================================================
class PidController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # ---------- 声明参数（可在命令行用 -p 覆盖）----------
        self.declare_parameter('target_x', 1.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_theta', 999.0)   # 999.0 = 不指定目标朝向
        # PID 参数（前向 / 横向 / 旋转）
        self.declare_parameter('kp_x', 0.8)
        self.declare_parameter('ki_x', 0.0)
        self.declare_parameter('kd_x', 0.05)
        self.declare_parameter('kp_y', 0.8)
        self.declare_parameter('ki_y', 0.0)
        self.declare_parameter('kd_y', 0.05)
        self.declare_parameter('kp_w', 1.5)
        self.declare_parameter('ki_w', 0.0)
        self.declare_parameter('kd_w', 0.1)
        # 运行参数
        self.declare_parameter('goal_tolerance', 0.05)  # 到达判定距离(米)
        self.declare_parameter('theta_tolerance', 0.05) # 朝向判定阈值(弧度)
        self.declare_parameter('v_max', 0.25)           # 平移速度上限(m/s)
        self.declare_parameter('w_max', 2.0)            # 角速度上限(rad/s)
        self.declare_parameter('control_hz', 50.0)      # 控制频率
        self.declare_parameter('odom_topic', '/agent0/odom')
        self.declare_parameter('gps_topic', '/agent0/gps')
        self.declare_parameter('compass_topic', '/agent0/compass/bearing')
        self.declare_parameter('cmd_topic', '/agent0/cmd_vel')
        self.declare_parameter('pose_source', 'gps')    # 'gps' 或 'odom'
        self.declare_parameter('log_csv', '')           # 数据记录CSV路径，空=不记录

        # ---------- 读取参数 ----------
        gx = self.get_parameter('target_x').value
        gy = self.get_parameter('target_y').value
        gtheta = self.get_parameter('target_theta').value
        self.goal = (gx, gy)
        self.has_target_theta = (abs(gtheta - 999.0) > 1e-3)
        self.goal_theta = gtheta if self.has_target_theta else 0.0
        self.tol = self.get_parameter('goal_tolerance').value
        self.tol_theta = self.get_parameter('theta_tolerance').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        control_hz = self.get_parameter('control_hz').value
        odom_topic = self.get_parameter('odom_topic').value
        gps_topic = self.get_parameter('gps_topic').value
        compass_topic = self.get_parameter('compass_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        self.pose_source = self.get_parameter('pose_source').value  # 'gps' 或 'odom'
        csv_path = self.get_parameter('log_csv').value

        # ---------- 当前位姿 ----------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False
        self.gps_received = False
        self.compass_received = False

        # ---------- 三个 PID ----------
        self.pid_x = PID(
            kp=self.get_parameter('kp_x').value, ki=self.get_parameter('ki_x').value,
            kd=self.get_parameter('kd_x').value,
            out_min=-self.v_max, out_max=self.v_max)
        self.pid_y = PID(
            kp=self.get_parameter('kp_y').value, ki=self.get_parameter('ki_y').value,
            kd=self.get_parameter('kd_y').value,
            out_min=-self.v_max, out_max=self.v_max)
        self.pid_w = PID(
            kp=self.get_parameter('kp_w').value, ki=self.get_parameter('ki_w').value,
            kd=self.get_parameter('kd_w').value,
            out_min=-self.w_max, out_max=self.w_max)

        # ---------- ROS 通信 ----------
        if self.pose_source == 'odom':
            self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        else:  # gps + compass
            self.create_subscription(PointStamped, gps_topic, self.gps_callback, 10)
            self.create_subscription(FloatStamped, compass_topic, self.compass_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # ---------- 数据记录 ----------
        self.csv_file = None
        if csv_path:
            self.csv_file = open(csv_path, 'w', encoding='utf-8')
            self.csv_file.write('t,x,y,theta,rho,ex_w,ey_w,vx,vy,w,reached\n')
            self.log_start = self.get_clock().now()
            self.get_logger().info(f'数据记录到: {csv_path}')

        # ---------- 状态 ----------
        self.reached = False
        self.reached_theta = False
        self.dt = 1.0 / control_hz

        # ---------- 控制定时器 ----------
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'目标点: ({gx:.3f}, {gy:.3f})' +
            (f', 目标朝向: {math.degrees(gtheta):.1f}°' if self.has_target_theta else ', 不调整朝向'))
        self.get_logger().info(
            f'PID 前向: kp={self.pid_x.kp} ki={self.pid_x.ki} kd={self.pid_x.kd} | '
            f'横向: kp={self.pid_y.kp} ki={self.pid_y.ki} kd={self.pid_y.kd} | '
            f'旋转: kp={self.pid_w.kp} ki={self.pid_w.ki} kd={self.pid_w.kd}')
        self.get_logger().info(f'控制频率 {control_hz:.0f}Hz | 到达容差 {self.tol}m')
        self.get_logger().info(f'位姿来源: {self.pose_source.upper()}' +
                               (' (GPS+compass)' if self.pose_source == 'gps' else ' (odom)'))

    # ---------- 里程计回调：更新当前位姿 ----------
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    # ---------- GPS 回调：更新位置（世界坐标系真值）----------
    def gps_callback(self, msg):
        self.x = msg.point.x
        self.y = msg.point.y
        self.gps_received = True

    # ---------- 罗盘回调：更新朝向（度数，0=东，逆时针为正）----------
    def compass_callback(self, msg):
        # Webots compass bearing: 度数，0=东(+x)，逆时针为正
        # 转成弧度，并映射到 [-pi, pi]
        self.theta = math.radians(msg.data)
        self.theta = wrap_angle(self.theta)
        self.compass_received = True

    # ---------- 主控制循环 ----------
    def control_loop(self):
        # 还没收到位姿数据，不动作
        if self.pose_source == 'odom':
            if not self.odom_received:
                return
        else:  # gps + compass
            if not (self.gps_received and self.compass_received):
                return

        gx, gy = self.goal
        x, y, theta = self.x, self.y, self.theta

        # 1) 世界系误差
        ex_w = gx - x
        ey_w = gy - y
        rho = math.hypot(ex_w, ey_w)   # 直线距离

        # 2) 是否到达
        if rho < self.tol:
            # 位置到达，处理朝向（如果有要求）
            if not self.reached:
                self.reached = True
                self.pid_x.reset()
                self.pid_y.reset()
                self.get_logger().info(
                    f'>>> 位置到达! 当前({x:.3f},{y:.3f}) 误差 {rho*100:.1f}cm')

            if self.has_target_theta:
                e_theta = wrap_angle(self.goal_theta - theta)
                if abs(e_theta) < self.tol_theta:
                    if not self.reached_theta:
                        self.reached_theta = True
                        self.pid_w.reset()
                        self.get_logger().info(
                            f'>>> 朝向到达! {math.degrees(theta):.1f}° 误差 {math.degrees(e_theta):.1f}°')
                    self.publish_cmd(0.0, 0.0, 0.0)
                    self.log_data(rho, ex_w, ey_w, 0.0, 0.0, 0.0)
                    if self.reached_theta:
                        self.get_logger().info('>>> 任务完成，已停车。')
                        self.timer.cancel()
                    return
                # 仍在调整朝向
                w = self.pid_w.update(e_theta, self.dt)
                w = clamp(w, -self.w_max, self.w_max)
                self.publish_cmd(0.0, 0.0, w)
                self.log_data(rho, ex_w, ey_w, 0.0, 0.0, w)
                return
            else:
                # 不要求朝向，位置到达即完成
                self.publish_cmd(0.0, 0.0, 0.0)
                self.log_data(rho, ex_w, ey_w, 0.0, 0.0, 0.0)
                self.get_logger().info('>>> 任务完成，已停车。')
                self.timer.cancel()
                return

        # 3) 坐标变换：世界系 -> 机器人系
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        ex_r =  ex_w * cos_t + ey_w * sin_t   # 前方误差
        ey_r = -ex_w * sin_t + ey_w * cos_t   # 左侧误差

        # 4) 三个 PID
        vx = self.pid_x.update(ex_r, self.dt)
        vy = self.pid_y.update(ey_r, self.dt)
        w = 0.0
        if self.has_target_theta:
            e_theta = wrap_angle(self.goal_theta - theta)
            w = self.pid_w.update(e_theta, self.dt)
            w = clamp(w, -self.w_max, self.w_max)

        # 5) 合速度限幅（保证总平移速度不超 v_max）
        v_total = math.hypot(vx, vy)
        if v_total > self.v_max:
            scale = self.v_max / v_total
            vx *= scale
            vy *= scale

        # 6) 发布
        self.publish_cmd(vx, vy, w)
        self.log_data(rho, ex_w, ey_w, vx, vy, w)

        # 调试日志（每秒打印一次，避免刷屏）
        if int(self.get_clock().now().nanoseconds / 1e9) % 1 == 0:
            self.get_logger().info(
                f'({x:.2f},{y:.2f}) θ={math.degrees(theta):.0f}° '
                f'ρ={rho:.3f}m  cmd=({vx:.3f},{vy:.3f},{w:.3f})',
                throttle_duration_sec=1.0)

    # ---------- 发布速度命令 ----------
    def publish_cmd(self, vx, vy, w):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w
        self.cmd_pub.publish(msg)

    # ---------- 记录数据 ----------
    def log_data(self, rho, ex_w, ey_w, vx, vy, w):
        if self.csv_file:
            t = (self.get_clock().now() - self.log_start).nanoseconds / 1e9
            self.csv_file.write(
                f'{t:.3f},{self.x:.4f},{self.y:.4f},{self.theta:.4f},'
                f'{rho:.4f},{ex_w:.4f},{ey_w:.4f},{vx:.4f},{vy:.4f},{w:.4f},'
                f'{1 if self.reached else 0}\n')
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
    node = PidController()
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
# 【前置】确保已编译（见下方说明）
#
# --- 终端 1：启动仿真 ---
# source /opt/ros/humble/setup.bash
# source ~/ros2_ws/install/setup.bash
# ros2 launch webots_ros2_robomaster robot_launch.py
#
# --- 终端 2：启动 PID 控制器 ---
# source /opt/ros/humble/setup.bash
# source ~/ros2_ws/install/setup.bash
#
# # 测试 1：正前方 1 米（直线前进）
# ros2 run webots_ros2_robomaster pid_controller --ros-args -p target_x:=1.0 -p target_y:=0.0
#
# # 测试 2：正左方 1 米（纯横移，全向底盘标志）
# ros2 run webots_ros2_robomaster pid_controller --ros-args -p target_x:=0.0 -p target_y:=1.0
#
# # 测试 3：后侧方（双轴协同）
# ros2 run webots_ros2_robomaster pid_controller --ros-args -p target_x:=-0.8 -p target_y:=0.8
#
# # 带目标朝向 + 数据记录
# ros2 run webots_ros2_robomaster pid_controller --ros-args \
#   -p target_x:=1.0 -p target_y:=1.0 -p target_theta:=1.5708 \
#   -p log_csv:=/home/chengxiaoyu/ros2_ws/pid_log_t1.csv
#
# --- 调参示例 ---
# ros2 run webots_ros2_robomaster pid_controller --ros-args \
#   -p target_x:=1.0 -p target_y:=0.0 \
#   -p kp_x:=1.2 -p kd_x:=0.08 -p kp_w:=2.0
#
# ============================================================
# 编译说明（首次使用需做一次）
# ============================================================
# 1. 把本文件复制到包目录：
#    cp pid_controller.py ~/ros2_ws/src/webots_ros2_robomaster/webots_ros2_robomaster/
#
# 2. 编辑 setup.py，在 entry_points 的 console_scripts 里加一行：
#    'pid_controller = webots_ros2_robomaster.pid_controller:main',
#
# 3. 重新编译：
#    cd ~/ros2_ws && colcon build --packages-select webots_ros2_robomaster
#
# 4. source 后即可 ros2 run
#    source ~/ros2_ws/install/setup.bash

