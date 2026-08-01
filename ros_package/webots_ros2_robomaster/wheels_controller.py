import rclpy
from geometry_msgs.msg import Twist
from sensing_msgs.msg import WheelSpeed
import time 


# ros2control.yml
# wheel_separation: 0.216  # 2 * y-offset (from URDF)
# wheel_radius: 0.06
HALF_DISTANCE_BETWEEN_WHEELS = 0.216 / 2
WHEEL_RADIUS = 0.06
# Constants
MAX_SPEED = 1.0
MAX_AGULAR_SPEED = 1.0
WHEEL_RADIUS = 0.26
LX = 0.238
LY = 0.285


class WheelsController:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__motor_fl = self.__robot.getDevice('front_left_wheel_joint')
        self.__motor_fr = self.__robot.getDevice('front_right_wheel_joint')
        self.__motor_bl = self.__robot.getDevice('back_left_wheel_joint')
        self.__motor_br = self.__robot.getDevice('back_right_wheel_joint')

        self.__motor_fl.setPosition(float('inf'))
        self.__motor_fr.setPosition(float('inf'))
        self.__motor_bl.setPosition(float('inf'))
        self.__motor_br.setPosition(float('inf'))

        self.__motor_fl.setVelocity(0)
        self.__motor_fr.setVelocity(0)
        self.__motor_bl.setVelocity(0)
        self.__motor_br.setVelocity(0)

        self.__last_cmd_time = time.time()
        self.__cmd_timeout = 1.5  # 秒數（超過這個時間就自動停止）

        self.__wheel_speed = WheelSpeed()
        self.__target_twist = Twist()

        self.__robot_name = self.__robot.getName()
        topic_name_wheelspeed = f'/{self.__robot_name}/cmd_wheels'
        topic_name_twist = f'/{self.__robot_name}/cmd_vel'

        if not rclpy.ok():
            rclpy.init(args=None)
        
        self.__node = rclpy.create_node('wheels_controller')
        self.__node.create_subscription(WheelSpeed, topic_name_wheelspeed, self.__wheel_speed_callback, 1)
        self.__node.create_subscription(Twist, topic_name_twist, self.__cmd_vel_callback, 1)

    def __wheel_speed_callback(self, speed):
        self.__wheel_speed = speed
        self.__node.get_logger().info(
            f"Received wheel speed command: FL={speed.fl}, FR={speed.fr}, RL={speed.rl}, RR={speed.rr}"
        )
        self.__last_cmd_time = time.time()  # Update the last command time


    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
        self.__last_cmd_time = time.time()  # Update the last command time

    def step(self):
            
        rclpy.spin_once(self.__node, timeout_sec=0.01)

        current_time = time.time()
        if current_time - self.__last_cmd_time > self.__cmd_timeout:
            self.__target_twist = Twist()
            self.__wheel_speed = WheelSpeed()
        
        # Use wheel speed if any value is set (assume > small threshold)
        # if abs(self.__wheel_speed.fl) > 0.001 or abs(self.__wheel_speed.fr) > 0.001 or abs(self.__wheel_speed.rl) > 0.001 or abs(self.__wheel_speed.rr) > 0.001:
        self.__motor_fl.setVelocity(self.__wheel_speed.fl)
        self.__motor_fr.setVelocity(self.__wheel_speed.fr)
        self.__motor_bl.setVelocity(self.__wheel_speed.rl)
        self.__motor_br.setVelocity(self.__wheel_speed.rr)
        self.__node.get_logger().info(
            f"Set speed: FL={self.__wheel_speed.fl}, FR={self.__wheel_speed.fr}, RL={self.__wheel_speed.rl}, RR={self.__wheel_speed.rr}"
        )
        # else:
        vx = self.__target_twist.linear.x
        vy = self.__target_twist.linear.y
        omega = self.__target_twist.angular.z

        len_xy = LX + LY
        motor_speed = [
            1 / WHEEL_RADIUS * (vx - vy - len_xy * omega),
            1 / WHEEL_RADIUS * (vx + vy + len_xy * omega),
            1 / WHEEL_RADIUS * (vx + vy - len_xy * omega),
            1 / WHEEL_RADIUS * (vx - vy + len_xy * omega),
        ]

        self.__motor_fl.setVelocity(motor_speed[0])
        self.__motor_fr.setVelocity(motor_speed[1])
        self.__motor_bl.setVelocity(motor_speed[2])
        self.__motor_br.setVelocity(motor_speed[3])