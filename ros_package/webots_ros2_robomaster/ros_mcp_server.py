"""
Part 5: Minimal ROS MCP server for natural-language robot control.

This module implements a small stdio MCP-compatible JSON-RPC server. It exposes
safe, structured tools that an LLM/MCP host can call instead of publishing raw
velocity commands directly.

The tools reuse the controllers from Part 2, Part 3, and Part 4:
- pid_controller for follower point-to-point navigation
- trajectory_tracker for follower trajectory tracking
- target_follower for visual target following

It also provides closed-loop leader motions for commands such as "move forward
1 meter" or "turn left 90 degrees".
"""

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from webots_ros2_msgs.msg import FloatStamped


PROTOCOL_VERSION = "2024-11-05"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    gps_ok: bool = False
    compass_ok: bool = False
    last_update: float = 0.0


class RosToolNode(Node):
    """ROS side of the MCP server: status subscriptions and safe cmd_vel output."""

    def __init__(self) -> None:
        super().__init__("ros_mcp_server")
        self.states: Dict[str, RobotState] = {
            "agent0": RobotState(),
            "agent1": RobotState(),
        }
        self.cmd_pubs = {
            "agent0": self.create_publisher(Twist, "/agent0/cmd_vel", 10),
            "agent1": self.create_publisher(Twist, "/agent1/cmd_vel", 10),
        }
        for robot in self.states:
            self.create_subscription(
                PointStamped,
                f"/{robot}/gps",
                self._make_gps_callback(robot),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                FloatStamped,
                f"/{robot}/compass/bearing",
                self._make_compass_callback(robot),
                qos_profile_sensor_data,
            )

    def _make_gps_callback(self, robot: str):
        def callback(msg: PointStamped) -> None:
            state = self.states[robot]
            state.x = float(msg.point.x)
            state.y = float(msg.point.y)
            state.gps_ok = True
            state.last_update = time.time()

        return callback

    def _make_compass_callback(self, robot: str):
        def callback(msg: FloatStamped) -> None:
            state = self.states[robot]
            state.theta = wrap_angle(math.radians(float(msg.data)))
            state.compass_ok = True
            state.last_update = time.time()

        return callback

    def publish_cmd(self, robot: str, vx: float, vy: float, wz: float) -> None:
        if robot not in self.cmd_pubs:
            raise ValueError(f"unknown robot: {robot}")
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pubs[robot].publish(msg)

    def stop_robot(self, robot: str) -> None:
        self.publish_cmd(robot, 0.0, 0.0, 0.0)

    def wait_for_pose(self, robot: str, timeout: float = 5.0) -> RobotState:
        deadline = time.time() + timeout
        while time.time() < deadline:
            state = self.states.get(robot)
            if state and state.gps_ok and state.compass_ok:
                return state
            time.sleep(0.05)
        raise RuntimeError(f"pose for {robot} is not available")


class RosMcpServer:
    def __init__(self) -> None:
        rclpy.init(args=None)
        self.node = RosToolNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.processes: Dict[str, subprocess.Popen] = {}
        self.log_dir = os.environ.get("PART5_MCP_LOG_DIR", "/mnt/d/桌面/ros机器人task")

    def serve(self) -> None:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
                response = self.handle_request(request)
                if response is not None:
                    self.write_response(response)
            except Exception as exc:
                self.write_response(
                    {
                        "jsonrpc": "2.0",
                        "id": None,
                        "error": {"code": -32603, "message": str(exc)},
                    }
                )

    def write_response(self, response: Dict[str, Any]) -> None:
        sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
        sys.stdout.flush()

    def handle_request(self, request: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        method = request.get("method")
        request_id = request.get("id")

        if method == "initialize":
            return {
                "jsonrpc": "2.0",
                "id": request_id,
                "result": {
                    "protocolVersion": PROTOCOL_VERSION,
                    "capabilities": {"tools": {}},
                    "serverInfo": {
                        "name": "webots-ros2-robomaster-mcp",
                        "version": "0.1.0",
                    },
                },
            }
        if method == "notifications/initialized":
            return None
        if method == "tools/list":
            return {"jsonrpc": "2.0", "id": request_id, "result": {"tools": self.tools()}}
        if method == "tools/call":
            params = request.get("params", {})
            result = self.call_tool(params.get("name"), params.get("arguments", {}))
            return {
                "jsonrpc": "2.0",
                "id": request_id,
                "result": {"content": [{"type": "text", "text": result}]},
            }
        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "error": {"code": -32601, "message": f"unknown method: {method}"},
        }

    def tools(self) -> list:
        return [
            self._tool(
                "get_robot_status",
                "Return GPS position, heading, and active MCP-managed tasks.",
                {"robot": {"type": "string", "enum": ["agent0", "agent1", "all"]}},
            ),
            self._tool(
                "leader_move_forward",
                "Move leader agent0 forward by a distance in meters using GPS/compass feedback.",
                {"distance_m": {"type": "number"}, "speed": {"type": "number"}},
            ),
            self._tool(
                "leader_turn",
                "Turn leader agent0 by angle_deg. Positive is left/counter-clockwise.",
                {"angle_deg": {"type": "number"}, "angular_speed": {"type": "number"}},
            ),
            self._tool(
                "leader_rotate_once",
                "Rotate leader agent0 once in place.",
                {"direction": {"type": "string", "enum": ["left", "right"]}},
            ),
            self._tool(
                "navigate_follower_to",
                "Start Part2 PID point-to-point navigation for follower agent1.",
                {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                    "theta": {"type": "number"},
                },
            ),
            self._tool(
                "track_follower_trajectory",
                "Start Part3 trajectory tracking for follower agent1.",
                {
                    "trajectory_type": {"type": "string", "enum": ["line", "square", "circle"]},
                    "radius": {"type": "number"},
                    "size": {"type": "number"},
                    "center_x": {"type": "number"},
                    "center_y": {"type": "number"},
                    "start_x": {"type": "number"},
                    "start_y": {"type": "number"},
                    "end_x": {"type": "number"},
                    "end_y": {"type": "number"},
                },
            ),
            self._tool(
                "start_target_following",
                "Start Part4 visual target following: agent1 follows agent0 ArUco tag.",
                {"distance": {"type": "number"}},
            ),
            self._tool(
                "stop_robot",
                "Stop one robot and terminate MCP-managed follower task if needed.",
                {"robot": {"type": "string", "enum": ["agent0", "agent1"]}},
            ),
            self._tool("stop_all", "Stop both robots and terminate all MCP-managed tasks.", {}),
        ]

    def _tool(self, name: str, description: str, properties: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "name": name,
            "description": description,
            "inputSchema": {
                "type": "object",
                "properties": properties,
                "additionalProperties": False,
            },
        }

    def call_tool(self, name: str, args: Dict[str, Any]) -> str:
        if name == "get_robot_status":
            return self.get_robot_status(args.get("robot", "all"))
        if name == "leader_move_forward":
            return self.leader_move_forward(float(args.get("distance_m", 1.0)), float(args.get("speed", 0.2)))
        if name == "leader_turn":
            return self.leader_turn(float(args.get("angle_deg", 90.0)), float(args.get("angular_speed", 0.5)))
        if name == "leader_rotate_once":
            return self.leader_rotate_once(str(args.get("direction", "left")))
        if name == "navigate_follower_to":
            theta = args.get("theta", None)
            return self.navigate_follower_to(float(args.get("x", 0.0)), float(args.get("y", 0.0)), theta)
        if name == "track_follower_trajectory":
            return self.track_follower_trajectory(args)
        if name == "start_target_following":
            return self.start_target_following(float(args.get("distance", 0.5)))
        if name == "stop_robot":
            return self.stop_robot(str(args.get("robot", "agent1")))
        if name == "stop_all":
            return self.stop_all()
        raise ValueError(f"unknown tool: {name}")

    def get_robot_status(self, robot: str = "all") -> str:
        robots = ["agent0", "agent1"] if robot == "all" else [robot]
        lines = []
        for name in robots:
            try:
                state = self.node.wait_for_pose(name, timeout=2.0)
            except RuntimeError:
                state = self.node.states.get(name)
            if state is None:
                lines.append(f"{name}: unknown")
                continue
            ok = state.gps_ok and state.compass_ok
            lines.append(
                f"{name}: pose_ok={ok} x={state.x:.3f} y={state.y:.3f} "
                f"theta={math.degrees(state.theta):.1f}deg"
            )
        active = ", ".join(sorted(self.processes)) or "none"
        lines.append(f"active_tasks={active}")
        return "\n".join(lines)

    def leader_move_forward(self, distance_m: float, speed: float = 0.2) -> str:
        reached, travelled = self._drive_robot_forward_distance(
            "agent0",
            distance_m,
            max_speed=abs(speed),
        )
        status = "reached" if reached else "timeout"
        return f"leader moved forward {distance_m:.2f}m ({status}, travelled={travelled:.2f}m)"

    def leader_turn(self, angle_deg: float, angular_speed: float = 0.5) -> str:
        state = self.node.wait_for_pose("agent0")
        target = wrap_angle(state.theta + math.radians(angle_deg))
        reached = self._turn_robot_to("agent0", target, max_w=abs(angular_speed))
        status = "reached" if reached else "command_sent"
        return f"leader turned {angle_deg:.1f}deg ({status})"

    def leader_rotate_once(self, direction: str = "left") -> str:
        sign = 1.0 if direction != "right" else -1.0
        speed = 0.5
        duration = 2.0 * math.pi / speed
        end_time = time.time() + duration
        while time.time() < end_time:
            self.node.publish_cmd("agent0", 0.0, 0.0, sign * speed)
            time.sleep(0.05)
        self.node.stop_robot("agent0")
        return f"leader rotated once toward {direction}"

    def navigate_follower_to(self, x: float, y: float, theta: Optional[Any] = None) -> str:
        self.stop_follower_tasks()
        params = [
            "-p", f"target_x:={x}",
            "-p", f"target_y:={y}",
            "-p", "gps_topic:=/agent1/gps",
            "-p", "compass_topic:=/agent1/compass/bearing",
            "-p", "cmd_topic:=/agent1/cmd_vel",
            "-p", "pose_source:=gps",
        ]
        if theta is not None:
            params.extend(["-p", f"target_theta:={float(theta)}"])
        self.processes["follower_pid"] = self._launch_ros_node("pid_controller", params)
        return f"started follower point navigation to ({x:.2f}, {y:.2f})"

    def track_follower_trajectory(self, args: Dict[str, Any]) -> str:
        self.stop_follower_tasks()
        traj = str(args.get("trajectory_type", "circle"))
        params = [
            "-p", f"trajectory_type:={traj}",
            "-p", "gps_topic:=/agent1/gps",
            "-p", "compass_topic:=/agent1/compass/bearing",
            "-p", "cmd_topic:=/agent1/cmd_vel",
        ]
        if traj == "circle":
            params.extend(["-p", f"circle_radius:={float(args.get('radius', 0.6))}"])
            params.extend(["-p", f"circle_center_x:={float(args.get('center_x', 0.0))}"])
            params.extend(["-p", f"circle_center_y:={float(args.get('center_y', 0.0))}"])
        elif traj == "square":
            params.extend(["-p", f"square_size:={float(args.get('size', 1.0))}"])
            params.extend(["-p", f"square_center_x:={float(args.get('center_x', 0.0))}"])
            params.extend(["-p", f"square_center_y:={float(args.get('center_y', 0.0))}"])
        elif traj == "line":
            params.extend(["-p", f"start_x:={float(args.get('start_x', 0.0))}"])
            params.extend(["-p", f"start_y:={float(args.get('start_y', 0.0))}"])
            params.extend(["-p", f"end_x:={float(args.get('end_x', 1.0))}"])
            params.extend(["-p", f"end_y:={float(args.get('end_y', 0.0))}"])
        else:
            raise ValueError("trajectory_type must be line, square, or circle")
        self.processes["follower_trajectory"] = self._launch_ros_node("trajectory_tracker", params)
        return f"started follower {traj} trajectory"

    def start_target_following(self, distance: float = 0.5) -> str:
        self.stop_follower_tasks()
        prealign_status = self._prepare_visual_following_start(distance)
        params = [
            "-p", "robot_name:=agent1",
            "-p", "aruco_dict:=DICT_6X6_250",
            "-p", f"target_distance:={distance}",
            "-p", "image_timeout:=1.5",
            "-p", "search_when_never_seen:=true",
            "-p", "search_angular_speed:=0.45",
            "-p", "debug:=false",
        ]
        self.processes["target_follower"] = self._launch_ros_node("target_follower", params)
        return f"started visual target following at {distance:.2f}m ({prealign_status})"

    def _prepare_visual_following_start(self, distance: float) -> str:
        """Use GPS/compass for coarse acquisition before visual servoing starts."""
        try:
            leader = self.node.wait_for_pose("agent0", timeout=12.0)
            follower = self.node.wait_for_pose("agent1", timeout=12.0)
        except RuntimeError as exc:
            return f"pre_align=skipped ({exc}; run status once or wait for Webots controllers)"

        stand_off = max(float(distance) + 0.15, 0.65)
        desired_x = leader.x - stand_off * math.cos(leader.theta)
        desired_y = leader.y - stand_off * math.sin(leader.theta)
        start_error = math.hypot(follower.x - desired_x, follower.y - desired_y)

        moved = True
        if start_error > 0.12:
            moved = self._drive_robot_to("agent1", desired_x, desired_y, max_speed=0.22)

        face_theta = math.atan2(leader.y - desired_y, leader.x - desired_x)
        turned = self._turn_robot_to("agent1", face_theta, max_w=0.8)

        move_status = "move_ok" if moved else "move_timeout"
        turn_status = "turn_ok" if turned else "turn_timeout"
        return (
            f"pre_align={move_status},{turn_status},"
            f"target=({desired_x:.2f},{desired_y:.2f})"
        )

    def stop_robot(self, robot: str) -> str:
        if robot == "agent1":
            self.stop_follower_tasks()
        self.node.stop_robot(robot)
        return f"stopped {robot}"

    def stop_all(self) -> str:
        self.stop_follower_tasks()
        self.node.stop_robot("agent0")
        self.node.stop_robot("agent1")
        return "stopped all robots and MCP-managed tasks"

    def _launch_ros_node(self, executable: str, ros_params: list) -> subprocess.Popen:
        cmd = ["ros2", "run", "webots_ros2_robomaster", executable, "--ros-args"] + ros_params
        env = os.environ.copy()
        os.makedirs(self.log_dir, exist_ok=True)
        log_path = os.path.join(self.log_dir, f"part5_{executable}.log")
        log_file = open(log_path, "a", encoding="utf-8")
        log_file.write(f"\\n--- launch: {' '.join(cmd)} ---\\n")
        log_file.flush()
        return subprocess.Popen(
            cmd,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )

    def stop_follower_tasks(self) -> None:
        for name in ["follower_pid", "follower_trajectory", "target_follower"]:
            proc = self.processes.pop(name, None)
            if proc is None:
                continue
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        self.node.stop_robot("agent1")

    def _drive_robot_forward_distance(self, robot: str, distance_m: float, max_speed: float = 0.18) -> tuple[bool, float]:
        start = self.node.wait_for_pose(robot)
        start_x = start.x
        start_y = start.y
        target_distance = abs(distance_m)
        if target_distance < 0.01:
            self.node.stop_robot(robot)
            return True, 0.0

        direction = 1.0 if distance_m >= 0.0 else -1.0
        speed = max(0.05, min(abs(max_speed), 0.25))
        deadline = time.time() + max(30.0, target_distance / speed * 8.0 + 12.0)
        travelled = 0.0
        reached = False
        while time.time() < deadline:
            state = self.node.wait_for_pose(robot, timeout=1.0)
            travelled = math.hypot(state.x - start_x, state.y - start_y)
            if travelled >= target_distance - 0.03:
                reached = True
                break
            remaining = target_distance - travelled
            cmd_speed = speed if remaining > 0.08 else min(speed, max(0.08, 0.8 * remaining))
            self.node.publish_cmd(robot, direction * cmd_speed, 0.0, 0.0)
            time.sleep(0.05)
        self.node.stop_robot(robot)
        return reached, travelled

    def _drive_robot_to(self, robot: str, target_x: float, target_y: float, max_speed: float = 0.18) -> bool:
        tol = 0.05
        kp = 0.8
        reached = False
        deadline = time.time() + 30.0
        while time.time() < deadline:
            state = self.node.wait_for_pose(robot, timeout=1.0)
            ex_w = target_x - state.x
            ey_w = target_y - state.y
            rho = math.hypot(ex_w, ey_w)
            if rho < tol:
                reached = True
                break
            cos_t = math.cos(state.theta)
            sin_t = math.sin(state.theta)
            ex_r = ex_w * cos_t + ey_w * sin_t
            ey_r = -ex_w * sin_t + ey_w * cos_t
            vx = clamp(kp * ex_r, -max_speed, max_speed)
            vy = clamp(kp * ey_r, -max_speed, max_speed)
            total = math.hypot(vx, vy)
            if total > max_speed:
                scale = max_speed / total
                vx *= scale
                vy *= scale
            self.node.publish_cmd(robot, vx, vy, 0.0)
            time.sleep(0.05)
        self.node.stop_robot(robot)
        return reached

    def _turn_robot_to(self, robot: str, target_theta: float, max_w: float = 0.5) -> bool:
        tol = math.radians(2.0)
        kp = 1.4
        reached = False
        deadline = time.time() + 20.0
        while time.time() < deadline:
            state = self.node.wait_for_pose(robot, timeout=1.0)
            err = wrap_angle(target_theta - state.theta)
            if abs(err) < tol:
                reached = True
                break
            wz = clamp(kp * err, -max_w, max_w)
            if 0.0 < abs(wz) < 0.08:
                wz = 0.08 if wz > 0 else -0.08
            self.node.publish_cmd(robot, 0.0, 0.0, wz)
            time.sleep(0.05)
        self.node.stop_robot(robot)
        return reached

    def shutdown(self) -> None:
        try:
            self.stop_all()
        finally:
            self.executor.shutdown()
            self.spin_thread.join(timeout=2.0)
            self.node.destroy_node()
            rclpy.shutdown()


def main() -> None:
    server = RosMcpServer()
    try:
        server.serve()
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
