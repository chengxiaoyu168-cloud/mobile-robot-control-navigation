"""
Small MCP demo client for Part 5 validation.

This script launches ros_mcp_server over stdio, sends initialize + one tool call,
prints JSON-RPC responses, and optionally keeps the server alive for a few
seconds so long-running tasks such as target following can be observed.
"""

import argparse
import json
import re
import subprocess
import sys
import time
from typing import Any, Dict


KNOWN_COMMANDS = {
    "status",
    "leader-forward",
    "leader-turn",
    "leader-rotate-once",
    "follower-goto",
    "follower-trajectory",
    "target-follow",
    "stop",
    "stop-all",
    "nl",
}


def normalize_chinese_number(text: str) -> str:
    table = {
        "零": "0",
        "一": "1",
        "二": "2",
        "两": "2",
        "三": "3",
        "四": "4",
        "五": "5",
        "六": "6",
        "七": "7",
        "八": "8",
        "九": "9",
    }
    for cn, digit in table.items():
        text = text.replace(cn, digit)
    text = text.replace("十", "10")
    return text


def first_number(text: str, default: float) -> float:
    text = normalize_chinese_number(text)
    match = re.search(r"[-+]?\d+(?:\.\d+)?", text)
    return float(match.group(0)) if match else default


def build_natural_language_tool_call(sentence: str) -> Dict[str, Any]:
    compact = sentence.replace(" ", "")
    compact = compact.replace("，", ",").replace("。", ".")

    if any(word in compact for word in ["状态", "位置", "位姿", "在哪"]):
        return {"name": "get_robot_status", "arguments": {"robot": "all"}}

    if any(word in compact for word in ["停止", "停车", "停下"]):
        if "领航" in compact or "leader" in compact or "agent0" in compact:
            return {"name": "stop_robot", "arguments": {"robot": "agent0"}}
        if "跟随" in compact or "follower" in compact or "agent1" in compact:
            return {"name": "stop_robot", "arguments": {"robot": "agent1"}}
        return {"name": "stop_all", "arguments": {}}

    if "绕当前点" in compact or "旋转一圈" in compact or "转一圈" in compact:
        direction = "right" if any(word in compact for word in ["右", "顺时针"]) else "left"
        return {"name": "leader_rotate_once", "arguments": {"direction": direction}}

    if any(word in compact for word in ["左转", "向左转", "逆时针"]):
        return {
            "name": "leader_turn",
            "arguments": {"angle_deg": first_number(compact, 90.0), "angular_speed": 0.5},
        }

    if any(word in compact for word in ["右转", "向右转", "顺时针"]):
        return {
            "name": "leader_turn",
            "arguments": {"angle_deg": -first_number(compact, 90.0), "angular_speed": 0.5},
        }

    if any(word in compact for word in ["后退", "向后", "倒退"]):
        return {
            "name": "leader_move_forward",
            "arguments": {"distance_m": -first_number(compact, 0.2), "speed": 0.2},
        }

    if any(word in compact for word in ["向前", "前进", "往前", "前移"]):
        return {
            "name": "leader_move_forward",
            "arguments": {"distance_m": first_number(compact, 0.2), "speed": 0.2},
        }

    if any(word in compact for word in ["开始跟随", "目标跟踪", "动态跟随", "跟随领航者"]):
        return {"name": "start_target_following", "arguments": {"distance": first_number(compact, 0.5)}}

    raise ValueError(
        "无法解析自然语言指令。示例：向前移动0.2米 / 向左转30度 / 绕当前点旋转一圈 / 开始动态跟随"
    )


def send(proc: subprocess.Popen, request: Dict[str, Any]) -> Dict[str, Any]:
    assert proc.stdin is not None
    assert proc.stdout is not None
    proc.stdin.write(json.dumps(request, ensure_ascii=False) + "\n")
    proc.stdin.flush()
    line = proc.stdout.readline()
    if not line:
        raise RuntimeError("MCP server closed stdout")
    return json.loads(line)


def build_tool_call(args: argparse.Namespace) -> Dict[str, Any]:
    command = args.command
    if command == "nl":
        return build_natural_language_tool_call(args.sentence)
    if command == "status":
        return {"name": "get_robot_status", "arguments": {"robot": args.robot}}
    if command == "leader-forward":
        return {
            "name": "leader_move_forward",
            "arguments": {"distance_m": args.distance, "speed": args.speed},
        }
    if command == "leader-turn":
        return {
            "name": "leader_turn",
            "arguments": {"angle_deg": args.angle, "angular_speed": args.angular_speed},
        }
    if command == "leader-rotate-once":
        return {"name": "leader_rotate_once", "arguments": {"direction": args.direction}}
    if command == "follower-goto":
        payload = {"x": args.x, "y": args.y}
        if args.theta is not None:
            payload["theta"] = args.theta
        return {"name": "navigate_follower_to", "arguments": payload}
    if command == "follower-trajectory":
        payload = {
            "trajectory_type": args.trajectory_type,
            "radius": args.radius,
            "size": args.size,
            "center_x": args.center_x,
            "center_y": args.center_y,
            "start_x": args.start_x,
            "start_y": args.start_y,
            "end_x": args.end_x,
            "end_y": args.end_y,
        }
        return {"name": "track_follower_trajectory", "arguments": payload}
    if command == "target-follow":
        return {"name": "start_target_following", "arguments": {"distance": args.distance}}
    if command == "stop":
        return {"name": "stop_robot", "arguments": {"robot": args.robot}}
    if command == "stop-all":
        return {"name": "stop_all", "arguments": {}}
    raise ValueError(f"unknown command: {command}")


def main() -> None:
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-") and sys.argv[1] not in KNOWN_COMMANDS:
        sys.argv = [sys.argv[0], "nl", " ".join(sys.argv[1:])]

    parser = argparse.ArgumentParser(description="Part5 MCP demo client")
    parser.add_argument("--hold", type=float, default=0.0, help="Keep server alive after tool call")

    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("nl", help="Parse a Chinese natural-language command into an MCP tool call")
    p.add_argument("sentence", help="例如：向前移动0.2米 / 向左转30度 / 绕当前点旋转一圈")

    p = sub.add_parser("status")
    p.add_argument("--robot", default="all", choices=["agent0", "agent1", "all"])

    p = sub.add_parser("leader-forward")
    p.add_argument("distance", type=float)
    p.add_argument("--speed", type=float, default=0.18)

    p = sub.add_parser("leader-turn")
    p.add_argument("angle", type=float)
    p.add_argument("--angular-speed", type=float, default=0.5)

    p = sub.add_parser("leader-rotate-once")
    p.add_argument("--direction", default="left", choices=["left", "right"])

    p = sub.add_parser("follower-goto")
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("--theta", type=float, default=None)

    p = sub.add_parser("follower-trajectory")
    p.add_argument("trajectory_type", choices=["line", "square", "circle"])
    p.add_argument("--radius", type=float, default=0.6)
    p.add_argument("--size", type=float, default=1.0)
    p.add_argument("--center-x", type=float, default=0.0)
    p.add_argument("--center-y", type=float, default=0.0)
    p.add_argument("--start-x", type=float, default=0.0)
    p.add_argument("--start-y", type=float, default=0.0)
    p.add_argument("--end-x", type=float, default=1.0)
    p.add_argument("--end-y", type=float, default=0.0)

    p = sub.add_parser("target-follow")
    p.add_argument("--distance", type=float, default=0.5)

    p = sub.add_parser("stop")
    p.add_argument("--robot", default="agent1", choices=["agent0", "agent1"])

    sub.add_parser("stop-all")

    args = parser.parse_args()

    proc = subprocess.Popen(
        ["ros2", "run", "webots_ros2_robomaster", "ros_mcp_server"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )

    try:
        init_response = send(
            proc,
            {
                "jsonrpc": "2.0",
                "id": 1,
                "method": "initialize",
                "params": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {},
                    "clientInfo": {"name": "part5-demo-client", "version": "0.1"},
                },
            },
        )
        print(json.dumps(init_response, ensure_ascii=False, indent=2))

        tool = build_tool_call(args)
        if args.command == "nl":
            print(f"[NL] {args.sentence} -> {json.dumps(tool, ensure_ascii=False)}")
        call_response = send(
            proc,
            {"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": tool},
        )
        print(json.dumps(call_response, ensure_ascii=False, indent=2))

        if args.hold > 0:
            print(f"Holding MCP server for {args.hold:.1f}s...")
            time.sleep(args.hold)
    finally:
        proc.terminate()
        try:
            _, stderr = proc.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            _, stderr = proc.communicate(timeout=3.0)
        if stderr:
            print(stderr, file=sys.stderr)


if __name__ == "__main__":
    main()
