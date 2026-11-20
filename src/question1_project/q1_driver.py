#!/usr/bin/env python3
"""题一：关节分步驱动与关键帧记录。

流程固定为 lift -> head -> arm -> gripper_open -> gripper_close，
目标姿态由 seed 在关节限位内均匀采样，保证输入可复现。
每段按 move_sec 缓动插值发布 /joint_states，到位后保持 pause_sec 再采样。
"""
import argparse
import json
import random
import time
from pathlib import Path

JOINTS = [
    "lift_joint", "pitch_joint",
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    "arm_finger_joint1", "arm_finger_joint2",
]
SEGMENTS = ["lift", "head", "arm", "gripper_open", "gripper_close"]
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
FINGERS = ["arm_finger_joint1", "arm_finger_joint2"]

PUBLISH_HZ = 50.0


def round6(value):
    return float(f"{value:.6f}")


class RosDriver:
    """向 /joint_states 发布完整 10 关节状态。"""

    def __init__(self):
        import rclpy
        from sensor_msgs.msg import JointState

        rclpy.init()
        self._rclpy = rclpy
        self._JointState = JointState
        self.node = rclpy.create_node("q1_driver")
        self.publisher = self.node.create_publisher(JointState, "/joint_states", 10)

    def publish(self, state):
        msg = self._JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [float(state[j]) for j in JOINTS]
        self.publisher.publish(msg)
        self._rclpy.spin_once(self.node, timeout_sec=0)

    def close(self):
        self.node.destroy_node()
        self._rclpy.shutdown()


class NullDriver:
    """无 ROS 模式：只计算状态，不发布、不等待。"""

    def publish(self, state):
        pass

    def close(self):
        pass


def hold(driver, state, seconds):
    """保持状态并持续发布，保证 RViz 中动作连续。"""
    if seconds <= 0:
        driver.publish(state)
        return
    steps = max(1, int(seconds * PUBLISH_HZ))
    for _ in range(steps):
        driver.publish(state)
        time.sleep(1.0 / PUBLISH_HZ)


def move_to(driver, start, goal, seconds):
    """从 start 平滑插值到 goal，沿路发布。

    用 smoothstep（3a^2-2a^3）代替线性，起停速度为 0，
    避免大角度段一启动就满速、看起来像突然弹出去。
    """
    steps = max(1, int(seconds * PUBLISH_HZ))
    for i in range(1, steps + 1):
        a = i / steps
        alpha = a * a * (3.0 - 2.0 * a)
        frame = {j: round6(start[j] + (goal[j] - start[j]) * alpha) for j in JOINTS}
        driver.publish(frame)
        time.sleep(1.0 / PUBLISH_HZ)


def plan_targets(seed, limits):
    """按 seed 在限位内均匀采样每段的目标姿态。"""
    rng = random.Random(seed)
    targets = {name: {} for name in SEGMENTS}

    bound = limits["lift_joint"]
    targets["lift"]["lift_joint"] = round6(rng.uniform(bound["lower"], bound["upper"]))

    bound = limits["pitch_joint"]
    targets["head"]["pitch_joint"] = round6(rng.uniform(bound["lower"], bound["upper"]))

    for joint in ARM_JOINTS:
        bound = limits[joint]
        targets["arm"][joint] = round6(rng.uniform(bound["lower"], bound["upper"]))

    finger_open = round6(limits["arm_finger_joint1"]["upper"])
    for finger in FINGERS:
        targets["gripper_open"][finger] = finger_open
        targets["gripper_close"][finger] = 0.0

    return targets


def run_record(record, limits, driver, paced):
    move_sec = float(record["move_sec"])
    pause_sec = float(record["pause_sec"])
    targets = plan_targets(int(record["seed"]), limits)

    state = {j: 0.0 for j in JOINTS}
    driver.publish(state)
    keyframes = []

    for name in SEGMENTS:
        goal = dict(state)
        goal.update(targets[name])
        if paced:
            move_to(driver, state, goal, move_sec)
            state = goal
            hold(driver, state, pause_sec)
        else:
            state = goal
        keyframes.append({"name": name, "joint_state": dict(state)})

    return {
        "id": record["id"],
        "step": record["step"],
        "keyframes": keyframes,
        "final_joint_state": dict(state),
    }


def main():
    parser = argparse.ArgumentParser(description="题一：关节分步驱动")
    parser.add_argument("--input", required=True, help="输入 jsonl 路径")
    parser.add_argument("--output", required=True, help="输出 jsonl 路径")
    parser.add_argument(
        "--limits",
        default=str(Path(__file__).resolve().parent / "config" / "joint_limits.json"),
        help="关节限位文件",
    )
    parser.add_argument("--ros", action="store_true", help="发布 /joint_states 供 RViz 显示")
    parser.add_argument(
        "--warmup",
        type=float,
        default=5.0,
        help="ROS 模式下先保持零位并发布的秒数，避免录像开头出现姿态跳变",
    )
    args = parser.parse_args()

    limits = json.loads(Path(args.limits).read_text(encoding="utf-8"))
    driver = RosDriver() if args.ros else NullDriver()
    try:
        if args.ros and args.warmup > 0:
            hold(driver, {j: 0.0 for j in JOINTS}, args.warmup)
            print(f"预热 {args.warmup:.1f}s 完成，开始动作", flush=True)
        results = []
        for line in Path(args.input).read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("step") != "all":
                raise ValueError(f"不支持的 step: {record.get('step')!r}，题面只定义 all")
            results.append(run_record(record, limits, driver, paced=args.ros))
    finally:
        driver.close()

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as fh:
        for item in results:
            fh.write(json.dumps(item, ensure_ascii=False) + "\n")
    print(f"已写入 {len(results)} 行 -> {out}")


if __name__ == "__main__":
    main()
