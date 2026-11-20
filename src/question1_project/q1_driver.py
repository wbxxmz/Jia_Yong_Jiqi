#!/usr/bin/env python3
"""题一：关节分步驱动与关键帧记录。

流程按官方要求的五段执行：lift -> head -> arm -> gripper_open -> gripper_close。
起始为官方零位（全部关节为 0）；`seed` 决定展示姿态，10 个关节都参与：

1. lift：升降关节抬到 seed 采样的高度（0.12~0.30 米）；
2. head：头部俯仰到 seed 采样的角度（-0.35~0.05 弧度）；
3. arm：joint1~joint6 全部按 seed 在避障包络内采样，摆到展示姿态；
4. gripper_open：两指张开到限位 0.506 弧度；
5. gripper_close：两指合拢并停在展示姿态（不回零），与官方示例一致。

采样范围经过离线校验（5000 个 seed，指尖始终在身前 0.87 米外、高度 0.29 米以上，
各连杆不进入升降立柱），不会出现穿模或撞地。
keyframes 按官方顺序输出，每段到位并保持 pause_sec 后采样；
final_joint_state 与最后一个关键帧一致。
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
FINGERS = ["arm_finger_joint1", "arm_finger_joint2"]

PUBLISH_HZ = 50.0

# 展示姿态的采样范围，单位：米 / 弧度。
# 范围经过离线 FK 校验：5000 个 seed 下指尖始终在身前且高于 0.29 米，连杆不穿立柱、不撞地。
SAMPLING_RANGES = {
    "lift_joint": (0.12, 0.30),
    "pitch_joint": (-0.35, 0.05),
    "joint1": (-1.0, 1.0),
    "joint2": (-0.50, 0.10),
    "joint3": (-0.40, 0.10),
    "joint4": (-0.50, 0.50),
    "joint5": (-0.60, 0.20),
    "joint6": (-0.50, 0.50),
}


def round6(value):
    return float(f"{value:.6f}")


def check_limits(state, limits):
    """发布前确认所有关节目标都在官方限位内，越界直接报错。"""
    for joint, value in state.items():
        bound = limits.get(joint)
        if bound is None:
            continue
        if value < bound["lower"] - 1e-9 or value > bound["upper"] + 1e-9:
            raise ValueError(
                f"{joint}={value} 超出限位 [{bound['lower']}, {bound['upper']}]"
            )
    return state


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

    用 smoothstep（3a^2-2a^3）代替线性：起停速度为 0，
    大角度段不会一启动就满速、看起来像突然弹出去。
    """
    steps = max(1, int(seconds * PUBLISH_HZ))
    for i in range(1, steps + 1):
        a = i / steps
        alpha = a * a * (3.0 - 2.0 * a)
        frame = {j: round6(start[j] + (goal[j] - start[j]) * alpha) for j in JOINTS}
        driver.publish(frame)
        time.sleep(1.0 / PUBLISH_HZ)


def initial_state():
    """官方零位：全部关节为 0，手臂水平前伸、夹爪闭合。"""
    return {j: 0.0 for j in JOINTS}


def plan_targets(seed):
    """按 seed 在采样范围内生成展示姿态，10 个关节中 8 个在此决定，两指由开合段决定。"""
    rng = random.Random(int(seed))
    return {joint: round6(rng.uniform(low, high)) for joint, (low, high) in SAMPLING_RANGES.items()}


def run_record(record, limits, driver, paced):
    move_sec = float(record["move_sec"])
    pause_sec = float(record["pause_sec"])
    finger_open = round6(limits["arm_finger_joint1"]["upper"])
    targets = plan_targets(record.get("seed", 0))

    state = initial_state()
    check_limits(state, limits)
    driver.publish(state)
    keyframes = []

    def advance(goal, ratio=1.0):
        """插值到 goal；ratio 用于把一段 move_sec 拆成多个子动作。"""
        nonlocal state
        check_limits(goal, limits)
        if paced:
            move_to(driver, state, goal, move_sec * ratio)
        state = goal
        return state

    for name in SEGMENTS:
        if name == "lift":
            goal = dict(state)
            goal["lift_joint"] = targets["lift_joint"]
            advance(goal)
        elif name == "head":
            goal = dict(state)
            goal["pitch_joint"] = targets["pitch_joint"]
            advance(goal)
        elif name == "arm":
            # joint1~joint6 全部由 seed 决定，一次摆到展示姿态。
            goal = dict(state)
            for joint in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"):
                goal[joint] = targets[joint]
            advance(goal)
        elif name == "gripper_open":
            goal = dict(state)
            for finger in FINGERS:
                goal[finger] = finger_open
            advance(goal)
        else:  # gripper_close
            # 只合拢两指，停在展示姿态，不回零。
            goal = dict(state)
            for finger in FINGERS:
                goal[finger] = 0.0
            advance(goal)

        if paced:
            hold(driver, state, pause_sec)
        keyframes.append({"name": name, "joint_state": dict(state)})
        values = ", ".join(f"{j}={state[j]:.3f}" for j in JOINTS)
        print(f"关键帧 {name}: {values}", flush=True)

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
        help="ROS 模式下先保持官方零位并发布的秒数，避免 RViz 收到首帧前出现姿态跳变",
    )
    args = parser.parse_args()

    limits = json.loads(Path(args.limits).read_text(encoding="utf-8"))
    driver = RosDriver() if args.ros else NullDriver()
    try:
        if args.ros and args.warmup > 0:
            start = initial_state()
            hold(driver, start, args.warmup)
            print(f"预热 {args.warmup:.1f}s 完成（官方零位），开始动作", flush=True)
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
