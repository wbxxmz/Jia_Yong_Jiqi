#!/usr/bin/env python3
"""题一：关节分步驱动与关键帧记录。

流程固定为官方要求的五段：lift -> head -> arm -> gripper_open -> gripper_close。
动作编排成一套完整展示（不依赖随机采样，画面每次一致、可复现）：

1. 起始：肩膀折叠，大臂抬起、小臂折回，夹爪收在肩后；
2. lift：升降抬到 0.15 米；
3. head：头部俯仰 -0.25 弧度；
4. arm：先用 60% 时间把肩膀伸展开（joint2/joint3 回零、手臂伸直），
   再用 40% 时间旋转底座（joint1=-0.8）并下压手腕（joint5=-0.8）倾斜；
5. gripper_open：两指张开到限位 0.506 弧度；
6. gripper_close：两指合拢，随后手臂收回起始折叠状态，作为流程结束姿态。

keyframes 仍按官方顺序输出，每段到位并保持 pause_sec 后采样；
final_joint_state 与最后一个关键帧一致。
"""
import argparse
import json
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

# 折叠姿态：大臂抬起、小臂折回，指尖落在肩后下方，属于“肩膀闭合”状态。
FOLDED_ARM = {
    "joint1": 0.0,
    "joint2": 1.5,
    "joint3": -2.5,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": 0.0,
}
# 展示姿态：底座左旋、大臂略抬、手腕下压，指尖在身前偏右、高度约 0.76 米。
ARM_TILT = {
    "joint1": -0.8,
    "joint2": 0.4,
    "joint3": 0.0,
    "joint4": 0.0,
    "joint5": -0.8,
    "joint6": 0.0,
}
LIFT_HEIGHT = 0.15
HEAD_PITCH = -0.25


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
    """起始状态：肩膀折叠、升降与头部件在零位、夹爪闭合。"""
    state = {j: 0.0 for j in JOINTS}
    state.update(FOLDED_ARM)
    return state


def run_record(record, limits, driver, paced):
    move_sec = float(record["move_sec"])
    pause_sec = float(record["pause_sec"])
    finger_open = round6(limits["arm_finger_joint1"]["upper"])

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
            goal["lift_joint"] = LIFT_HEIGHT
            advance(goal)
        elif name == "head":
            goal = dict(state)
            goal["pitch_joint"] = HEAD_PITCH
            advance(goal)
        elif name == "arm":
            # 第一步：肩膀伸展，把小臂从折叠状态伸直。
            straight = dict(state)
            straight.update({"joint2": 0.0, "joint3": 0.0})
            advance(straight, 0.6)
            # 第二步：底座旋转、手腕下压，倾斜到展示位置。
            tilted = dict(state)
            tilted.update(ARM_TILT)
            advance(tilted, 0.4)
        elif name == "gripper_open":
            goal = dict(state)
            for finger in FINGERS:
                goal[finger] = finger_open
            advance(goal)
        else:  # gripper_close
            # 先合拢夹爪，再收回肩膀折叠状态，作为流程结束姿态。
            closed = dict(state)
            for finger in FINGERS:
                closed[finger] = 0.0
            advance(closed, 0.35)
            folded = dict(closed)
            folded.update(FOLDED_ARM)
            advance(folded, 0.65)

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
        help="ROS 模式下先保持起始折叠姿态并发布的秒数，避免录像开头出现姿态跳变",
    )
    args = parser.parse_args()

    limits = json.loads(Path(args.limits).read_text(encoding="utf-8"))
    driver = RosDriver() if args.ros else NullDriver()
    try:
        if args.ros and args.warmup > 0:
            start = initial_state()
            hold(driver, start, args.warmup)
            print(f"预热 {args.warmup:.1f}s 完成（起始折叠姿态），开始动作", flush=True)
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
