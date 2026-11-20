"""把官方 URDF 展开成可直接喂给 robot_state_publisher 的字符串。

官方 xacro 通过 $(find bajie_robot) 引用自身，正常做法是 colcon 构建。
这里不改官方仓库，而是临时造一个 ament 前缀（软链指向官方 urdf 目录），
用子进程跑 xacro，再把 package:// 改成绝对 file://，RViz 无需安装工作空间。
"""
import os
import subprocess
import tempfile
from pathlib import Path


def expand_urdf(urdf_root):
    urdf_root = Path(urdf_root).resolve()
    xacro_file = urdf_root / "urdf" / "bajie_robot.urdf.xacro"
    if not xacro_file.is_file():
        raise FileNotFoundError(f"找不到 {xacro_file}")

    prefix = Path(tempfile.mkdtemp(prefix="bajie_ament_"))
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "bajie_robot").touch()
    os.symlink(urdf_root, prefix / "share" / "bajie_robot")

    env = dict(os.environ)
    env["AMENT_PREFIX_PATH"] = f"{prefix}:{env.get('AMENT_PREFIX_PATH', '')}"
    result = subprocess.run(
        ["xacro", xacro_file.name],
        cwd=str(xacro_file.parent),
        env=env,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"xacro 展开失败:\n{result.stderr.strip()}")

    return result.stdout.replace("package://bajie_robot/", f"file://{urdf_root}/")
