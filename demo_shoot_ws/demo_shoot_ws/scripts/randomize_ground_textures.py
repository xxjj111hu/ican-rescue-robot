#!/usr/bin/env python3

"""随机替换射击任务点地面贴图，并初始化共享运行权限。

射击场有 5 个地面视觉点：0 号点为 task_point_0，小车无需导航到此点；
1～4 号点为实际射击任务点。每次从 5 类地面贴图中各选一张，再随机分配
到 0～4 号点。材质中的 ``rotate 90`` 保持不变，任务点几何、位姿和碰撞
不被修改。
"""

from __future__ import annotations

import argparse
import grp
import json
import logging
import os
import pwd
import random
import re
import shutil
import stat
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path


SHARED_GROUP = "ican_shoot_sim"
LOGGER = logging.getLogger("randomize_ground_textures")
LOG_FILE: Path | None = None
MATERIAL_NAMES = [
    "ShootingGroundTarget0",
    "ShootingGroundTarget1",
    "ShootingGroundTarget2",
    "ShootingGroundTarget3",
    "ShootingGroundTarget4",
]
WORLD_MATERIAL_MODELS = [
    ("task_point_0", "ShootingGroundTarget0"),
    ("task_point_1", "ShootingGroundTarget1"),
    ("task_point_2", "ShootingGroundTarget2"),
    ("task_point_3", "ShootingGroundTarget3"),
    ("task_point_4", "ShootingGroundTarget4"),
]
GROUND_TASK_GEOMETRY = {
    "task_point_0": ("1.115218 -1.356470 0.006 0 0 0", "0.52971 0.38454 0.012"),
    "task_point_1": ("0.030951 -1.359743 0.006 0 0 0", "0.52971 0.38454 0.012"),
    "task_point_2": ("0.027137 -2.700000 0.006 0 0 0", "0.52971 0.38454 0.012"),
    "task_point_3": ("1.120240 -1.913890 0.006 0 0 1.5708", "0.52971 0.38454 0.012"),
    "task_point_4": ("1.120112 -0.784766 0.006 0 0 1.5708", "0.52971 0.38454 0.012"),
}
GROUND_POOLS = {
    "collapsed_ruins": ["collapsed_ruins_01.png", "collapsed_ruins_02.jpg"],
    "landslide": ["landslide_01.jpg", "landslide_02.jpg"],
    "trapped_people": ["trapped_people_01.png", "trapped_people_02.jpg"],
    "damaged_vehicle": ["damaged_vehicle_01.png", "damaged_vehicle_02.jpg"],
    "rescue_camp": ["rescue_camp_01.png", "rescue_camp_02.jpg"],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="随机更换射击任务点地面贴图，或一次性配置共享写权限。"
    )
    parser.add_argument("--seed", type=int, default=None, help="可复现本次随机结果的种子。")
    parser.add_argument("--dry-run", action="store_true", help="只显示映射，不修改文件。")
    parser.add_argument(
        "--setup-permissions",
        action="store_true",
        help="初始化运行资源、贴图并配置共享组权限；只需执行一次。",
    )
    parser.add_argument(
        "--add-user",
        action="append",
        default=[],
        metavar="USER",
        help="初始化时把其他用户加入共享组，可重复指定。",
    )
    parser.add_argument("--runtime-share", type=Path, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--pool-dir", type=Path, default=None, help=argparse.SUPPRESS)
    return parser.parse_args()


def package_share_directory() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("ican_land_sim"))
    except Exception:
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
            if not prefix:
                continue
            candidate = Path(prefix) / "share" / "ican_land_sim"
            if (candidate / "worlds").is_dir():
                return candidate
    raise RuntimeError(
        "找不到已安装的 ican_land_sim。请先执行：source scripts/setup_env.sh"
    )


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[1]


def configure_logging() -> None:
    """同时输出到终端和工作空间日志文件。"""
    global LOG_FILE
    log_dir = workspace_root() / "logs"
    LOG_FILE = log_dir / "randomize_ground_textures.log"
    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stdout)]
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        handlers.append(logging.FileHandler(LOG_FILE, encoding="utf-8"))
    except OSError as error:
        print(f"警告：无法写入日志文件 {LOG_FILE}：{error}", file=sys.stderr)

    formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
    )
    for handler in handlers:
        handler.setFormatter(formatter)
    LOGGER.setLevel(logging.INFO)
    LOGGER.handlers.clear()
    for handler in handlers:
        LOGGER.addHandler(handler)
    LOGGER.propagate = False


def log_path_text() -> str:
    return str(LOG_FILE) if LOG_FILE else "日志文件不可用"


def source_pool_directory() -> Path:
    root = workspace_root()
    candidates = [
        root / "src" / "ican_land_sim" / "materials" / "textures" / "point_targets",
        root / "assets" / "point_targets",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    raise RuntimeError("工作空间中找不到 point_targets 贴图目录。")


def all_ground_images() -> list[str]:
    return [name for names in GROUND_POOLS.values() for name in names]


def validate_pool(pool_dir: Path) -> None:
    missing = [name for name in all_ground_images() if not (pool_dir / name).is_file()]
    if missing:
        raise RuntimeError(
            "以下地面候选贴图不存在于 "
            f"{pool_dir}：\n" + "\n".join(f"  - {name}" for name in missing)
        )


def choose_assignment(rng: random.Random) -> tuple[dict[str, str], dict[str, str]]:
    categories = rng.sample(list(GROUND_POOLS), len(MATERIAL_NAMES))
    assignment: dict[str, str] = {}
    ground_types: dict[str, str] = {}
    for material_name, category in zip(MATERIAL_NAMES, categories):
        assignment[material_name] = rng.choice(GROUND_POOLS[category])
        ground_types[material_name] = category
    return assignment, ground_types


def material_block(material_name: str, texture_name: str) -> str:
    return f"""material {material_name}
{{
  technique
  {{
    pass
    {{
      ambient 1 1 1 1
      diffuse 1 1 1 1
      texture_unit
      {{
        texture {texture_name}
        rotate 90
      }}
    }}
  }}
}}"""


def replace_material_texture(content: str, material_name: str, texture_name: str) -> str:
    start_match = re.search(
        rf"^material\s+{re.escape(material_name)}\s*\{{", content, re.MULTILINE
    )
    if start_match is None:
        return content.rstrip() + "\n\n" + material_block(material_name, texture_name) + "\n"

    depth = 0
    block_end = None
    for index in range(start_match.start(), len(content)):
        if content[index] == "{":
            depth += 1
        elif content[index] == "}":
            depth -= 1
            if depth == 0:
                block_end = index + 1
                break
    if block_end is None:
        raise RuntimeError(f"材质块没有正常结束：{material_name}")

    block = content[start_match.start() : block_end]
    updated, count = re.subn(
        r"(^[ \t]*texture[ \t]+)(\S+)([ \t]*)$",
        rf"\g<1>{texture_name}\g<3>",
        block,
        count=1,
        flags=re.MULTILINE,
    )
    if count != 1:
        raise RuntimeError(f"材质块中找不到 texture 行：{material_name}")
    if not re.search(r"^[ \t]*rotate[ \t]+90[ \t]*$", updated, re.MULTILINE):
        updated = updated.replace("\n      }", "\n        rotate 90\n      }", 1)
    return content[: start_match.start()] + updated + content[block_end:]


def target_material_xml(material_name: str) -> str:
    return (
        "<material><script>"
        "<uri>model://ican_land_sim/materials/scripts</uri>"
        "<uri>model://ican_land_sim/materials/textures</uri>"
        "<uri>model://ican_land_sim/materials/textures/point_targets</uri>"
        f"<name>{material_name}</name>"
        "</script></material>"
    )


def task_point_0_model_xml() -> str:
    return f'''<model name="task_point_0">
      <static>true</static>
      <pose>1.115218 -1.356470 0.006 0 0 0</pose>
      <link name="link"><visual name="visual"><geometry><box><size>0.52971 0.38454 0.012</size></box></geometry>{target_material_xml("ShootingGroundTarget0")}<cast_shadows>false</cast_shadows></visual></link>
    </model>'''


def update_world_materials(content: str) -> str:
    # Migrate older installations where target 0 was incorrectly attached to
    # command_center and task_point_0 did not exist yet.
    if re.search(r'<model\s+name="task_point_0"\s*>', content) is None:
        task_one_match = re.search(
            r'<model\s+name="task_point_1"\s*>.*?</model>', content, re.DOTALL
        )
        if task_one_match is None:
            raise RuntimeError("世界文件中找不到 task_point_1，无法插入 task_point_0")
        content = (
            content[: task_one_match.start()]
            + task_point_0_model_xml()
            + "\n\n    "
            + content[task_one_match.start() :]
        )

    command_center_match = re.search(
        r'<model\s+name="command_center"\s*>.*?</model>', content, re.DOTALL
    )
    if command_center_match is None:
        raise RuntimeError("世界文件中找不到 command_center")
    command_center = command_center_match.group(0)
    command_center, count = re.subn(
        r"<material>.*?</material>",
        "<material><ambient>0.65 1.0 0.05 1</ambient><diffuse>0.65 1.0 0.05 1</diffuse></material>",
        command_center,
        count=1,
        flags=re.DOTALL,
    )
    if count != 1:
        raise RuntimeError("command_center 中找不到 visual material")
    content = (
        content[: command_center_match.start()]
        + command_center
        + content[command_center_match.end() :]
    )

    for model_name, material_name in WORLD_MATERIAL_MODELS:
        model_match = re.search(
            rf"<model\s+name=\"{re.escape(model_name)}\"\s*>.*?</model>",
            content,
            re.DOTALL,
        )
        if model_match is None:
            raise RuntimeError(f"世界文件中找不到 {model_name}")
        model = model_match.group(0)
        pose, size = GROUND_TASK_GEOMETRY[model_name]
        model, pose_count = re.subn(
            r"(<static>true</static>\s*)<pose>[^<]+</pose>",
            rf"\g<1><pose>{pose}</pose>",
            model,
            count=1,
        )
        model, size_count = re.subn(
            r"<geometry><box><size>[^<]+</size></box></geometry>",
            f"<geometry><box><size>{size}</size></box></geometry>",
            model,
            count=1,
        )
        if pose_count != 1 or size_count != 1:
            raise RuntimeError(f"{model_name} 中找不到 pose 或 geometry")
        updated_model, count = re.subn(
            r"<material>.*?</material>",
            target_material_xml(material_name),
            model,
            count=1,
            flags=re.DOTALL,
        )
        if count != 1:
            raise RuntimeError(f"{model_name} 中找不到 visual material")
        content = content[: model_match.start()] + updated_model + content[model_match.end() :]
    return content


def atomic_write(path: Path, content: str, mode: int = 0o664) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", suffix=".tmp", delete=False
    ) as temporary:
        temporary_path = Path(temporary.name)
        temporary.write(content)
        temporary.flush()
        os.fsync(temporary.fileno())
    os.chmod(temporary_path, mode)
    os.replace(temporary_path, path)


def print_assignment(assignment: dict[str, str], ground_types: dict[str, str]) -> None:
    LOGGER.info("本次射击地面贴图映射：")
    for material_name in MATERIAL_NAMES:
        LOGGER.info(
            f"  {material_name}: {ground_types[material_name]} -> "
            f"{assignment[material_name]}"
        )
    LOGGER.info("  五个地面类别各使用一次；0 号点仅用于视觉贴图，不参与导航。")


def current_user() -> str:
    return os.environ.get("SUDO_USER") or os.environ.get("USER") or pwd.getpwuid(os.getuid()).pw_name


def run_setup_as_root(args: argparse.Namespace, runtime_share: Path, pool_dir: Path) -> int:
    if os.geteuid() == 0:
        return setup_as_root(args, runtime_share, pool_dir)

    command = [
        "sudo",
        sys.executable,
        str(Path(__file__).resolve()),
        "--setup-permissions",
        "--runtime-share",
        str(runtime_share),
        "--pool-dir",
        str(pool_dir),
    ]
    for username in args.add_user:
        command.extend(["--add-user", username])
    LOGGER.info("需要管理员权限，准备调用 sudo 初始化：%s", " ".join(command))
    subprocess.run(command, check=True)
    LOGGER.info("权限初始化完成。当前用户请重新登录，或执行：newgrp ican_shoot_sim")
    return 0


def setup_as_root(args: argparse.Namespace, runtime_share: Path, pool_dir: Path) -> int:
    LOGGER.info("开始初始化运行资源和共享权限")
    LOGGER.info("运行资源目录：%s", runtime_share)
    LOGGER.info("候选贴图目录：%s", pool_dir)
    LOGGER.info("当前有效用户：%s，EUID=%s", current_user(), os.geteuid())
    validate_pool(pool_dir)
    materials_dir = runtime_share / "materials"
    scripts_dir = materials_dir / "scripts"
    textures_dir = materials_dir / "textures"
    runtime_pool = textures_dir / "point_targets"
    worlds_dir = runtime_share / "worlds"
    material_file = scripts_dir / "ican_land.material"
    world_file = worlds_dir / "land_shoot.world"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = runtime_share / f"manual_backup_{timestamp}"
    LOGGER.info("创建旧文件备份：%s", backup_dir)
    backup_dir.mkdir(parents=True, exist_ok=True)
    for old_file in (material_file, world_file):
        if old_file.is_file():
            shutil.copy2(old_file, backup_dir / old_file.name)

    runtime_pool.mkdir(parents=True, exist_ok=True)
    for image_name in all_ground_images():
        source_image = pool_dir / image_name
        target_image = runtime_pool / image_name
        # The release deb already stores the candidate images in runtime_pool.
        # Avoid shutil.copy2() on the same path during permission setup.
        if source_image.resolve() != target_image.resolve():
            shutil.copy2(source_image, target_image)
    LOGGER.info("已准备 %d 张地面候选贴图", len(all_ground_images()))

    default_assignment = {
        MATERIAL_NAMES[0]: "collapsed_ruins_01.png",
        MATERIAL_NAMES[1]: "landslide_02.jpg",
        MATERIAL_NAMES[2]: "rescue_camp_01.png",
        MATERIAL_NAMES[3]: "damaged_vehicle_01.png",
        MATERIAL_NAMES[4]: "trapped_people_02.jpg",
    }
    material_text = material_file.read_text(encoding="utf-8") if material_file.exists() else ""
    for material_name, image_name in default_assignment.items():
        material_text = replace_material_texture(material_text, material_name, image_name)
    world_text = world_file.read_text(encoding="utf-8")
    updated_world_text = update_world_materials(world_text)
    # Validate both generated contents before replacing either file, so a
    # malformed world cannot leave a half-updated runtime installation.
    atomic_write(material_file, material_text)
    atomic_write(world_file, updated_world_text)

    try:
        gid = grp.getgrnam(SHARED_GROUP).gr_gid
    except KeyError:
        LOGGER.info("共享组不存在，创建系统组：%s", SHARED_GROUP)
        subprocess.run(["groupadd", "--system", SHARED_GROUP], check=True)
        gid = grp.getgrnam(SHARED_GROUP).gr_gid

    users = {current_user(), *args.add_user}
    for username in sorted(users):
        try:
            pwd.getpwnam(username)
        except KeyError as error:
            raise RuntimeError(f"系统中不存在用户：{username}") from error
        subprocess.run(["usermod", "-aG", SHARED_GROUP, username], check=True)
        LOGGER.info("已将用户加入共享组：%s -> %s", username, SHARED_GROUP)

    editable_dirs = [materials_dir, scripts_dir, textures_dir, runtime_pool, worlds_dir]
    for directory in editable_dirs:
        directory.mkdir(parents=True, exist_ok=True)
        os.chown(directory, 0, gid)
        os.chmod(directory, 0o2775)

    editable_files = [material_file, world_file, *runtime_pool.glob("*")]
    for file_path in editable_files:
        if file_path.is_file():
            os.chown(file_path, 0, gid)
            os.chmod(file_path, 0o664)

    # The setup subprocess runs as root. Return the log file to the invoking
    # user so later non-root runs can continue appending to it.
    if LOG_FILE is not None and LOG_FILE.is_file():
        try:
            invoking_user = pwd.getpwnam(current_user())
            os.chown(LOG_FILE, invoking_user.pw_uid, invoking_user.pw_gid)
            os.chmod(LOG_FILE, 0o664)
        except OSError as log_error:
            LOGGER.warning("无法把日志文件交还给用户：%s：%s", LOG_FILE, log_error)

    for required_path in (material_file, world_file, runtime_pool):
        mode = stat.S_IMODE(required_path.stat().st_mode)
        LOGGER.info(
            "权限检查：%s owner=%s:%s mode=%o",
            required_path,
            required_path.stat().st_uid,
            required_path.stat().st_gid,
            mode,
        )

    LOGGER.info("运行资源已更新：%s", runtime_share)
    LOGGER.info("共享组：%s", SHARED_GROUP)
    LOGGER.info("旧文件备份：%s", backup_dir)
    LOGGER.info("已加入用户：%s", ", ".join(sorted(users)))
    return 0


def write_assignment_report(
    runtime_share: Path,
    assignment: dict[str, str],
    ground_types: dict[str, str],
    seed: int | None,
) -> None:
    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "seed": seed,
        "runtime_share": "share/ican_land_sim",
        "material_file": "share/ican_land_sim/materials/scripts/ican_land.material",
        "ground_type_assignment": ground_types,
        "assignment": assignment,
    }
    report_path = runtime_share / "materials" / "shooting_texture_assignment.json"
    atomic_write(report_path, json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    LOGGER.info("映射报告已写入：%s", report_path)


def main() -> int:
    configure_logging()
    LOGGER.info("开始运行，参数：%s", " ".join(sys.argv[1:]) or "(无)")
    args = parse_args()
    try:
        runtime_share = args.runtime_share or package_share_directory()
        pool_dir = args.pool_dir or source_pool_directory()
        LOGGER.info("运行资源目录：%s", runtime_share)
        LOGGER.info("候选贴图目录：%s", pool_dir)
        validate_pool(pool_dir)

        if args.setup_permissions:
            return run_setup_as_root(args, runtime_share, pool_dir)

        rng = random.Random(args.seed)
        assignment, ground_types = choose_assignment(rng)
        print_assignment(assignment, ground_types)
        if args.dry_run:
            LOGGER.info("试运行完成，未修改文件。")
            return 0

        material_file = runtime_share / "materials/scripts/ican_land.material"
        world_file = runtime_share / "worlds/land_shoot.world"
        if not material_file.is_file() or not os.access(material_file, os.W_OK):
            try:
                file_stat = material_file.stat()
                LOGGER.error(
                    "材质文件不可写：%s owner=%s:%s mode=%o euid=%s groups=%s",
                    material_file,
                    file_stat.st_uid,
                    file_stat.st_gid,
                    stat.S_IMODE(file_stat.st_mode),
                    os.geteuid(),
                    os.getgroups(),
                )
            except OSError as stat_error:
                LOGGER.error("无法检查材质文件权限：%s：%s", material_file, stat_error)
            raise RuntimeError(
                "运行贴图文件不可写。首次请执行："
                " ./scripts/randomize_ground_textures.sh --setup-permissions"
            )

        if not world_file.is_file() or not os.access(world_file, os.W_OK):
            try:
                file_stat = world_file.stat()
                LOGGER.error(
                    "世界文件不可写：%s owner=%s:%s mode=%o euid=%s groups=%s",
                    world_file,
                    file_stat.st_uid,
                    file_stat.st_gid,
                    stat.S_IMODE(file_stat.st_mode),
                    os.geteuid(),
                    os.getgroups(),
                )
            except OSError as stat_error:
                LOGGER.error("无法检查世界文件权限：%s", stat_error)
            raise RuntimeError(
                "运行世界文件不可写。首次请执行："
                " ./scripts/randomize_ground_textures.sh --setup-permissions"
            )

        content = material_file.read_text(encoding="utf-8")
        for material_name in MATERIAL_NAMES:
            content = replace_material_texture(
                content, material_name, assignment[material_name]
            )
        world_content = world_file.read_text(encoding="utf-8")
        updated_world_content = update_world_materials(world_content)
        atomic_write(material_file, content)
        atomic_write(world_file, updated_world_content)
        write_assignment_report(runtime_share, assignment, ground_types, args.seed)
        LOGGER.info("材质文件和世界文件已更新，请重启 Gazebo。")
        return 0
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        LOGGER.exception("贴图替换失败：%s", error)
        LOGGER.error("详细日志：%s", log_path_text())
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
