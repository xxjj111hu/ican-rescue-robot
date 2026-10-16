# iCAN 射击练习仿真 Demo 工作空间

这是基于最简射击工作空间生成的 Demo 空间，增加了 0～4 号地面贴图、随机贴图替换脚本和权限配置说明。`packages/` 中的 deb 只包含最简运行所需的 `ican_land_control`、`ican_land_sim` 和 `ican_shoot_sim`，不包含完整工作空间的其他功能包。示例启动时使用内置靶标配置，不需要手动输入 ID。

## 目录

```text
demo_shoot_ws/
  assets/
    point_targets/             # 10 张地面候选贴图
  src/
    ican_example_navigation/
    ican_example_ring_aim/
    ican_example_mission/
  packages/
    ican-shoot-sim-practice_0.1.0_amd64.deb
    SHA256SUMS
  scripts/
    00_install_ros2_gazebo.sh
    01_install_sim_deb.sh
    setup_env.sh
    run_sim.sh
    run_example_mission.sh
    randomize_ground_textures.sh
    randomize_ground_textures.py
    check_sim_env.sh
  docs/
    快速开始.md
    仿真环境与接口说明.md
```

## 推荐系统

- Ubuntu 22.04 LTS amd64
- ROS 2 Humble
- Gazebo Classic

## 首次安装

进入本目录后执行：

```bash
bash scripts/00_install_ros2_gazebo.sh
bash scripts/01_install_sim_deb.sh
```

安装完成后打开一个新终端，或在当前终端执行：

```bash
source scripts/setup_env.sh
```

该脚本会同时加载 ROS 2、Gazebo Classic 的系统资源（包括 shader 和媒体文件）以及 iCAN 仿真包，不需要另外手动设置 `GAZEBO_RESOURCE_PATH`。

## 启动仿真

```bash
bash scripts/run_sim.sh
```

无图形界面或远程服务器可用：

```bash
bash scripts/run_sim.sh gui:=false
```

## 检查环境

```bash
bash scripts/check_sim_env.sh
```

## 示例代码

`demo_shoot_ws` 本身就是示例代码的 colcon 工作空间；首次运行或修改源码后，先在本目录构建：

```bash
colcon build --symlink-install
source install/setup.bash
```

启动示例流程：

```bash
bash scripts/run_example_mission.sh
```

脚本会打开三个终端标签，分别启动仿真、Nav2 导航和示例程序。示例使用预设的 2/3/4 号靶配置（ID 分别为 1、6、8），不会再等待手动输入；启动后会导航到 1 号任务点，发布 `shoot_1` 点位消息并执行射击。

## 更换地面贴图

本空间自带 10 张地面候选贴图，脚本会按照识别任务的规则，把五个类别各选一张，随机分配到 0～4 号地面点。0 号点是地图中的 `task_point_0`，不参与小车导航；原点 `command_center` 不使用任务贴图。

首次使用或其他用户需要运行时，配置 `/opt/ican_shoot_sim` 的共享写权限：

```bash
source scripts/setup_env.sh
./scripts/randomize_ground_textures.sh --setup-permissions --add-user "$USER"
```

为其他用户配置权限时，把用户名追加到命令中即可：

```bash
./scripts/randomize_ground_textures.sh --setup-permissions --add-user 用户名
```

权限配置完成后，直接随机替换贴图：

```bash
./scripts/randomize_ground_textures.sh
```

常用选项：

```bash
./scripts/randomize_ground_textures.sh --seed 1234  # 固定本次随机结果
./scripts/randomize_ground_textures.sh --dry-run     # 只显示映射，不修改
```

脚本会同时更新材质和世界文件，运行日志保存在 `logs/randomize_ground_textures.log`，最新映射保存在 `/opt/ican_shoot_sim/share/ican_land_sim/materials/shooting_texture_assignment.json`。如果提示权限不足，先查看日志，再重新执行 `--setup-permissions`；替换完成后重启 Gazebo。

示例导航仍然调用 ROS 2 官方 `nav2_bringup`，但默认使用适配当前练习场地和麦轮小车的参数：

```text
src/ican_example_navigation/config/nav2_params.yaml
```

如需对比 ROS 2 官方默认参数，可启动时传入：

```bash
ros2 launch ican_example_navigation navigation.launch.py params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

RViz 中很多绿色小箭头通常是 AMCL 粒子云显示。当前示例默认使用 `src/ican_example_navigation/rviz/example_nav.rviz`，不显示粒子云。

更多接口说明见 [docs/仿真环境与接口说明.md](docs/仿真环境与接口说明.md)。
