# 节点源码已迁出

`ican_recognize` 已迁移到工作空间，以便 `colcon build` 直接编译：

```
sim_ws/src/ican_recognize/
```

本目录不再保留源码，避免两份代码漂移。

如需运行：

```bash
cd ~/你的工作空间路径/sim_ws
colcon build --packages-select ican_recognize
source install/setup.bash
ros2 launch ican_recognize recognize.launch.py
```

参数文件：`sim_ws/src/ican_recognize/config/recognize_params.yaml`
使用说明：`交付包/识别/README.md`
