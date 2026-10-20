# ASUS 20261020 部署脚本快照

这些是已用于 `192.168.1.164:2233` 的独立部署脚本，包含该机的实际目录映射；不是通用安装器。
详细结果见 `docs/ASUS_LIVE_20261020.md`。复制脚本到
`/home/asus/nav_2027_vehicle_config_164/` 后使用，不包含 SSH 或 GitHub 凭据。

`remote_env_20261020.bash` 只加载新 release、独立 TDT 依赖、只读 Livox/small_point_lio overlay，
以及必要的私有重建消息库。PAL/controller-manager/control 消息重建复用系统同版本原始接口定义，
不修改字段，不升级系统包。没有部署原版 point_lio。

已运行时不要重复启动：

```bash
bash start_driver_20261020.bash
# 另开终端
bash start_state_20261020.bash
# 另开终端
bash start_foxglove_20261020.bash
# 另开终端；需要保存原始关键帧时
bash start_mapping_20261020.bash
```

脚本中没有串口节点或物理运动输出。不能用此观察实例声称已经通过正式地图导航验收。

新增在线回环后端，复用已运行 recorder 的实际 session，不重启雷达：

```bash
bash start_online_mapping_20261020.bash /绝对路径/当前的session
```

该脚本显式加载新 `install_online` overlay。Foxglove 导入 `mapping_online.json`，参考为 mapping_odom。
回环、历史地图更新与资源限制详见 `docs/ONLINE_MAPPING.md`。
