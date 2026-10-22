# ASUS独立部署：2026-10-22

主机 `asus@192.168.1.164:2233`，部署代码 `8a9a9b12cc2a33abec5040e7a7f55aad12cbe0be`，分支 `audit-p0-tf-costmap`。

旧release有大量未提交修改，未覆盖、重置或删除。原车工作区、已有MCAP和旧release原样保留。新建：

- 源码：`/home/asus/nav_2027_vehicle_config_164/releases/20261022_fit153_console`
- 运行目录：`/home/asus/nav_2027_vehicle_config_164/runtime_20261022_fit153_console`

仅从旧运行目录复制已有MID-360驱动配置与small_point_lio参数，没有重设网卡或修改原车启动服务。控制脚本通过NAV_BASE/NAV_RELEASE/NAV_RUNTIME选中当前独立目录。

## 现在怎么打开

```bash
ssh -t -p 2233 asus@192.168.1.164 "bash /home/asus/nav_2027_vehicle_config_164/runtime_20261022_fit153_console/start_console.bash"
```

本机还提供 `output/ASUS_20261022/打开车载导航菜单.cmd`，双击后通过SSH进入菜单，按SSH提示输入凭据。三份Foxglove布局在同目录的 `Foxglove` 文件夹。

菜单2启动建图采集；5看终端日志；6看实际ROS消息；f打印Foxglove配置。物理串口是独立选项8/9，默认关闭。退出菜单不会停止后台任务，s才请求停止本项目全部组件。

## 实际构建和测试

通过Git bundle克隆全新源码，执行 `tools/deployment/20261022/deploy_asus.bash`：

```bash
source /home/asus/nav_2027_vehicle_config_164/remote_env_20261020.bash
cd /home/asus/nav_2027_vehicle_config_164/releases/20261022_fit153_console
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
export ROS_DOMAIN_ID=198
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
colcon test --executor sequential --return-code-on-test-failure
colcon test-result --verbose
python3 tools/test_nav_console.py
```

- ASUS全14包构建通过，2分36秒；沿用small_point_lio依赖，不构建原版point_lio。
- ASUS64 tests：0 errors、0 failures、1 skipped。
- 控制台无设备CLI测试3项通过。
- 实际运行新目录的控制台status和mapping dry-run通过，路径指向新管理器；各组件当前均未启动。
- 新release工作区干净；原release未提交修改保留。
- 主机网口enp86s0为UP，串口by-id存在。这只证明设备枚举，不等于雷达通信/定位或电控执行验收。
- 未启动雷达/LIO、未录制新数据、未发物理命令。避免在底盘/云台初始化工况未知时自动建图。

证据在 [部署日志目录](evidence/asus_deploy_20261022)。原始完整测试日志仍保存在ASUS运行目录和release的colcon日志中。

## 开发是否结束

没有。基础软件功能已有实现，并且本次实际部署/构建/软件回归通过；以下现场验收尚未结束：

1. 底盘和云台停稳初始化，随后单独旋转云台，检查拟合偏心补偿及墙面重影。
2. 遥控完成一圈真实场地采集，检查累计地图完整性、真实回环和资源稳定性。
3. 导出检查包、确认自由空间与地图坐标，发布本地场地MapBundle，验证重定位。
4. 检查真实速度方向、障碍停车、定位/雷达/通信故障行为，完成Nav2任务实车闭环。

15.3cm参数仍是用户批准的录包拟合debug配置，不是完整底盘精测；轮速/真实停车反馈也未因此自动具备。UI脚本不自动把draft地图升级成正式地图。只有上述验收通过，才能宣布当前基础自主导航交付完成，不能用64项软件测试替代它们。
