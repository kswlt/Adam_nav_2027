# small_point_lio 上游接入验收

来源为 [Yancey2023/small_point_lio](https://github.com/Yancey2023/small_point_lio)，
固定 ros2 提交 `688d75cfa780049ae532e5100ca64f46ad8b1a93`，保留上游 MIT 许可。
外部源码位于 `/home/asus/nav_deps/src/small_point_lio`，不复制成新的自研滤波器。

## 构建与接口补丁

```bash
source /opt/ros/jazzy/setup.bash
RM_NAV_DEPS_ROOT=/home/asus/nav_deps bash tools/bootstrap_small_point_lio.sh
source /home/asus/nav_deps/install_lio/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=98 python3 tools/smoke_small_point_lio.py
```

远端使用 Git SSH 拉取公共依赖；首次下载可指定
`RM_NAV_LIO_REPO_URL=git@github.com:Yancey2023/small_point_lio.git`。
bootstrap 接受原始固定提交或本项目完整补丁，不覆盖额外修改。
安装为独立 ROS overlay `install_lio`，工程自己的 14 个包仍在 `rm_nav_v2/install`。

集中补丁 `tools/patches/small_point_lio_raw_state.patch` 的 SHA256：
`255e111bd62e8fad265a4813a6eb57d98554c423e16291979589a8a2b502efd4`。
补丁只调整发布边界，没有修改 predict/update、点面残差或状态滤波方程：

- 上游 kf 状态是 IMU 位姿，发布 `/lio/sensor_odometry`，child frame 为真实 IMU frame。
  原始速度不直接冒充底盘速度，后续 EKF 融合位姿。
- 暴露滤波器已有的 6×6 位姿协方差，并将旋转右扰动的协方差转换到世界轴。
  这是发布坐标转换，不修改滤波器 covariance 更新。
- 点云核心已按各点时间投影到 odom，ROS 层直接发布 `/lio/deskewed_odom_cloud`，
  去除原 ROS 层的第二次底盘外参变换。
- 新增 odom/state frame 参数和 `publish_tf`，默认 false。
  生产链唯一 odom→底盘 TF 来自 robot_localization，LIO 不争抢该 TF。

目前构建没有安装 Livox CustomMsg 驱动，支持上游 `livox_pointcloud2`、其他可用适配器。
PointCloud2 的 Livox 模式需要 FLOAT32 XYZ、UINT8 tag、FLOAT64 timestamp；
每点 timestamp 为绝对纳秒，不能随意把秒、相对 offset 或消息时间戳代入。
实车应先安装官方驱动并核对实际字段、硬件时间与 IMU 量纲，不自研 UDP 协议。

## 2026-10-09 软件结果

在 asus/Jazzy 构建上游原版及补丁版均成功。
测试实际启动 small_point_lio_node，输入明确的合成正交平面点云与静止 IMU，
进行真实初始化与 Point-LIO 计算，生成 48 组位姿、48 组点云。
验证无输入不输出、位姿有限/单位四元数、静止漂移小于测试阈值 0.15 m、
滤波协方差存在、odom 点云没有再次外参变换，以及 publish_tf=false 时不发布 TF。
日志：`log/small_point_lio_smoke.log`；合成配置：`log/lio_fixture.yaml`。

这证明真实算法与发布接口可运行，不证明实际传感器去畸变、动态云台运动或定位精度。
合成测试的单位外参只能用于测试，不能复制为实车标定。
真实传感器字段/时间检查、Sensor Hub、回放、标定和长期运行仍需接入。

ROS_DOMAIN_ID=100 python3 tools/smoke_small_point_lio.py --with-state 验收上游→resolver→EKF→状态桥→观测→子地图→GICP，输入仍是合成静态三平面/IMU/编码器，不代替实车动态回放。见 [集成说明](state_observation_acceptance.md)。
