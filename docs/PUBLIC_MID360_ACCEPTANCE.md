# 公开 MID360 数据回放（2026-10-13 本机执行）

来源：Kenji Koide，AIST， [Driving SLAM Test with Livox MID360](https://zenodo.org/records/14841855)，
DOI 10.5281/zenodo.14841855，官方记录许可 CC BY 4.0。原 ZIP 517088133 字节，保留于
本机 `.dev/datasets/koide_mid360/original.zip`，原始 sqlite3 bag 未改写时间、frame 或消息。

bag 长 277.166836670 s，2772 个 PointCloud2 与 55435 个 IMU；没有 ground truth、车体安装 TF、
绝对云台编码器或真实底盘反馈。逐点 timestamp 为绝对纳秒，首帧跨度约 99.526912 ms；
原始加速度均值 z≈0.9862，使用上游 acc_norm=1.0。

在隔离 Domain 193、use_sim_time、rosbag2 `/clock` 下，实际运行固定版本 small_point_lio，
保留上游内部外参，不生成 TF、车体或健康替身。第二轮增加进程树资源采样，不覆盖首次结果。

第二轮结果：2738 组 LIO odometry/去畸变云，输出跨度 276.097358227 s，
2738 条轨迹消息、552 条实时地图预览，预览上限 200000 点。
进程树采样峰值 RSS 151668 KiB、CPU 38.12 s（包含 ros2 run launcher，共享页可能重复计数）。
结束时最后原始帧起点到 LIO 时间差 -98.744284 ms；使用逐点末端比较为 +1.484911 ms。
这是末端源时间差，**不是端到端 UI 延迟**。

估计最大位移 341.4621 m、p99 步间速度 6.1370 m/s、最终相对位移 28.5151 m。
没有真值，不能把这些估计值作为误差/实际路程；不计算 ATE/RPE，不宣称姿态、闭环或精度通过。
输入 2772 帧不等于 2738 输出帧；初始化和负载下未覆盖所有帧，尚未证明零丢帧。
两轮观测计数不同，不能声称逐消息完全确定性回放。
诊断运动边界为公开驾驶场景单独显式设置，与此前静止/人工 1 m 数据的门限无关。

复现：

```bash
ROS_DOMAIN_ID=193 python3 tools/smoke_mid360_bag.py \
  --bag /原始/rosbag2_2024_04_16-14_17_01 --output /新的/报告目录 \
  --deps "$RM_NAV_DEPS_ROOT" --max-bag-duration 300 \
  --max-speed 30 --max-displacement 1000 --max-rotation 3.141592654 --with-visualization
python3 tools/inspect_mid360_bag.py --bag /原始/bag --output /新的/raw_report.json
```

原始与报告目录：WSL `/root/adam_nav_complete/public_koide_lio`、`public_koide_resources`、
`public_raw_timing.json`。数据不足以检验实际底盘 resolver、EKF 安装、冻结地图真值、回环正确性、
实车 Nav2 或监管停车；这些项目保持 NOT VERIFIED。相应 ROS 软件链另由明确的 TEST_ONLY 集成验证。
