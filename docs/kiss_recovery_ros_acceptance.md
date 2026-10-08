# 受限 KISS 恢复 ROS 调度验收

本步骤把真实 KISS→GICP 后端接入冻结地图 ROS 链，提供有边界的恢复请求、
会话隔离和候选复核。默认停在候选确认阶段；显式启用后续
[停车与提交事务](recovery_transaction_acceptance.md) 才可提交大修正，仍不自动恢复运动权限。

## 运行约定

MapOdomManager 仍是唯一 map→odom 发布者，也是恢复会话的发起者。
默认 `enable_recovery=false`。开启时必须提供实际地图对应的矩形场地范围
`field_bounds=[min_x,max_x,min_y,max_y]`；缺失、退化或非有限范围会拒绝启动。
这些值必须由实际 MapBundle/场地测量提供，验收脚本中的范围只是合成场景数据。

```bash
ros2 launch rm_nav_bringup frozen_map_localization.launch.py \
  map_version:=<地图版本> enable_recovery:=true \
  field_bounds:='[<min_x>,<max_x>,<min_y>,<max_y>]'
```

服务 `/localization/request_recovery` 类型 `rm_nav_interfaces/srv/RequestRecovery`。
请求包含地图版本、map 坐标中的最后可靠/出生区域中心、odom 坐标中的机器人原点，
以及距失定位的时间。当前由调用方提供这些输入，尚未实现自动失定位触发与来源核验。
机器人原点不是点云均值；在 odom 坐标中必须随实际运动更新。

半径由服务器计算：`recovery_max_speed * lost_time + recovery_margin`。
默认最大速度 2 m/s、余量 1 m、最大半径 6 m。超过上限直接拒绝，不能无限扩大。
请求中心必须位于场地范围内，非有限输入和地图版本不符均拒绝。
成功请求先发布健康 false，再发布带唯一会话编号的 RecoveryRequest。

matcher 使用场地矩形与 `radius + target_feature_range` 圆形交集裁剪目标地图，
后者为候选机器人位置周围保留点云特征，并非扩大允许的机器人位置。
默认特征范围 8 m，需按实际传感器有效范围调整。source/target 各最多 50000 点，
恢复匹配最高 1 Hz，超量直接拒绝，当前没有自动体素降采样来满足此上限。
结果中的机器人原点必须仍落在原候选半径及场地范围内。

## 候选确认与运动约束

- 只接受本会话的 KISS_GICP 结果，保留源观测时间并进行年龄、顺序、质量检查。
- 新鲜的普通 LOCAL_GICP 结果不能退出恢复状态或恢复健康。
- 两次有效结果需相隔至少 0.1 s，平移差不超过 0.1 m、旋转差不超过 0.05 rad。
- 点云数据指纹相同，即使换了时间戳，也不能作为第二次独立确认。
- 新请求清除旧候选与确认；旧会话结果被拒绝。
- 超时停止匹配、清除确认，保持健康 false；不自动退回普通定位。
- `/localization/recovery_confirmed` 是诊断信息，不是 TF 提交或恢复运动的许可。

指纹只防止完全相同数据重用，不证明真实传感器独立观测。
实车仍需通过去畸变、源帧编号及新的 LocalSubmap 构建过程验证观测独立性。
恢复期间 motion_gate 会把持续输入的非零命令置零；尚未接物理串口，
软件零速度不是下位机停车确认。

## 验收与剩余工作

```bash
source install/setup.bash
ROS_DOMAIN_ID=91 python3 tools/smoke_kiss_recovery.py
```

启动真实 matcher、MapOdomManager 和 motion_gate，使用 5000 点合成云及远处干扰点。
验证真实 KISS/GICP 大偏差恢复、目标裁剪、请求边界、重复数据拒绝、第二次候选确认、
运动阻断、会话重置、旧会话拒绝和超时安全保持。
2026-10-09 在 asus 通过；该合成数据平移误差 `0.00000320 m`、RMSE `0.000586 m`。
这是软件算法及编排验收，不能推断实车定位精度或下位机停车状态。

Nav2 取消、唯一 TF 提交和双 costmap 清理已在后续事务接入，必须有独立实测速度输入。
自动丢失触发、odom/最后可靠位置门、新规划与稳定定位放行已接入，见
[任务监管验收](task_supervisor_acceptance.md)。尚需真实 odom/LIO 与反馈适配、
TDT 旧轨迹失效及实车回放。
recovery_confirmed=true 本身不能提交 TF，也不能恢复运动。
