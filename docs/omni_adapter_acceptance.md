# Omni 适配器超时与前馈回归

检查基线：`d55a764dc3e5fe0d9a7ec28aebfb20f5f8843783`。日期：2026-10-11。
远端 feature 已 fetch 并确认基线，无更新覆盖；原有未提交串口 provenance 文件及第三方副本保留。

## 根因与修复范围

| 严重度 | 旧行为 | 新行为 |
| --- | --- | --- |
| 高 | require_measured_feedback=false 仍解引用 feedback_，可空指针崩溃 | 无反馈时纯速度前馈，绕过 PID，不制造反馈 |
| 高 | safe、odom、drive_ready 永久缓存 | 检查源时间与单调接收时间，就绪心跳超时，失效归零平移并清空命令 |
| 高 | PID output_limit 只限制修正量，前馈叠加后超限 | 最终 XY 向量限幅及 yaw 速率限幅 |
| 高 | 全零 safe 仍可能输出非零 PID 修正或保留旧 yaw 目标 | 零平移并以新鲜当前 yaw 重置目标 |
| 中 | yaw wrap 使用循环；非有限/极大输入不稳定 | remainder 有界计算，拒绝非法速率参数，支持失效重置 |
| 中 | MPPI launch 不读取 adapter YAML | 加载 YAML，显式反馈模式参数，默认仍关闭物理输出 |

复用现有 rclcpp、ROS 消息、PID/YawManager 及 my_serial_py，通信字节协议未改。
修改限于 rm_nav_control、MPPI 启动配置与说明。默认物理输出保持 false。

## 测试方法

在车载隔离工作区执行：

```bash
bash tools/vehicle_164_build.bash
```

该脚本执行 colcon build、colcon test、colcon test-result --verbose。
新增 CTest `omni_pid_adapter_regression` 内有三个 ROS 场景：
关闭输出、强制真实反馈、无反馈纯前馈。独立 Domain 199、仅本机发现，所有输入输出
重映射到 `/test_only/omni/*`，不运行驱动和串口，不触碰原导航进程。

首次测试 fixture 将 quaternion.w 赋整数 0，触发 ROS Python 消息转换断言；
已改为 0.0。保留失败日志 `omni_adapter_fixture_failure.log`，清理该次孤立测试进程。
修正后最终结果：13 个导航/通信包构建通过；41 tests，0 errors，0 failures，1 skipped。
新增 ROS 回归包含 3 个场景，整体约 16 秒；最终结果记录于 `omni_adapter_build_test.log`。

## 尚未验证与兼容影响

- MID-360 网口 carrier=0；真实 LIO/TF、MCAP 闭环及实车移动 NOT TESTED。
- drive_ready 原来可一次性发布，现在必须持续心跳；缺少真实来源时适配器保持零平移。
- 无轮速模式不是闭环速度控制，不能声称 PID 已调参或底盘速度精度通过。
- 原协议没有 yaw 失能字段；serial_bridge 只对平移超时归零并重复发送旧 yaw 目标。
  里程计失效时无法仅凭本节点保证旋转停车。下位机 watchdog、yaw 零点/方向/坐标系
  和停止语义待硬件核验，未接通串口输出。
- ros2_control HardwareInterface 实际 IO 仍待完成；本报告不代表最终控制整车验收。
