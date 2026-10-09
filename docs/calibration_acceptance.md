# 实测 CalibrationBundle 验收

生产定位链只接受 `status: verified` 的实测 Bundle。模板和上游示例外参继续拒绝启动。

```bash
python3 tools/validate_calibration_bundle.py \
  --file /absolute/measured_calibration.yaml \
  --report /absolute/calibration_validation.json
```

必须测量并记录：

- `base_footprint → chassis`
- `chassis → big_gimbal_yaw`
- `big_gimbal_yaw → front_mid360_imu`
- `front_mid360_imu → front_mid360`
- 四个刚体变换的米制平移和 xyzw 单位四元数
- 绝对编码器 joint、方向和零位（弧度）
- frame 名称和唯一性
- 传感器时钟偏移、同步方法和误差上限

验证器不会把 draft 改成 verified，也不会根据零值猜测外参。
