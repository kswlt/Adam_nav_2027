# 更换 MID-360 后的网络诊断

2026-10-11，目标 `asus@192.168.1.164:2233`，主机名 ASUS。
用户确认本车更换了雷达，网口灯闪烁；新设备 IP 未知。
原 `192.168.1.152` 仅是旧雷达配置，不能作为新设备身份或地址证据。

## 实际检查

- 系统只有一个物理以太网接口 enp86s0，Intel I226-V/igc，MAC 88:ae:dd:68:a9:71；
  Wi-Fi 为 wlo1。无 USB 以太网设备；当前网络命名空间无其他以太网接口。
- enp86s0 管理状态 UP，但 NO-CARRIER、carrier=0、Link detected: no、速度未知。
- 软件和 NIC 硬件统计均 rx_packets=0、tx_packets=0。
- 地址 192.168.1.50/32；对旧雷达 .152 的专用路由原本走 enp86s0，
  Wi-Fi 默认路由保持 wlo1/.164。
- 两次 any-interface 抓取 UDP 56000–56510 与 ARP，只看到 Wi-Fi ARP，未发现雷达报文。
- 重启自动协商及仅重启 enp86s0 后未恢复。接口 down/up 导致临时专用路由消失，
  已显式恢复 .152/32 经 enp86s0，并复查 Wi-Fi 默认路由。
- EEE 已关闭，运行时电源管理为 active/control=on。
- 临时强制 100 Mbps/full 作诊断，仍 carrier=0，6 秒有线抓包为 0；
  随后恢复自动协商及原有全部 advertised link modes，未写入持久配置。

## 结论与下一步

尚未连接到新雷达，也没有证据确认其 IP。旧 JSON 地址可能不适用于新设备，
但地址错误不能解释网卡始终没有 carrier。不能仅凭这些观察断言线缆坏了；
仍需核实实际连接位置、转接板、物理链路及网卡协商兼容性。

优先获取 SN/Viewer 地址并配置新设备的专用路由，再抓取官方驱动发现及点云/IMU。
若保留出厂 IP，官方规则为 192.168.1.1XX（SN 末两位）；改过地址则不能据此确定。
来源：[Livox MID-360 官方快速入门](https://dl.djicdn.com/downloads/Livox/Mid-360/QSG/Livox_Mid-360_Quick_Start_Guide_multi.pdf)。

不要将有线直接配置成覆盖 Wi-Fi 的优先 /24 路由：同网段 SSH 可能被转往无链路接口。
当前未改 Wi-Fi、未重载网卡模块、未修改原导航代码或启动串口。
