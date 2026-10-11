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

## 主动发现补查（2026-10-12）

基线 aca0735；已检查 git status 并 fetch，保留原有未提交修改。
以 root 检查所有网络命名空间：Docker rm_win_nav_local 为 host 网络，
accounts-daemon、rtkit、VS Code 命名空间只有 lo，没有隐藏雷达接口。
旧开发机 192.168.1.145:22 本轮 TCP 探测不可达。

新增 tools/livox_search_probe.cpp，只复用官方 CommPort、SdkProtocol、FastCRC，
发送 search 命令到 UDP 56000；目的为限域广播及候选 .52、旧 .152。
不初始化 DeviceManager，不执行网络设置、工作模式、数据接收地址或串口命令。
完整 SDK viewer 初始化存在自动 UpdateLidarCfg 路径，因此未使用其设备管理流程。
参考：[官方设备发现源码](https://github.com/Livox-SDK/Livox-SDK2/blob/c0796f04c143143899c87a773d9f6b7136453c0b/sdk_core/device_manager.cpp)。

SDK 源码独立位于 diagnostics/Livox-SDK2，revision c0796f04c143143899c87a773d9f6b7136453c0b。
从 diagnostics 目录构建：

```bash
c++ -std=c++17 -O2 -I Livox-SDK2/sdk_core -I Livox-SDK2/include -I Livox-SDK2/3rdparty \
  livox_search_probe.cpp Livox-SDK2/sdk_core/comm/comm_port.cpp \
  Livox-SDK2/sdk_core/comm/sdk_protocol.cpp Livox-SDK2/3rdparty/FastCRC/FastCRCsw.cpp \
  -o livox_search_probe
./livox_search_probe --self-test
sudo ./livox_search_probe wlo1
sudo ./livox_search_probe enp86s0
```

构建及独立编解码 self-test 通过：合成 search ACK 可解析，截断包和非 ACK 被拒绝。
仅为辅助诊断工具，本轮未重复运行导航 colcon/MCAP 测试。
两接口各 10 秒，send_accepted=30、send_errors=0、datagrams=10、devices=0。
收到的报文没有有效雷达 search ACK；广播可在本机回送，send_accepted 不代表物理发送成功。
enp86s0 仍 carrier=0。对 .52 的临时有线路由已撤销；Wi-Fi 默认路由保持原样。
ethtool 灯光定位操作返回 Operation not supported，未据此判断用户观察的灯来自哪一个接口。

最终状态：尚未发现或连接新雷达，未把 .52、.152 或普通 ARP 设备标记为雷达。
下一步需要从插雷达的电脑本机核对当前 IP/接口 MAC，以及新雷达 SN 或 Viewer 地址，
以排除远程目标电脑与实际接线电脑不一致。持久网络设置未改，原导航源码与进程未改。
