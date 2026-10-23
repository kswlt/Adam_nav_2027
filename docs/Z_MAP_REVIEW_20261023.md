# 本地场地地图检查、发布UI与先验定位复验

检查基线：`28444126f6ec164f1ac5ce3b1e92fd5c875f11f3`，分支`audit-p0-tf-costmap`。
本轮继续已保存地图的导航交付流程；不使用虚构自由空间，不接物理输出。

## 已执行

1. 保留正在运行的Domain98建图（PID11850），driver3966、Foxglove4270不变。
2. 导入已保存原生地图`maps/live_20261023`，生成`maps/live_20261023_draft/map_bundle.json`。
3. 真实地图包含3587关键帧、15719点；PCD保留二进制XYZI原本，draft为XYZ副本。
4. 在UI生成492×413、5cm/格的俯视检查图，原点[-17.1,-11.3,0]。
   障碍投影z范围暂用当前导航scan配置的[-0.10,1.50]m，**相对本次建图原点，不是离地高度**。
   2054个障碍格，其余未知，自由格0；没有自动把空白区域当空地。
5. 页面实际加载检查图、区域圈选和目标点输入控件；只是滚动/检查，没有代用户圈选或勾选审核。

本轮实际地图预览：[current_site_review.png](evidence/z_localization_20261023/current_site_review.png)。
当前图有稀疏观测/未知区域，不能据此宣布整个场地可通行或地图完整。

## 新UI怎么用

操作台`http://127.0.0.1:8878/`经现有SSH隧道访问。

1. 在“2 · 地图与导航”选择已经保存的原生地图。
2. 在“2a”设置分辨率和实际适用的障碍物高度范围，点“生成所选地图检查图”。
3. 结合现场和3D地图检查高度/地面/障碍物。在俯视图依次点击顶点，点“完成当前区域”。
   可圈选多个区域；黑色障碍观测始终保留，灰色未圈选区域仍是未知。
4. 填写实际检查人，确认检查内容后勾选复选框，点“确认并发布本地导航地图”。
   没有实际检查时不要勾选。程序拒绝缺少确认、空区域、自交、越界和无自由格输入。
5. 复用`publish_map_bundle.py`生成占据栅格、地图YAML、观测地形参考、原生Z包副本和正式local_site manifest。
   使用本次建图坐标、恒等变换，**不宣称官方场地对齐**。不覆盖旧图，临时目录失败回滚。
   生成的terrain是minimum_observed_z，仍标记traversability_verified=false，不是已验证地面坡度模型。
6. 新manifest自动填入导航路径。完成当前采集后，先保存并停止当前mapping，再启动影子导航。
7. 重新初始化状态链并等待定位健康；“2b”输入map系x/y和yaw弧度，点“发送影子目标”。
   目标经过既有`/nav/submit_goal`服务；服务拒绝会报错，不把accepted=false当成功。
   物理串口始终未启用。

本轮尚无真实操作者确认的自由空间；**没有发布navigation_ready地图，没有启动Nav2整链，也没有发送目标/实车速度**。
UI没有绕过原MapBundle/导航健康契约。

## 先验地图定位实际测试

从现场实时驱动另录约22秒原始PointCloud2/IMU，录制进程持续25秒（包含发现话题时间）。
bag在ASUS：`runtime_live_20261023/prior_test_raw_20261023/`，原始MCAP约110.24MiB。
本机原始备份：桌面`新导航框架/实车原始数据备份/20261023_Z先验定位验收/prior_test_raw_20261023/`。
未重写任何CDR、header或接收时间；本次使用原接收顺序，无派生调度。

```bash
source "$NAV_Z_WORKSPACE/src/Adam_nav_2027/tools/z_environment.bash"
export ROS_DOMAIN_ID=226 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 tools/smoke_z_localization_bag.py \
  --bag "$NAV_Z_WORKSPACE/runtime_live_20261023/prior_test_raw_20261023" \
  --prior-map "$NAV_Z_WORKSPACE/maps/live_20261023" \
  --output "$NAV_Z_WORKSPACE/prior_test_live_20261023" --rate 1 --max-wall-seconds 120
```

实际结果：pipeline_pass=true，3849里程计、195去畸变云、3849轴参考、3472后端位姿；先验图收到1次。
Z后端接纳prior scan-to-map观测，末尾NORMAL/epoch0，健康曾通过，LIO覆盖数据末尾，map→odom唯一父关系通过。
测试进程正常退出；Domain98现场建图不切换、不重启。无真值，不报告ATE/RPE或精度合格。
这段短记录只证明已保存图的重载与近期现场数据定位链，不等于Nav2整链或长期稳定性验收。

证据：[current_site_prior_22s.json](evidence/z_localization_20261023/current_site_prior_22s.json)。

## 故障与修复

- 页面之前用Windows Date.now减ASUS接收时间，实测两机相差约10.26秒，误报“消息已过期”。
  现由ASUS单调时钟计算接收年龄，UI读取age_s；没有改操作系统时钟或放宽传感器源年龄门限。
- 操作台重启会丢失准备好的检查图，现仅缓存预览及ID；不缓存审核通过声明、不替用户确认。
- 当前现场轴参考已锁存`LIO pose motion exceeds physical continuity limits`，后端仍可NORMAL。
  **这是真实未解决故障，不能被时钟显示修复掩盖。**保持导航输出关闭，保留日志与当前会话；
  后续导航模式初始化后仍须检查参考状态健康，复发则需采集故障前后的原始数据/位姿分析。
- 最初部署命令在错误本机目录运行，上传未发生，ASUS测试缺少test_map_review模块；已纠正目录后重新部署并测试通过。
  没有把该次失败当作有效测试结果。

当前现场状态快照：[operator_review_status.json](evidence/z_localization_20261023/operator_review_status.json)。

## 验证、影响与剩余事项

```bash
python3 -m unittest discover -s tools/z_operator -p 'test_*.py' -v
python3 -m unittest discover -s tools -p test_publish_map_bundle.py -v
node --check tools/z_operator/review.js
```

ASUS操作台17项、既有MapBundle9项，全部通过。覆盖未知不变自由、障碍保留、多边形自交/越界拒绝、
明确审核要求、地图变化拒绝、目标编码和拒绝返回、单调时间年龄，以及原PTY/进程归属回归。
页面JS语法检查通过，原生地图导入/真实检查图和独立先验回放已实际执行。
复用Pillow画多边形及现有import/publish工具、Nav2任务服务与Z定位器，不新增SLAM或控制算法。
没有ROS包代码变更，本阶段未重复全量colcon；没有将尚未完成的人工审核/发布/实车步骤计为通过。

新风险：操作者圈选错误或高度过滤错误仍会影响地图；圈选并不能证明地面承载性或动态障碍安全。
现有PCD障碍保留、未知区域保留和导航健康联锁继续生效。
下一步依赖实际场地检查，不用假数据填充reviewer、编码器或停车反馈。
提交/推送分支保持audit-p0-tf-costmap，未合并main；最终提交与远端状态见本轮回报。
