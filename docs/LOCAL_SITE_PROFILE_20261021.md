# 本地场地地图与真实定位后端的软件回归

此前只有官方对齐 profile，用户选择本次建图坐标后仍无法显式加载本地地图。新增 `publish_map_bundle.py --profile local_site`，保留默认 official；本地仅接受明确的 session_mapping_odom 单位变换，local_source 必填，不能伪装官方对齐。

frozen_map_localization、sensor_reference_navigation、tdt_navigation 显式 `map_profile:=local_site` 才接受本地交付；默认拒绝。无人工审核自由空间的地图仅可发布为定位用，navigation_ready=false；人工 mask 需匹配地图来源、坐标、分辨率、审核人及审核范围，未知格不自动变成可通行区域。draft 依旧拒绝。实际往返导出的 draft 也未自动提升。

修复集成回归发现的 TDT 精确时间 TF 等待：原20ms短于 MapOdomManager 50ms发布周期，实际报 future extrapolation；改为可配置 transform_timeout 默认100ms且不得大于 pose input_timeout，等待后重新检查pose年龄。仍查询原始时间，不回退到latest，不发布静态 map→odom。雷达参考圆形包络保持0.54m，inflation至少0.65m。

ASUS全14包 colcon build、colcon test、colcon test-result --verbose：58 tests，0 errors，0 failures，1 skipped。新增本地 profile 6项单测，原官方回归保留。

独立 Domain 211 的 `tools/smoke_local_site_navigation.py` 使用明确 TEST_ONLY 地图/理想动力学，实际 GICP/MapOdomManager/冻结地图启动器/Nav2/TDT/task_supervisor 链路。fixture不发布定位健康、map TF或导航权限，地图由实际map_server加载。实际35条GICP估计、205条map TF；0.8m目标动作成功，最终位置约(-0.000275,0.655301)，仍受默认约0.15m到达容差约束；后续停止子图输入触发定位失效并停止运动输出。

失败的动态TF日志保留在ASUS runtime；修复后报告 `releases/20261021_sensor_reference/log/local_site_navigation_report.json`。该回归不连接物理串口，不能当作实车冻结地图定位、实际路径精度或自主导航验收。
