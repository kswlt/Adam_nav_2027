# Z迁移实测证据

源码提交5aaf827，ASUS独立目录z_migration_20261023，数据为既有20261022原始MCAP。

| 文件 | 解释 |
|---|---|
| test_result.txt | 18包工作区测试汇总；364项零错误/零失败，1可选原库交叉检查跳过 |
| mapping_45s.json | 原始前45秒的建图/保存管线 |
| prior_45s.json | 上述原生地图的先验定位管线 |
| full_acquisition_replay.json | 完整327秒采集时序派生播放，含末尾状态/覆盖率检查 |
| full_map_prior_replay.json | 583关键帧完整地图重新加载后的45秒定位 |
| scheduling.json | 派生播放调度说明；原消息字节/header/frame不改，接收时间另存CSV |
| receive_burst_expected_rejection.json | 故障证据，pipeline_pass=false是正确拒绝；不可算成功回放 |
| tf_authority.txt | 私有测试图中/tf发布者与QoS |

报告中的pose变化都是估计值，无外部真值，不能据此报告ATE/RPE或宣称实车精度通过。
内存树包含rosbag2预读器，不等于在线导航进程本身的资源消耗。
原始文件、失败日志和初版观察脚本误判的全长结果仍保留在ASUS，见上级迁移报告。
