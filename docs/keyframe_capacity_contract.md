# 关键帧容量契约

当前 GTSAM PoseGraph 和 `offline_graph_optimizer` 的单图上限是 500 个关键帧。
录制器现将 `max_keyframes` 默认值和允许范围统一为 500：

- 100、300、500 帧：允许录制，离线优化器可以读取。
- 第 501 帧：触发明确 quota fault，停止当前 session；不删除、不覆盖已有 CDR，
  不静默丢弃已接受数据。
- 超过 500 帧的场地：应停止当前 session，记录下一段的起点并启动新的 archive session。
  当前版本没有把多个 session 自动拼成一个 GTSAM 图，不能宣称跨段优化已经完成。

这样保留原始关键帧完整性，同时让录制器和优化器的容量契约一致。后续若引入 Submap
分段，需要另行定义跨段边界、内存上限和结果合并格式。
