# rm_nav_planning

Nav2 path preprocessing, TDT trajectory backend and validation.

This package is part of the RM Nav V2 staged implementation.

`TimedTrajectory` carries map/environment versions and an expiry timestamp. A controller must reject a trajectory after a map correction, environment update, or expiry.

`MinimumJerkGenerator` 生成五次多项式 minimum-jerk 轨迹，采用最短 yaw 角、速度/加速度/jerk
边界和 map/environment 版本绑定；它是 TDT 的软件后端基础，输出仍需通过真实 footprint、制动和
执行器验收后才能进入物理底盘。

`PathPreprocessor` removes only redundant closely spaced points. It does not replace Nav2 planning or the final swept-footprint collision validator.

`SweptFootprintValidator` samples interpolated translation and yaw poses through an environment query. The query owns the real polygon footprint and costmap/ESDF collision logic.
