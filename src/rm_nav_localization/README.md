# rm_nav_localization

LIO adapter, chassis resolver, state estimation and relocalization.

This package is part of the RM Nav V2 staged implementation.

The first implemented component is `ChassisResolver`. It applies the PDF-defined relation:

```text
world_T_chassis = world_T_lidar * inverse(chassis_T_lidar(t))
```

It does not filter, publish TF, or invent timestamps. Those responsibilities stay with the Sensor Hub, `robot_localization`, and the TF authority layer.

`CandidateRegionGenerator` limits global recovery search to `max_speed * lost_time + safety_margin`, clipped to official field bounds. KISS and GICP adapters consume this region but do not own its policy.

`MapOdomManager` is the sole state holder for `map->odom`; it rejects `UNVERIFIED`, `CANDIDATE`, and `REJECTED` registration results. Point-LIO odometry remains continuous and is never reset here.

`LocalSubmapBuilder` maintains the configured rolling time window and transforms each input frame into `odom`. It reports timestamps and point count for registration quality checks.

`RelocalizationStateMachine` enforces the safe sequence `TRACKING → SUSPECT → RELOCALIZING → SAFE_STOP`; motion is disabled during global recovery and after unrecoverable failure.
