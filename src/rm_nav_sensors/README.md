# rm_nav_sensors

Sensor adapters and ObservationFrame/ObservationBatch glue.

This package is part of the RM Nav V2 staged implementation.

`ObservationFrame` and `ObservationBatch` are the Sensor Hub contract. Each sensor keeps its own origin, timestamp, frame and health; downstream modules must not assume an already merged point cloud is the only source of truth.
