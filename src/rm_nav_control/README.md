# rm_nav_control

Controller baseline, Omni PID and yaw authority.

This package is part of the RM Nav V2 staged implementation.

`CommandSynthesizer` clamps world-frame velocity commands and emits zero during a safety gate. It does not own serial transport or strategy policy.
