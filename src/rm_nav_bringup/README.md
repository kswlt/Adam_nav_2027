# rm_nav_bringup

Launch files, profiles and NavSupervisor integration.

This package is part of the RM Nav V2 staged implementation.

`NavSupervisor` gates motion permission and speed by system state. It does not implement task planning or serial transport.

`config/profiles.yaml` keeps `stable_baseline` as the default. Enhanced modules are explicit opt-in and must pass the same replay and real-robot checks before activation.
