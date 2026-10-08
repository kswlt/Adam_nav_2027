# rm_nav_registration

Unified small_gicp and KISS registration adapters.

This package is part of the RM Nav V2 staged implementation.

`registration_types.hpp` is the shared result contract for local GICP, KISS coarse registration, AMCL/GICP and loop closure. It intentionally does not implement a registration algorithm or accept a result based only on a library `converged` flag.

`localization_validator.hpp` applies configurable inlier, residual, degeneracy, jump and confidence gates. Large corrections become `CANDIDATE` and require a second observation before `map->odom` can be updated.
