# rm_nav_registration

Unified small_gicp and KISS registration adapters.

This package is part of the RM Nav V2 staged implementation.

`registration_types.hpp` is the shared result contract for local GICP, KISS coarse registration, AMCL/GICP and loop closure. It intentionally does not implement a registration algorithm or accept a result based only on a library `converged` flag.
