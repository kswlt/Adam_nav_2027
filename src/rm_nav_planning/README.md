# rm_nav_planning

Nav2 path preprocessing, TDT trajectory backend and validation.

This package is part of the RM Nav V2 staged implementation.

`TimedTrajectory` carries map/environment versions and an expiry timestamp. A controller must reject a trajectory after a map correction, environment update, or expiry.

`PathPreprocessor` removes only redundant closely spaced points. It does not replace Nav2 planning or the final swept-footprint collision validator.
