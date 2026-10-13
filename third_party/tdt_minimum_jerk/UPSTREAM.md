# TDT polynomial backend

Source: https://github.com/T-DT-Algorithm-2026/tdt-nav-kit
Revision: `814d5ce0237acc7faa8c070e5c5e61e2ddc81996`
Files: `src/MinimumSnapOsqp/minimumSnap.*`, `sfcSquare.*`; MIT license retained in LICENSE.

Project patches are recorded in `tools/patches/tdt_timing_output.patch`:
expose final segment times and descending-power coefficients after collision refinement;
record normalized-time mode; add explicit standard includes to the corridor header.
No QP, corridor, time-allocation or collision-refinement algorithm was replaced.

Six coefficients (`setOrder(6)`) mean degree five; `setMaxDx(3)` selects the third-derivative objective.
The wrapper does not infer execution timing or velocities from the upstream visualization polyline.
XY is solved in grid-local coordinates to avoid mixing rotated grid origins with world coordinates.
Yaw is an independent scalar TDT polynomial with current wz and a zero terminal wz; intermediate
Nav2 waypoint headings are path tangents and do not constrain the chassis yaw policy.
