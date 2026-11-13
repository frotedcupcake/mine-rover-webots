# Meshes

Put the Fusion 360 export of the chassis here as `chassis.stl` (or `.obj`).

- Units: **metres**. Axes: **X forward, Y left, Z up**. Origin at the centre of the
  0.25 x 0.20 x 0.07 m chassis box.
- Turn it on with `useMeshes TRUE` on the `MineRover` node (see the main README).
- The mesh is visual only. Collision stays simple boxes and cylinders.

`chassis.stl` in this folder is a plain box placeholder, used to check that the
`useMeshes` path loads.
