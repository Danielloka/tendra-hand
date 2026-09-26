# CAD

Source design files for the hand.

- `*.f3d`: Fusion 360 archives (the source of truth)
- `*.step`: neutral exports so people without Fusion can open and modify the design

When the design changes, export both, and re-export the URDF into `../robot_description/fusion_export/`.
