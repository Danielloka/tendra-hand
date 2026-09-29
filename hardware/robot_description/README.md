# Robot description

Kinematic/physical models of the hand.

- `fusion_export/`: the **raw, unedited** export from Fusion 360 (made with the `fusion2urdf` plugin, a ROS 1 package format).
  Don't edit it by hand: re-export from Fusion instead, so nothing gets lost.
  Its `LICENSE` and `package.xml` are template leftovers from the plugin and don't describe this project.

- `v1_export/`: the v1 hand (20 DOF), written by the `TendraHandV1` Fusion script (stage `export`): `hand_v1.json` (parts + joints, world mm, all joints at 0; Fusion's axis signs are arbitrary), `meshes/*.stl`, and `tendon_routes.json` from `hardware/cad/tendon_router.py`. `sim/convert_v1.py` turns it into `sim/models/tendra_hand_v1.xml`.

The cleaned models (correct PLA masses, recomputed inertias, descriptive joint names, flexion-positive axes, MuJoCo MJCF) are **generated** from this export by a script in `sim/`.

### Known issues in the current export
- Material is set to steel (total mass 821 g instead of about 130 g)
- Inertias rounded to 1e-6; two are invalid
- Placeholder effort/velocity limits (100)
- Thumb joints 7 and 8 have opposite axis signs
