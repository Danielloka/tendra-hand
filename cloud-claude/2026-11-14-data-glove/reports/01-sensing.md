# 01 — Finger-angle sensing technologies (research agent, Sonnet)

About 6 searches and 1 page read. Numbers marked "judgment" are the agent's estimates, not verified.

| Technology | Accuracy | Drift | Slim | Notes |
|---|---|---|---|---|
| **3D magnetometer + magnet across the joint** (TMAG5273, MLX90393) | TAG (arXiv 2603.28542, 2026): < 1°, σ 0.215° on a bench; 21 DoF for < $500 | ≈ 0.02° over 1000 s | yes (chip + 4–6 mm magnet, dorsal) | 0.24° error from a PC chassis at 5 cm vs 5.69° for a Manus glove. Soft-glove slip costs accuracy compared with TAG's rigid hinges |
| Magnetic angle encoders (AS5600, MT6701) on an exoskeleton | ~0.5–1° (judgment) | none | no (8–10 mm hinges) | best for reference-grade joints; bulky, fits hands poorly |
| IMU per phalanx | 2–5° (judgment) | yaw drift | wiring heavy | good as a supplement (wrist / thumb base) |
| Flex / stretch / textile | 5–10° (judgment) | yes | yes | hysteresis, fatigue, sweat; region bend, not a hinge angle |
| String excursion (LucidGloves) | no published figure | — | fairly | curl only; multi-anchor strings could separate joints but friction/slack add error |
| Fingernail force (Mascaro & Asada PPG) | ~1 N normal, 0.5 N shear | — | yes | per-subject calibration, light sensitive; research-grade |
| sEMG (Meta wristband, emg2pose) | several degrees (judgment) | — | yes | not DIY |

**Ranking (cheap + slim + accurate + DIY):** 1) 3D magnetometer + magnet per joint; 2) magnetic encoders on thin hinges where reference accuracy is needed; 3) multi-anchor strings as a fallback. Use the wrist camera to cross-check and calibrate.

Sources: TAG https://arxiv.org/html/2603.28542v1 · ExoStart https://arxiv.org/pdf/2506.11775 · MILE https://arxiv.org/html/2512.00324v2 · OSMO https://www.emergentmind.com/papers/2512.08920 · LucidGloves https://hackaday.io/project/178243 · Mascaro thesis https://dspace.mit.edu/handle/1721.1/29920 · emg2pose https://arxiv.org/html/2412.02725v1
