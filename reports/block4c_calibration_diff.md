# Block 4c calibration diff

stored: `handeye-2026-11-17T12:12` -> candidate: `handeye-2026-11-18T11-block4c`

## Lens

| item | stored | candidate |
|---|---|---|
| fx | 312.774 | 831.897 |
| fy | 312.774 | 833.014 |
| cx | 320.000 | 320.000 |
| cy | 240.000 | 240.000 |
| distortion | [0.0, 0.0, 0.0, 0.0, 0.0] | [-0.102251, 0.685733, 0.001112, 0.003239, 0.0] |
| horizontal FOV | 91.3 deg | 42.1 deg |

## Mount (`camera_in_tool`, OpenGL camera axes in the so101:gripper body frame)

```
stored
[[-0.85762   0.190169 -0.477832  0.06437 ]
 [ 0.13828   0.980174  0.141906 -0.002449]
 [ 0.495344  0.055627 -0.866914 -0.024773]
 [ 0.        0.        0.        1.      ]]
candidate
[[-0.845366 -0.218268  0.48756   0.067656]
 [ 0.155227 -0.973702 -0.166759 -0.002916]
 [ 0.511136 -0.06529   0.857016 -0.010885]
 [ 0.        0.        0.        1.      ]]
```

- camera displacement: **14.28 mm**
- rotation difference: **178.69 deg**
- translation change in the tool frame: [4.0, 0.94, -13.68] mm

The rotation difference is not a 3-D small angle: the stored matrix is the
OpenCV-convention solve with its camera y/z axes flipped, which puts the
board behind the lens for every consumer that reads the file as OpenGL
(see `tools/block4c_conv.py`: stored-as-OpenGL rejects all 264 corners,
the candidate reprojects them at 1.7 px median).

## Board pose this mount produces

- solved: x -9.92 mm, y -3.13 mm, z 431.35 mm, yaw 5.67 deg
- live/board.json: x -13.74, y -3.57, z 409.5, yaw 7.14
- delta: {'x_mm': 3.82, 'y_mm': 0.44, 'z_mm': 21.85, 'yaw_deg': -1.47}

## Nothing else changed

`chain`, `width`, `height` are carried over unchanged.

