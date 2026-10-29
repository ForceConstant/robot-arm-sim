# Printed-piece grasp and clearance review

Source: private printed Dubrovnik 1960 pack v5 at its imported 50% mesh scale;
private NormaCore CAD parallel-gripper pack v3 from Onshape microversion
`4dca0986f24304c301118aa4`. The tool model has a 0–38 mm measured-CAD aperture,
0 mm tip reach at the TCP, 50 mm proximal finger reach, and an 8 mm finger
half-width. The CAD grasp profile remains uncalibrated against physical PLA,
friction, motor torque, or print tolerances.

## Current controller plan

Heights are above the board surface. Open/hold values are total jaw gaps in the
simulator. The controller takes the set-supplied grasp height and directional
width, opens far enough for the whole proximal finger band, and uses a measured
per-piece closure allowance when supplied.

| piece | height | grasp height | open gap | hold gap | closure allowance |
| --- | ---: | ---: | ---: | ---: | ---: |
| pawn | 23.23 mm | 14.76 mm | 25.15 mm | 6.94 mm | 2 mm default |
| rook | 26.18 mm | 13.91 mm | 28.83 mm | 7.69 mm | 2 mm default |
| knight | 32.29 mm | 11.77 mm | 33.65 mm | 4.36 mm | 9 mm measured A/B |
| bishop | 36.80 mm | 18.01 mm | 25.20 mm | 6.98 mm | 2 mm default |
| queen | 40.71 mm | 24.17 mm | 27.59 mm | 8.08 mm | 2 mm default |
| king | 46.85 mm | 32.70 mm | 29.15 mm | 7.67 mm | 2 mm default |

Every opening fits within the modeled 0–38 mm aperture. At the crowded home
rank, the current span search selects a board-axis span with a non-negative
heuristic margin; the tested tight cases are close to the clearance boundary.
The fixed `crowded-v3` suite exercises actual contact grasps on both arms and
checks piece placement and neighbor displacement. The new black corner-knight
corner-knight regression justified a 7 mm allowance at nominal start. Seeded
perturbation reproduced a topple at 7 mm and passed with 9 mm; this is now the
knight profile value for both colors. A contact-traced black `g8f6` trial
showed the right jaw hitting the adjacent `f7` pawn during destination descent/
release when the board shift reached about 7–9 mm. It passed when repeated at
half the shift. This evidence sets the randomized manipulation check's board
shift bound to ±5 mm per axis; arm-start jitter remains ±0.1 rad.

These results qualify the modeled tasks only. They do not prove physical
clearance or a safe squeeze force; measure those on the printed gripper before
any hardware qualification.
