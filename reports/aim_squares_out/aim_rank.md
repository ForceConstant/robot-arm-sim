
## Where it wins and where it loses (by rank)

Median aim error, per rank, over the 8 files:

| rank | tip mode mm | aim_point mm | n |
|---|---|---|---|
| 1 | 10.0 | 9.5 | 7 / 7 |
| 2 | 11.8 | 5.0 | 8 / 8 |
| 3 | 12.0 | 3.8 | 8 / 8 |
| 4 | 9.1 | 4.9 | 8 / 7 |
| 5 | 7.0 | 10.3 | 8 / 8 |
| 6 | 7.6 | 7.3 | 8 / 7 |
| 7 | 5.6 | 4.6 | 8 / 8 |
| 8 | 7.4 | 5.4 | 6 / 6 |

Aim mode is a large, unambiguous win on ranks 2-4 (11.6 -> 4.6 mm median), which
is where the reach geometry tilts the tool and the ray is furthest from the tip.
It is a small loss on rank 5 (7.0 -> 10.3 mm), where the tip already lands
inside 5-8 mm and the correction is below the deadband: the attempt adds a few
millimetres of noise and no travel. It also loses by more than 1 mm on 13
squares in total (a5, a6, a7, b5, c5, d5, e5, e6, f5, f6, f8, g4, g7).

**Operating rule this supports:** command the aim ray only when the initial aim
offset exceeds ~10 mm (about the deadband in board millimetres at this height).
Below that, leave the tip placement alone - the correction cannot be actuated and
the loop only perturbs a good pose.
