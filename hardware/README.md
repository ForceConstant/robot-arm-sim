# hardware

**Current simulator layout:** [current_layout.json](current_layout.json) and
[qualification](../docs/LAYOUT_OPTIMIZATION.md): 90 mm gap to the base-frame
origin, extended 30 mm base (`base_so101_v2_ext30.stl`, 462.4 mm frame datum)
and 3.2 mm board. The base bottom is coplanar with the board underside at
430 mm, so a single base board can carry both; no separate riser is used. Older
generated rig drawings/parts below retain their historical dimensions; they
were not regenerated or physically qualified by this simulation-only layout
experiment. `product_twin.json` is the dated stock-tool Phase 8 baseline.

Generated build files for the physical rig. The guide, with the drawings and
what to buy or print, is [docs/hardware.md](../docs/hardware.md).

| | |
| --- | --- |
| `dimensions.json` | the whole build sheet, as generated from `chess_sim` |
| `stl/` | four printable parts: riser, mast plate, mast tube, capture tray |
| `parts.scad` | the same parts parametrically, for adjusting them |
| `board_254mm.png` | the board artwork at 300 dpi, print at 100% |
| `generate.py` | rewrites all of the above, plus the figures in `docs/media/` |
| `diagrams.py` | the scale drawings, as SVG |

```bash
MUJOCO_GL=egl python hardware/generate.py
```

Nothing here is hand-written: change `chess_sim/config.py` and re-run.
