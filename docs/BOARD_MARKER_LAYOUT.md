# Board and marker layout — measured options

Two ways to carry the four 40 mm ArUco markers were built and measured against
the same simulation gates. Only one keeps the qualified manipulation envelope.

## Requirement

A `DICT_4X4_50` marker is 6 modules across and needs a quiet zone of at least one
module, so a border of width B can host at most `0.75 × B`: 30 mm in a 40 mm
border, 41 mm in a 55 mm border. At the qualified scan poses a 40 mm marker
projects to ~44.7 px (7.4 px/module) and a 30 mm marker to ~33.5 px
(5.6 px/module), which is below the ~7 px/module this pipeline needs.

## Option A — 254 mm board, 15 mm border, markers on a thin mat (selected)

* Markers at ±152 mm on a 0.5 mm sheet under the slab; sheet 368 mm overall.
* Playing surface 3.7 mm above the table, marker face 0.5 mm above the table.
* Arm base 90 mm from the board edge, 30 mm above the table.
* **Qualified:** 64/64 source squares, 4/4 tray slots; registration 8/8 within
  2 mm / 1° in the Phase 9 held-out runs.

## Option B — 334 mm board, 55 mm border, markers in the border (rejected)

* Border 55 mm hosts a 40 mm marker with 7.5 mm quiet zones at ±139.5 mm.
* Play area pinned: same square centres, same arm base position (the mount gap
  drops from 90 to 50 mm so the base does not move).
* Reach probes pass: a1→a2, b1→b2, h1→h2, h8→h7 all complete at 0.8–1.4 mm with
  no arm-to-board contact.
* **Fails the square matrix: 51/64.** The failing squares are a1–g1 and b2, all
  rejected with "no jaw span with clearance and IK". The border now sits at
  playing height where the jaw must overhang the board edge to straddle an edge
  piece; on Option A that space is the 0.5 mm mat, low enough for the fingers.
* Registration still works from the border markers (0.88 mm / 0.10° in a smoke
  run, from one visible marker), so the failure is manipulation, not vision.

## Consequence for the build

Do not print a 55 mm border at playing height. Either keep the thin mat
(Option A), or change the border's *height* rather than its width — a border
2–3 mm below the playing surface would restore the jaw overhang, but that is a
new geometry and needs its own 64-square run before it can be claimed.

The code supports both placements: `BoardConfig.marker_plane` selects `"mat"`
or `"board"`, and `MarkerLayout.for_board()` builds the matching marker
positions (border-mounted layouts validate the quiet zone and refuse a border
that is too narrow).
