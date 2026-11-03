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

## Option B — 334 mm board, 55 mm border, markers in the border (not yet usable)

* Border 55 mm hosts a 40 mm marker with 7.5 mm quiet zones at ±139.5 mm.
* Play area pinned: same square centres, same arm base (the mount gap drops from
  90 to 50 mm so the base does not move).
* Reach probes pass: a1→a2, b1→b2, h1→h2, h8→h7 complete at 0.8–1.4 mm with no
  arm-to-board contact.
* Registration works from the border markers (0.88 mm / 0.10° in a smoke run).
* **Open question, not a border verdict.** On the same all-types scan this
  layout scored 51/64 while the unchanged 254 mm/15 mm layout scored 52/64,
  failing the same near-rank squares with IK error 8–13 mm against an 8 mm
  gate; the border was therefore not implicated. The historical 64/64 receipt
  counted "at least one piece type per square, with king/rook retries", which is
  weaker than every type passing.
* **Seeding fix.** `grasp_span` now retries each jaw-span direction from a
  warm start (same target, no tool offset) and a fixed set of fallback postures,
  because the local QP can miss a reachable target from one seed. That lifted
  the all-types scan from 52/64 to **56/64** on the unchanged layout. The eight
  remaining squares fail per *piece type*, not per square: every one is
  reachable from an arbitrary posture (b1 at 0.0 mm), so what remains is a
  global IK fallback rather than a geometric limit.

## What that means for the overhang argument

An earlier version of this note blamed the border for blocking an open jaw that
had to pass tens of millimetres outside the board edge. That was wrong. The open
jaw is 28.8 mm against a 28 mm square, so a finger passes the edge by well under
a millimetre, and `jaw_clearance` only measures clearance to other pieces, not to
the board at all. The border is a mount-clearance cost (40 mm of the 90 mm gap),
not a grasp-volume conflict.

## Consequence for the build

Keep the thin mat (Option A) for the current build: it is qualified, needs no
mount change, and the border alternative has not been shown to hold the same
matrix. The 55 mm border is still a legitimate option on marker grounds, but it
costs 40 mm of mount clearance and must be requalified — including resolving why
the all-types scan fails the near rank in both layouts.

The code supports both placements: `BoardConfig.marker_plane` selects `"mat"`
or `"board"`, and `MarkerLayout.for_board()` builds the matching marker
positions (border-mounted layouts validate the quiet zone and refuse a border
that is too narrow).
