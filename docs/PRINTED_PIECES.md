# Printed STL piece sets

The browser lab supports `ROBOT_PIECE_SET=printed` with
`ROBOT_PIECE_ASSETS=/absolute/path/to/pack`. Missing requested assets fail startup;
there is no silent fallback to the generated pieces. Without these settings the
public checkout retains the original set and needs no private geometry.

## Import once, then use locally

```sh
python tools/import_printed_set.py --source /path/to/six-stls \
  --output .private/piece-assets --name 'My printed set · 50%' --scale 0.5
ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS="$PWD/.private/piece-assets" \
  MUJOCO_GL=egl .venv/bin/python -m uvicorn robot_web.app:app --port 18098
```

Offline import requires `trimesh`, `numpy`, `shapely`, and `pillow`; it is not
part of the runtime requirements. Supply six z-up millimetre STLs named
Pawn/Rook/Knight/Bishop/Queen/King.stl. The importer preserves orientation, centres
the foot, places its lowest vertex at z=0 and applies one uniform mm-to-m scale.
Both colours use identical meshes, dimensions and inertials, with material tint
only. Appearance `piece_scale=1` preserves these absolute dimensions; no automatic
per-piece height fitting or square-dependent resizing. Oversized printed pieces
are rejected instead of independently shrinking the pieces.

Detailed OBJ visuals retain all supplied triangles. Real contact uses cached
horizontal convex compounds per disconnected solid (at most 0.75 mm tall), not one convex hull over the whole
piece and not the legacy four-cylinder proxy. Within-slab concavities are convexified, including underside weight pockets;
these pockets are filled for stable table contact, not reproduced as usable holes.
The manifest records this approximation, slab height and source hashes.
A 48-slab conservative radial profile is used only for controller clearance;
it does not replace the asymmetric knight's physical contact geometry. The
existing controller still uses heuristic/oracle-state planning, not a full
orientation-aware world collision planner.

Mass, COM and inertia come from the supplied closed meshes at an **estimated
effective print density of 600 kg/m³**, without extra nuts, felt or magnets.
Changing infill or adding weights requires remeasuring mass/inertia. Contact
friction is unchanged and is not calibrated PLA-on-PLA hardware data. No gripper
geometry, force limits, success tolerances or hidden attachments are changed.

## Local geometry and publication boundary

The supplied Dubrovnik 1960 source is attributed to **ezee.one**, MakerWorld model
[406463](https://makerworld.com/en/models/406463-dubrovnik-1960-bobby-fischer-chess-set).
Its listing uses the Standard Digital File License; do not redistribute source
STLs, converted meshes or derived collisions in the public repository, runtime
archive or registry images. Geometry is supplied only in the authorized private
LAN Docker build context, alongside the existing private gripper pack.

`set.json` holds `scale="absolute"`, `name`, `uniform_source_scale`, and `pieces`
keyed by lowercase type. Each piece contains `collision_meshes` relative to the
pack root, `profile` slabs in metres, `mass_kg`, `center_mass_m`, and the full
`inertia_kg_m2` tensor. Visuals and light texture live at `<type>/<type>.obj` and
`<type>/material_0.png`.

The user-supplied set is imported at 50% on the unchanged 28 mm board. Approximate
height/base-width pairs in mm: king 46.85/19.64, queen 40.71/19.24,
bishop 36.80/17.29, knight 32.29/18.08, rook 26.18/17.69, pawn 23.23/15.69.
These are geometry measurements, not a claim of reliable manipulation.

Test source/pack geometry, reset/settling, both independent arms and real browser
controls before deployment. New geometry invalidates old incident playback hashes
by design. Run `crowded-v1` after every authorized redeploy; report individual
manipulation failures as data, never hide or loosen the scoring.
