# Clip Hull Collision System

## Overview

GoldSrc BSP maps use Minkowski-expanded "clip hulls" for player collision.
The BSP compiler pre-expands world geometry by the player's bounding box
dimensions so that the engine can treat the player as a point during
collision checks. There are four hulls:

| Hull | Purpose    | Half-extents (GS units) | BSP struct   |
|------|------------|-------------------------|--------------|
| 0    | Point/vis  | 0, 0, 0                 | BSPNode/Leaf |
| 1    | Standing   | 16, 16, 36              | BSPClipNode  |
| 2    | Large      | 32, 32, 32              | BSPClipNode  |
| 3    | Crouching  | 16, 16, 18              | BSPClipNode  |

Hull 0 matches the visible geometry exactly. Hulls 1-3 are stored as
separate BSP trees of `BSPClipNode` entries with direct contents encoding
(negative child value = contents directly: EMPTY=-1, SOLID=-2, SKY=-6).

CLIP brushes are invisible brushes that only exist in hulls 1-3 (not hull 0).
They create collision walls with no corresponding visible geometry.

## Contents values

| Value | Name  | Playable? |
|-------|-------|-----------|
| -1    | EMPTY | Yes       |
| -2    | SOLID | No        |
| -3    | WATER | Yes       |
| -4    | SLIME | Yes       |
| -5    | LAVA  | Yes       |
| -6    | SKY   | **No**    |

**CONTENTS_SKY is non-playable and must be treated identically to
CONTENTS_SOLID** in all clip hull logic. Sky brushes (CONTENTS_SKY in hull 0)
are expanded by the BSP compiler into hull 1 as SOLID — exactly like world
geometry. Players are stopped at the sky trimesh boundary before reaching sky
volumes, so SKY space is unreachable from the player's perspective.

Failing to treat SKY as non-playable (checking only `== CONTENTS_SOLID`)
causes false-positive clip brush detections: the Minkowski expansion ring of
sky brushes creates hull 1 SOLID in adjacent EMPTY space, and without the SKY
check that ring is mistaken for user-added CLIP brushes.

## Pipeline

The clip hull extraction pipeline in `src/bsp_hull.cpp` and
`src/nodes/goldsrc_bsp.cpp` runs in five stages:

### Stage 1: Walk clip tree

`walk_clip_tree(hull1_root, CONTENTS_SOLID)` recursively walks the hull 1
`BSPClipNode` tree. At each internal node, the splitting plane is pushed onto
an accumulator. The front child gets the **negated** plane; the back child gets
the **original** plane. Each plane records an `unexpand_sign` (±1) for
un-expansion.

When a CONTENTS_SOLID leaf is reached, the accumulated planes define a convex
cell. The world model bounding-box planes are added to cap infinite cells that
touch the map boundary.

### Stage 2: Un-expand hull 1 planes

Each hull 1 cell plane was offset outward by the Minkowski support during BSP
compilation. Un-expansion reverses this:

```
support = |nx| * hx + |ny| * hy + |nz| * hz
plane.dist -= support * unexpand_sign
```

If un-expansion collapses a cell to fewer than 4 vertices, a binary search over
scale ∈ [0, 1] finds the maximum un-expansion factor that still yields valid
geometry (fallback rescues ~95% of collapsed cells). If the binary search also
fails, the cell is discarded.

### Stage 3: Clip against hull 0

`clip_cell_by_hull0()` clips each un-expanded cell against the hull 0 BSP tree,
keeping only fragments that lie in non-SOLID regions (EMPTY, SKY, WATER, etc.).
This discards the portions of un-expanded cells that overlap visible world
geometry.

After this stage, each surviving fragment lies in hull 0 open space — either
inside a genuine CLIP brush volume, or in the Minkowski expansion ring of nearby
world/sky geometry that wasn't fully eliminated by un-expansion.

### Stage 4: Filter — sky pre-filter

`filter_clip_brush_cells()` begins by discarding cells that are entirely within
sky brush volumes. `touches_playable_leaf()` recursively clips a cell against
the hull 0 BSP and returns `true` if any fragment overlaps a playable leaf
(EMPTY, WATER, SLIME, or LAVA — **not SKY**). Cells that return `false` are
purely inside sky space and are unreachable.

### Stage 5: Filter — vertex and expansion ring filters

Two further stages within `filter_clip_brush_cells()` separate genuine CLIP
brush cells from world-geometry expansion ring remnants:

**Stage 1 (vertex filter):** For each cell, checks:
- Whether any vertex is in hull 0 SOLID (suggesting an expansion artifact that
  overlaps world geometry)
- Per-vertex hull 1 classification: a vertex that is h1 SOLID *and* not near a
  wall is a "clip indicator" — strong evidence the cell overlaps a real CLIP
  brush
- Cell dimension plausibility (size gates to reject slivers and thin artifacts)

**Stage 2 (expansion ring filter):** For each surviving cell, checks whether
all or most vertices are "near wall" via `vert_near_wall()`. Cells where every
vertex is inside the expansion ring of world/sky geometry are discarded.

Key sub-stages in Stage 2:

- **2aa (tolerance rescue):** Cells with all h1=EMPTY vertices (centroid
  h1=SOLID) are rescued if the centroid is h1=SOLID AND not near wall. This
  targets expansion ring artifacts at ceiling boundaries; their centroids are
  in the ring (h1=SOLID at tolerance but h1=EMPTY at exact) so they correctly
  fail and are killed.

- **2f (r_big majority-near-wall, all h1=SOLID):** Large cells where most or
  all vertices are near wall and all are h1=SOLID. The primary rescue is
  `cell_has_clip_leaf()`: if any hull 0 PLAYABLE leaf whose AABB-centre lies
  inside the cell also passes h1=SOLID + `!vert_near_wall`, the cell is
  confirmed as a clip brush. This test is identical to the Python
  `check_clip_hulls.py` 3-step test and is the most reliable discriminator
  available. Expansion ring artifacts have no such leaf (their leaf centres are
  always within `he` of the nearest wall surface); real CLIP brushes have at
  least one leaf centre in the open interior of the brush.

### cell_has_clip_leaf

`cell_has_clip_leaf(cell, …)` walks the full hull 0 BSP tree. For each
PLAYABLE leaf (EMPTY / WATER / SLIME / LAVA):

1. Computes the leaf AABB centre `lc = (mins + maxs) / 2`
2. Checks whether `lc` is inside the cell (all halfplanes: `dot ≤ dist + ε`)
3. Checks `h1(lc) == SOLID or SKY`
4. Checks `!vert_near_wall(lc)`

Returns `true` on the first match. This is a direct C++ implementation of the
Python 3-step test; the two tools agree by construction.

## vert_near_wall

`vert_near_wall(nodes, leafs, planes, hull0_root, v, he)` returns `true` if the
player AABB `[v−he, v+he]` intersects any hull 0 region the player cannot
occupy — SOLID or SKY.

It uses an **exact AABB–BSP intersection**: at each hull 0 split plane, the
AABB's signed-distance interval `[dot−support, dot+support]` is compared to
`plane.dist`. If the interval is entirely on one side, only that child is
visited. If the interval straddles the plane, **both children** are visited.

```
dot     = nx*vx + ny*vy + nz*vz
support = |nx|*hx + |ny|*hy + |nz|*hz

if dot - support >= plane.dist → front child only
if dot + support <  plane.dist → back child only
else                           → both children
```

At leaf nodes, `CONTENTS_SOLID` or `CONTENTS_SKY` returns `true` immediately.

This correctly handles all geometry regardless of orientation, including:
- Thin brushes (thinner than the hull half-extents)
- Diagonal walls at any distance
- Sky ceilings (the critical SKY case — corner sampling missed these)

The older 8-corner or 14-point sampling approaches had two failure modes:
1. They only checked `CONTENTS_SOLID`, missing SKY expansion rings entirely
2. Corner samples at `±he` overshoot thin brushes (samples land past the brush)

## Detecting user-added CLIP brushes

`tools/check_clip_hulls.py` identifies maps that have user-added CLIP brushes
(invisible collision geometry not derivable from Minkowski expansion of world
brushes). For each PLAYABLE hull 0 leaf, it samples the AABB centre point `p`
and applies three tests:

1. `classify_hull0(p)` must be PLAYABLE (EMPTY / WATER / SLIME / LAVA)
2. `classify_hull1(p)` must be SOLID or SKY — both indicate "blocked"
3. `vert_near_wall(p, HULL1_HE)` must return `False`

If all three hold, `p` is in hull 1 blocked space that is not the Minkowski
expansion ring of any nearby geometry — confirmed CLIP brush.

**Maps with user-added CLIP brushes (19):**
ww_atrocity, ww_castle, ww_december, ww_golem, ww_hiddenforest, ww_hunt,
ww_keep, ww_leyline, ww_library, ww_memnate, ww_monoliths, ww_osaka,
ww_rage, ww_ravine2, ww_roc2, ww_shrinkspell, ww_storm, ww_volcano, ww_wolteg

**Maps without user-added CLIP brushes (5):**
ww_2fort, ww_chasm, ww_checkmate, ww_feudal, ww_ravine

## Files

- `src/bsp_hull.cpp` — `walk_clip_tree`, `clip_cell_by_hull0`,
  `filter_clip_brush_cells`, `vert_near_wall`, `touches_playable_leaf`,
  `compute_cell_vertices`
- `src/bsp_hull.h` — shared types (`ConvexCell`, `HullPlane`, `CellVertex`)
  and function declarations
- `src/nodes/goldsrc_bsp.cpp` — Godot importer: runs the full pipeline and
  produces `ConvexPolygonShape3D` collision shapes + debug mesh
- `src/test_hull_csg.cpp` — standalone regression test harness
- `tools/check_clip_hulls.py` — Python tool that scans BSP files to detect
  which maps have user-added CLIP brushes

## Building the test harness

```bash
cd src
clang++ -std=c++17 -O2 -I. test_hull_csg.cpp parsers/bsp_parser.cpp bsp_hull.cpp -o test_hull_csg
./test_hull_csg ../../res/maps/
```

## Key formulas

**Minkowski support** (hull expansion distance for a plane):
```
support = |nx| * hx + |ny| * hy + |nz| * hz
```

**AABB–plane interval** (used by vert_near_wall):
```
dot     = n · v
support = |nx|*hx + |ny|*hy + |nz|*hz
interval = [dot - support, dot + support]
```

**GoldSrc to Godot coordinate transform:**
```
godot = Vector3(-gs_x * scale, gs_z * scale, gs_y * scale)
```
where `scale = 0.025`.
