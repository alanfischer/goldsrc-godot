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
(negative child value = contents: -2 = SOLID, -1 = EMPTY).

CLIP brushes are invisible brushes that only exist in hulls 1-3 (not hull 0).
They create collision walls with no corresponding visible geometry.

## Problem

To use these hulls for Godot collision, we need to:

1. Walk the clip BSP tree to extract convex solid cells
2. **Un-expand** each cell back to its original brush geometry (reverse the
   Minkowski expansion) so collision matches the actual wall surfaces
3. Generate `ConvexPolygonShape3D` collision shapes from the result

The un-expansion (step 2) is the hard part. Naively subtracting the
Minkowski support from every plane causes cells to collapse or produce
incorrect geometry.

## Algorithm

### Tree Walk

`walk_clip_tree()` recursively walks the `BSPClipNode` tree. At each node,
it pushes the splitting plane onto an accumulator:
- Front child gets the **negated** plane (point is on the front side)
- Back child gets the **original** plane (point is on the back side)

Each plane also records its `sibling_child` — the opposite child at that
split, which tells us what's on the other side of that face.

When a CONTENTS_SOLID leaf is reached, the accumulated planes define a
convex cell.

### Cell Splitting

For each face of a cell, the sibling subtree may itself be a sub-tree
(not a single leaf). `split_cell_for_face()` walks the sibling subtree
and splits the cell along any splitting planes that cross the face, so
each sub-cell's face borders exactly one leaf. This gives us per-face
knowledge of whether the neighbor is SOLID or EMPTY.

### Selective Un-expansion

For each plane of each sub-cell, we decide whether to un-expand:

**Pass 1 — Determine what to un-expand:**
- If sibling = EMPTY (or water, etc.): un-expand (this face borders open space)
- If sibling = SOLID: keep expanded (internal face between solid cells),
  UNLESS a thickness-gated probe fired one support distance outside the
  face lands in EMPTY (detects thin BSP artifact layers)

**Pass 2 — Antiparallel slab clamping:**
For nearly-antiparallel plane pairs (dot < -0.8) that are both being
un-expanded, check if the slab between them is too thin:
- `slab_gap = di + dj` (direct plane-distance gap, NOT vertex-based thickness)
- If `total_support > slab_gap`, clamp proportionally, preserving a minimum
  8 GS unit wall thickness

**Pass 3 — Apply:** Subtract (clamped) Minkowski support from each plane's
distance.

### Fallback: Binary Search

If a cell collapses after un-expansion (< 4 vertices), restore original
distances and binary search over scale [0, 1] to find the maximum
un-expansion that produces valid geometry. This rescues ~95% of
collapsed cells.

### Thin-Cell Filter

BSP tree splitting creates degenerate slivers (e.g. 1 GS unit thick
horizontal cells at hull expansion boundaries). After un-expansion,
any cell with an axis-aligned bounding box dimension < 4 GS units is
discarded.

### Collision Shape Generation

Each surviving cell's vertex point cloud (already in Godot coordinates)
is fed to `ConvexPolygonShape3D::set_points()`. All shapes are children
of a single `StaticBody3D` on collision layer 1.

A semi-transparent purple debug mesh is also generated from the same
cells for visual verification.

## Key Formulas

**Minkowski support** (how much a plane is expanded for a given hull):
```
support = |nx| * hx + |ny| * hy + |nz| * hz
```

**Slab gap** (distance between antiparallel planes, for clamping):
```
slab_gap = di + dj    (where nj ≈ -ni)
```

**GoldSrc to Godot coordinate transform:**
```
godot = Vector3(-gs_x * scale, gs_z * scale, gs_y * scale)
```
where `scale = 0.025`.

## Files

- `src/nodes/goldsrc_bsp.cpp` — `build_clip_hull_debug()` and supporting
  functions in anonymous namespace (`walk_clip_tree`, `split_cell_for_face`,
  `compute_cell_vertices`, `triangulate_convex_cell`, `clip_tree_contents`,
  `minkowski_support`)
- `src/nodes/goldsrc_bsp.h` — method declaration
- `test_clip_hull.cpp` — standalone regression tests (no Godot dependency)

## Testing

Build and run the regression tests against any GoldSrc BSP:

```bash
cd extern/goldsrc-godot
clang++ -std=c++17 -O2 -I src -o test_clip_hull test_clip_hull.cpp src/parsers/bsp_parser.cpp
./test_clip_hull ../../res/maps/ww_golem.bsp
```

The tests verify three historical bugs that were fixed:

1. **angled_wall** — Antiparallel plane clamping used vertex-based thickness
   (overstated) instead of slab gap formula. Walls collapsed.
2. **clip_brush_wall** — Pure CLIP brush walls (hull 0 EMPTY) were clamped
   to near-zero thickness. Minimum wall thickness of 8 GS units fixes this.
3. **no_thin_artifact** — BSP tree splitting artifacts (1 GS unit thick
   horizontal slivers) survived as floating geometry. Min-dimension filter
   removes them.

## Known Issues

- Some cells still collapse and can't be rescued by binary search (~1%)
- The min-dimension filter (4 GS) might occasionally remove legitimate
  thin geometry
- Hull 3 (crouching) is hardcoded; hull 1 (standing) would need different
  half-extents

## Credits

Algorithm design and implementation by Alan Fischer and Claude (Anthropic).
The selective un-expansion approach, slab gap formula, binary search fallback,
and thin-cell filter were developed iteratively through testing against
ww_golem.bsp with the standalone regression test harness.
