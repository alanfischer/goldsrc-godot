#!/usr/bin/env python3
"""
check_clip_hulls.py

Detects user-added CLIP brushes in GoldSrc BSP v30 files.

Algorithm:
  Walk hull 0's BSP tree to visit every PLAYABLE leaf (EMPTY / WATER / SLIME /
  LAVA).  For each leaf, sample its centre point p.  When p is also solid in
  hull 1 (the standing-player clip hull), determine whether that hull-1 solid
  is a Minkowski expansion of nearby geometry, or a user-added CLIP brush.

  Disambiguation via AABB–BSP intersection (vert_near_wall):
    A point p is in the Minkowski expansion ring of any non-playable brush iff
    the player-AABB [p−he, p+he] intersects any hull 0 non-PLAYABLE region
    (SOLID, SKY, or any other non-playable content).

    We test this with an exact AABB–BSP intersection: at each BSP split plane
    we compute the AABB's signed-distance interval against that plane and only
    descend the children the AABB can actually reach.  This correctly handles
    all geometry — axis-aligned, diagonal, and thin brushes — without relying
    on sparse corner sampling.

    • If the AABB [p−he, p+he] touches any non-PLAYABLE hull 0 leaf → p is in
      the Minkowski expansion ring of world / sky geometry → skip.
    • Otherwise → p is in open playable space → confirmed CLIP brush.

  SKY is excluded from PLAYABLE_CONTENTS because the sky trimesh already stops
  the player before reaching SKY volumes, so CLIP brushes there serve no
  purpose.  Sky expansion rings are correctly filtered by vert_near_wall.

Usage:
  python3 check_clip_hulls.py ../../maps/
  python3 check_clip_hulls.py map1.bsp map2.bsp ...
"""

import struct
import sys
from pathlib import Path

# ── GoldSrc BSP format constants ──────────────────────────────────────────────

HLBSP_VERSION  = 30
MAX_BSP_LUMPS  = 15

LUMP_PLANES    = 1
LUMP_NODES     = 5
LUMP_LEAFS     = 10
LUMP_CLIPNODES = 9
LUMP_MODELS    = 14

CONTENTS_EMPTY = -1
CONTENTS_SOLID = -2
CONTENTS_WATER = -3
CONTENTS_SLIME = -4
CONTENTS_LAVA  = -5
CONTENTS_SKY   = -6

# Hull 0 leaf contents where a player can actually be present.
# SKY is intentionally excluded: the sky trimesh stops the player before
# they reach any SKY volume, so CLIP brushes there serve no purpose.
PLAYABLE_CONTENTS = {CONTENTS_EMPTY, CONTENTS_WATER, CONTENTS_SLIME, CONTENTS_LAVA}

# Hull 1 half-extents: standing player 16×16×36
HULL1_HE = (16.0, 16.0, 36.0)

# ── BSP structs (all #pragma pack(1)) ────────────────────────────────────────
#
#   BSPPlane:     float normal[3], float dist, int32 type         → 20 bytes
#   BSPNode:      int32 planenum, int16 children[2],
#                 int16 mins/maxs[3], uint16 firstface/numfaces   → 24 bytes
#   BSPLeaf:      int32 contents, int32 visofs,
#                 int16 mins/maxs[3], uint16[2], uint8[4]         → 28 bytes
#   BSPClipNode:  int32 planenum, int16 children[2]               →  8 bytes
#   BSPModel:     float mins/maxs/origin[9], int32 headnode[4],
#                 int32 visleafs/firstface/numfaces                → 64 bytes

# ── Parsing ───────────────────────────────────────────────────────────────────

class BSP:
    __slots__ = ('planes', 'nodes', 'leafs', 'leaf_centers', 'clipnodes', 'models')

    def __init__(self):
        self.planes      = []  # (nx, ny, nz, dist)
        self.nodes       = []  # (planenum, child0, child1)
        self.leafs       = []  # int contents
        self.leaf_centers = [] # (cx, cy, cz) midpoint of each leaf AABB
        self.clipnodes   = []  # (planenum, child0, child1)
        self.models      = []  # (headnode[0..3],)


def load_bsp(path):
    """Parse a GoldSrc BSP file. Returns BSP on success, None on failure."""
    data = Path(path).read_bytes()
    if len(data) < 4:
        return None

    version, = struct.unpack_from('<i', data, 0)
    if version != HLBSP_VERSION:
        return None

    lumps = []
    for i in range(MAX_BSP_LUMPS):
        ofs, size = struct.unpack_from('<ii', data, 4 + i * 8)
        lumps.append((ofs, size))

    bsp = BSP()

    # Planes (20 bytes each)
    ofs, size = lumps[LUMP_PLANES]
    for i in range(size // 20):
        nx, ny, nz, d, _ = struct.unpack_from('<ffffi', data, ofs + i * 20)
        bsp.planes.append((nx, ny, nz, d))

    # Nodes (24 bytes each) — only need planenum + 2 children
    ofs, size = lumps[LUMP_NODES]
    for i in range(size // 24):
        pnum, c0, c1 = struct.unpack_from('<ihh', data, ofs + i * 24)
        bsp.nodes.append((pnum, c0, c1))

    # Leafs (28 bytes each) — contents + AABB centre
    # Layout: int32 contents, int32 visofs, int16 mins[3], int16 maxs[3], ...
    ofs, size = lumps[LUMP_LEAFS]
    for i in range(size // 28):
        base = ofs + i * 28
        contents, = struct.unpack_from('<i', data, base)
        mn = struct.unpack_from('<hhh', data, base + 8)
        mx = struct.unpack_from('<hhh', data, base + 14)
        bsp.leafs.append(contents)
        bsp.leaf_centers.append(((mn[0]+mx[0])*0.5,
                                  (mn[1]+mx[1])*0.5,
                                  (mn[2]+mx[2])*0.5))

    # Clipnodes (8 bytes each)
    ofs, size = lumps[LUMP_CLIPNODES]
    for i in range(size // 8):
        pnum, c0, c1 = struct.unpack_from('<ihh', data, ofs + i * 8)
        bsp.clipnodes.append((pnum, c0, c1))

    # Models (64 bytes each) — skip 36 bytes of floats to get headnodes
    ofs, size = lumps[LUMP_MODELS]
    for i in range(size // 64):
        h0, h1, h2, h3 = struct.unpack_from('<iiii', data, ofs + i * 64 + 36)
        bsp.models.append((h0, h1, h2, h3))

    return bsp


# ── BSP tree queries ──────────────────────────────────────────────────────────

def classify_hull0(bsp, root, p):
    """Classify point p against hull 0. Returns leaf contents."""
    idx = root
    while idx >= 0:
        pnum, c0, c1 = bsp.nodes[idx]
        nx, ny, nz, d = bsp.planes[pnum]
        idx = c0 if (nx*p[0] + ny*p[1] + nz*p[2]) >= d else c1
    leaf = -(idx + 1)
    return bsp.leafs[leaf] if 0 <= leaf < len(bsp.leafs) else CONTENTS_SOLID


def classify_hull1(bsp, root, p):
    """Classify point p against hull 1 (clipnodes). Returns CONTENTS value."""
    idx = root
    while idx >= 0:
        if idx >= len(bsp.clipnodes):
            return CONTENTS_EMPTY
        pnum, c0, c1 = bsp.clipnodes[idx]
        if pnum >= len(bsp.planes):
            return CONTENTS_EMPTY
        nx, ny, nz, d = bsp.planes[pnum]
        idx = c0 if (nx*p[0] + ny*p[1] + nz*p[2]) >= d else c1
    return idx  # negative contents value


def vert_near_wall(bsp, hull0_root, p, he):
    """
    Returns True if the player AABB [p−he, p+he] intersects any non-PLAYABLE
    hull 0 region (SOLID, SKY, or any other non-playable content).

    Uses an exact AABB–BSP intersection test: at each split plane we check
    whether the AABB can reach either child, descending both when it straddles
    the plane.  This correctly detects all geometry regardless of orientation
    or thickness — including thin brushes and sky ceilings.

    A point is in the Minkowski expansion ring of world/sky geometry iff this
    returns True.  CLIP brush points in open playable space return False.
    """
    hx, hy, hz = he
    stack = [hull0_root]
    while stack:
        idx = stack.pop()
        if idx < 0:
            # Leaf node: check contents
            leaf = -(idx + 1)
            if leaf < 0 or leaf >= len(bsp.leafs):
                return True  # outside-world → treat as SOLID
            if bsp.leafs[leaf] not in PLAYABLE_CONTENTS:
                return True  # SOLID, SKY, or other non-playable content
            continue
        pnum, c0, c1 = bsp.nodes[idx]
        nx, ny, nz, d = bsp.planes[pnum]
        dot     = nx*p[0] + ny*p[1] + nz*p[2]
        support = abs(nx)*hx + abs(ny)*hy + abs(nz)*hz
        # AABB range against this plane: [dot-support, dot+support]
        if dot - support >= d:
            stack.append(c0)          # AABB entirely on front side
        elif dot + support < d:
            stack.append(c1)          # AABB entirely on back side
        else:
            stack.append(c0)          # AABB straddles: check both
            stack.append(c1)
    return False


# ── Hull 0 leaf enumeration ───────────────────────────────────────────────────

def collect_hull0_playable_leaf_indices(bsp, root):
    """
    Walk hull 0, collecting the indices of all PLAYABLE leaves.
    Uses an explicit stack to avoid Python recursion limits.
    """
    stack = [root]
    indices = []
    while stack:
        idx = stack.pop()
        if idx < 0:
            leaf = -(idx + 1)
            if 0 <= leaf < len(bsp.leafs) and bsp.leafs[leaf] in PLAYABLE_CONTENTS:
                indices.append(leaf)
        else:
            _, c0, c1 = bsp.nodes[idx]
            stack.append(c0)
            stack.append(c1)
    return indices


# ── Main detection ────────────────────────────────────────────────────────────

def find_clip_brush_positions(bsp):
    """
    Returns a list of (x, y, z) hull 0 leaf centres confirmed as user-added
    CLIP brush positions.  Empty list means no CLIP brushes found.

    For each PLAYABLE hull 0 leaf, sample its AABB centre p and apply the
    clip-indicator test:

      1. Confirm p is in playable hull 0 space (rounding guard).
      2. Hull 1 must be SOLID (or SKY — treat both as "blocked").
      3. vert_near_wall(p) must be False: the player AABB around p must NOT
         touch any non-playable hull 0 region.  If it does, p is inside the
         Minkowski expansion ring of world/sky geometry, not a CLIP brush.

    If all three hold → confirmed user-added CLIP brush.
    """
    if not bsp.models:
        return []

    hull0_root = bsp.models[0][0]  # headnode[0]: world BSP
    hull1_root = bsp.models[0][1]  # headnode[1]: clip hull (standing player)

    if hull1_root < 0:
        return []

    hits = []
    for leaf_idx in collect_hull0_playable_leaf_indices(bsp, hull0_root):
        p = bsp.leaf_centers[leaf_idx]

        if classify_hull0(bsp, hull0_root, p) not in PLAYABLE_CONTENTS:
            continue
        h1 = classify_hull1(bsp, hull1_root, p)
        if h1 not in (CONTENTS_SOLID, CONTENTS_SKY):
            continue
        if not vert_near_wall(bsp, hull0_root, p, HULL1_HE):
            hits.append(p)

    return hits


def has_user_clip_brushes(bsp):
    return len(find_clip_brush_positions(bsp)) > 0


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    verbose = '--positions' in sys.argv
    args = [a for a in sys.argv[1:] if a != '--positions']

    bsp_files = []
    for arg in args:
        p = Path(arg)
        if p.is_dir():
            bsp_files.extend(sorted(p.glob('**/*.bsp')))
        elif p.is_file() and p.suffix.lower() == '.bsp':
            bsp_files.append(p)
        else:
            print(f"warning: skipping {arg} (not a .bsp file or directory)", file=sys.stderr)

    if not bsp_files:
        print("No .bsp files found.")
        sys.exit(1)

    has_clips = []
    no_clips  = []
    errors    = []

    for path in bsp_files:
        try:
            bsp = load_bsp(path)
            if bsp is None:
                errors.append((path.name, "not a valid GoldSrc BSP v30"))
                continue
            positions = find_clip_brush_positions(bsp)
            if positions:
                has_clips.append(path.name)
                if verbose:
                    print(f"\n  {path.name} — {len(positions)} CLIP indicator(s):")
                    for p in positions:
                        print(f"    ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f})")
            else:
                no_clips.append(path.name)
        except Exception as e:
            errors.append((path.name, str(e)))

    total = len(bsp_files)
    print(f"\nClip hull analysis — {total} BSP file{'s' if total != 1 else ''}")
    print("=" * 60)

    if has_clips:
        print(f"\n[+] HAS user-added CLIP brushes ({len(has_clips)}):")
        for name in sorted(has_clips):
            print(f"      {name}")

    if no_clips:
        print(f"\n[-] NO user-added CLIP brushes ({len(no_clips)}):")
        for name in sorted(no_clips):
            print(f"      {name}")

    if errors:
        print(f"\n[!] Errors / skipped ({len(errors)}):")
        for name, msg in errors:
            print(f"      {name}: {msg}")

    print()


if __name__ == '__main__':
    main()
