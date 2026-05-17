# goldsrc-godot

A Godot 4.3+ GDExtension for loading GoldSrc (Half-Life 1) engine assets: BSP maps, MDL models, SPR sprites, and WAD texture archives.

![TFC ravine.bsp loaded in the Godot editor](docs/ravine_godot_editor.png)

## Features

### BSP Maps
- Full BSP30 format with face-based mesh generation
- Atlas-packed lightmaps with 64 lightstyle channels and runtime rebaking
- Embedded and WAD-referenced textures with transparency (`{` prefix alpha-scissor)
- Animated textures (`+0name` … `+9name` / `+aname` … `+jname` alternates) — frames collected at import and stored as `tex_anim_frames` metadata on each `MeshInstance3D`; driven at 10 FPS by `TextureAnimator` at runtime with no C++ dependency
- Water/liquid textures (`!` and `*` prefix) rendered with a turbulent UV-warp shader (sine-wave distortion animated via `TIME`)
- Hull 0 collision (StaticBody3D for worldspawn, AnimatableBody3D for brush entities)
- Water volume extraction as Area3D with ConvexPolygonShape3D
- Automatic occluder generation (OccluderInstance3D + PolygonOccluder3D) — see [Occluder Generation](#occluder-generation) below
- PVS (Potentially Visible Set) data parsing with RLE decompression — exposed for runtime visibility culling via `VisibilityManager` and used by `debug_occluders` mode to validate occluder effectiveness; a stripped PVS blob (PLANES, VISIBILITY, NODES, LEAFS, MODELS only) is baked into each `.scn` as `pvs_data` metadata so `VisibilityManager` can initialize without the original `.bsp`
- Worldspawn spatial splitting — walks the BSP tree to group faces into spatial clusters, producing separate MeshInstance3D nodes per group for better frustum culling
- Brush entity root nodes are AnimatableBody3D instances — meshes and collision shapes are direct children, so entity scripts extend AnimatableBody3D directly without a body wrapper
- Point entity nodes (Node3D) with entity properties stored as metadata — classname, targetname, origin, angles, and all other key-value pairs are accessible from GDScript via `node.get_meta("entity")`
- Entity lump parsing (key-value dictionaries accessible from GDScript)
- **Ambient cube light grid baking** — traces rays in 6 directions from a 3D grid through the BSP tree, samples lightmaps at hit points, and outputs slice images for `ImageTexture3D` construction. Includes flood-fill of solid cells to prevent trilinear interpolation artifacts. Provides spatially-varying directional ambient lighting for dynamic models

### MDL Models
- Skeleton3D with full bone hierarchy
- Skinned meshes with per-vertex bone weights
- All animation sequences as AnimationPlayer tracks
- Chrome and additive material flags
- Configurable scale factor

### SPR Sprites
- All frame types (single and grouped)
- Texture formats: normal, additive, index-alpha, alpha-test
- Sprite types: parallel, facing-upright, oriented, etc.
- Frame textures and origins accessible individually from GDScript
- `build_scene()` produces a self-animating Sprite3D with a `SpriteAnimationPlayer` child (GDScript node, no C++ dependency at runtime) — exported properties: `fps` (default 10.0), `loop_animation` (default true), `autoplay` (default "default"); non-looping animations auto-hide the sprite on completion

### WAD Textures
- WAD2/WAD3 format support
- Palette-based to RGBA conversion with auto-generated mipmaps
- Case-insensitive texture lookup
- Per-texture caching

## Editor Import Plugins

Drop files into a project and they auto-import:

| Format | Extension | Output | Description |
|--------|-----------|--------|-------------|
| BSP | `.bsp` | `.scn` | PackedScene with meshes, lightmaps, collision |
| MDL | `.mdl` | `.scn` | PackedScene with Skeleton3D, meshes, animations |
| SPR | `.spr` | `.scn` | PackedScene with self-animating Sprite3D and SpriteAnimationPlayer child |
| WAD | `.wad` | `.png` files | Extracts individual textures as PNGs |

All imported scenes contain only standard Godot types (Node3D, MeshInstance3D, ArrayMesh, Skeleton3D, AnimationPlayer, StaticBody3D, AnimatableBody3D, OccluderInstance3D, etc.) and do **not** require the GDExtension at runtime.

### BSP Entity Model Instantiation

The BSP importer automatically instantiates MDL/SPR scenes for point entities (monsters, props, sprites, etc.). Entity `model` keys are resolved as `res://` + the entity path, mirroring the GoldSrc layout at the project root — so `models/headcrab.mdl` maps to `res://models/headcrab.mdl`.

If the asset is not found, a small placeholder `BoxMesh` (0.3 m³) is placed at the entity position so it remains visible in the editor.

**Typical workflow:**
1. Drop your game's `models/` and `sprites/` folders into the project root (`res://`).
2. Godot auto-imports all `.mdl` and `.spr` files.
3. Import your `.bsp` — entities with models/sprites appear in the scene automatically.

### Headless Batch Conversion

Convert BSP maps from the command line without opening the editor:

```bash
godot --path <project-dir> --script res://tools/batch_convert_bsp.gd -- \
  --bsp map1.bsp --bsp map2.bsp \
  --wad-dir /path/to/wads \
  --output-dir /path/to/output \
  --scale 0.025 \
  --shader-lightstyles \
  --overbright 2.0 \
  --rotate
```

Options:
- `--bsp` — input BSP file (repeat for multiple maps)
- `--wad-dir` — directory containing `.wad` files for texture lookup
- `--output-dir` — where to write `.scn` files
- `--scale` — coordinate scale factor (default: `0.025`)
- `--shader-lightstyles` — use shader-based lightstyle animation
- `--overbright` — lightmap brightness multiplier (default: `1.0`)
- `--rotate` — rotate 180 degrees around Y to match alternate coordinate conventions

Outputs a `.scn` PackedScene file per map with all geometry, collision, occluders, entity nodes, and a `pvs_data` PackedByteArray stored as root node metadata for `VisibilityManager` initialization.

## Occluder Generation

The importer automatically generates `OccluderInstance3D` + `PolygonOccluder3D` nodes for worldspawn wall geometry. To use them at runtime, enable **Project Settings > Rendering > Occlusion Culling > Use Occlusion Culling**.

### Algorithm

1. **Face collection** — worldspawn wall faces are gathered. Sky, water, transparent, and tool textures are excluded.
2. **Coplanar grouping** — faces are grouped by quantized plane key (normal + distance) to find walls that share a plane.
3. **Connected components** — within each plane group, union-find on shared vertices identifies contiguous face patches.
4. **Boundary edge merging** — shared interior edges cancel out, leaving only the true outer boundary of each patch (plus any interior holes from doorways/windows).
5. **Loop classification**:
   - **Single loop** → solid wall, one merged occluder.
   - **Multiple loops, holes BSP-solid** → the "holes" are backed by solid geometry (e.g. recessed detail), so one merged occluder from the outer loop is safe.
   - **Multiple loops, real openings** → doorways/windows detected via BSP tree traversal. The algorithm re-runs edge cancellation on only the qualifying faces (area ≥ `occluder_min_area`). Adjacent solid panels merge into one occluder; the doorway spaces become the natural exterior boundary rather than interior holes.
6. **PVS-coverage filtering** — a greedy pass drops candidates whose marginal contribution to covering BSP-visible leaf pairs falls below `occluder_pvs_min_gain`. This removes occluders that add geometry overhead without meaningfully improving culling coverage. Runs before the area sort so the count cap applies to survivors only.
7. **Importance sorting and capping** — surviving candidates are sorted by polygon area (largest first). If `occluder_max_count` is non-zero, only the top N are kept.
8. **Polygon cleanup** — duplicate and collinear vertices are removed, and each polygon is pre-validated against Godot's triangulator before being committed as an occluder.

### Import Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `occluder_min_area` | 65535 | Minimum face area in GoldSrc units² for a face to qualify as an occluder. ~256×256 at default. Raise to get fewer, larger occluders; lower to include smaller walls. |
| `occluder_pvs_min_gain` | 500 | Greedy PVS-coverage filter threshold. Candidates whose marginal BSP-visible leaf-pair coverage is below this value are dropped before area sorting. `0` = disabled (keep all candidates). |
| `occluder_max_count` | 0 | Maximum number of occluders after sorting by area. `0` = unlimited. Applied after the PVS-coverage filter. |

These can be set per-map in the Godot Import tab or directly in the `.bsp.import` file:

```ini
[params]
occluder_min_area=65535.0
occluder_pvs_min_gain=500
occluder_max_count=0
```

### Debug Mode

Set `debug_occluders = true` on the `GoldSrcBSP` node before calling `build_mesh()` to print a full pipeline report including: face counts, component breakdown by type (solid/solid-holes/real-openings/walk-failures), occluder coverage percentage, overfill checks on merged polygons, and PVS validation (what fraction of BSP-invisible leaf pairs have an occluder plane between them).

## Building

Requires CMake 3.22+ and a C++17 compiler.

```bash
git clone --recursive https://github.com/alanfischer/goldsrc-godot.git
cd goldsrc-godot
mkdir build && cd build
cmake ..
make -j8
```

The compiled library goes to `addons/goldsrc/bin/`. Open the project in Godot and enable the GoldSrc plugin under Project Settings > Plugins.

## GDScript API

### GoldSrcBSP

```gdscript
var bsp = GoldSrcBSP.new()
bsp.scale_factor = 0.025

var wad = GoldSrcWAD.new()
wad.load_wad("textures.wad")
bsp.add_wad(wad)

bsp.load_bsp("map.bsp")
bsp.build_mesh()

# Entity data is on the child nodes as metadata:
for child in bsp.get_children():
    if child.has_meta("entity"):
        var ent = child.get_meta("entity")  # Dictionary
        print(ent.get("classname", ""))
        print(ent.get("targetname", ""))

# Or get raw entity dictionaries:
var entities = bsp.get_entities()  # Array of Dictionaries

# Optional: tune occluder generation (before build_mesh)
bsp.occluder_min_area = 65535.0   # min face area in GoldSrc units²
bsp.occluder_pvs_min_gain = 500   # greedy PVS-coverage filter; 0 = disabled
bsp.occluder_max_count = 0        # cap on occluder count after PVS filter (0 = unlimited)
bsp.debug_occluders = true        # prints PVS validation, overfill checks, pipeline stats

# PVS query API (useful for runtime visibility culling)
var leaf_count = bsp.get_leaf_count()                  # total BSP leaf count
var leaf = bsp.point_to_leaf(Vector3(0, 0, 0))         # leaf index for a world position
var visible_leaves = bsp.get_leaf_pvs(leaf)            # PackedInt32Array of PVS-visible leaf indices
var aabb_leaves = bsp.get_leaves_in_aabb(my_aabb)      # PackedInt32Array of leaves touching an AABB

# Export a stripped PVS blob (PLANES, VISIBILITY, NODES, LEAFS, MODELS only).
# Embed this in .scn metadata so VisibilityManager can initialize without the .bsp.
var pvs_blob: PackedByteArray = bsp.get_pvs_blob()

# Load a stripped PVS blob for PVS-only queries (no WADs or build_mesh needed)
bsp.load_bsp_from_data(pvs_blob)

# Bake ambient cube light grid (call after build_mesh)
var grid = bsp.bake_light_grid(32.0)  # cell size in GoldSrc units
# Returns Dictionary with:
#   grid_origin: Vector3 — world-space origin in Godot coords
#   grid_dims: Vector3i — grid dimensions in Godot coords (X, Y, Z)
#   cell_size: float — cell size in Godot units
#   dir_slices: Array of 6 Arrays of Images — one per axis direction
#     (+X, -X, +Y, -Y, +Z, -Z), each array contains depth-slice Images
#     for ImageTexture3D construction
```

### VisibilityManager

BSP PVS-based runtime visibility culling. Load the BSP once (no WADs or mesh build needed) and query observer/entity visibility per-frame.

```gdscript
var vm = VisibilityManager.new()
add_child(vm)

# Preferred: initialize from pvs_data metadata baked into the .scn at import time.
# No .bsp file required at runtime.
var pvs_blob: PackedByteArray = map_root.get_meta("pvs_data", PackedByteArray())
var leaf_count = vm.setup_from_data(pvs_blob, 0.025)  # returns leaf count; 0 = failure

# Alternative: load directly from a .bsp file
var leaf_count = vm.setup("maps/mymap.bsp", 0.025)

# Observers — a viewpoint whose PVS is cached and updated lazily (only on leaf change)
var cam_handle = vm.add_observer(0, camera.global_position)   # peer_id=0 for local camera
vm.update_observer_position(cam_handle, camera.global_position)

# Entities — tracked by world-space AABB; leaf set recomputes when AABB changes
var ent_handle = vm.register_entity(my_node.get_aabb())
vm.update_entity_aabb(ent_handle, my_node.get_aabb())
var visible = vm.is_entity_visible_to_observer(ent_handle, cam_handle)

# Leaf sets — for pre-baked leaf arrays (e.g. worldspawn spatial groups)
var set_handle = vm.register_leaf_set(packed_int32_leaves)
var ls_visible = vm.is_leaf_set_visible_to_observer(set_handle, cam_handle)

# Point queries
var leaf_vis = vm.is_leaf_visible_to_observer(leaf_index, cam_handle)
var pos_vis  = vm.is_position_visible_to_observer(some_pos, cam_handle)

# Cleanup
vm.remove_observer(cam_handle)
vm.unregister_entity(ent_handle)
vm.unregister_leaf_set(set_handle)
vm.teardown()
```

### GoldSrcMDL

```gdscript
var mdl = GoldSrcMDL.new()
mdl.scale_factor = 0.025
mdl.load_mdl("model.mdl")
mdl.build_model()

print(mdl.get_sequence_count())    # Number of animations
print(mdl.get_sequence_name(0))    # First animation name
print(mdl.get_bone_count())        # Number of bones
```

### GoldSrcSPR

```gdscript
var spr = GoldSrcSPR.new()
spr.load_spr("sprite.spr")

var frame_count = spr.get_frame_count()
var texture = spr.get_frame_texture(0)  # ImageTexture
var origin = spr.get_frame_origin(0)    # Vector2i — up/left extent from entity position
var spr_type = spr.get_type()           # SPR_VP_PARALLEL, etc.

# Build a self-animating Sprite3D scene (no C++ dependency at runtime):
var sprite = spr.build_scene()          # Sprite3D with SpriteAnimationPlayer child
# Override animation properties before adding to tree:
var sap = sprite.get_node("SpriteAnimationPlayer")
sap.fps = 20.0
sap.loop_animation = false
# Frames and origins are stored as metadata on the Sprite3D:
#   sprite.get_meta("tex_anim_frames")   # Array[ImageTexture]
#   sprite.get_meta("tex_anim_origins")  # Array of [ox, oy] pairs
```

### GoldSrcWAD

```gdscript
var wad = GoldSrcWAD.new()
wad.load_wad("textures.wad")

var names = wad.get_texture_names()     # PackedStringArray
var tex = wad.get_texture("concrete1")  # ImageTexture
```

## Coordinate System

GoldSrc uses Z-up; Godot uses Y-up. The plugin converts automatically:
- Positions: `(x, y, z)` &rarr; `(-x * scale, z * scale, y * scale)`
- Quaternions: conjugation by -90&deg; X rotation

Default scale factor is `0.025` (1 GoldSrc unit = 0.025 Godot units).
