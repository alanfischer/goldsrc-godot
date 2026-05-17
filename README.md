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
- Worldspawn spatial splitting — walks the BSP tree to group faces into spatial clusters, producing separate MeshInstance3D nodes per group for better frustum culling; each group node has `pvs_leaves` metadata (PackedInt32Array) for use with `VisibilityManager.register_leaf_set()`
- Brush entity root nodes are AnimatableBody3D instances — meshes and collision shapes are direct children
- Point entity root nodes are plain Node3D instances
- Entity properties stored as metadata on each node — classname, targetname, origin, angles, and all other key-value pairs are accessible from GDScript via `node.get_meta("entity")`
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

## PVS Runtime Visibility Culling

### Concept

BSP maps are divided into convex regions called *leaves*. During map compilation, the compiler precomputes which leaves can see each other — the Potentially Visible Set (PVS) — stored as a run-length–encoded bitfield. Knowing an observer's current leaf lets you look up the full set of visible leaves in O(1) with no raycasting.

`VisibilityManager` wraps this data with two abstractions:

- **Observers** — viewpoints (camera or network peers) whose PVS is cached and updated lazily: the PVS bitfield is only recomputed when the observer crosses a BSP leaf boundary. Positions within the same leaf reuse the cached result at no cost.
- **Leaf sets** — sets of BSP leaves that represent a tracked object, either derived from an AABB pushed through the BSP tree (for moving entities) or pre-baked from import metadata (for static mesh groups). Visibility is a single bitset scan across the leaf set.

### Data Flow

```
.bsp import
    └─ BSP importer strips PVS-relevant lumps (PLANES, VISIBILITY, NODES, LEAFS, MODELS)
       and bakes them into .scn as pvs_data metadata on the root node
           └─ runtime: setup_from_data(pvs_data, scale_factor)
                  └─ VisibilityManager ready — no .bsp file needed at runtime
```

The `pvs_data` blob is written by both the editor importer and the headless batch converter. Retrieve it from the instantiated map scene:

```gdscript
var pvs_blob: PackedByteArray = map_root.get_meta("pvs_data", PackedByteArray())
var leaf_count = vm.setup_from_data(pvs_blob, 0.025)
if leaf_count == 0:
    push_error("PVS setup failed — pvs_data missing or corrupt")
```

### Use Case 1: Rendering Culling

The BSP importer groups worldspawn faces into spatial clusters and emits a separate `MeshInstance3D` per group. Each group node has `pvs_leaves` metadata (a `PackedInt32Array`) stamped by the builder. Register these as leaf sets once on map load, then cull per-frame.

```gdscript
var vm: VisibilityManager
var cam_handle: int
var mesh_leaf_sets: Dictionary  # MeshInstance3D -> leaf_set handle

func _ready() -> void:
    vm = VisibilityManager.new()
    add_child(vm)

    var pvs_blob = map_root.get_meta("pvs_data", PackedByteArray())
    if vm.setup_from_data(pvs_blob, 0.025) == 0:
        return

    cam_handle = vm.add_observer(0, camera.global_position)

    for mesh_node in map_root.find_children("*", "MeshInstance3D", true, false):
        if mesh_node.has_meta("pvs_leaves"):
            var leaves: PackedInt32Array = mesh_node.get_meta("pvs_leaves")
            mesh_leaf_sets[mesh_node] = vm.register_leaf_set(leaves)

func _process(_delta: float) -> void:
    vm.update_observer_position(cam_handle, camera.global_position)
    for mesh_node in mesh_leaf_sets:
        var handle: int = mesh_leaf_sets[mesh_node]
        mesh_node.visible = vm.is_leaf_set_visible_to_observer(handle, cam_handle)
```

### Use Case 2: Network Entity Culling

On a multiplayer server, give each connected peer its own observer. Skip sending entity updates to peers whose PVS doesn't cover the entity's leaves.

```gdscript
var vm: VisibilityManager
var peer_handles: Dictionary   # peer_id -> observer handle
var entity_handles: Dictionary # node -> entity handle

func _ready() -> void:
    vm = VisibilityManager.new()
    add_child(vm)
    var pvs_blob = map_root.get_meta("pvs_data", PackedByteArray())
    vm.setup_from_data(pvs_blob, 0.025)

func on_peer_connected(peer_id: int, initial_pos: Vector3) -> void:
    peer_handles[peer_id] = vm.add_observer(peer_id, initial_pos)

func on_peer_disconnected(peer_id: int) -> void:
    vm.remove_observer(peer_handles[peer_id])
    peer_handles.erase(peer_id)

func on_peer_moved(peer_id: int, pos: Vector3) -> void:
    vm.update_observer_position(peer_handles[peer_id], pos)

func register_game_entity(node: Node3D) -> void:
    entity_handles[node] = vm.register_entity(node.get_aabb())

func on_entity_moved(node: Node3D) -> void:
    vm.update_entity_aabb(entity_handles[node], node.get_aabb())

func send_updates_to_peer(peer_id: int) -> void:
    var obs = peer_handles.get(peer_id, -1)
    if obs == -1:
        return
    for node in entity_handles:
        if vm.is_entity_visible_to_observer(entity_handles[node], obs):
            send_entity_update(peer_id, node)
```

### Handle Lifetime

All handles are stable integers until the matching `remove_observer` / `unregister_entity` / `unregister_leaf_set` call. Handles from removed entries are recycled internally — do not use a handle after removal.

Call `teardown()` (or free the node) when unloading a map. All registered observers, entities, and leaf sets are released automatically.

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

BSP PVS-based runtime visibility culling. See [PVS Runtime Visibility Culling](#pvs-runtime-visibility-culling) for end-to-end examples.

```gdscript
var vm = VisibilityManager.new()
add_child(vm)

# Preferred: initialize from pvs_data metadata baked into the .scn at import time.
# No .bsp file required at runtime.
var pvs_blob: PackedByteArray = map_root.get_meta("pvs_data", PackedByteArray())
var leaf_count = vm.setup_from_data(pvs_blob, 0.025)  # returns leaf count; 0 = failure

# Alternative: load directly from a .bsp file (no WADs or mesh build needed)
var leaf_count = vm.setup("maps/mymap.bsp", 0.025)

# Observers — a viewpoint with a cached PVS bitfield (one bool per BSP leaf).
# PVS is recomputed only when the observer crosses a BSP leaf boundary.
# peer_id=0 is conventional for the local camera; use the multiplayer peer ID for remote players.
var cam_handle = vm.add_observer(0, camera.global_position)
vm.update_observer_position(cam_handle, camera.global_position)

# Entities — tracked by world-space AABB; leaf set recomputes when AABB changes.
var ent_handle = vm.register_entity(my_node.get_aabb())
vm.update_entity_aabb(ent_handle, my_node.get_aabb())
var visible = vm.is_entity_visible_to_observer(ent_handle, cam_handle)

# Leaf sets — for pre-baked leaf arrays (worldspawn spatial groups from pvs_leaves metadata).
# The leaf set never changes after registration.
var set_handle = vm.register_leaf_set(packed_int32_leaves)
var ls_visible = vm.is_leaf_set_visible_to_observer(set_handle, cam_handle)

# Point queries
var leaf_vis = vm.is_leaf_visible_to_observer(leaf_index, cam_handle)
var pos_vis  = vm.is_position_visible_to_observer(some_pos, cam_handle)

# Cleanup — or call teardown() / free the node to release everything at once.
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
