extends RefCounted
## Builds a self-contained map scene from a .bsp: the one sequence the editor importer and any
## runtime converter both need, so the two cannot drift apart on what a converted map carries.
##
## The returned root is a plain Node3D (no GDExtension types), with the BSP's nodes under it and
## these root metadata entries:
##
##   bsp_data            PackedByteArray  stripped BSP tree for PVS and hull collision (always)
##   pipeline_id         String           which build of this extension made the scene (always)
##   lightstyle_image    Image            64x1 lightstyle values, when shader_lightstyles is on
##   lightstyle_texture  ImageTexture     the texture the world materials sample, likewise
##   light_grid          Dictionary       ambient-cube grid, when light_grid_cell_size > 0
##
## Owners are NOT set: a caller that adds nodes of its own (the importer's entity models) sets
## them afterwards with set_owner_recursive(), then packs.
##
## No `class_name` on purpose: global class names resolve from a cache only an editor run
## writes, so a bare headless run would fail to parse anything naming it. Preload it instead:
##
##   const BspSceneBuilder = preload("res://addons/goldsrc/bsp_scene_builder.gd")

const PIPELINE_META := &"pipeline_id"

const DEFAULT_OPTIONS := {
	# Godot units per GoldSrc unit.
	"scale_factor": 0.025,
	# Load every .wad in this directory instead of the ones worldspawn declares.
	"wad_directory": "",
	# Where declared WADs are looked for, in order. Empty means beside the .bsp, then res://.
	"wad_search_dirs": [],
	# WAD basenames loaded even when worldspawn does not declare them (e.g. decals.wad).
	"extra_wads": [],
	# Composite the four lightmap styles in the world shader from a 64-entry lightstyle
	# texture, so styles can be animated at runtime. GoldSrcBSP's own default.
	"shader_lightstyles": true,
	# Lightmap multiplier on world materials (GoldSrc's is 2.0). 0 leaves them as built: the
	# lightstyle shader is built at 2.0, the plain StandardMaterial3D path at 1.0.
	"overbright": 0.0,
	# Ambient-cube light grid cell size in GoldSrc units; 0 skips the bake.
	"light_grid_cell_size": 0.0,
}


## Build the scene for bsp_path. Options not given take DEFAULT_OPTIONS. Returns null (after
## reporting why) when the extension is missing or the file will not parse.
static func build(bsp_path: String, options: Dictionary = {}) -> Node3D:
	if not ClassDB.class_exists(&"GoldSrcBSP"):
		push_error("BspSceneBuilder: GoldSrcBSP is not available; build the goldsrc extension")
		return null
	var opts := DEFAULT_OPTIONS.merged(options, true)

	var bsp = ClassDB.instantiate(&"GoldSrcBSP")
	bsp.set_scale_factor(opts.scale_factor)
	bsp.shader_lightstyles = opts.shader_lightstyles
	var err: Error = bsp.load_bsp(bsp_path)
	if err != OK:
		push_error("BspSceneBuilder: could not load %s: %s" % [bsp_path, error_string(err)])
		bsp.free()
		return null

	var wad_directory: String = opts.wad_directory
	if not wad_directory.is_empty():
		_load_wad_directory(bsp, wad_directory)
	else:
		var search_dirs := PackedStringArray(opts.wad_search_dirs)
		if search_dirs.is_empty():
			search_dirs = PackedStringArray([bsp_path.get_base_dir(), "res://"])
		_load_declared_wads(bsp, search_dirs, PackedStringArray(opts.extra_wads))

	bsp.build_mesh()
	if opts.overbright > 0.0:
		apply_overbright(bsp, opts.overbright)

	var root := Node3D.new()
	var root_name := bsp_path.get_file().get_basename()
	root.name = root_name if not root_name.is_empty() else "bsp_root"

	# build_mesh() parked the stripped tree on the BSP node, which is freed below. Read it back
	# rather than calling get_bsp_blob() again, which would rebuild the whole multi-MB payload;
	# PackedByteArray is copy-on-write, so this is a refcount.
	var bsp_blob: PackedByteArray = bsp.get_meta("bsp_data", PackedByteArray())
	if not bsp_blob.is_empty():
		root.set_meta("bsp_data", bsp_blob)
	root.set_meta(PIPELINE_META, pipeline_id())

	if opts.shader_lightstyles:
		var ls_image: Image = bsp.get_lightstyle_image()
		var ls_texture: ImageTexture = bsp.get_lightstyle_texture()
		if ls_image and ls_texture:
			root.set_meta("lightstyle_image", ls_image)
			root.set_meta("lightstyle_texture", ls_texture)

	if opts.light_grid_cell_size > 0.0:
		var light_grid: Dictionary = bsp.bake_light_grid(opts.light_grid_cell_size)
		if not light_grid.is_empty():
			root.set_meta("light_grid", light_grid)

	for child in bsp.get_children():
		bsp.remove_child(child)
		root.add_child(child)
	bsp.free()
	return root


## Scale lightmapped world materials by `scale`: the world shader's overbright uniform, and the
## albedo of the detail-textured standard materials the builder falls back to.
static func apply_overbright(node: Node, scale: float) -> void:
	for child in node.get_children():
		if child is MeshInstance3D:
			var mesh: Mesh = child.mesh
			if mesh == null:
				continue
			for surf_idx in mesh.get_surface_count():
				var mat := mesh.surface_get_material(surf_idx)
				if mat is ShaderMaterial:
					mat.set_shader_parameter("overbright", scale)
				elif mat is StandardMaterial3D and mat.detail_enabled:
					mat.albedo_color = Color(scale, scale, scale)
		else:
			apply_overbright(child, scale)


## Give every node under `node` that owner so PackedScene.pack() keeps it. Instanced sub-scenes
## keep their own internals: owning their root is enough.
static func set_owner_recursive(node: Node, owner: Node) -> void:
	for child in node.get_children():
		child.owner = owner
		if child.scene_file_path.is_empty():
			set_owner_recursive(child, owner)


## This build's conversion-pipeline id: a hash of the extension's own C++ source
## (cmake/pipeline_id.cmake), so every platform's build of one source tree agrees. "" when
## the extension is missing or too old to report one.
static func pipeline_id() -> String:
	if not ClassDB.class_exists(&"GoldSrcBSP"):
		return ""
	if not ClassDB.class_has_method(&"GoldSrcBSP", &"get_pipeline_id", true):
		return ""
	return str(ClassDB.class_call_static(&"GoldSrcBSP", &"get_pipeline_id"))


## The pipeline id a packed map was built by, or "" when it predates the stamp.
static func scene_pipeline_id(scene: PackedScene) -> String:
	return str(scene_root_meta(scene, PIPELINE_META, ""))


## True when a packed map was built by a different build of the extension than this one, and
## so should be rebuilt. An extension too old to report an id never calls anything stale.
static func is_scene_stale(scene: PackedScene) -> bool:
	var current := pipeline_id()
	return not current.is_empty() and scene_pipeline_id(scene) != current


## Read one root metadata entry off a packed scene without instantiating it. Loading the
## PackedScene is cheap; the node tree is not built until instantiate().
static func scene_root_meta(scene: PackedScene, key: StringName, fallback: Variant = null) -> Variant:
	if scene == null:
		return fallback
	var state := scene.get_state()
	if state == null or state.get_node_count() == 0:
		return fallback
	var property := "metadata/" + key
	for i in state.get_node_property_count(0):
		if state.get_node_property_name(0, i) == property:
			return state.get_node_property_value(0, i)
	return fallback


static func _load_wad_directory(bsp, wad_directory: String) -> void:
	var dir := DirAccess.open(wad_directory)
	if dir == null:
		push_warning("BspSceneBuilder: could not open WAD directory '%s'" % wad_directory)
		return
	for file_name in dir.get_files():
		if file_name.get_extension().to_lower() == "wad":
			_add_wad(bsp, wad_directory.path_join(file_name))


## Load the WADs worldspawn's "wad" key declares (a ';'-separated list of GoldSrc paths, of
## which only the basename is used), then any extras, each from the first search dir holding it.
static func _load_declared_wads(bsp, search_dirs: PackedStringArray, extra_wads: PackedStringArray) -> void:
	var wad_string := ""
	for ent in bsp.get_entities():
		if ent.get("classname", "") == "worldspawn":
			wad_string = ent.get("wad", "")
			break

	var names := PackedStringArray()
	for wad_path in wad_string.split(";", false) + extra_wads:
		var wad_name := wad_path.get_file().to_lower()
		if not wad_name.is_empty() and not names.has(wad_name):
			names.append(wad_name)

	for wad_name in names:
		var found := false
		for base in search_dirs:
			var full_path := base.path_join(wad_name)
			if FileAccess.file_exists(full_path):
				_add_wad(bsp, full_path)
				found = true
				break
		if not found:
			push_warning("BspSceneBuilder: WAD '%s' not found in %s" % [wad_name, ", ".join(search_dirs)])


static func _add_wad(bsp, path: String) -> void:
	var wad = ClassDB.instantiate(&"GoldSrcWAD")
	if wad.load_wad(path) != OK:
		push_warning("BspSceneBuilder: failed to load WAD %s" % path)
		return
	bsp.add_wad(wad)
