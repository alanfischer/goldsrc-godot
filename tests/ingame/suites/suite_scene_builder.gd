extends "res://tests/ingame/suite.gd"
## GoldSrcBSP.build_scene(): what a converted map carries on its root, what each property
## changes, and the pipeline_id stamp that tells a consumer the scene came from another build.
## The properties are read by build_mesh()/build_scene(), not load_bsp(), so they can be set
## on load_map()'s node after loading.


func run() -> void:
	var root := _build({})
	if root == null:
		return
	_defaults(root)
	_stamp_round_trips(root)
	_unloaded_is_null()
	_build_mesh_carries_the_metadata()
	_declared_wads()
	_properties()


## A scene from a freshly loaded map with `props` set on its GoldSrcBSP.
func _build(props: Dictionary) -> Node3D:
	var bsp := load_map()
	if bsp == null:
		return null
	for key in props:
		bsp.set(key, props[key])
	var root := bsp.build_scene()
	check_eq(bsp.get_child_count(), 0, "build_scene() leaves the GoldSrcBSP empty")
	if root == null:
		fail("build_scene() returned null with %s" % props)
		return null
	track(root)
	return root


func _defaults(root: Node3D) -> void:
	check_eq(String(root.name), "ww_ravine", "the root is named after the map")
	check(root.get_child_count() > 0, "the BSP's nodes are moved under the root")
	check(root.find_children("*", "GoldSrcBSP", true, false).is_empty(),
		"no GoldSrcBSP survives in the scene, so it loads without the extension")
	check(root.find_children("*", "", true, false).all(func(n): return n.owner == root),
		"every node is owned by the root, so the scene packs as-is")

	var blob: PackedByteArray = root.get_meta("bsp_data", PackedByteArray())
	check(blob.size() > 4, "bsp_data is baked onto the root")
	if blob.size() > 4:
		check_eq(blob.decode_s32(0), 30, "bsp_data is still a BSP30 file")

	var id := GoldSrcBSP.get_pipeline_id()
	check(not id.is_empty(), "the extension reports a pipeline id")
	check_eq(root.get_meta("pipeline_id", ""), id, "the root is stamped with it")

	check(root.get_meta("lightstyle_texture", null) is ImageTexture,
		"shader lightstyles are on by default, so the lightstyle texture rides along")
	check(root.get_meta("lightstyle_image", null) is Image, "and so does its image")
	check(not root.has_meta("light_grid"), "no light grid unless a cell size is asked for")
	var values := _overbright_values(root)
	check(not values.is_empty(), "the world shader has an overbright uniform")
	check(values.all(func(v): return is_equal_approx(v, 2.0)), "overbright defaults to GoldSrc's 2.0")


func _stamp_round_trips(root: Node3D) -> void:
	var scene := PackedScene.new()
	if not check_eq(scene.pack(root), OK, "the built root packs"):
		return
	check_eq(GoldSrcBSP.scene_pipeline_id(scene), GoldSrcBSP.get_pipeline_id(),
		"the stamp reads back off the packed scene without instantiating it")
	check_eq(GoldSrcBSP.scene_pipeline_id(null), "", "a null scene has no stamp")

	var unstamped_root := Node3D.new()
	var unstamped := PackedScene.new()
	unstamped.pack(unstamped_root)
	unstamped_root.free()
	check_eq(GoldSrcBSP.scene_pipeline_id(unstamped), "", "a scene from before the stamp reads as no stamp")


func _unloaded_is_null() -> void:
	var bsp := GoldSrcBSP.new()
	check(bsp.build_scene() == null, "build_scene() with nothing loaded builds nothing")
	bsp.free()


## A caller that keeps the GoldSrcBSP in its tree instead gets the same metadata on it.
func _build_mesh_carries_the_metadata() -> void:
	var bsp := load_map()
	if bsp == null:
		return
	bsp.build_mesh()
	check_eq(bsp.get_meta("pipeline_id", ""), GoldSrcBSP.get_pipeline_id(), "build_mesh() stamps its own node")
	check(bsp.has_meta("bsp_data") and bsp.has_meta("lightstyle_texture"),
		"build_mesh() puts the rest of the root metadata on its own node too")


## The repo carries no WADs, so every one ww_ravine declares comes back missing.
func _declared_wads() -> void:
	var bsp := load_map()
	if bsp == null:
		return
	var missing := bsp.load_declared_wads(PackedStringArray(["res://maps"]))
	check(not missing.is_empty(), "ww_ravine declares WADs the repo does not carry")
	check(missing.has("halflife.wad"), "declared paths come back as lowercase basenames")
	check(Array(missing).all(func(n): return missing.count(n) == 1), "each missing WAD is reported once")


func _properties() -> void:
	var plain := _build({"shader_lightstyles": false, "overbright": 1.5})
	if plain:
		check(not plain.has_meta("lightstyle_texture"), "shader lightstyles off: no lightstyle texture")
		var albedos := _detail_albedos(plain)
		check(not albedos.is_empty(), "shader lightstyles off: lightmaps are detail-textured materials")
		# Textured materials start white and untextured ones 0.5 grey; overbright scales both.
		check(albedos.all(func(c): return is_equal_approx(c.r, 1.5) or is_equal_approx(c.r, 0.75)),
			"overbright scales every detail-lightmap albedo")

	var gridded := _build({"overbright": 1.5, "light_grid_cell_size": 64.0})
	if gridded:
		var values := _overbright_values(gridded)
		check(not values.is_empty(), "overbright reaches the world shader")
		check(values.all(func(v): return is_equal_approx(v, 1.5)),
			"every world shader takes the overbright asked for")
		var grid: Dictionary = gridded.get_meta("light_grid", {})
		check(grid.has("grid_dims"), "a light grid cell size bakes the grid onto the root")


## The overbright uniform of every world ShaderMaterial that has one.
func _overbright_values(root: Node) -> Array[float]:
	var out: Array[float] = []
	for mat in _materials(root):
		if mat is ShaderMaterial and mat.get_shader_parameter("overbright") != null:
			out.append(float(mat.get_shader_parameter("overbright")))
	return out


func _detail_albedos(root: Node) -> Array[Color]:
	var out: Array[Color] = []
	for mat in _materials(root):
		if mat is StandardMaterial3D and mat.detail_enabled:
			out.append(mat.albedo_color)
	return out


func _materials(root: Node) -> Array[Material]:
	var out: Array[Material] = []
	for mi in root.find_children("*", "MeshInstance3D", true, false):
		var mesh: Mesh = (mi as MeshInstance3D).mesh
		if mesh == null:
			continue
		for i in mesh.get_surface_count():
			var mat := mesh.surface_get_material(i)
			if mat:
				out.append(mat)
	return out
