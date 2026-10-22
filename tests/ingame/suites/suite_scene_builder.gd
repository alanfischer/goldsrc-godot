extends "res://tests/ingame/suite.gd"
## BspSceneBuilder: what a converted map carries on its root, what each option changes, and
## the pipeline_id stamp that tells a consumer the scene came from another build.

const BspSceneBuilder = preload("res://addons/goldsrc/bsp_scene_builder.gd")


func run() -> void:
	var root := _build({})
	if root == null:
		return
	_defaults(root)
	_stamp_round_trips(root)
	_unbuildable_is_null()
	_options()


func _build(options: Dictionary) -> Node3D:
	var root := BspSceneBuilder.build(MAP_PATH, options)
	if root == null:
		fail("BspSceneBuilder.build(%s, %s) returned null" % [MAP_PATH, options])
		return null
	track(root)
	return root


func _defaults(root: Node3D) -> void:
	check_eq(String(root.name), "ww_ravine", "the root is named after the map")
	check(root.get_child_count() > 0, "the BSP's nodes are moved under the root")
	check(root.find_children("*", "GoldSrcBSP", true, false).is_empty(),
		"no GoldSrcBSP survives in the scene, so it loads without the extension")

	var blob: PackedByteArray = root.get_meta("bsp_data", PackedByteArray())
	check(blob.size() > 4, "bsp_data is baked onto the root")
	if blob.size() > 4:
		check_eq(blob.decode_s32(0), 30, "bsp_data is still a BSP30 file")

	var id := GoldSrcBSP.get_pipeline_id()
	check(not id.is_empty(), "the extension reports a pipeline id")
	check_eq(root.get_meta(BspSceneBuilder.PIPELINE_META, ""), id, "the root is stamped with it")

	check(root.get_meta("lightstyle_texture", null) is ImageTexture,
		"shader lightstyles are on by default, so the lightstyle texture rides along")
	check(root.get_meta("lightstyle_image", null) is Image, "and so does its image")
	check(not root.has_meta("light_grid"), "no light grid unless a cell size is asked for")
	var values := _overbright_values(root)
	check(not values.is_empty(), "the world shader has an overbright uniform")
	check(values.all(func(v): return is_equal_approx(v, 2.0)),
		"overbright 0 leaves the world shader at the 2.0 it was built with")


func _stamp_round_trips(root: Node3D) -> void:
	BspSceneBuilder.set_owner_recursive(root, root)
	var scene := PackedScene.new()
	if not check_eq(scene.pack(root), OK, "the built root packs"):
		return

	check_eq(BspSceneBuilder.scene_pipeline_id(scene), GoldSrcBSP.get_pipeline_id(),
		"the stamp reads back off the packed scene without instantiating it")
	check(not BspSceneBuilder.is_scene_stale(scene), "a scene this build made is not stale")
	check_eq(BspSceneBuilder.scene_root_meta(scene, &"no_such_key", 7), 7,
		"a missing key reads as the fallback")
	check_eq(BspSceneBuilder.scene_pipeline_id(null), "", "a null scene has no stamp")

	var other := Node3D.new()
	other.set_meta(BspSceneBuilder.PIPELINE_META, "some-other-build")
	var other_scene := PackedScene.new()
	other_scene.pack(other)
	check(BspSceneBuilder.is_scene_stale(other_scene), "another build's stamp is stale")

	other.remove_meta(BspSceneBuilder.PIPELINE_META)
	var unstamped := PackedScene.new()
	unstamped.pack(other)
	check(BspSceneBuilder.is_scene_stale(unstamped), "a scene from before the stamp is stale")
	other.free()


func _unbuildable_is_null() -> void:
	check(BspSceneBuilder.build("res://maps/no_such_map.bsp") == null, "a missing .bsp builds nothing")


func _options() -> void:
	var plain := _build({"shader_lightstyles": false, "overbright": 1.5})
	if plain:
		check(not plain.has_meta("lightstyle_texture"), "shader lightstyles off: no lightstyle texture")
		var albedos := _detail_albedos(plain)
		check(not albedos.is_empty(), "shader lightstyles off: lightmaps are detail-textured materials")
		check(albedos.all(func(c): return c.is_equal_approx(Color(1.5, 1.5, 1.5))),
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
