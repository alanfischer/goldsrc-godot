extends "res://tests/ingame/suite.gd"
## build_scene: the half of goldsrc_bsp.cpp that turns parsed faces into a Godot scene
## tree. Nothing outside an engine can call it at all, which is the reason this
## harness exists.


func run() -> void:
	_scale_factor()
	_builds_a_scene_tree()
	_builds_once_per_load()
	_face_axes()


func _scale_factor() -> void:
	var bsp := GoldSrcBSP.new()

	check_near(bsp.get_scale_factor(), 0.025, 1e-6,
		"default scale is the GoldSrc-to-metres factor")
	bsp.set_scale_factor(0.5)
	check_near(bsp.get_scale_factor(), 0.5, 1e-6, "scale factor round-trips")
	bsp.set_scale_factor(0.025)


func _builds_a_scene_tree() -> void:
	var bsp := load_map()
	if bsp == null:
		return

	var root := bsp.build_scene()
	if not check(root != null, "a loaded map builds"):
		return
	track(root)

	# One child per entity: worldspawn plus every brush entity.
	check_eq(root.get_child_count(), 75, "build_scene emits a node per entity")

	var worldspawn := root.get_node_or_null("worldspawn")
	check(worldspawn != null, "worldspawn is present by name")
	if worldspawn != null:
		check(worldspawn.get_child_count() > 0,
			"worldspawn is split into spatial groups")

	var stats := _walk(root)
	check(stats.meshes > 0, "the tree contains MeshInstance3D nodes (got %d)" % stats.meshes)
	check_eq(stats.meshes_without_surfaces, 0,
		"every emitted mesh has at least one surface")
	check(stats.bodies > 0,
		"the tree contains collision bodies (got %d)" % stats.bodies)


## The lightmap atlases and lightstyles belong to the one scene built from them, so a second
## build_scene() from the same load builds nothing; loading again builds the same scene.
func _builds_once_per_load() -> void:
	var bsp := load_map()
	if bsp == null:
		return

	var first := bsp.build_scene()
	if not check(first != null, "the first build_scene() builds"):
		return
	track(first)
	check(bsp.build_scene() == null, "a second build_scene() from the same load builds nothing")
	check_eq(bsp.load_bsp(MAP_PATH), OK, "the map loads again")
	var again := bsp.build_scene()
	if not check(again != null, "a fresh load builds again"):
		return
	track(again)
	check_eq(again.get_child_count(), first.get_child_count(), "and builds the same tree")


## get_face_axes returns the texture S and T axes for the face under a point, used to align
## decals. Two vectors, always.
func _face_axes() -> void:
	var bsp := load_map()
	if bsp == null:
		return

	var axes: Array = bsp.get_face_axes(Vector3(-26.4, 49.6, 36.4), Vector3(0, 1, 0))
	check_eq(axes.size(), 2, "face axes come back as an S/T pair")
	if axes.size() == 2:
		check(axes[0] is Vector3 and axes[1] is Vector3, "both axes are Vector3")
		check_near((axes[0] as Vector3).length(), 1.0, 0.01, "the S axis is unit length")
		check_near((axes[1] as Vector3).length(), 1.0, 0.01, "the T axis is unit length")


func _walk(root: Node) -> Dictionary:
	var out := {"meshes": 0, "meshes_without_surfaces": 0, "bodies": 0}
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		if n is MeshInstance3D:
			out.meshes += 1
			var m: Mesh = (n as MeshInstance3D).mesh
			if m == null or m.get_surface_count() == 0:
				out.meshes_without_surfaces += 1
		if n.is_class("StaticBody3D"):
			out.bodies += 1
		for c in n.get_children():
			stack.append(c)
	return out
