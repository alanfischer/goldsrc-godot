extends "res://tests/ingame/suite.gd"
## build_mesh: the 3,500-line half of goldsrc_bsp.cpp that turns parsed faces into a Godot
## scene tree. Nothing outside an engine can call it at all, which is the reason this
## harness exists.


func run() -> void:
	_scale_factor()
	_builds_a_scene_tree()
	_is_idempotent()
	_face_axes()


func _scale_factor() -> void:
	var bsp := GoldSrcBSP.new()
	track(bsp)

	check_near(bsp.get_scale_factor(), 0.025, 1e-6,
		"default scale is the GoldSrc-to-metres factor")
	bsp.set_scale_factor(0.5)
	check_near(bsp.get_scale_factor(), 0.5, 1e-6, "scale factor round-trips")
	bsp.set_scale_factor(0.025)


func _builds_a_scene_tree() -> void:
	var bsp := load_map()
	if bsp == null:
		return

	check_eq(bsp.get_child_count(), 0, "nothing is built until build_mesh is called")
	bsp.build_mesh()

	# One child per entity: worldspawn plus every brush entity.
	check_eq(bsp.get_child_count(), 643, "build_mesh emits a node per entity")

	var worldspawn := bsp.get_node_or_null("worldspawn")
	check(worldspawn != null, "worldspawn is present by name")
	if worldspawn != null:
		check(worldspawn.get_child_count() > 0,
			"worldspawn is split into spatial groups")

	var stats := _walk(bsp)
	check(stats.meshes > 0, "the tree contains MeshInstance3D nodes (got %d)" % stats.meshes)
	check_eq(stats.meshes_without_surfaces, 0,
		"every emitted mesh has at least one surface")
	check(stats.bodies > 0,
		"the tree contains collision bodies (got %d)" % stats.bodies)


## Rebuilding must replace the tree, not append to it. Getting this wrong doubles a map's
## geometry every time a level reloads, which is invisible until frame rate collapses.
func _is_idempotent() -> void:
	var bsp := load_map()
	if bsp == null:
		return

	bsp.build_mesh()
	var first := bsp.get_child_count()
	bsp.build_mesh()
	check_eq(bsp.get_child_count(), first, "building twice does not duplicate the tree")


## get_face_axes returns the texture S and T axes for the face under a point, used to align
## decals. Two vectors, always.
func _face_axes() -> void:
	var bsp := load_map()
	if bsp == null:
		return

	var axes: Array = bsp.get_face_axes(Vector3(-35.875, 6.05, 40.65), Vector3(0, 1, 0))
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
