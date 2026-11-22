extends "res://tests/ingame/suite.gd"
## Sun fraction: the sky's share of each luxel's light, and the sun's direct share of it, bound to
## world materials as lm_sun (r, g) and added to the light grid as sun_slices. Only a shader hook
## reads it, so only a hook bakes it.

const HOOK := "res://examples/shader_hook/hook.gdshaderinc"


func run() -> void:
	_not_baked_without_a_hook()
	_brush_entities_get_the_sun()
	_the_direct_sun_is_split_out()


func _not_baked_without_a_hook() -> void:
	var bsp := load_map()
	if bsp == null:
		return
	var root := track(bsp.build_scene())
	check_eq(_lm_sun_means(root, false).size(), 0, "no hook, no lm_sun on any surface")
	check(not root.has_meta("sun"), "no hook, no sun meta")
	check(not bsp.bake_light_grid(64.0).has("sun_slices"), "no hook, no sun_slices in the grid")


## Doors, walls and ladders stand in the same sun as the ground around them; at night they
## have to go dark with it.
func _brush_entities_get_the_sun() -> void:
	var bsp := load_map()
	if bsp == null:
		return
	bsp.shader_hook = HOOK
	var root := track(bsp.build_scene())
	var means := _lm_sun_means(root, true)
	check(means.size() > 0, "ww_ravine's brush entities have lm_sun surfaces")
	var lit := means.filter(func(m: float) -> bool: return m > 0.05).size()
	check(lit > means.size() / 2,
		"most outdoor brush-entity surfaces take a sun share (%d of %d)" % [lit, means.size()])
	check(bsp.bake_light_grid(64.0).has("sun_slices"), "with a hook the grid has sun_slices")


## For a game that lights the sun itself: how much of each luxel came straight from it, where it is
## and how bright, and which faces cast its shadows.
func _the_direct_sun_is_split_out() -> void:
	var bsp := load_map()
	if bsp == null:
		return
	bsp.shader_hook = HOOK
	var root := track(bsp.build_scene())
	var sun: Dictionary = root.get_meta("sun", {})
	check(not sun.is_empty(), "the root says where the sun is")
	if not sun.is_empty():
		check(sun["dir"].y >= 0.0, "not below the horizon (%v)" % sun["dir"])
		check(float(sun["light"]) > 0.1, "and bright (%.3f)" % float(sun["light"]))
	var sky := _lm_sun_means(root, false, 0)
	var direct := _lm_sun_means(root, false, 1)
	var lit := 0
	var within := true
	for i in sky.size():
		if direct[i] > 0.05:
			lit += 1
		if direct[i] > sky[i] + 0.02:
			within = false
	check(lit > 0, "sunlit surfaces have a direct share (%d of %d)" % [lit, sky.size()])
	check(within, "never more than the sky's whole share")
	var grid := bsp.bake_light_grid(64.0)
	var slices: Array = grid.get("sun_slices", [])
	var sunlit := false
	for img: Image in slices:
		check_eq(img.get_format(), Image.FORMAT_RG8, "sun_slices are RG")
		for y in img.get_height():
			for x in img.get_width():
				if img.get_pixel(x, y).g > 0.5:
					sunlit = true
		break
	check(grid.has("sun_color") and (root.get_meta("sun", {}) as Dictionary).has("dir"),
		"the grid says how it baked the sun, the root where it is")
	var sky_casts := 0
	var world_casts := 0
	for mi: MeshInstance3D in root.find_children("*", "MeshInstance3D", true, false):
		if mi.has_meta("is_sky_surface"):
			sky_casts += int(mi.cast_shadow != GeometryInstance3D.SHADOW_CASTING_SETTING_OFF)
		elif not mi.has_meta("is_water_surface"):
			world_casts += int(mi.cast_shadow == GeometryInstance3D.SHADOW_CASTING_SETTING_DOUBLE_SIDED)
	check_eq(sky_casts, 0, "the sun shines in through sky faces")
	check(world_casts > 0, "and the world casts from both sides")
	var entity_casts := 0
	for child in root.get_children():
		var ent: Dictionary = child.get_meta("entity", {})
		if child.name == "worldspawn" or int(ent.get("zhlt_lightflags", "0")) & 2:
			continue
		for mi: MeshInstance3D in child.find_children("*", "MeshInstance3D", true, false):
			entity_casts += int(mi.cast_shadow != GeometryInstance3D.SHADOW_CASTING_SETTING_OFF)
	check_eq(entity_casts, 0, "brush entities block no sun unless flagged opaque, as in hlrad")
	if not sunlit:
		check(true, "(the lowest grid slice is all in shade)")


## Mean lm_sun under each surface's vertices, over brush entities (or every node); `channel` 0 is
## the sky's share, 1 the direct sun's.
func _lm_sun_means(root: Node, brush_only: bool, channel := 0) -> Array[float]:
	var means: Array[float] = []
	for child in root.get_children():
		if brush_only and child.name == "worldspawn":
			continue
		for mi: MeshInstance3D in child.find_children("*", "MeshInstance3D", true, false):
			if mi.mesh == null:
				continue
			for s in mi.mesh.get_surface_count():
				var mat := mi.mesh.surface_get_material(s) as ShaderMaterial
				var tex: Texture2D = mat.get_shader_parameter("lm_sun") if mat else null
				if tex == null:
					continue
				var uv2s: PackedVector2Array = mi.mesh.surface_get_arrays(s)[Mesh.ARRAY_TEX_UV2]
				if uv2s.is_empty():
					continue
				var img := tex.get_image()
				var sum := 0.0
				for uv in uv2s:
					var c := img.get_pixel(clampi(int(uv.x * img.get_width()), 0, img.get_width() - 1),
						clampi(int(uv.y * img.get_height()), 0, img.get_height() - 1))
					sum += c.g if channel == 1 else c.r
				means.append(sum / uv2s.size())
	return means
