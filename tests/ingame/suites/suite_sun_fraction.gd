extends "res://tests/ingame/suite.gd"
## Sun fraction: the sky's share of each luxel's light, bound to world materials as lm_sun and
## added to the light grid as sun_slices. Only a shader hook reads it, so only a hook bakes it.

const HOOK := "res://examples/shader_hook/hook.gdshaderinc"


func run() -> void:
	_not_baked_without_a_hook()
	_brush_entities_get_the_sun()


func _not_baked_without_a_hook() -> void:
	var bsp := load_map()
	if bsp == null:
		return
	bsp.build_mesh()
	check_eq(_lm_sun_means(bsp, false).size(), 0, "no hook, no lm_sun on any surface")
	check(not bsp.bake_light_grid(64.0).has("sun_slices"), "no hook, no sun_slices in the grid")


## Doors, walls and ladders stand in the same sun as the ground around them; at night they
## have to go dark with it.
func _brush_entities_get_the_sun() -> void:
	var bsp := load_map()
	if bsp == null:
		return
	bsp.shader_hook = HOOK
	bsp.build_mesh()
	var means := _lm_sun_means(bsp, true)
	check(means.size() > 0, "ww_ravine's brush entities have lm_sun surfaces")
	var lit := means.filter(func(m: float) -> bool: return m > 0.05).size()
	check(lit > means.size() / 2,
		"most outdoor brush-entity surfaces take a sun share (%d of %d)" % [lit, means.size()])
	check(bsp.bake_light_grid(64.0).has("sun_slices"), "with a hook the grid has sun_slices")


## Mean lm_sun under each surface's vertices, over brush entities (or every node).
func _lm_sun_means(bsp: GoldSrcBSP, brush_only: bool) -> Array[float]:
	var means: Array[float] = []
	for child in bsp.get_children():
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
					sum += img.get_pixel(clampi(int(uv.x * img.get_width()), 0, img.get_width() - 1),
						clampi(int(uv.y * img.get_height()), 0, img.get_height() - 1)).r
				means.append(sum / uv2s.size())
	return means
