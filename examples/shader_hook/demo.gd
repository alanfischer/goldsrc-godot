@tool
extends Node3D
## Drives hook.gdshaderinc on the map under this node. Open demo.tscn and drag the sliders in the
## inspector: the map changes in the editor viewport, no running needed.
##
## The map's materials come out of its import, so this sets their uniforms the same way a game
## would at runtime: walk the map, find the ShaderMaterials, set_shader_parameter. Nothing here
## is saved into the imported scene.

## Moonlight, as the multiplier on the sky's light at full night.
const MOON := Vector3(0.12, 0.14, 0.24)

@export_range(0.0, 1.0) var night := 0.0:
	set(v):
		night = v
		_apply()
@export_range(0.0, 1.0) var wetness := 0.0:
	set(v):
		wetness = v
		_apply()
@export_range(0.0, 2.0) var lightning := 0.0:
	set(v):
		lightning = v
		_apply()


func _ready() -> void:
	_apply()


func _apply() -> void:
	if not is_inside_tree():
		return
	var sky_light := Vector3.ONE.lerp(MOON, night) + Vector3.ONE * lightning
	var map := get_child(0) if get_child_count() > 0 else null
	var rain_floor: Dictionary = map.get_meta("rain_floor", {}) if map else {}
	var floor_tex: Texture2D = null
	for mat in _materials(map, []):
		mat.set_shader_parameter("sky_light", sky_light)
		mat.set_shader_parameter("flash_color", Vector3.ONE * lightning)
		mat.set_shader_parameter("wetness", wetness)
		if not rain_floor.is_empty() and mat.get_shader_parameter("rain_floor") == null:
			if floor_tex == null:
				floor_tex = ImageTexture.create_from_image(rain_floor["image"])
			mat.set_shader_parameter("rain_floor", floor_tex)
			mat.set_shader_parameter("rain_floor_origin", rain_floor["origin"])
			mat.set_shader_parameter("rain_floor_size", rain_floor["size"])
			mat.set_shader_parameter("has_rain_floor", true)


## Every ShaderMaterial under `node`, once each: the importer shares one per texture.
func _materials(node: Node, out: Array[ShaderMaterial]) -> Array[ShaderMaterial]:
	if node == null:
		return out
	if node is MeshInstance3D and (node as MeshInstance3D).mesh:
		var mesh := (node as MeshInstance3D).mesh
		for i in mesh.get_surface_count():
			var mat := mesh.surface_get_material(i) as ShaderMaterial
			if mat and not mat in out:
				out.append(mat)
	for child in node.get_children():
		_materials(child, out)
	return out
