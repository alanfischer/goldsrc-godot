@tool
extends EditorImportPlugin


func _get_importer_name() -> String:
	return "goldsrc.mdl"


func _get_visible_name() -> String:
	return "GoldSrc MDL"


func _get_recognized_extensions() -> PackedStringArray:
	return PackedStringArray(["mdl"])


func _get_save_extension() -> String:
	return "scn"


func _get_resource_type() -> String:
	return "PackedScene"


func _get_preset_count() -> int:
	return 1


func _get_preset_name(preset_index: int) -> String:
	return "Default"


## What shape of scene this importer builds. 1: meshes merged into one MeshInstance3D per
## bodypart. 2: triangle corners welded back into shared vertices. 3: animation keys cut to the
## ones interpolation can't reproduce, skins stored as lossless WebP.
##
## Godot records it in each .import file but does not act on a bump — neither --import nor an
## editor session reimports on its own (checked on 4.7). Changing the built scene means clearing
## .godot/imported/*.mdl-* by hand; the number is here so it is visible which build a cache came
## from.
func _get_format_version() -> int:
	return 4


func _get_priority() -> float:
	return 1.0


func _get_import_order() -> int:
	return 0


func _can_import_threaded() -> bool:
	return false


func _get_import_options(_path: String, _preset_index: int) -> Array[Dictionary]:
	return [
		{
			"name": "scale_factor",
			"default_value": 0.025,
			"property_hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.001,1.0,0.001",
		},
	]


func _get_option_visibility(_path: String, _option_name: StringName, _options: Dictionary) -> bool:
	return true


func _import(source_file: String, save_path: String, options: Dictionary,
		_platform_variants: Array[String], _gen_files: Array[String]) -> Error:
	var scale_factor: float = options.get("scale_factor", 0.025)

	var mdl := GoldSrcMDL.new()
	mdl.set_scale_factor(scale_factor)

	var err = mdl.load_mdl(source_file)
	if err != OK:
		push_error("MDL Importer: Failed to load '%s': %s" % [source_file, error_string(err)])
		return err

	var root := mdl.build_scene()
	var scene := PackedScene.new()
	var pack_err := scene.pack(root)
	root.free()
	if pack_err != OK:
		push_error("MDL Importer: Failed to pack scene: %s" % error_string(pack_err))
		return pack_err

	return ResourceSaver.save(scene, save_path + "." + _get_save_extension())

