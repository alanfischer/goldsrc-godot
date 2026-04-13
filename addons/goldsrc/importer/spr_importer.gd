@tool
extends EditorImportPlugin


func _get_importer_name() -> String:
	return "goldsrc.spr"


func _get_visible_name() -> String:
	return "GoldSrc SPR"


func _get_recognized_extensions() -> PackedStringArray:
	return PackedStringArray(["spr"])


func _get_save_extension() -> String:
	return "scn"


func _get_resource_type() -> String:
	return "PackedScene"


func _get_preset_count() -> int:
	return 1


func _get_preset_name(preset_index: int) -> String:
	return "Default"


func _get_priority() -> float:
	return 1.0


func _get_import_order() -> int:
	return 0


func _can_import_threaded() -> bool:
	return false


func _get_import_options(_path: String, _preset_index: int) -> Array[Dictionary]:
	return [
		{
			"name": "animation_fps",
			"default_value": 10.0,
			"property_hint": PROPERTY_HINT_RANGE,
			"hint_string": "1.0,60.0,0.5",
		},
		{
			"name": "loop",
			"default_value": true,
		},
	]


func _get_option_visibility(_path: String, _option_name: StringName, _options: Dictionary) -> bool:
	return true


func _import(source_file: String, save_path: String, options: Dictionary,
		_platform_variants: Array[String], _gen_files: Array[String]) -> Error:
	var fps: float = options.get("animation_fps", 10.0)
	var loop: bool = options.get("loop", true)

	var spr := GoldSrcSPR.new()
	var err = spr.load_spr(source_file)
	if err != OK:
		push_error("SPR Importer: Failed to load '%s': %s" % [source_file, error_string(err)])
		return err

	var frame_count = spr.get_frame_count()
	if frame_count == 0:
		push_error("SPR Importer: No frames in '%s'" % source_file)
		return ERR_INVALID_DATA

	var sprite = spr.build_scene()
	if not sprite:
		push_error("SPR Importer: build_scene() failed for '%s'" % source_file)
		return ERR_CANT_CREATE

	# Bake import options into the SpriteAnimationPlayer child
	var sap = sprite.get_node_or_null(^"SpriteAnimationPlayer")
	if sap:
		sap.fps = fps
		sap.loop_animation = loop

	var scene := PackedScene.new()
	var pack_err := scene.pack(sprite)
	sprite.free()
	if pack_err != OK:
		push_error("SPR Importer: Failed to pack scene for '%s': %s" % [source_file, error_string(pack_err)])
		return pack_err

	return ResourceSaver.save(scene, save_path + "." + _get_save_extension())
