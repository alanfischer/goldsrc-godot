@tool
extends EditorImportPlugin
## Imports a GoldSrc WAD file as a directory of PNG textures.
##
## The WAD itself is saved as a minimal .tres marker. The real output is the
## generated PNG files written next to the source WAD, one per texture.


func _get_importer_name() -> String:
	return "goldsrc.wad"


func _get_visible_name() -> String:
	return "GoldSrc WAD"


func _get_recognized_extensions() -> PackedStringArray:
	return PackedStringArray(["wad"])


func _get_save_extension() -> String:
	return "tres"


func _get_resource_type() -> String:
	return "Resource"


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
			"name": "extract_textures",
			"default_value": true,
		},
		{
			"name": "output_directory",
			"default_value": "",
			"property_hint": PROPERTY_HINT_DIR,
			"hint_string": "Leave empty to extract next to the WAD file",
		},
	]


func _get_option_visibility(_path: String, _option_name: StringName, _options: Dictionary) -> bool:
	if _option_name == "output_directory":
		return _options.get("extract_textures", true)
	return true


func _import(source_file: String, save_path: String, options: Dictionary,
		_platform_variants: Array[String], _gen_files: Array[String]) -> Error:
	var extract: bool = options.get("extract_textures", true)

	var wad := GoldSrcWAD.new()
	var err := wad.load_wad(source_file)
	if err != OK:
		push_error("WAD Importer: Failed to load '%s': %s" % [source_file, error_string(err)])
		return err

	if extract:
		var output_dir: String = options.get("output_directory", "")
		if output_dir.is_empty():
			# Default: create a folder named after the WAD next to it
			output_dir = source_file.get_base_dir().path_join(
				source_file.get_file().get_basename())

		DirAccess.make_dir_recursive_absolute(output_dir)

		var names := wad.get_texture_names()
		var count := 0
		for tex_name in names:
			var tex := wad.get_texture(tex_name)
			if tex == null:
				continue
			var img := tex.get_image()
			if img == null:
				continue
			var png_path := output_dir.path_join(tex_name.to_lower() + ".png")
			img.save_png(png_path)
			count += 1

		print("WAD Importer: Extracted %d textures to %s" % [count, output_dir])

	# Save a minimal marker resource so Godot considers the import complete
	var res := Resource.new()
	return ResourceSaver.save(res, save_path + "." + _get_save_extension())
