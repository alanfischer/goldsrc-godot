@tool
extends EditorImportPlugin


func _get_importer_name() -> String:
	return "goldsrc.bsp"


func _get_visible_name() -> String:
	return "GoldSrc BSP"


func _get_recognized_extensions() -> PackedStringArray:
	return PackedStringArray(["bsp"])


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


# Must run on main thread — ArrayMesh/ImageTexture serialization deadlocks
# when ResourceSaver.save() is called from import worker threads.
func _can_import_threaded() -> bool:
	return false


func _get_import_options(_path: String, _preset_index: int) -> Array[Dictionary]:
	return [
		{
			"name": "wad_directory",
			"default_value": "",
			"property_hint": PROPERTY_HINT_DIR,
			"hint_string": "",
		},
		{
			"name": "scale_factor",
			"default_value": 0.025,
			"property_hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.001,1.0,0.001",
		},
		{
			"name": "occluder_min_area",
			"default_value": 65535.0,
			"property_hint": PROPERTY_HINT_RANGE,
			"hint_string": "0,262144,1",
		},
		{
			"name": "occluder_boundary_margin",
			"default_value": 512.0,
			"property_hint": PROPERTY_HINT_RANGE,
			"hint_string": "0,1024,1",
		},
	]


func _get_option_visibility(_path: String, _option_name: StringName, _options: Dictionary) -> bool:
	return true


func _import(source_file: String, save_path: String, options: Dictionary,
		_platform_variants: Array[String], _gen_files: Array[String]) -> Error:
	var wad_directory: String = options.get("wad_directory", "")
	var scale_factor: float = options.get("scale_factor", 0.025)
	var occluder_min_area: float = options.get("occluder_min_area", 16384.0)
	var occluder_boundary_margin: float = options.get("occluder_boundary_margin", 64.0)

	# Create the BSP node and configure it
	var bsp := GoldSrcBSP.new()
	bsp.set_scale_factor(scale_factor)
	bsp.set_occluder_min_area(occluder_min_area)
	bsp.set_occluder_boundary_margin(occluder_boundary_margin)

	# Load WAD files if a directory is specified
	if wad_directory != "":
		var dir := DirAccess.open(wad_directory)
		if dir:
			dir.list_dir_begin()
			var file_name := dir.get_next()
			while file_name != "":
				if not dir.current_is_dir() and file_name.get_extension().to_lower() == "wad":
					var wad := GoldSrcWAD.new()
					var wad_path := wad_directory.path_join(file_name)
					wad.load_wad(wad_path)
					bsp.add_wad(wad)
				file_name = dir.get_next()
			dir.list_dir_end()
		else:
			push_warning("BSP Importer: Could not open WAD directory '%s'" % wad_directory)

	# Load and build the BSP
	bsp.load_bsp(source_file)
	bsp.build_mesh()

	# Reparent children to a plain Node3D so the saved scene is GDExtension-independent
	var root := Node3D.new()
	root.name = source_file.get_file().get_basename()

	var children := bsp.get_children()
	for child in children:
		bsp.remove_child(child)
		root.add_child(child)
	_set_owner_recursive(root, root)

	# Pack and save
	var scene := PackedScene.new()
	var pack_err := scene.pack(root)
	if pack_err != OK:
		push_error("BSP Importer: Failed to pack scene: %s" % error_string(pack_err))
		return pack_err

	var save_err := ResourceSaver.save(scene, save_path + "." + _get_save_extension())
	return save_err


func _set_owner_recursive(node: Node, owner: Node) -> void:
	for child in node.get_children():
		child.owner = owner
		_set_owner_recursive(child, owner)
