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

	var err := mdl.load_mdl(source_file)
	if err != OK:
		push_error("MDL Importer: Failed to load '%s': %s" % [source_file, error_string(err)])
		return err

	mdl.build_model()

	# Reparent children to a plain Node3D so the saved scene is GDExtension-independent
	var root := Node3D.new()
	root.name = source_file.get_file().get_basename()
	# GoldSrc models face -Z after axis swizzle; rotate to match GoldSrc game orientation
	root.rotation_degrees.y = 90.0

	var children := mdl.get_children()
	for child in children:
		mdl.remove_child(child)
		root.add_child(child)
	_set_owner_recursive(root, root)

	var scene := PackedScene.new()
	var pack_err := scene.pack(root)
	if pack_err != OK:
		push_error("MDL Importer: Failed to pack scene: %s" % error_string(pack_err))
		return pack_err

	return ResourceSaver.save(scene, save_path + "." + _get_save_extension())


func _set_owner_recursive(node: Node, owner: Node) -> void:
	for child in node.get_children():
		child.owner = owner
		_set_owner_recursive(child, owner)
