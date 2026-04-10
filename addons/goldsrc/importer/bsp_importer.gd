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
	return 1 # Must run after MDL/SPR importers (order 0) so their scenes are available.


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
		# Base res:// path for resolving entity model/sprite references.
		# Set this to the folder that mirrors your GoldSrc game root inside the
		# project (e.g. res://res), so "models/headcrab.mdl" resolves correctly.
		# Leave empty when your project root already mirrors the game root.
		{
			"name": "model_directory",
			"default_value": "",
			"property_hint": PROPERTY_HINT_DIR,
			"hint_string": "",
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
		{
			"name": "occluder_exclude_normal",
			"default_value": Vector3(0, 1, 0),
		},
		{
			"name": "occluder_exclude_threshold",
			"default_value": 0.7,
			"property_hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,1.0,0.01",
		},
		{
			"name": "occluder_max_count",
			"default_value": 0,
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
	var model_directory: String = options.get("model_directory", "")
	var occluder_min_area: float = options.get("occluder_min_area", 16384.0)
	var occluder_boundary_margin: float = options.get("occluder_boundary_margin", 64.0)
	var occluder_exclude_normal: Vector3 = options.get("occluder_exclude_normal", Vector3(0, 1, 0))
	var occluder_exclude_threshold: float = options.get("occluder_exclude_threshold", 0.7)
	var occluder_max_count: int = options.get("occluder_max_count", 0)

	# Create the BSP node and configure it
	var bsp := GoldSrcBSP.new()
	bsp.set_scale_factor(scale_factor)
	bsp.set_occluder_min_area(occluder_min_area)
	bsp.set_occluder_boundary_margin(occluder_boundary_margin)
	bsp.set_occluder_exclude_normal(occluder_exclude_normal)
	bsp.set_occluder_exclude_threshold(occluder_exclude_threshold)
	bsp.set_occluder_max_count(occluder_max_count)

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

	_instantiate_entity_models(root, model_directory)

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
		# Don't recurse into instanced sub-scenes — their internal nodes belong
		# to that scene, not to this one. Setting owner on the root is enough.
		if child.scene_file_path == "":
			_set_owner_recursive(child, owner)


# Instantiate MDL/SPR scenes for each point entity that has a "model" key.
# Resolves entity paths relative to model_dir (if set) or res:// (if empty),
# mirroring the GoldSrc layout at that root.
# Falls back to a placeholder box if the asset is not found.
func _instantiate_entity_models(root: Node3D, model_dir: String) -> void:
	var base := model_dir if model_dir != "" else "res://"
	for child in root.get_children():
		if not child.has_meta("entity"):
			continue
		var ent: Dictionary = child.get_meta("entity")
		var model_path: String = ent.get("model", "")

		# Brush entities reference internal BSP sub-models ("*N") — skip them.
		if model_path.is_empty() or model_path.begins_with("*"):
			continue

		var res_path := base.path_join(model_path)
		var instance: Node3D = null

		if ResourceLoader.exists(res_path):
			var scene := load(res_path) as PackedScene
			if scene:
				instance = scene.instantiate() as Node3D

		if instance == null:
			var mi := MeshInstance3D.new()
			var box := BoxMesh.new()
			box.size = Vector3(0.3, 0.3, 0.3)
			mi.mesh = box
			instance = mi
			push_warning("BSP Importer: model not found, using placeholder for '%s'" % res_path)

		instance.name = model_path.get_file().get_basename()
		child.add_child(instance)
