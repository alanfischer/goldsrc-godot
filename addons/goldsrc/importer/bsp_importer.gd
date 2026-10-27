@tool
extends EditorImportPlugin

## Bump when this script changes what an import produces. Godot itself only compares it when
## something else already sent a file to be re-tested (its source or .import changed), so the
## plugin's own check is what acts on it — see import_key().
const FORMAT_VERSION := 1
## Project metadata section recording, per .bsp, the import_key() it was last imported with.
const IMPORTED_WITH := "goldsrc_bsp_imported_with"


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


func _get_format_version() -> int:
	return FORMAT_VERSION


## What an import made now would be built by: this extension build and this script's version.
## A map recorded with any other key is stale (plugin.gd reimports it).
static func import_key() -> String:
	return "%s/%d" % [GoldSrcBSP.get_pipeline_id(), FORMAT_VERSION]


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
		# Comma-separated WAD paths searched after the declared ones, for a project that ships
		# its own stand-ins for WADs it can't redistribute. Set it project-wide in
		# [importer_defaults]. A declared WAD that's present still wins.
		{
			"name": "fallback_wads",
			"default_value": "",
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
		# Composite the lightmap styles in the world shader from a 64-entry lightstyle texture,
		# so switchable and flickering lights can be animated at runtime. The texture rides in
		# the scene's "lightstyle_texture" metadata.
		{
			"name": "shader_lightstyles",
			"default_value": true,
		},
		# Lightmap multiplier on world materials. GoldSrc's is 2.0.
		{
			"name": "overbright",
			"default_value": 2.0,
			"property_hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,4.0,0.05",
		},
		# Bake an ambient-cube light grid into the scene's "light_grid" metadata, for lighting
		# models by where they stand. Cell size in GoldSrc units; 0 skips the bake.
		{
			"name": "light_grid_cell_size",
			"default_value": 0.0,
			"property_hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.0,256.0,1.0",
		},
	]


func _get_option_visibility(_path: String, _option_name: StringName, _options: Dictionary) -> bool:
	return true


func _import(source_file: String, save_path: String, options: Dictionary,
		_platform_variants: Array[String], _gen_files: Array[String]) -> Error:
	var bsp := GoldSrcBSP.new()
	bsp.scale_factor = options.get("scale_factor", 0.025)
	bsp.shader_lightstyles = options.get("shader_lightstyles", true)
	bsp.overbright = options.get("overbright", 2.0)
	bsp.light_grid_cell_size = options.get("light_grid_cell_size", 0.0)
	var err := bsp.load_bsp(source_file)
	if err != OK:
		bsp.free()
		return err

	# An explicit wad_directory loads every WAD in it. Otherwise the WADs worldspawn declares
	# are looked up beside the .bsp, then at res://, so a stock project textures correctly
	# with no per-file import config.
	var wad_directory: String = options.get("wad_directory", "")
	if wad_directory != "":
		_load_wad_directory(bsp, wad_directory)
	else:
		var missing := bsp.load_declared_wads()
		var fallbacks := _load_fallback_wads(bsp, options.get("fallback_wads", ""))
		if fallbacks == 0:
			for name in missing:
				push_warning("BSP Importer: WAD '%s' not found for %s" % [name, source_file])

	var root := bsp.build_scene()
	bsp.free()
	_instantiate_entity_models(root, options.get("model_directory", ""))

	var scene := PackedScene.new()
	var pack_err := scene.pack(root)
	root.free()
	if pack_err != OK:
		push_error("BSP Importer: Failed to pack scene: %s" % error_string(pack_err))
		return pack_err

	var save_err := ResourceSaver.save(scene, save_path + "." + _get_save_extension())
	if save_err == OK:
		EditorInterface.get_editor_settings().set_project_metadata(IMPORTED_WITH, source_file, import_key())
	return save_err


func _load_fallback_wads(bsp: GoldSrcBSP, paths: String) -> int:
	var loaded := 0
	for path in paths.split(",", false):
		path = path.strip_edges()
		var wad := GoldSrcWAD.new()
		if wad.load_wad(path) == OK:
			bsp.add_wad(wad)
			loaded += 1
		else:
			push_warning("BSP Importer: fallback WAD '%s' could not be loaded" % path)
	return loaded


func _load_wad_directory(bsp: GoldSrcBSP, wad_directory: String) -> void:
	var dir := DirAccess.open(wad_directory)
	if dir == null:
		push_warning("BSP Importer: Could not open WAD directory '%s'" % wad_directory)
		return
	for file_name in dir.get_files():
		if file_name.get_extension().to_lower() == "wad":
			var wad := GoldSrcWAD.new()
			if wad.load_wad(wad_directory.path_join(file_name)) == OK:
				bsp.add_wad(wad)


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

		var node_name := model_path.get_file().get_basename()
		if node_name.is_empty():
			node_name = "unnamed_entity_model"
		instance.name = node_name
		child.add_child(instance)
		# build_scene() owned everything it built; the model has to be owned too to be packed.
		instance.owner = root
