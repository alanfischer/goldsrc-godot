@tool
extends EditorPlugin

const BspSceneBuilder = preload("res://addons/goldsrc/bsp_scene_builder.gd")

var bsp_importer: EditorImportPlugin
var mdl_importer: EditorImportPlugin
var spr_importer: EditorImportPlugin
var wad_importer: EditorImportPlugin


func _enter_tree() -> void:
	bsp_importer = preload("res://addons/goldsrc/importer/bsp_importer.gd").new()
	mdl_importer = preload("res://addons/goldsrc/importer/mdl_importer.gd").new()
	spr_importer = preload("res://addons/goldsrc/importer/spr_importer.gd").new()
	wad_importer = preload("res://addons/goldsrc/importer/wad_importer.gd").new()
	add_import_plugin(bsp_importer)
	add_import_plugin(mdl_importer)
	add_import_plugin(spr_importer)
	add_import_plugin(wad_importer)
	_reimport_stale_maps.call_deferred()


func _exit_tree() -> void:
	remove_import_plugin(bsp_importer)
	remove_import_plugin(mdl_importer)
	remove_import_plugin(spr_importer)
	remove_import_plugin(wad_importer)
	bsp_importer = null
	mdl_importer = null
	spr_importer = null
	wad_importer = null


## Most of what a .bsp imports to is decided in C++, but Godot reimports only when the source,
## its options or the importer's format version changes — a rebuilt extension would leave every
## map as the old build made it. So compare each imported map's pipeline_id stamp against this
## build's, and reimport the ones that differ. The extension only changes across an editor
## restart, and the last id checked is remembered, so an ordinary start skips the scan.
func _reimport_stale_maps() -> void:
	var current := BspSceneBuilder.pipeline_id()
	if current.is_empty():
		return
	var settings := EditorInterface.get_editor_settings()
	if settings.get_project_metadata("goldsrc", "checked_pipeline_id", "") == current:
		return

	var fs := EditorInterface.get_resource_filesystem()
	while fs.is_scanning():
		await fs.filesystem_changed

	var stale := PackedStringArray()
	_collect_stale_maps(fs.get_filesystem(), stale)
	settings.set_project_metadata("goldsrc", "checked_pipeline_id", current)
	if stale.is_empty():
		return
	print("GoldSrc: reimporting %d map(s) built by another build of the extension: %s"
			% [stale.size(), ", ".join(stale)])
	fs.reimport_files(stale)


func _collect_stale_maps(dir: EditorFileSystemDirectory, out: PackedStringArray) -> void:
	for i in dir.get_file_count():
		var path := dir.get_file_path(i)
		if path.get_extension().to_lower() != "bsp" or not dir.get_file_import_is_valid(i):
			continue
		var scene := ResourceLoader.load(path, "PackedScene", ResourceLoader.CACHE_MODE_IGNORE) as PackedScene
		if scene and BspSceneBuilder.is_scene_stale(scene):
			out.append(path)
	for i in dir.get_subdir_count():
		_collect_stale_maps(dir.get_subdir(i), out)
