@tool
extends EditorPlugin

## Off in a project that converts maps itself at runtime and never loads the imported scenes,
## which would otherwise pay for a full reimport of every map each time the extension changes.
const REIMPORT_SETTING := "goldsrc/import/reimport_on_extension_change"
const BspImporter := preload("res://addons/goldsrc/importer/bsp_importer.gd")

var bsp_importer: EditorImportPlugin
var mdl_importer: EditorImportPlugin
var spr_importer: EditorImportPlugin
var wad_importer: EditorImportPlugin


func _enter_tree() -> void:
	bsp_importer = BspImporter.new()
	mdl_importer = preload("res://addons/goldsrc/importer/mdl_importer.gd").new()
	spr_importer = preload("res://addons/goldsrc/importer/spr_importer.gd").new()
	wad_importer = preload("res://addons/goldsrc/importer/wad_importer.gd").new()
	add_import_plugin(bsp_importer)
	add_import_plugin(mdl_importer)
	add_import_plugin(spr_importer)
	add_import_plugin(wad_importer)
	_declare_settings()
	_reimport_stale_maps.call_deferred()
	# goldsrc.gdextension is reloadable, so a rebuild can land without an editor restart.
	GDExtensionManager.extensions_reloaded.connect(_reimport_stale_maps)


func _exit_tree() -> void:
	GDExtensionManager.extensions_reloaded.disconnect(_reimport_stale_maps)
	remove_import_plugin(bsp_importer)
	remove_import_plugin(mdl_importer)
	remove_import_plugin(spr_importer)
	remove_import_plugin(wad_importer)
	bsp_importer = null
	mdl_importer = null
	spr_importer = null
	wad_importer = null


func _declare_settings() -> void:
	if not ProjectSettings.has_setting(REIMPORT_SETTING):
		ProjectSettings.set_setting(REIMPORT_SETTING, true)
	ProjectSettings.set_initial_value(REIMPORT_SETTING, true)
	ProjectSettings.add_property_info({"name": REIMPORT_SETTING, "type": TYPE_BOOL})


## Godot re-tests an import only when its source or .import file changes, so neither a rebuilt
## extension nor a bumped importer would ever reach a map imported before. The importer records
## the key each map was built with in project metadata; any map recorded with another key (or
## none) is reimported here, on editor start and on an extension hot reload.
func _reimport_stale_maps() -> void:
	if not ProjectSettings.get_setting(REIMPORT_SETTING, true):
		return
	var fs := EditorInterface.get_resource_filesystem()
	while fs.is_scanning():
		await fs.filesystem_changed

	var stale := PackedStringArray()
	_collect_stale_maps(fs.get_filesystem(), BspImporter.import_key(), stale)
	if stale.is_empty():
		return
	print("GoldSrc: reimporting %d map(s) imported by another build of the extension or importer: %s"
			% [stale.size(), ", ".join(stale)])
	fs.reimport_files(stale)


func _collect_stale_maps(dir: EditorFileSystemDirectory, key: String, out: PackedStringArray) -> void:
	var settings := EditorInterface.get_editor_settings()
	for i in dir.get_file_count():
		var path := dir.get_file_path(i)
		if path.get_extension().to_lower() != "bsp" or not dir.get_file_import_is_valid(i):
			continue
		if settings.get_project_metadata(BspImporter.IMPORTED_WITH, path, "") != key:
			out.append(path)
	for i in dir.get_subdir_count():
		_collect_stale_maps(dir.get_subdir(i), key, out)
