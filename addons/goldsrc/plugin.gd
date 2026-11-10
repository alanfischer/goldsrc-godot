@tool
extends EditorPlugin

## Off in a project that converts maps and models itself at runtime and never loads the imported
## scenes, which would otherwise pay for a full reimport of every one each time the extension
## changes.
const REIMPORT_SETTING := "goldsrc/import/reimport_on_extension_change"
const BspImporter := preload("res://addons/goldsrc/importer/bsp_importer.gd")
const MdlImporter := preload("res://addons/goldsrc/importer/mdl_importer.gd")

var bsp_importer: EditorImportPlugin
var mdl_importer: EditorImportPlugin
var spr_importer: EditorImportPlugin
var wad_importer: EditorImportPlugin


func _enter_tree() -> void:
	bsp_importer = BspImporter.new()
	mdl_importer = MdlImporter.new()
	spr_importer = preload("res://addons/goldsrc/importer/spr_importer.gd").new()
	wad_importer = preload("res://addons/goldsrc/importer/wad_importer.gd").new()
	add_import_plugin(bsp_importer)
	add_import_plugin(mdl_importer)
	add_import_plugin(spr_importer)
	add_import_plugin(wad_importer)
	_declare_settings()
	_reimport_stale.call_deferred()
	# goldsrc.gdextension is reloadable, so a rebuild can land without an editor restart.
	GDExtensionManager.extensions_reloaded.connect(_reimport_stale)


func _exit_tree() -> void:
	GDExtensionManager.extensions_reloaded.disconnect(_reimport_stale)
	var fs := EditorInterface.get_resource_filesystem()
	if fs.filesystem_changed.is_connected(_reimport_stale):
		fs.filesystem_changed.disconnect(_reimport_stale)
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
## extension nor a bumped importer would ever reach a map or model imported before. The importers
## record the key each file was built with in project metadata; any file recorded with another
## key (or none) is reimported here, on editor start and on an extension hot reload. Godot
## reimports in importer order, so models are rebuilt before the maps that place them.
func _reimport_stale() -> void:
	if not ProjectSettings.get_setting(REIMPORT_SETTING, true):
		return
	var fs := EditorInterface.get_resource_filesystem()
	# A connection, not an await: a headless export frees the plugin mid-scan, and an await would
	# then resume on a freed instance. Godot drops a freed object's connections itself.
	if fs.is_scanning():
		if not fs.filesystem_changed.is_connected(_reimport_stale):
			fs.filesystem_changed.connect(_reimport_stale, CONNECT_ONE_SHOT)
		return

	var stale := PackedStringArray()
	_collect_stale(fs.get_filesystem(), stale)
	if stale.is_empty():
		return
	print("GoldSrc: reimporting %d file(s) imported by another build of the extension or importer: %s"
			% [stale.size(), ", ".join(stale)])
	fs.reimport_files(stale)


func _collect_stale(dir: EditorFileSystemDirectory, out: PackedStringArray) -> void:
	var settings := EditorInterface.get_editor_settings()
	var imported_with := {"bsp": BspImporter.IMPORTED_WITH, "mdl": MdlImporter.IMPORTED_WITH}
	var keys := {"bsp": BspImporter.import_key(), "mdl": MdlImporter.import_key()}
	for i in dir.get_file_count():
		var path := dir.get_file_path(i)
		var ext := path.get_extension().to_lower()
		if not ext in keys or not dir.get_file_import_is_valid(i):
			continue
		if settings.get_project_metadata(imported_with[ext], path, "") != keys[ext]:
			out.append(path)
	for i in dir.get_subdir_count():
		_collect_stale(dir.get_subdir(i), out)
