@tool
extends EditorPlugin

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


func _exit_tree() -> void:
	remove_import_plugin(bsp_importer)
	remove_import_plugin(mdl_importer)
	remove_import_plugin(spr_importer)
	remove_import_plugin(wad_importer)
	bsp_importer = null
	mdl_importer = null
	spr_importer = null
	wad_importer = null
