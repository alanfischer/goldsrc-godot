@tool
extends EditorPlugin

var bsp_importer: EditorImportPlugin


func _enter_tree() -> void:
	bsp_importer = preload("res://addons/goldsrc/importer/bsp_importer.gd").new()
	add_import_plugin(bsp_importer)


func _exit_tree() -> void:
	remove_import_plugin(bsp_importer)
	bsp_importer = null
