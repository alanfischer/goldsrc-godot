extends SceneTree
## Headless BSP-to-SCN conversion script.
##
## Usage:
##   godot --path <goldsrc-godot-dir> --headless --script res://tools/batch_convert_bsp.gd -- \
##     --bsp <path1.bsp> --bsp <path2.bsp> \
##     --wad-dir <wad-directory> \
##     --output-dir <output-directory> \
##     --scale <float> \
##     --shader-lightstyles \
##     --overbright <float> \
##     --rotate


func _init() -> void:
	var args := _parse_args()
	var bsp_paths: PackedStringArray = args.get("bsp", PackedStringArray())
	var wad_dir: String = args.get("wad_dir", "")
	var output_dir: String = args.get("output_dir", "")
	var scale: float = float(args.get("scale", "0.025"))
	var use_shader: bool = args.get("shader_lightstyles", false)
	var overbright: float = float(args.get("overbright", "1.0"))
	var do_rotate: bool = args.get("rotate", false)

	if bsp_paths.is_empty():
		printerr("No --bsp arguments provided")
		quit(1)
		return

	if output_dir.is_empty():
		printerr("No --output-dir argument provided")
		quit(1)
		return

	# Pre-load WAD files once
	var wads: Array[GoldSrcWAD] = []
	if wad_dir != "":
		var dir := DirAccess.open(wad_dir)
		if dir:
			dir.list_dir_begin()
			var file_name := dir.get_next()
			while file_name != "":
				if not dir.current_is_dir() and file_name.get_extension().to_lower() == "wad":
					var wad := GoldSrcWAD.new()
					wad.load_wad(wad_dir.path_join(file_name))
					wads.append(wad)
					print("  Loaded WAD: %s" % file_name)
				file_name = dir.get_next()
			dir.list_dir_end()
		else:
			printerr("Could not open WAD directory: %s" % wad_dir)

	# Ensure output directory exists
	DirAccess.make_dir_recursive_absolute(output_dir)

	print("Options: scale=%.4f shader_lightstyles=%s overbright=%.1f" % [
		scale, use_shader, overbright])

	var ok := true
	for bsp_path in bsp_paths:
		if not FileAccess.file_exists(bsp_path):
			printerr("BSP not found: %s" % bsp_path)
			ok = false
			continue

		var basename := bsp_path.get_file().get_basename()
		print("Converting %s ..." % basename)

		if not _convert_one(bsp_path, wads, output_dir, basename, scale,
				use_shader, overbright, do_rotate):
			ok = false

	quit(0 if ok else 1)


func _convert_one(bsp_path: String, wads: Array[GoldSrcWAD],
		output_dir: String, basename: String, scale: float,
		use_shader: bool, overbright: float, do_rotate: bool) -> bool:
	var t0 := Time.get_ticks_msec()

	# Create and configure GoldSrcBSP
	var bsp := GoldSrcBSP.new()
	bsp.set_scale_factor(scale)
	if bsp.has_method("set_shader_lightstyles"):
		bsp.set_shader_lightstyles(use_shader)
	for wad in wads:
		bsp.add_wad(wad)

	bsp.load_bsp(bsp_path)
	_log_time("load_bsp", t0)

	var t1 := Time.get_ticks_msec()
	bsp.build_mesh()
	_log_time("build_mesh", t1)

	# Apply overbright
	if overbright != 1.0:
		t1 = Time.get_ticks_msec()
		_apply_overbright(bsp, overbright)
		_log_time("overbright", t1)

	# --- Save .scn ---
	t1 = Time.get_ticks_msec()

	# Reparent children to a plain Node3D (GDExtension-independent)
	var root := Node3D.new()
	root.name = basename

	# Store lightstyle resources as metadata (for shader path)
	if use_shader and bsp.has_method("get_lightstyle_image"):
		var ls_image = bsp.get_lightstyle_image()
		var ls_texture = bsp.get_lightstyle_texture()
		if ls_image and ls_texture:
			root.set_meta("lightstyle_image", ls_image)
			root.set_meta("lightstyle_texture", ls_texture)

	var children := bsp.get_children()
	for child in children:
		bsp.remove_child(child)
		root.add_child(child)
	bsp.free()
	_set_owner_recursive(root, root)

	# Optionally rotate 180 deg around Y so C++ convention (-x, z, y) matches
	# Python convention (x, z, -y). Use --rotate to enable.
	if do_rotate:
		root.rotation.y = PI

	var scene := PackedScene.new()
	var pack_err := scene.pack(root)
	if pack_err != OK:
		printerr("Failed to pack %s: %s" % [basename, error_string(pack_err)])
		root.free()
		return false

	var scn_path := output_dir.path_join(basename + ".scn")
	var save_err := ResourceSaver.save(scene, scn_path)
	root.free()
	if save_err != OK:
		printerr("Failed to save %s: %s" % [scn_path, error_string(save_err)])
		return false
	_log_time("save_scene", t1)

	var total := Time.get_ticks_msec() - t0
	print("  Done: %s (%dms)" % [scn_path, total])
	return true


func _apply_overbright(node: Node, scale: float) -> void:
	for child in node.get_children():
		if child is MeshInstance3D:
			var mesh: Mesh = child.mesh
			if not mesh:
				continue
			for surf_idx in mesh.get_surface_count():
				var mat := mesh.surface_get_material(surf_idx)
				if mat is ShaderMaterial:
					mat.set_shader_parameter("overbright", scale)
				elif mat is StandardMaterial3D and mat.detail_enabled:
					mat.albedo_color = Color(scale, scale, scale)
		else:
			_apply_overbright(child, scale)


func _log_time(label: String, since: int) -> void:
	print("  %s: %dms" % [label, Time.get_ticks_msec() - since])


func _set_owner_recursive(node: Node, owner: Node) -> void:
	for child in node.get_children():
		child.owner = owner
		_set_owner_recursive(child, owner)


func _parse_args() -> Dictionary:
	var result := {}
	var bsp_list := PackedStringArray()
	var raw := OS.get_cmdline_user_args()  # args after "--"
	var i := 0
	while i < raw.size():
		match raw[i]:
			"--bsp":
				i += 1
				if i < raw.size():
					bsp_list.append(raw[i])
			"--wad-dir":
				i += 1
				if i < raw.size():
					result["wad_dir"] = raw[i]
			"--output-dir":
				i += 1
				if i < raw.size():
					result["output_dir"] = raw[i]
			"--scale":
				i += 1
				if i < raw.size():
					result["scale"] = raw[i]
			"--shader-lightstyles":
				result["shader_lightstyles"] = true
			"--rotate":
				result["rotate"] = true
			"--overbright":
				i += 1
				if i < raw.size():
					result["overbright"] = raw[i]
		i += 1
	result["bsp"] = bsp_list
	return result
