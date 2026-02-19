extends SceneTree
## Headless BSP-to-SCN conversion script.
##
## Usage:
##   godot --path <goldsrc-godot-dir> --headless --script res://tools/batch_convert_bsp.gd -- \
##     --bsp <path1.bsp> --bsp <path2.bsp> \
##     --wad-dir <wad-directory> \
##     --output-dir <output-directory> \
##     --scale <float>

const PYTHON_CONVENTION := true  # (x,y,z) -> (x*s, z*s, -y*s)


func _init() -> void:
	var args := _parse_args()
	var bsp_paths: PackedStringArray = args.get("bsp", PackedStringArray())
	var wad_dir: String = args.get("wad_dir", "")
	var output_dir: String = args.get("output_dir", "")
	var scale: float = float(args.get("scale", "0.025"))

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

	var ok := true
	for bsp_path in bsp_paths:
		if not FileAccess.file_exists(bsp_path):
			printerr("BSP not found: %s" % bsp_path)
			ok = false
			continue

		var basename := bsp_path.get_file().get_basename()
		print("Converting %s ..." % basename)

		if not _convert_one(bsp_path, wads, output_dir, basename, scale):
			ok = false

	quit(0 if ok else 1)


func _convert_one(bsp_path: String, wads: Array[GoldSrcWAD],
		output_dir: String, basename: String, scale: float) -> bool:
	# Create and configure GoldSrcBSP
	var bsp := GoldSrcBSP.new()
	bsp.set_scale_factor(scale)
	for wad in wads:
		bsp.add_wad(wad)

	bsp.load_bsp(bsp_path)
	bsp.build_mesh()

	# --- Save .scn ---
	# Reparent children to a plain Node3D (GDExtension-independent)
	var root := Node3D.new()
	root.name = basename

	var children := bsp.get_children()
	for child in children:
		bsp.remove_child(child)
		root.add_child(child)
	_set_owner_recursive(root, root)

	# Rotate 180 deg around Y so C++ convention (-x, z, y) matches
	# Python convention (x, z, -y) used by existing wanderyn code.
	root.rotation.y = PI

	var scene := PackedScene.new()
	var pack_err := scene.pack(root)
	if pack_err != OK:
		printerr("Failed to pack %s: %s" % [basename, error_string(pack_err)])
		return false

	var scn_path := output_dir.path_join(basename + ".scn")
	var save_err := ResourceSaver.save(scene, scn_path)
	if save_err != OK:
		printerr("Failed to save %s: %s" % [scn_path, error_string(save_err)])
		return false
	print("  Saved %s" % scn_path)

	# --- Save .entities.json ---
	var entities := bsp.get_entities()
	var scaled_entities: Array = []
	for ent in entities:
		var d: Dictionary = ent.duplicate()
		if d.has("origin"):
			var parts: PackedStringArray = str(d["origin"]).split(" ")
			if parts.size() >= 3:
				var x := float(parts[0])
				var y := float(parts[1])
				var z := float(parts[2])
				# Python convention: (x, y, z) -> (x*s, z*s, -y*s)
				var gx := x * scale
				var gy := z * scale
				var gz := -y * scale
				d["origin"] = "%s %s %s" % [gx, gy, gz]
				d["origin_vec"] = [gx, gy, gz]
		scaled_entities.append(d)

	var json_path := output_dir.path_join(basename + ".entities.json")
	var json_str := JSON.stringify(scaled_entities, "  ")
	var file := FileAccess.open(json_path, FileAccess.WRITE)
	if file == null:
		printerr("Failed to write %s" % json_path)
		return false
	file.store_string(json_str)
	file.close()
	print("  Saved %s (%d entities)" % [json_path, scaled_entities.size()])

	return true


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
		i += 1
	result["bsp"] = bsp_list
	return result
