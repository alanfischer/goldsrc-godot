extends "res://tests/ingame/suite.gd"
## GoldSrcBSP loading: the two entry points, what an unloaded node reports, and what each
## rejection path returns.
##
## The counts below are anchored to the shipped maps/ww_2fort.bsp. They are exact on
## purpose — a parser change that quietly alters how many faces or leafs a real map yields
## is precisely the regression the standalone C++ tests, built on synthetic fixtures,
## cannot see.


func run() -> void:
	_unloaded_node_is_inert()
	_loads_the_shipped_map()
	_rejection_paths()
	_both_entry_points_agree()


## Every accessor must be safe before a map is loaded — these are public API and callers
## reach for them before checking.
func _unloaded_node_is_inert() -> void:
	var bsp := GoldSrcBSP.new()
	track(bsp)

	check_eq(bsp.get_leaf_count(), 0, "unloaded: leaf count")
	check_eq(bsp.get_entities().size(), 0, "unloaded: entity count")
	check_eq(bsp.get_pvs_blob().size(), 0, "unloaded: PVS blob is empty")
	check_eq(bsp.get_leaf_pvs(0).size(), 0, "unloaded: leaf PVS is empty")
	check_eq(bsp.point_contents(Vector3.ZERO), CONTENTS_EMPTY,
		"unloaded: everywhere reads as empty space")
	check_eq(bsp.point_to_leaf(Vector3.ZERO), -1, "unloaded: no leaf contains a point")


func _loads_the_shipped_map() -> void:
	var bsp := load_map()
	if bsp == null:
		return

	check_eq(bsp.get_leaf_count(), 3488, "ww_2fort leaf count")
	check_eq(bsp.get_entities().size(), 643, "ww_2fort entity count")
	check(bsp.get_pvs_blob().size() > 0, "a loaded map exposes a PVS blob")


func _rejection_paths() -> void:
	var bsp := GoldSrcBSP.new()
	track(bsp)

	check_eq(bsp.load_bsp("res://maps/does_not_exist.bsp"), ERR_FILE_NOT_FOUND,
		"missing file is reported as not found")
	check_eq(bsp.load_bsp_from_data(PackedByteArray()), ERR_INVALID_DATA,
		"empty buffer is rejected as invalid")

	var junk := PackedByteArray()
	junk.resize(256) # zeroed: right size to be a header, wrong version
	check_eq(bsp.load_bsp_from_data(junk), ERR_PARSE_ERROR,
		"a buffer that is not a BSP is rejected as a parse error")

	# A failed load must not leave half-parsed state behind.
	check_eq(bsp.get_leaf_count(), 0, "after a failed load the node is still empty")


## load_bsp is load_bsp_from_data plus a file read, so the two must agree on the same
## bytes. Cheap to state, and it pins the file-reading path against the in-memory one.
func _both_entry_points_agree() -> void:
	var from_path := load_map()
	if from_path == null:
		return

	var f := FileAccess.open(MAP_PATH, FileAccess.READ)
	if f == null:
		fail("could not read %s for the in-memory comparison" % MAP_PATH)
		return
	var bytes := f.get_buffer(f.get_length())
	f.close()

	var from_data := GoldSrcBSP.new()
	track(from_data)
	check_eq(from_data.load_bsp_from_data(bytes), OK, "the same bytes load from memory")
	check_eq(from_data.get_leaf_count(), from_path.get_leaf_count(),
		"both entry points agree on leaf count")
	check_eq(from_data.get_entities().size(), from_path.get_entities().size(),
		"both entry points agree on entity count")
