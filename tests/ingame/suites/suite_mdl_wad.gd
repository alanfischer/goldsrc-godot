extends "res://tests/ingame/suite.gd"
## GoldSrcMDL and GoldSrcWAD.
##
## The repo ships no .mdl or .wad, so this covers what can be asserted without one: that a
## fresh node reports empty rather than garbage, that every accessor is safe before a load,
## and that a failed load is reported and leaves nothing behind. The parsing itself is
## covered by the standalone C++ suite, which builds its own fixtures.
##
## If a small .mdl and .wad are ever added to the repo, this is where the round-trip
## assertions belong — bone counts, sequence names, texture lookup by name.


func run() -> void:
	_mdl_unloaded()
	_wad_unloaded()


func _mdl_unloaded() -> void:
	var mdl := GoldSrcMDL.new()
	track(mdl)

	check_eq(mdl.get_sequence_count(), 0, "unloaded MDL has no sequences")
	check_eq(mdl.get_bone_count(), 0, "unloaded MDL has no bones")
	check_eq(mdl.get_bodypart_count(), 0, "unloaded MDL has no bodyparts")
	check_eq(mdl.get_skin_count(), 0, "unloaded MDL has no skins")

	# Indexed accessors on an empty model must not read past the end.
	check_eq(mdl.get_sequence_name(0), "", "sequence name of an absent sequence is empty")
	check_eq(mdl.get_sequence_name(-1), "", "sequence name of a negative index is empty")
	check_eq(mdl.get_bodypart_name(0), "", "bodypart name of an absent part is empty")

	check_eq(mdl.load_mdl("res://models/does_not_exist.mdl"), ERR_FILE_NOT_FOUND,
		"a missing model is reported as not found")
	check_eq(mdl.get_bone_count(), 0, "a failed load leaves the model empty")

	check_near(mdl.get_scale_factor(), 0.025, 1e-6, "MDL shares the world scale factor")
	mdl.set_scale_factor(0.1)
	check_near(mdl.get_scale_factor(), 0.1, 1e-6, "MDL scale factor round-trips")


func _wad_unloaded() -> void:
	var wad := GoldSrcWAD.new()

	check_eq(wad.get_texture_count(), 0, "unloaded WAD has no textures")
	check_eq(wad.get_texture_names().size(), 0, "unloaded WAD lists no names")
	check_eq(wad.has_texture("anything"), false, "unloaded WAD contains nothing")
	check(wad.get_texture("anything") == null, "unloaded WAD returns no texture")

	check_eq(wad.load_wad("res://wads/does_not_exist.wad"), ERR_FILE_NOT_FOUND,
		"a missing WAD is reported as not found")
	check_eq(wad.get_texture_count(), 0, "a failed load leaves the WAD empty")
