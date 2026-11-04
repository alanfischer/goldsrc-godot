extends "res://tests/ingame/suite.gd"
## Entity extraction. The entity lump is how a map tells the game where players spawn, what
## the sky is and which WADs it needs, so this is the parser output the game leans on most.


func run() -> void:
	var bsp := load_map()
	if bsp == null:
		return

	var ents: Array = bsp.get_entities()
	_worldspawn(ents)
	_every_entity_is_well_formed(ents)
	_player_spawns(ents)
	_entity_rotation()


## Entity 0 is worldspawn by format definition, and carries the map-wide settings.
func _worldspawn(ents: Array) -> void:
	if ents.is_empty():
		fail("no entities parsed")
		return

	var ws: Dictionary = ents[0]
	check_eq(ws.get("classname", ""), "worldspawn", "entity 0 is worldspawn")
	check_eq(ws.get("skyname", ""), "night", "worldspawn carries ww_ravine's skyname")
	check(ws.has("wad"), "worldspawn lists its WAD dependencies")
	# The wad list is a semicolon-separated string ("halflife.wad;tfc.wad;"); the
	# separators must survive parsing intact, since a botched split would drop entries.
	check(String(ws.get("wad", "")).contains("halflife.wad"),
		"the WAD list keeps its separators")


func _every_entity_is_well_formed(ents: Array) -> void:
	var missing_classname := 0
	var not_a_dict := 0
	for e in ents:
		if typeof(e) != TYPE_DICTIONARY:
			not_a_dict += 1
			continue
		if not (e as Dictionary).has("classname"):
			missing_classname += 1
	check_eq(not_a_dict, 0, "every entity is a Dictionary")
	check_eq(missing_classname, 0, "every entity has a classname")


func _player_spawns(ents: Array) -> void:
	var spawns: Array = []
	var solo_starts := 0
	for e in ents:
		match String((e as Dictionary).get("classname", "")):
			"info_player_teamspawn":
				spawns.append(e)
			"info_player_start":
				solo_starts += 1

	# 8 team spawns plus a single info_player_start. Matched exactly rather than by an
	# "info_player" prefix, which conflates the two and gives 9.
	check_eq(spawns.size(), 8, "ww_ravine's team spawn count")
	check_eq(solo_starts, 1, "ww_ravine has one non-team start")
	if spawns.is_empty():
		return

	var first: Dictionary = spawns[0]
	check_eq(first.get("origin", ""), "1056 1456 1984", "first team spawn origin")
	check_eq(first.get("team_no", ""), "1", "first team spawn belongs to team 1")


## Forward (-Z) for an entity's keys, in Godot's axes.
func _forward(ent: Dictionary) -> Vector3:
	return -Basis.from_euler(GoldSrcBSP.entity_rotation(ent)).z


func _near(a: Vector3, b: Vector3, what: String) -> void:
	check(a.is_equal_approx(b) or a.distance_to(b) < 1e-5, "%s: %s, expected %s" % [what, a, b])


## GoldSrc's +X is Godot's -X and +Y is Godot's +Z (goldsrc_to_godot); a facing goes the same way.
func _entity_rotation() -> void:
	_near(_forward({}), Vector3(-1, 0, 0), "no angles faces GoldSrc +X")
	_near(_forward({"angles": "0 90 0"}), Vector3(0, 0, 1), "yaw 90 faces GoldSrc +Y")
	_near(_forward({"angle": "180"}), Vector3(1, 0, 0), "angle is a yaw")
	_near(_forward({"angles": "-90 0 0"}), Vector3(0, 1, 0), "pitch -90 looks up, as view angles")
	_near(_forward({"angles": "45 0 0"}), Vector3(-1, -1, 0).normalized(), "pitch 45 looks down")
	_near(_forward({"angle": "-1"}), Vector3(0, 1, 0), "angle -1 is straight up")
	_near(_forward({"angle": "-2"}), Vector3(0, -1, 0), "angle -2 is straight down")
	_near(_forward({"angles": "0 90 0", "angle": "0"}), Vector3(0, 0, 1), "angles wins over angle")
	var rolled := Basis.from_euler(GoldSrcBSP.entity_rotation({"angles": "0 0 90"}))
	_near(-rolled.z, Vector3(-1, 0, 0), "roll keeps the facing")
	# AngleVectors: up = (cr·sp·cy + sr·sy, cr·sp·sy - sr·cy, cr·cp), so roll 90 tips it to -Y.
	_near(rolled.y, Vector3(0, 0, -1), "roll 90 tips the top to GoldSrc -Y, as AngleVectors")
