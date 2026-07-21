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


## Entity 0 is worldspawn by format definition, and carries the map-wide settings.
func _worldspawn(ents: Array) -> void:
	if ents.is_empty():
		fail("no entities parsed")
		return

	var ws: Dictionary = ents[0]
	check_eq(ws.get("classname", ""), "worldspawn", "entity 0 is worldspawn")
	check_eq(ws.get("skyname", ""), "wiz", "worldspawn carries ww_2fort's skyname")
	check(ws.has("wad"), "worldspawn lists its WAD dependencies")
	# Keys with backslashes and semicolons survive intact — the wad list is the one field
	# where escaping mistakes would show up.
	check(String(ws.get("wad", "")).contains("halflife.wad"),
		"the WAD list keeps its path separators")


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

	# 24 team spawns plus a single info_player_start. Matched exactly rather than by an
	# "info_player" prefix, which conflates the two and gives 25.
	check_eq(spawns.size(), 24, "ww_2fort's team spawn count")
	check_eq(solo_starts, 1, "ww_2fort has one non-team start")
	if spawns.is_empty():
		return

	var first: Dictionary = spawns[0]
	check_eq(first.get("origin", ""), "1435 1626 242", "first team spawn origin")
	check_eq(first.get("team_no", ""), "1", "first team spawn belongs to team 1")
	check(first.has("angles"), "a spawn carries a facing angle")
