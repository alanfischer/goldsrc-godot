extends "res://tests/ingame/suite.gd"
## The spatial queries gameplay depends on: point_contents, point_to_leaf, the PVS, and
## AABB leaf gathering. A regression here is a live bug — players falling through floors or
## geometry vanishing — rather than something that fails loudly at load time.

## The first info_player_teamspawn in ww_2fort, at GoldSrc (1435 1626 242), converted with
## the extension's own mapping: godot = (-x*s, z*s, y*s) at the default scale of 0.025.
## Hardcoded rather than read from the entity list so a failure here means a query bug, not
## an entity-parsing bug — that is asserted separately in suite_bsp_entities.
const SPAWN := Vector3(-35.875, 6.05, 40.65)


func run() -> void:
	var bsp := load_map()
	if bsp == null:
		return

	_contents_around_a_spawn(bsp)
	_contents_over_a_grid(bsp)
	_point_to_leaf(bsp)
	_pvs(bsp)
	_leaves_in_aabb(bsp)


## A player spawn is by construction standing in open space on top of solid floor. If this
## ever inverts, players are spawning inside geometry.
func _contents_around_a_spawn(bsp: GoldSrcBSP) -> void:
	check_eq(bsp.point_contents(SPAWN), CONTENTS_EMPTY,
		"a player spawn stands in empty space")
	check_eq(bsp.point_contents(SPAWN - Vector3(0, 5, 0)), CONTENTS_SOLID,
		"there is solid floor beneath a player spawn")

	# Far outside the map is the void. Whatever it reports, it must be a real contents
	# value and must not crash the node walk.
	var far := bsp.point_contents(Vector3(1e6, 1e6, 1e6))
	check(far in [CONTENTS_EMPTY, CONTENTS_SOLID, CONTENTS_WATER, CONTENTS_SKY],
		"a point far outside the map yields a known contents value (got %d)" % far)


## Two sample points are not a net. A fixed grid across the map pins the whole
## point-to-leaf walk: an off-by-one in the leaf-index decode, a swapped node child or a
## dropped axis all move these counts, where a pair of hand-picked points can easily land
## on neighbours that happen to agree.
##
## The totals are anchored to the shipped ww_2fort.bsp. If they change, either the map
## changed or the BSP walk did — both worth a human look rather than a silent re-baseline.
func _contents_over_a_grid(bsp: GoldSrcBSP) -> void:
	var solid := 0
	var empty := 0
	var other := 0

	for ix in range(12):
		for iy in range(6):
			for iz in range(12):
				var p := Vector3(-60.0 + ix * 10.0, 2.0 + iy * 6.0, -60.0 + iz * 10.0)
				match bsp.point_contents(p):
					CONTENTS_SOLID: solid += 1
					CONTENTS_EMPTY: empty += 1
					_: other += 1

	check_eq(solid + empty + other, 864, "the grid samples every point")
	check_eq(solid, 756, "solid sample count across ww_2fort")
	check_eq(empty, 108, "open-space sample count across ww_2fort")


func _point_to_leaf(bsp: GoldSrcBSP) -> void:
	var leaf := bsp.point_to_leaf(SPAWN)
	check_between(leaf, 0, bsp.get_leaf_count() - 1,
		"a spawn resolves to a leaf inside the map")

	# The same point must land in the same leaf every time — the walk is pure.
	check_eq(bsp.point_to_leaf(SPAWN), leaf, "point_to_leaf is deterministic")


func _pvs(bsp: GoldSrcBSP) -> void:
	var leaf := bsp.point_to_leaf(SPAWN)
	var pvs: PackedInt32Array = bsp.get_leaf_pvs(leaf)

	check(pvs.size() > 0, "a spawn's leaf can see something")
	check(leaf in pvs, "a leaf is visible from itself")

	var leaf_count := bsp.get_leaf_count()
	var out_of_range := 0
	for i in pvs:
		if i < 0 or i >= leaf_count:
			out_of_range += 1
	check_eq(out_of_range, 0, "every PVS entry is a valid leaf index")

	check_eq(bsp.get_leaf_pvs(-1).size(), 0, "PVS of a negative leaf is empty")
	check_eq(bsp.get_leaf_pvs(leaf_count).size(), 0, "PVS past the last leaf is empty")


func _leaves_in_aabb(bsp: GoldSrcBSP) -> void:
	var leaf_count := bsp.get_leaf_count()

	var everything := bsp.get_leaves_in_aabb(
		AABB(Vector3(-10000, -10000, -10000), Vector3(20000, 20000, 20000)))
	check(everything.size() > 0, "an all-encompassing AABB gathers leaves")

	# The interesting property is not how many come back but that none of them is a bogus
	# index — an out-of-range leaf here would be read straight into a lookup by callers.
	var bad := 0
	for i in everything:
		if i < 0 or i >= leaf_count:
			bad += 1
	check_eq(bad, 0, "every gathered leaf index is in range")

	var at_spawn := bsp.get_leaves_in_aabb(AABB(SPAWN - Vector3.ONE, Vector3.ONE * 2.0))
	check(at_spawn.size() > 0, "a small AABB at a spawn gathers at least one leaf")
	check(at_spawn.size() < everything.size(),
		"a small AABB gathers fewer leaves than the whole map")
