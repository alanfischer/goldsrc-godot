extends "res://tests/ingame/suite.gd"
## VisibilityManager's PVS and PAS queries, from an observer standing at a ww_ravine spawn.

const SPAWN := Vector3(-26.4, 49.6, 36.4)


func run() -> void:
	var vm: VisibilityManager = track(VisibilityManager.new())
	check(vm.setup(MAP_PATH, 0.025) > 0, "the map loads")
	var obs := vm.add_observer(1, SPAWN)

	_pvs_and_pas_over_a_grid(vm, obs)
	_fail_open(vm, obs)


## The counts are anchored to the shipped ww_ravine.bsp and were cross-checked against an
## independent reader of the map's vis lump.
func _pvs_and_pas_over_a_grid(vm: VisibilityManager, obs: int) -> void:
	var bsp := load_map()
	if bsp == null:
		return
	var open := 0
	var visible := 0
	var audible := 0
	var seen_not_heard := 0
	for ix in range(14):
		for iy in range(14):
			for iz in range(41):
				var p := Vector3(-48.0 + ix * 4.0, 1.0 + iy * 4.0, -80.0 + iz * 4.0)
				if bsp.point_contents(p) != CONTENTS_EMPTY:
					continue
				open += 1
				var v := vm.is_position_visible_to_observer(p, obs)
				var a := vm.is_position_audible_to_observer(p, obs)
				visible += int(v)
				audible += int(a)
				seen_not_heard += int(v and not a)

	check(vm.is_position_audible_to_observer(SPAWN, obs), "a sound at the observer is heard")
	check_eq(seen_not_heard, 0, "whatever is seen is heard")
	check_eq(open, 1569, "open grid points")
	check_eq(visible, 175, "visible grid points")
	check_eq(audible, 1145, "audible grid points")


func _fail_open(vm: VisibilityManager, obs: int) -> void:
	check(vm.is_position_audible_to_observer(SPAWN - Vector3(0, 15, 0), obs),
		"a sound in solid reaches everyone")
	check(vm.is_position_audible_to_observer(SPAWN, -1), "an unknown observer hears everything")
	vm.teardown()
	check(vm.is_position_audible_to_observer(SPAWN, obs), "with no map, everything is heard")
