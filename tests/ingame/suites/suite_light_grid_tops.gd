extends "res://tests/ingame/suite.gd"
## The light grid's up face under open sky: it takes the floor's lightmap, so what faces up out in
## the open is lit like the ground it stands on, not by the bare sky fill.

const UP := 2  # dir_slices are in Godot order: +X -X +Y -Y +Z -Z
const DOWN := 3


func run() -> void:
	var bsp := load_map()
	if bsp == null:
		return
	var grid := bsp.bake_light_grid(64.0)
	var ups: Array = grid["dir_slices"][UP]
	var downs: Array = grid["dir_slices"][DOWN]
	var lit := 0
	var dark_tops := 0
	var floor_tops := 0
	for s in ups.size():
		var up: Image = ups[s]
		var down: Image = downs[s]
		for y in up.get_height():
			for x in up.get_width():
				var u := up.get_pixel(x, y)
				var d := down.get_pixel(x, y)
				var dl := d.r + d.g + d.b
				if dl < 0.15:
					continue
				lit += 1
				if u.r + u.g + u.b < dl * 0.5:
					dark_tops += 1
				if u.is_equal_approx(d):
					floor_tops += 1
	# Measured on ww_ravine with the flat sky fill on the up face: 38% dark tops, 252 cells (0.4%)
	# whose up face matched the floor. With the floor's light: 8% (under real ceilings) and 28%.
	check(lit > 1000, "a map with lit floors (%d cells)" % lit)
	check(dark_tops < lit * 0.15, "few tops darker than half the floor below (%d of %d)" % [dark_tops, lit])
	check(floor_tops > lit * 0.15, "open to the sky, the up face is the floor's light (%d of %d)" % [floor_tops, lit])
