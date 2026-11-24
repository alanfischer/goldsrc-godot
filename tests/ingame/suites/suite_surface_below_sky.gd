extends "res://tests/ingame/suite.gd"
## The surface below the sky (bake_surface_below_sky): per column, the first surface under the sky
## brushes, and the open air under it.

const NO_GAP := -1e6


func run() -> void:
	var bsp := load_map()
	if bsp == null:
		return
	var floor := bsp.bake_surface_below_sky(16.0)
	var heights: Image = floor["image"]
	var under: Image = floor["under"]
	check_eq(under.get_size(), heights.get_size(), "an air gap per column")
	check_eq(under.get_format(), Image.FORMAT_RGBAF, "underside, ground, share and the rest's height")
	var top: float = floor["top"]
	var sheltered := 0
	var bad := 0
	var edges := 0
	var bad_shares := 0
	for y in heights.get_height():
		for x in heights.get_width():
			var landed := heights.get_pixel(x, y).r
			var gap := under.get_pixel(x, y)
			# A cell part covered: its share is under 1 and the rest lies lower.
			if gap.b < 1.0:
				edges += 1
				if gap.b < 0.0 or gap.a >= landed:
					bad_shares += 1
			elif gap.b != 1.0 or gap.a != landed:
				bad_shares += 1
			if gap.r <= gap.g:
				if gap.r != NO_GAP:
					bad += 1
				continue
			sheltered += 1
			# Air under cover is under the surface, at least 0.3 m tall, within 4 m of the top.
			if gap.r > landed + 0.01 or gap.r - gap.g < 0.29 or gap.r < landed - 4.01 or landed >= top:
				bad += 1
				continue
			# The middle of it is empty space in the map, not inside a wall.
			var origin: Vector3 = floor["origin"]
			var cell: Vector2 = Vector2(floor["size"]) / Vector2(heights.get_size())
			var p := Vector3(origin.x - (x + 0.5) * cell.x, (gap.r + gap.g) * 0.5, origin.z + (y + 0.5) * cell.y)
			if bsp.point_contents(p) == -2:
				bad += 1
	check(edges > 0, "ww_ravine has cells a roof's edge crosses (%d)" % edges)
	check_eq(bad_shares, 0, "and each part-covered cell's rest lies under its top, the others all covered")
	check(sheltered > 0, "ww_ravine has air under cover (%d cells)" % sheltered)
	check_eq(bad, 0, "and every gap is under the roof, tall enough, and open")
	# The ravine itself, mid-map, is open to the sky: the surface is its floor, nothing over it.
	var mid := heights.get_size() / 2
	check(heights.get_pixel(mid.x, mid.y).r < top, "the ravine's middle is in the map")
	check(under.get_pixel(mid.x, mid.y).r <= under.get_pixel(mid.x, mid.y).g, "and open to the sky")
