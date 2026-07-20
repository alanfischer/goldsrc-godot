extends "res://tests/ingame/suite.gd"
## Lightstyle animation: the per-style brightness table GoldSrc uses for flickering and
## pulsing lights. Indices come straight from map data, so out-of-range access has to be
## safe rather than merely unlikely.


func run() -> void:
	var bsp := load_map()
	if bsp == null:
		return

	_defaults(bsp)
	_round_trips(bsp)
	_out_of_range_is_safe(bsp)
	_shader_toggle(bsp)


func _defaults(bsp: GoldSrcBSP) -> void:
	check_near(bsp.get_lightstyle(0), 1.0, 1e-6, "style 0 defaults to full brightness")


func _round_trips(bsp: GoldSrcBSP) -> void:
	bsp.set_lightstyle(3, 0.42)
	check_near(bsp.get_lightstyle(3), 0.42, 1e-5, "a set style reads back")

	bsp.set_lightstyle(3, 0.0)
	check_near(bsp.get_lightstyle(3), 0.0, 1e-6, "a style can be driven to zero")

	# Styles are independent — writing one must not disturb another.
	check_near(bsp.get_lightstyle(0), 1.0, 1e-6, "style 0 is untouched by writes to style 3")
	bsp.set_lightstyle(3, 1.0)


func _out_of_range_is_safe(bsp: GoldSrcBSP) -> void:
	check_near(bsp.get_lightstyle(9999), 0.0, 1e-6, "a style past the table reads as zero")
	check_near(bsp.get_lightstyle(-1), 0.0, 1e-6, "a negative style reads as zero")

	# The assertion is that these do not crash or corrupt the table; the engine keeps
	# running and neighbouring styles keep their values.
	bsp.set_lightstyle(9999, 0.5)
	bsp.set_lightstyle(-1, 0.5)
	check_near(bsp.get_lightstyle(0), 1.0, 1e-6,
		"out-of-range writes leave the real table intact")


func _shader_toggle(bsp: GoldSrcBSP) -> void:
	var was := bsp.get_shader_lightstyles()
	check(was, "shader lightstyles are on by default")

	bsp.set_shader_lightstyles(false)
	check_eq(bsp.get_shader_lightstyles(), false, "the shader lightstyle flag round-trips")
	bsp.set_shader_lightstyles(was)
