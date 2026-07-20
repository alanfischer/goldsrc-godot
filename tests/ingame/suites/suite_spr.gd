extends "res://tests/ingame/suite.gd"
## GoldSrcSPR against the shipped sprites/fire.spr. Unlike the BSP suites this exercises
## the Resource-derived side of the extension, including texture creation, which only
## exists inside an engine.


func run() -> void:
	_unloaded_is_inert()
	_loads_the_shipped_sprite()
	_frame_access_is_bounds_checked()


func _unloaded_is_inert() -> void:
	var spr := GoldSrcSPR.new()
	check_eq(spr.get_frame_count(), 0, "unloaded sprite has no frames")
	check_eq(spr.load_spr("res://sprites/does_not_exist.spr"), ERR_FILE_NOT_FOUND,
		"a missing sprite is reported as not found")


func _loads_the_shipped_sprite() -> void:
	var spr := GoldSrcSPR.new()
	check_eq(spr.load_spr(SPRITE_PATH), OK, "fire.spr loads")
	check_eq(spr.get_frame_count(), 15, "fire.spr frame count")

	# 4 = SPR_VP_PARALLEL_ORIENTED, 1 = SPR_ADDITIVE. Both come from the file header and
	# decide how the sprite is drawn, so a misread here changes rendering silently.
	check_eq(spr.get_type(), 4, "fire.spr orientation type")
	check_eq(spr.get_texture_format(), 1, "fire.spr is additive")

	var tex: Texture2D = spr.get_frame_texture(0)
	check(tex != null, "frame 0 produces a texture")
	if tex != null:
		check_eq(tex.get_size(), Vector2(32, 96), "frame 0 dimensions")

	check_eq(spr.get_frame_origin(0), Vector2i(-16, 48), "frame 0 origin")

	# Every frame must produce a usable texture, not just the first.
	var missing := 0
	for i in spr.get_frame_count():
		if spr.get_frame_texture(i) == null:
			missing += 1
	check_eq(missing, 0, "every frame produces a texture")


func _frame_access_is_bounds_checked() -> void:
	var spr := GoldSrcSPR.new()
	if spr.load_spr(SPRITE_PATH) != OK:
		fail("could not load %s" % SPRITE_PATH)
		return

	check(spr.get_frame_texture(999) == null, "a frame past the end has no texture")
	check(spr.get_frame_texture(-1) == null, "a negative frame has no texture")
	check_eq(spr.get_frame_origin(999), Vector2i(0, 0),
		"a frame past the end has a zero origin")
