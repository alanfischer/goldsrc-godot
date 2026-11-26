extends "res://tests/ingame/suite.gd"
## GoldSrcSPR against the shipped sprites/fire.spr. Unlike the BSP suites this exercises
## the Resource-derived side of the extension, including texture creation, which only
## exists inside an engine.


func run() -> void:
	_unloaded_is_inert()
	_loads_the_shipped_sprite()
	_frame_access_is_bounds_checked()
	_scene_is_a_flipbook()
	_looping_play_runs_on_the_shader_clock()
	_one_shot_steps_and_finishes()
	_restyle_and_unique_material()


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


## A built sprite, in the tree as it would be in a game: its animator puts the material on as
## it enters.
func _built() -> Sprite3D:
	var spr := GoldSrcSPR.new()
	if spr.load_spr(SPRITE_PATH) != OK:
		fail("could not load %s" % SPRITE_PATH)
		return null
	return _enter(spr.build_scene() as Sprite3D)


func _enter(sprite: Sprite3D) -> Sprite3D:
	track(sprite)
	(Engine.get_main_loop() as SceneTree).current_scene.add_child(sprite)
	return sprite


func _scene_is_a_flipbook() -> void:
	var spr := GoldSrcSPR.new()
	spr.load_spr(SPRITE_PATH)
	var outside := track(spr.build_scene()) as Sprite3D
	check(outside.material_override == null, "out of the tree the sprite is a plain Sprite3D")
	var library := outside.get_meta(&"animation_library", null) as AnimationLibrary
	check(library != null and library.has_animation(&"loop") and library.has_animation(&"once"),
		"the scene carries an AnimationLibrary with loop and once")

	var sprite := _built()
	if sprite == null:
		return
	var mat := sprite.material_override as ShaderMaterial
	check(mat != null, "the sprite draws with the flipbook ShaderMaterial")
	if mat == null:
		return
	var sap := sprite.get_node(^"SpriteAnimationPlayer")
	check_eq(sap.layers.size(), 15, "the scene carries all 15 frames as Images")
	sap._ensure_frames()
	var frames := mat.get_shader_parameter(&"frames") as Texture2DArray
	check(frames != null and frames.get_layers() == 15, "they become the layers of one array")
	# fire.spr's frames share one 32x96 rect at origin (-16, 48), so the canvas is that rect, centred.
	check_eq(sprite.texture.get_size(), Vector2(32, 96), "the canvas is the union of the frame rects")
	check_eq(sprite.offset, Vector2(0, 0), "the canvas sits where frame 0 did")
	check(mat.shader.code.contains("blend_add"), "an additive SPR draws additively")
	check(mat.shader.code.contains("MAIN_CAM_INV_VIEW_MATRIX"), "a parallel-oriented SPR billboards")


func _looping_play_runs_on_the_shader_clock() -> void:
	var built := _built()
	if built == null:
		return
	# Two instances of one packed scene, as an imported sprite is used.
	var scene := PackedScene.new()
	scene.pack(built)
	var sprite := _enter(scene.instantiate() as Sprite3D)
	var sap := sprite.get_node(^"SpriteAnimationPlayer")
	check_eq(sap.layers.size(), 15, "a packed sprite keeps its frames")
	sap.fps = 12.0
	sap.play()
	check(not sap.is_processing(), "a looping play runs no per-frame script")
	check_eq(sprite.get_instance_shader_parameter(&"fps"), 12.0,
		"the shader clocks the loop at the sprite's fps")
	var other := _enter(scene.instantiate() as Sprite3D)
	var osap := other.get_node(^"SpriteAnimationPlayer")
	osap.fps = 7.0
	osap.play()
	check_eq(other.get_instance_shader_parameter(&"fps"), 7.0, "each sprite keeps its own rate")
	check(other.material_override != null and other.material_override == sprite.material_override,
		"sprites looping at different rates share a material")


func _one_shot_steps_and_finishes() -> void:
	var sprite := _built()
	if sprite == null:
		return
	var sap := sprite.get_node(^"SpriteAnimationPlayer")
	sap.loop_animation = false
	var finished := []
	sap.animation_finished.connect(func(n: String) -> void: finished.append(n))
	sap.play()
	check_eq(sprite.get_instance_shader_parameter(&"fps"), 0.0, "a one-shot is not on the shader clock")
	sap._step()
	check_eq(sprite.get_instance_shader_parameter(&"frame"), 1.0, "a step shows the next frame")
	for i in 13:
		sap._step()
	check(finished.is_empty() and sprite.visible, "still playing on the last frame")
	sap._step()
	check_eq(finished, ["default"], "stepping past the last frame finishes the animation")
	check(not sprite.visible and not sap.is_playing(), "a finished one-shot hides and stops")


func _restyle_and_unique_material() -> void:
	var sprite := _built()
	if sprite == null:
		return
	var sap := sprite.get_node(^"SpriteAnimationPlayer")
	var shared := sprite.material_override
	var own: ShaderMaterial = sap.unique_material()
	check(own != shared, "unique_material gives the sprite its own copy")
	sap.restyle(BaseMaterial3D.BILLBOARD_FIXED_Y, BaseMaterial3D.BLEND_MODE_MIX, false)
	check(sprite.material_override == own, "restyling keeps the sprite's own material")
	check(own.shader.code.contains("blend_mix") and own.shader.code.contains("fog_disabled"),
		"restyle swaps the blend and fog")
	check(own.shader.code.contains("vec4(0.0, 1.0, 0.0, 0.0)"), "restyle swaps the billboard")
