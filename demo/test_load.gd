extends Node3D

## Test — loads a BSP map, an animated player model, and an animated sprite.
## Attach this to the root Node3D of test.tscn and run.

var ASSET_DIR := "res://assets/"
var sprite_node: Sprite3D
var spr_resource: GoldSrcSPR
var spr_frame := 0
var spr_timer := 0.0
var spr_fps := 10.0

func _ready():
	# Load WADs
	var wads: Array[GoldSrcWAD] = []
	for wad_name in ["wwwad.wad", "halflife.wad", "tfc.wad", "tfc2.wad"]:
		var wad = GoldSrcWAD.new()
		if wad.load_wad(ASSET_DIR + wad_name) == OK:
			wads.append(wad)
			print("  WAD: ", wad_name, " — ", wad.get_texture_count(), " textures")

	# Load BSP
	var bsp = GoldSrcBSP.new()
	for w in wads:
		bsp.add_wad(w)
	if bsp.load_bsp(ASSET_DIR + "maps/ww_chasm.bsp") == OK:
		add_child(bsp)
		bsp.build_mesh()

	# Load a player model and play an animation
	var mdl = GoldSrcMDL.new()
	if mdl.load_mdl(ASSET_DIR + "models/player/fire.mdl") == OK:
		mdl.position = Vector3(0, 0, 0)
		add_child(mdl)
		mdl.build_model()

		# Find the AnimationPlayer and play "idle"
		for child in mdl.get_children():
			if child is AnimationPlayer:
				var anim_player: AnimationPlayer = child
				var anims = anim_player.get_animation_list()
				print("  Available animations: ", anims.size())
				for a in anims:
					print("    ", a)
				# Play idle animation
				if anim_player.has_animation("idle"):
					anim_player.play("idle")
					print("  >> Playing 'idle' animation")
				elif anims.size() > 0:
					anim_player.play(anims[0])
					print("  >> Playing '", anims[0], "' animation")
				break

	# Load an animated sprite (fire.spr — 15 frames)
	spr_resource = GoldSrcSPR.new()
	if spr_resource.load_spr(ASSET_DIR + "sprites/fire.spr") == OK:
		print("  SPR: fire.spr — ", spr_resource.get_frame_count(), " frames")
		if spr_resource.get_frame_count() > 0:
			sprite_node = Sprite3D.new()
			sprite_node.texture = spr_resource.get_frame_texture(0)
			sprite_node.pixel_size = 0.005
			sprite_node.position = Vector3(1.5, 1.0, 0)
			sprite_node.billboard = BaseMaterial3D.BILLBOARD_ENABLED
			sprite_node.transparent = true
			add_child(sprite_node)

	# Find info_player_start spawn point
	var spawn_pos := Vector3(0, 2, 5)
	var spawn_yaw := 0.0
	for ent in bsp.get_entities():
		if ent.get("classname") == "info_player_start":
			if ent.has("origin"):
				var parts = ent["origin"].split(" ")
				if parts.size() >= 3:
					var gx = float(parts[0])
					var gy = float(parts[1])
					var gz = float(parts[2])
					spawn_pos = Vector3(-gx * 0.025, gz * 0.025, gy * 0.025)
			if ent.has("angles"):
				var angles = ent["angles"].split(" ")
				if angles.size() >= 2:
					var yaw = float(angles[1])
					spawn_yaw = deg_to_rad(180 - yaw)
			print("  Spawn: ", spawn_pos, " yaw=", rad_to_deg(spawn_yaw))
			break

	# FPS walking camera at spawn point
	var player = CharacterBody3D.new()
	player.set_script(load("res://fps_camera.gd"))
	player.position = spawn_pos
	player.rotation.y = spawn_yaw
	add_child(player)

	# Add basic lighting
	var light = DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-45, -45, 0)
	add_child(light)

	var env = WorldEnvironment.new()
	var environment = Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Color(0.2, 0.2, 0.3)
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = Color(0.4, 0.4, 0.4)
	env.environment = environment
	add_child(env)


func _process(delta: float):
	# Animate the sprite by cycling frames
	if sprite_node and spr_resource and spr_resource.get_frame_count() > 1:
		spr_timer += delta
		if spr_timer >= 1.0 / spr_fps:
			spr_timer -= 1.0 / spr_fps
			spr_frame = (spr_frame + 1) % spr_resource.get_frame_count()
			sprite_node.texture = spr_resource.get_frame_texture(spr_frame)
