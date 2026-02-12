extends SceneTree

func _init():
	var sprites = ["fire.spr", "firestone.spr", "firez.spr", "ballsmoke.spr", "grayfire.spr", "glow01.spr"]
	for name in sprites:
		var spr = GoldSrcSPR.new()
		if spr.load_spr("res://assets/sprites/" + name) == OK:
			print("  %s: %d frames" % [name, spr.get_frame_count()])
	quit()
