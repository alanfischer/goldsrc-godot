#include "goldsrc_spr.h"

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/gd_script.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <cstring>

using namespace godot;
using namespace std;

// Inline GDScript embedded into the .scn produced by build_scene().
// Mirrors the AnimationPlayer API so SPR and MDL scenes can be driven identically.
// Frames and origins are read from tex_anim_frames / tex_anim_origins metadata on
// the parent Sprite3D.  fps / loop_animation / autoplay are @export properties so
// the importer or converter can bake non-default values into the saved scene.
// No C++ plugin dependency at runtime — pure GDScript baked into the .scn.
static const char *SPRITE_ANIMATOR_SCRIPT =
"extends Node\n"
"signal animation_finished(anim_name: String)\n"
"signal animation_started(anim_name: String)\n"
"@export var fps: float = 10.0\n"
"@export var loop_animation: bool = true\n"
"@export var autoplay: String = \"default\"\n"
"var speed_scale: float = 1.0\n"
"var current_animation: String = \"\"\n"
"var current_animation_position: float:\n"
"\tget:\n"
"\t\treturn _tick\n"
"var _playing: bool = false\n"
"var _tick: float = 0.0\n"
"var _last_idx: int = -1\n"
"var _frames: Array = []\n"
"var _origins: Array = []\n"
"func _ready() -> void:\n"
"\tvar spr := get_parent()\n"
"\tif spr.has_meta(&\"tex_anim_frames\"):\n"
"\t\t_frames = spr.get_meta(&\"tex_anim_frames\")\n"
"\tif spr.has_meta(&\"tex_anim_origins\"):\n"
"\t\t_origins = spr.get_meta(&\"tex_anim_origins\")\n"
"\tset_process(false)\n"
"\tif autoplay != \"\" and _frames.size() > 0:\n"
"\t\tplay(autoplay)\n"
"func play(anim_name: String = \"default\", _blend: float = -1.0, _speed: float = 1.0, _from_end: bool = false) -> void:\n"
"\tif _frames.is_empty():\n"
"\t\treturn\n"
"\tvar restart := current_animation != anim_name or not _playing\n"
"\tcurrent_animation = anim_name\n"
"\tif restart:\n"
"\t\t_tick = 0.0\n"
"\t\t_last_idx = -1\n"
"\t\tanimation_started.emit(anim_name)\n"
"\t_playing = true\n"
"\tset_process(true)\n"
"\t_apply_frame(_frame_at(_tick))\n"
"func pause() -> void:\n"
"\t_playing = false\n"
"\tset_process(false)\n"
"func stop(keep_state: bool = true) -> void:\n"
"\t_playing = false\n"
"\tset_process(false)\n"
"\tif not keep_state:\n"
"\t\t_tick = 0.0\n"
"\t\t_last_idx = -1\n"
"\t\tcurrent_animation = \"\"\n"
"func is_playing() -> bool:\n"
"\treturn _playing\n"
"func seek(seconds: float, update: bool = false) -> void:\n"
"\t_tick = seconds\n"
"\t_last_idx = -1\n"
"\tif update:\n"
"\t\t_apply_frame(_frame_at(_tick))\n"
"func get_animation_list() -> PackedStringArray:\n"
"\tif _frames.size() > 0:\n"
"\t\treturn PackedStringArray([\"default\"])\n"
"\treturn PackedStringArray()\n"
"func has_animation(anim_name: StringName) -> bool:\n"
"\treturn anim_name == &\"default\" and _frames.size() > 0\n"
"func _process(delta: float) -> void:\n"
"\t_tick += delta * speed_scale\n"
"\tvar idx := _frame_at(_tick)\n"
"\tif idx < 0:\n"
"\t\t_playing = false\n"
"\t\tset_process(false)\n"
"\t\tget_parent().visible = false\n"
"\t\tvar finished_anim := current_animation\n"
"\t\tcurrent_animation = \"\"\n"
"\t\tanimation_finished.emit(finished_anim)\n"
"\t\treturn\n"
"\tif idx == _last_idx:\n"
"\t\treturn\n"
"\t_apply_frame(idx)\n"
"func _frame_at(t: float) -> int:\n"
"\tvar raw := int(t * fps)\n"
"\tif loop_animation:\n"
"\t\treturn raw % _frames.size()\n"
"\tif raw >= _frames.size():\n"
"\t\treturn -1\n"
"\treturn raw\n"
"func _apply_frame(idx: int) -> void:\n"
"\tif idx < 0 or idx >= _frames.size():\n"
"\t\treturn\n"
"\t_last_idx = idx\n"
"\tvar spr: Sprite3D = get_parent() as Sprite3D\n"
"\tif not spr:\n"
"\t\treturn\n"
"\tvar tex: ImageTexture = _frames[idx]\n"
"\tspr.texture = tex\n"
"\tif idx < _origins.size():\n"
"\t\tvar origin: Array = _origins[idx]\n"
"\t\tspr.offset = Vector2(float(origin[0]) + tex.get_width() * 0.5, float(origin[1]) - tex.get_height() * 0.5)\n"
"\tvar mat := spr.material_override\n"
"\tif mat is StandardMaterial3D:\n"
"\t\tmat.albedo_texture = tex\n";

void GoldSrcSPR::_bind_methods() {
	ClassDB::bind_method(D_METHOD("load_spr", "path"), &GoldSrcSPR::load_spr);
	ClassDB::bind_method(D_METHOD("get_frame_count"), &GoldSrcSPR::get_frame_count);
	ClassDB::bind_method(D_METHOD("get_frame_texture", "index"), &GoldSrcSPR::get_frame_texture);
	ClassDB::bind_method(D_METHOD("get_frame_origin", "index"), &GoldSrcSPR::get_frame_origin);
	ClassDB::bind_method(D_METHOD("get_type"), &GoldSrcSPR::get_type);
	ClassDB::bind_method(D_METHOD("get_texture_format"), &GoldSrcSPR::get_texture_format);
	ClassDB::bind_method(D_METHOD("build_scene"), &GoldSrcSPR::build_scene);
}

Error GoldSrcSPR::load_spr(const String &path) {
	Ref<FileAccess> file = FileAccess::open(path, FileAccess::READ);
	if (file.is_null()) {
		UtilityFunctions::printerr("[GoldSrc] Cannot open SPR file: ", path);
		return ERR_FILE_NOT_FOUND;
	}

	int64_t len = file->get_length();
	PackedByteArray data = file->get_buffer(len);

	parser = make_unique<goldsrc::SPRParser>();
	if (!parser->parse(data.ptr(), data.size())) {
		UtilityFunctions::printerr("[GoldSrc] Failed to parse SPR file: ", path);
		parser.reset();
		return ERR_PARSE_ERROR;
	}

	frame_cache.clear();
	frame_cache.resize(parser->get_data().frames.size());
	UtilityFunctions::print("[GoldSrc] Loaded SPR: ", path, " (", get_frame_count(), " frames)");
	return OK;
}

int GoldSrcSPR::get_frame_count() const {
	if (!parser) return 0;
	return (int)parser->get_data().frames.size();
}

Ref<ImageTexture> GoldSrcSPR::get_frame_texture(int index) const {
	if (!parser || index < 0 || index >= get_frame_count()) {
		return Ref<ImageTexture>();
	}

	if (frame_cache[index].is_valid()) {
		return frame_cache[index];
	}

	const goldsrc::SPRFrame &frame = parser->get_data().frames[index];

	PackedByteArray pixels;
	pixels.resize(frame.data.size());
	memcpy(pixels.ptrw(), frame.data.data(), frame.data.size());

	Ref<Image> image = Image::create_from_data(frame.width, frame.height,
		false, Image::FORMAT_RGBA8, pixels);

	Ref<ImageTexture> texture = ImageTexture::create_from_image(image);
	frame_cache[index] = texture;
	return texture;
}

Vector2i GoldSrcSPR::get_frame_origin(int index) const {
	if (!parser || index < 0 || index >= get_frame_count()) {
		return Vector2i(0, 0);
	}
	const goldsrc::SPRFrame &frame = parser->get_data().frames[index];
	return Vector2i(frame.origin_x, frame.origin_y);
}

int GoldSrcSPR::get_type() const {
	if (!parser) return 0;
	return (int)parser->get_data().type;
}

int GoldSrcSPR::get_texture_format() const {
	if (!parser) return 0;
	return (int)parser->get_data().texture_format;
}

Sprite3D *GoldSrcSPR::build_scene() const {
	if (!parser) {
		UtilityFunctions::printerr("[GoldSrc] build_scene: SPR not loaded");
		return nullptr;
	}

	Sprite3D *sprite = memnew(Sprite3D);

	int count = get_frame_count();
	Array frames;
	Array origins;
	for (int i = 0; i < count; i++) {
		Ref<ImageTexture> tex = get_frame_texture(i);
		if (tex.is_valid()) {
			frames.push_back(tex);
		}
		Vector2i origin = get_frame_origin(i);
		Array pair;
		pair.push_back(origin.x);
		pair.push_back(origin.y);
		origins.push_back(pair);
	}

	if (frames.size() > 0) {
		Ref<ImageTexture> first = frames[0];
		if (first.is_valid()) {
			sprite->set_texture(first);
		}
	}
	sprite->set_meta("tex_anim_frames", frames);
	sprite->set_meta("tex_anim_origins", origins);

	if (frames.size() > 1) {
		Ref<GDScript> script;
		script.instantiate();
		script->set_source_code(String(SPRITE_ANIMATOR_SCRIPT));
		if (script->reload() == OK) {
			Node *animator = memnew(Node);
			animator->set_name("SpriteAnimationPlayer");
			animator->set_script(script);
			sprite->add_child(animator);
			animator->set_owner(sprite);
		} else {
			UtilityFunctions::printerr("[GoldSrc] Failed to compile sprite animator script");
		}
	}

	return sprite;
}
