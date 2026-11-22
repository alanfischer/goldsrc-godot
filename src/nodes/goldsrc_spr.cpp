#include "goldsrc_spr.h"

#include <godot_cpp/classes/base_material3d.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/gd_script.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/sprite3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <climits>
#include <cstring>

using namespace godot;
using namespace std;

// Inline GDScript embedded into the .scn produced by build_scene(). @tool so bake() runs inside
// the editor's importer; playback stays off in the editor.
// Mirrors the AnimationPlayer API so SPR and MDL scenes can be driven identically. The frames are
// one Texture2DArray and the sprite's ShaderMaterial picks the layer: a looping play runs on the
// shader clock and costs nothing per frame; one-shots, pauses and seeks step a frame uniform from
// a Timer. No C++ plugin dependency at runtime: pure GDScript baked into the .scn.
static const char *SPRITE_ANIMATOR_SCRIPT = R"GD(@tool
extends Node
signal animation_finished(anim_name: String)
signal animation_started(anim_name: String)
@export var fps: float = 10.0:
	set(v):
		fps = v
		_sync()
@export var loop_animation: bool = true:
	set(v):
		loop_animation = v
		_sync()
@export var autoplay: String = "default"
@export var frame_count: int = 0
@export var base_material: ShaderMaterial
## The frames as Images: a Texture2DArray made during a headless import saves empty (its layers
## live in the renderer), so the array is built from these at runtime, once per sprite file.
@export var layers: Array[Image] = []
@export var billboard_mode: int = BaseMaterial3D.BILLBOARD_ENABLED
@export var blend_mode: int = BaseMaterial3D.BLEND_MODE_MIX
@export var alpha_scissor: bool = false
@export var fog_enabled: bool = true
var speed_scale: float = 1.0:
	set(v):
		speed_scale = v
		_sync()
var current_animation: String = ""
var current_animation_position: float:
	get:
		return _frame / maxf(fps * speed_scale, 0.001)
var _playing: bool = false
var _frame: int = 0
var _unique: bool = false
var _timer: Timer = null
static var _shared := {}

const _BILLBOARD := {
	BaseMaterial3D.BILLBOARD_ENABLED: "\tMODELVIEW_MATRIX = VIEW_MATRIX * mat4(MAIN_CAM_INV_VIEW_MATRIX[0], MAIN_CAM_INV_VIEW_MATRIX[1], MAIN_CAM_INV_VIEW_MATRIX[2], MODEL_MATRIX[3]);\n\tMODELVIEW_NORMAL_MATRIX = mat3(MODELVIEW_MATRIX);\n",
	BaseMaterial3D.BILLBOARD_FIXED_Y: "\tMODELVIEW_MATRIX = VIEW_MATRIX * mat4(vec4(normalize(cross(vec3(0.0, 1.0, 0.0), MAIN_CAM_INV_VIEW_MATRIX[2].xyz)), 0.0), vec4(0.0, 1.0, 0.0, 0.0), vec4(normalize(cross(MAIN_CAM_INV_VIEW_MATRIX[0].xyz, vec3(0.0, 1.0, 0.0))), 0.0), MODEL_MATRIX[3]);\n\tMODELVIEW_NORMAL_MATRIX = mat3(MODELVIEW_MATRIX);\n",
}
const _BLEND := ["blend_mix", "blend_add", "blend_sub", "blend_mul", "blend_premul_alpha"]

func _ready() -> void:
	if Engine.is_editor_hint():
		return
	if autoplay != "" and frame_count > 1:
		play(autoplay)
	else:
		_sync()

## Import time: the material over `frames`, the sprite's frames padded to one canvas each.
func bake(frames: Array[Image]) -> void:
	layers = frames
	frame_count = frames.size()
	var mat := ShaderMaterial.new()
	mat.shader = Shader.new()
	mat.shader.code = _code()
	mat.set_shader_parameter(&"frame_count", float(frame_count))
	base_material = mat
	get_parent().material_override = mat

func _ensure_frames() -> void:
	if base_material and base_material.get_shader_parameter(&"frames") == null and not layers.is_empty():
		var array := Texture2DArray.new()
		array.create_from_images(layers)
		base_material.set_shader_parameter(&"frames", array)

## Change how the sprite is drawn: a BaseMaterial3D billboard and blend mode, and whether fog applies.
func restyle(billboard: int, blend: int, fog := true) -> void:
	billboard_mode = billboard
	blend_mode = blend
	fog_enabled = fog
	if _unique:
		var mat := get_parent().material_override as ShaderMaterial
		mat.shader = _shader(_code())
	_sync()

## This sprite's own material, for per-instance parameters (render_priority, the "albedo" tint).
func unique_material() -> ShaderMaterial:
	var spr := get_parent() as Sprite3D
	_ensure_frames()
	if not _unique:
		var mat := (spr.material_override as ShaderMaterial).duplicate() as ShaderMaterial
		mat.shader = _shader(_code())
		spr.material_override = mat
		_unique = true
		_sync()
	return spr.material_override as ShaderMaterial

func play(anim_name: String = "default", _blend: float = -1.0, _speed: float = 1.0, _from_end: bool = false) -> void:
	if frame_count < 2:
		return
	var restart := current_animation != anim_name or not _playing
	current_animation = anim_name
	if restart:
		_frame = 0
		if _timer:
			_timer.stop()
		animation_started.emit(anim_name)
	_playing = true
	_sync()

func pause() -> void:
	_playing = false
	_sync()

func stop(keep_state: bool = true) -> void:
	_playing = false
	if not keep_state:
		_frame = 0
		current_animation = ""
	_sync()

func is_playing() -> bool:
	return _playing

func seek(seconds: float, update: bool = false) -> void:
	_frame = _frame_at(seconds)
	if update:
		_sync()

func get_animation_list() -> PackedStringArray:
	if frame_count > 1:
		return PackedStringArray(["default"])
	return PackedStringArray()

func has_animation(anim_name: StringName) -> bool:
	return anim_name == &"default" and frame_count > 1

func _frame_at(t: float) -> int:
	var raw := int(t * fps)
	if loop_animation:
		return raw % maxi(frame_count, 1)
	return mini(raw, frame_count - 1)

func _sync() -> void:
	var spr := get_parent() as Sprite3D
	if spr == null or base_material == null or Engine.is_editor_hint():
		return
	_ensure_frames()
	var rate := fps * speed_scale
	var clocked := _playing and loop_animation and rate > 0.0
	if _unique:
		var mat := spr.material_override as ShaderMaterial
		mat.set_shader_parameter(&"fps", rate if clocked else 0.0)
		mat.set_shader_parameter(&"frame", float(_frame))
	elif clocked or _frame == 0:
		spr.material_override = _shared_material(rate if clocked else 0.0)
	else:
		unique_material()
		return
	var stepping := _playing and not loop_animation and rate > 0.0
	if stepping and _timer == null:
		_timer = Timer.new()
		_timer.timeout.connect(_step)
		add_child(_timer)
	if _timer:
		if stepping:
			if _timer.is_stopped() or not is_equal_approx(_timer.wait_time, 1.0 / rate):
				_timer.start(1.0 / rate)
		else:
			_timer.stop()

func _step() -> void:
	_frame += 1
	if _frame < frame_count:
		(get_parent().material_override as ShaderMaterial).set_shader_parameter(&"frame", float(_frame))
		return
	_frame = frame_count - 1
	_playing = false
	_timer.stop()
	get_parent().visible = false
	var finished_anim := current_animation
	current_animation = ""
	animation_finished.emit(finished_anim)

func _shared_material(rate: float) -> ShaderMaterial:
	var code := _code()
	var key := [base_material, code, rate]
	var mat: ShaderMaterial = _shared.get(key)
	if mat == null:
		mat = base_material.duplicate() as ShaderMaterial
		mat.shader = _shader(code)
		mat.set_shader_parameter(&"fps", rate)
		mat.set_shader_parameter(&"frame", 0.0)
		_shared[key] = mat
	return mat

## One Shader per look across every sprite file (Engine meta, as each imported scene carries its
## own copy of this script), so a look compiles once however many sprites use it.
func _shader(code: String) -> Shader:
	if not Engine.has_meta(&"goldsrc_sprite_shaders"):
		Engine.set_meta(&"goldsrc_sprite_shaders", {})
	var shaders: Dictionary = Engine.get_meta(&"goldsrc_sprite_shaders")
	var shader: Shader = shaders.get(code)
	if shader == null:
		if base_material and base_material.shader and base_material.shader.code == code:
			shader = base_material.shader
		else:
			shader = Shader.new()
			shader.code = code
		shaders[code] = shader
	return shader

func _code() -> String:
	var modes := PackedStringArray(["unshaded", "cull_back", "depth_draw_opaque", _BLEND[blend_mode]])
	if not fog_enabled:
		modes.append("fog_disabled")
	var alpha := "\tALPHA = albedo.a * tex.a;\n"
	if alpha_scissor:
		alpha += "\tALPHA_SCISSOR_THRESHOLD = 0.5;\n"
	return ("shader_type spatial;\nrender_mode %s;\n" % ", ".join(modes)
		+ "uniform sampler2DArray frames : source_color, filter_linear_mipmap, repeat_enable;\n"
		+ "uniform float frame_count = 1.0;\nuniform float fps = 0.0;\nuniform float frame = 0.0;\n"
		+ "uniform vec4 albedo : source_color = vec4(1.0);\n"
		+ "void vertex() {\n%s}\n" % _BILLBOARD.get(billboard_mode, "")
		+ "void fragment() {\n"
		+ "\tfloat layer = fps > 0.0 ? mod(floor(TIME * fps), frame_count) : frame;\n"
		+ "\tvec4 tex = texture(frames, vec3(UV, layer));\n"
		+ "\tALBEDO = albedo.rgb * tex.rgb;\n" + alpha + "}\n")
)GD";

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
	const goldsrc::SPRData &data = parser->get_data();
	int count = get_frame_count();
	if (count == 0) {
		UtilityFunctions::printerr("[GoldSrc] build_scene: SPR has no frames");
		return nullptr;
	}

	Sprite3D *sprite = memnew(Sprite3D);

	Array frames;
	Array origins;
	int left = INT_MAX, right = INT_MIN, top = INT_MIN, bottom = INT_MAX;
	for (int i = 0; i < count; i++) {
		const goldsrc::SPRFrame &f = data.frames[i];
		frames.push_back(get_frame_texture(i));
		Array pair;
		pair.push_back(f.origin_x);
		pair.push_back(f.origin_y);
		origins.push_back(pair);
		left = std::min(left, f.origin_x);
		right = std::max(right, f.origin_x + (int)f.width);
		top = std::max(top, f.origin_y);
		bottom = std::min(bottom, f.origin_y - (int)f.height);
	}
	sprite->set_meta("tex_anim_frames", frames);
	sprite->set_meta("tex_anim_origins", origins);

	// Every frame on one canvas, the union of their rects about the sprite origin: a frame's own
	// origin becomes where it sits, so the quad never has to move between frames.
	int w = right - left, h = top - bottom;
	TypedArray<Image> layers;
	for (int i = 0; i < count; i++) {
		const goldsrc::SPRFrame &f = data.frames[i];
		PackedByteArray canvas;
		canvas.resize((int64_t)w * h * 4);
		memset(canvas.ptrw(), 0, canvas.size());
		int x0 = f.origin_x - left, y0 = top - f.origin_y;
		for (int y = 0; y < (int)f.height; y++)
			memcpy(canvas.ptrw() + ((int64_t)(y0 + y) * w + x0) * 4, f.data.data() + (size_t)y * f.width * 4, (size_t)f.width * 4);
		layers.push_back(Image::create_from_data(w, h, false, Image::FORMAT_RGBA8, canvas));
	}
	sprite->set_texture(ImageTexture::create_from_image(layers[0]));
	sprite->set_offset(Vector2(left + w * 0.5f, top - h * 0.5f));

	Ref<GDScript> script;
	script.instantiate();
	script->set_source_code(String(SPRITE_ANIMATOR_SCRIPT));
	if (script->reload() != OK) {
		UtilityFunctions::printerr("[GoldSrc] Failed to compile sprite animator script");
		return sprite;
	}
	Node *animator = memnew(Node);
	animator->set_name("SpriteAnimationPlayer");
	animator->set_script(script);
	sprite->add_child(animator);
	animator->set_owner(sprite);

	// Orientation and blend come from the SPR header.
	int billboard;
	switch (data.type) {
		case goldsrc::SPR_VP_PARALLEL_UPRIGHT:
		case goldsrc::SPR_FACING_UPRIGHT:
			billboard = BaseMaterial3D::BILLBOARD_FIXED_Y;
			break;
		case goldsrc::SPR_ORIENTED:
			billboard = BaseMaterial3D::BILLBOARD_DISABLED;
			break;
		default: // VP_PARALLEL, VP_PARALLEL_ORIENTED
			billboard = BaseMaterial3D::BILLBOARD_ENABLED;
			break;
	}
	animator->set("billboard_mode", billboard);
	animator->set("blend_mode", data.texture_format == goldsrc::SPR_ADDITIVE
		? BaseMaterial3D::BLEND_MODE_ADD : BaseMaterial3D::BLEND_MODE_MIX);
	animator->set("alpha_scissor", data.texture_format == goldsrc::SPR_ALPHATEST);
	animator->call("bake", layers);
	return sprite;
}
