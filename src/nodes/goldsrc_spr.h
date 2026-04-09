#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/classes/sprite3d.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector2i.hpp>

#include "../parsers/spr_parser.h"
#include <memory>

class GoldSrcSPR : public godot::Resource {
	GDCLASS(GoldSrcSPR, godot::Resource)

protected:
	static void _bind_methods();

public:
	GoldSrcSPR() = default;
	~GoldSrcSPR() = default;

	godot::Error load_spr(const godot::String &path);
	int get_frame_count() const;
	godot::Ref<godot::ImageTexture> get_frame_texture(int index) const;
	godot::Vector2i get_frame_origin(int index) const;
	int get_type() const;
	int get_texture_format() const;

	/// Build a Sprite3D scene node with tex_anim_frames metadata and an inline
	/// GDScript animator (if multi-frame). Caller takes ownership and must free().
	godot::Sprite3D *build_scene() const;

private:
	std::unique_ptr<goldsrc::SPRParser> parser;
	mutable std::vector<godot::Ref<godot::ImageTexture>> frame_cache;
};
