#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/variant/string.hpp>

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
	int get_type() const;
	int get_texture_format() const;

private:
	std::unique_ptr<goldsrc::SPRParser> parser;
	mutable std::vector<godot::Ref<godot::ImageTexture>> frame_cache;
};
