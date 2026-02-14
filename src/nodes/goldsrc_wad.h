#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>

#include "../parsers/wad_parser.h"
#include <memory>

class GoldSrcWAD : public godot::Resource {
	GDCLASS(GoldSrcWAD, godot::Resource)

protected:
	static void _bind_methods();

public:
	GoldSrcWAD() = default;
	~GoldSrcWAD() = default;

	godot::Error load_wad(const godot::String &path);
	godot::Ref<godot::ImageTexture> get_texture(const godot::String &name) const;
	godot::PackedStringArray get_texture_names() const;
	bool has_texture(const godot::String &name) const;
	int get_texture_count() const;

private:
	std::unique_ptr<goldsrc::WADParser> parser;
	mutable std::map<std::string, godot::Ref<godot::ImageTexture>> texture_cache;
};
