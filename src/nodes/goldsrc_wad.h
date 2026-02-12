#ifndef GOLDSRC_WAD_H
#define GOLDSRC_WAD_H

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>

#include "../parsers/wad_parser.h"
#include <memory>

using namespace godot;

class GoldSrcWAD : public Resource {
	GDCLASS(GoldSrcWAD, Resource)

protected:
	static void _bind_methods();

public:
	GoldSrcWAD();
	~GoldSrcWAD();

	Error load_wad(const String &path);
	Ref<ImageTexture> get_texture(const String &name) const;
	PackedStringArray get_texture_names() const;
	bool has_texture(const String &name) const;
	int get_texture_count() const;

private:
	std::unique_ptr<goldsrc::WADParser> parser;
	mutable std::map<std::string, Ref<ImageTexture>> texture_cache;
};

#endif // GOLDSRC_WAD_H
