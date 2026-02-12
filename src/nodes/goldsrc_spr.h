#ifndef GOLDSRC_SPR_H
#define GOLDSRC_SPR_H

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/variant/string.hpp>

#include "../parsers/spr_parser.h"
#include <memory>

using namespace godot;

class GoldSrcSPR : public Resource {
	GDCLASS(GoldSrcSPR, Resource)

protected:
	static void _bind_methods();

public:
	GoldSrcSPR();
	~GoldSrcSPR();

	Error load_spr(const String &path);
	int get_frame_count() const;
	Ref<ImageTexture> get_frame_texture(int index) const;
	int get_type() const;
	int get_texture_format() const;

private:
	std::unique_ptr<goldsrc::SPRParser> parser;
	mutable std::vector<Ref<ImageTexture>> frame_cache;
};

#endif // GOLDSRC_SPR_H
