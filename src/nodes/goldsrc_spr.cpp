#include "goldsrc_spr.h"

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <cstring>

using namespace godot;
using namespace std;

void GoldSrcSPR::_bind_methods() {
	ClassDB::bind_method(D_METHOD("load_spr", "path"), &GoldSrcSPR::load_spr);
	ClassDB::bind_method(D_METHOD("get_frame_count"), &GoldSrcSPR::get_frame_count);
	ClassDB::bind_method(D_METHOD("get_frame_texture", "index"), &GoldSrcSPR::get_frame_texture);
	ClassDB::bind_method(D_METHOD("get_type"), &GoldSrcSPR::get_type);
	ClassDB::bind_method(D_METHOD("get_texture_format"), &GoldSrcSPR::get_texture_format);
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

int GoldSrcSPR::get_type() const {
	if (!parser) return 0;
	return (int)parser->get_data().type;
}

int GoldSrcSPR::get_texture_format() const {
	if (!parser) return 0;
	return (int)parser->get_data().texture_format;
}
