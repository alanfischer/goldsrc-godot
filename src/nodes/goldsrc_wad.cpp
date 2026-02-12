#include "goldsrc_wad.h"

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

GoldSrcWAD::GoldSrcWAD() {
}

GoldSrcWAD::~GoldSrcWAD() {
}

void GoldSrcWAD::_bind_methods() {
	ClassDB::bind_method(D_METHOD("load_wad", "path"), &GoldSrcWAD::load_wad);
	ClassDB::bind_method(D_METHOD("get_texture", "name"), &GoldSrcWAD::get_texture);
	ClassDB::bind_method(D_METHOD("get_texture_names"), &GoldSrcWAD::get_texture_names);
	ClassDB::bind_method(D_METHOD("has_texture", "name"), &GoldSrcWAD::has_texture);
	ClassDB::bind_method(D_METHOD("get_texture_count"), &GoldSrcWAD::get_texture_count);
}

Error GoldSrcWAD::load_wad(const String &path) {
	Ref<FileAccess> file = FileAccess::open(path, FileAccess::READ);
	if (file.is_null()) {
		UtilityFunctions::printerr("[GoldSrc] Cannot open WAD file: ", path);
		return ERR_FILE_NOT_FOUND;
	}

	int64_t len = file->get_length();
	PackedByteArray data = file->get_buffer(len);

	parser = std::make_unique<goldsrc::WADParser>();
	if (!parser->parse(data.ptr(), data.size())) {
		UtilityFunctions::printerr("[GoldSrc] Failed to parse WAD file: ", path);
		parser.reset();
		return ERR_PARSE_ERROR;
	}

	texture_cache.clear();
	UtilityFunctions::print("[GoldSrc] Loaded WAD: ", path, " (", parser->get_texture_count(), " textures)");
	return OK;
}

Ref<ImageTexture> GoldSrcWAD::get_texture(const String &name) const {
	if (!parser) return Ref<ImageTexture>();

	std::string sname = name.utf8().get_data();

	// Check cache
	auto it = texture_cache.find(sname);
	if (it != texture_cache.end()) {
		return it->second;
	}

	const goldsrc::WADTexture *tex = parser->get_texture(sname);
	if (!tex) return Ref<ImageTexture>();

	// Create Godot Image from RGBA data
	PackedByteArray pixels;
	pixels.resize(tex->data.size());
	memcpy(pixels.ptrw(), tex->data.data(), tex->data.size());

	Ref<Image> image = Image::create_from_data(tex->width, tex->height,
		false, Image::FORMAT_RGBA8, pixels);

	Ref<ImageTexture> texture = ImageTexture::create_from_image(image);
	texture_cache[sname] = texture;
	return texture;
}

PackedStringArray GoldSrcWAD::get_texture_names() const {
	PackedStringArray result;
	if (!parser) return result;

	for (const auto &name : parser->get_texture_names()) {
		result.push_back(String(name.c_str()));
	}
	return result;
}

bool GoldSrcWAD::has_texture(const String &name) const {
	if (!parser) return false;
	return parser->has_texture(name.utf8().get_data());
}

int GoldSrcWAD::get_texture_count() const {
	if (!parser) return 0;
	return parser->get_texture_count();
}
