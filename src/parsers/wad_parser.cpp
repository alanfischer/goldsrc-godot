#include "wad_parser.h"
#include <cstring>

namespace goldsrc {

bool WADParser::parse(const uint8_t *data, size_t size) {
	if (size < sizeof(WADHeader)) return false;

	const WADHeader *header = reinterpret_cast<const WADHeader *>(data);

	if (std::strncmp(header->identification, "WAD2", 4) != 0 &&
		std::strncmp(header->identification, "WAD3", 4) != 0) {
		return false;
	}

	int32_t numlumps = header->numlumps;
	int32_t infotableofs = header->infotableofs;

	if (infotableofs < 0 || (size_t)infotableofs + numlumps * sizeof(WADLumpInfo) > size) {
		return false;
	}

	const WADLumpInfo *lumps = reinterpret_cast<const WADLumpInfo *>(data + infotableofs);

	for (int32_t i = 0; i < numlumps; i++) {
		const WADLumpInfo &lump = lumps[i];

		if (lump.filepos < 0 || (size_t)lump.filepos + lump.disksize > size) {
			continue;
		}

		// Type 0x43 = miptex (WAD3), Type 0x44 = miptex (WAD2 palette)
		if (lump.type != 0x43 && lump.type != 0x44) {
			continue;
		}

		const WADMipTex *miptex = reinterpret_cast<const WADMipTex *>(data + lump.filepos);

		if (miptex->width == 0 || miptex->height == 0 || miptex->offsets[0] == 0) {
			continue;
		}

		decode_texture(miptex, data + lump.filepos);
	}

	return true;
}

void WADParser::decode_texture(const WADMipTex *miptex, const uint8_t *base) {
	uint32_t w = miptex->width;
	uint32_t h = miptex->height;

	if (w == 0 || h == 0 || miptex->offsets[0] == 0) return;

	// Palette is after all 4 mip levels + 2 byte count
	uint32_t pixels = w * h;
	uint32_t datasize = pixels + (pixels / 4) + (pixels / 16) + (pixels / 64);
	const uint8_t *palette = base + miptex->offsets[0] + datasize + 2;

	const uint8_t *src = base + miptex->offsets[0];

	bool has_transparency = (miptex->name[0] == '{');

	WADTexture tex;
	tex.name = std::string(miptex->name, strnlen(miptex->name, 16));
	tex.width = w;
	tex.height = h;
	tex.has_transparency = has_transparency;
	tex.data.resize(w * h * 4);

	for (uint32_t j = 0; j < pixels; j++) {
		uint8_t index = src[j];
		uint8_t r = palette[index * 3 + 0];
		uint8_t g = palette[index * 3 + 1];
		uint8_t b = palette[index * 3 + 2];

		if (has_transparency && r == 0 && g == 0 && b == 255) {
			tex.data[j * 4 + 0] = 0;
			tex.data[j * 4 + 1] = 0;
			tex.data[j * 4 + 2] = 0;
			tex.data[j * 4 + 3] = 0;
		} else {
			tex.data[j * 4 + 0] = r;
			tex.data[j * 4 + 1] = g;
			tex.data[j * 4 + 2] = b;
			tex.data[j * 4 + 3] = 255;
		}
	}

	std::string lower_name = to_lower(tex.name);
	texture_names.push_back(tex.name);
	textures[lower_name] = std::move(tex);
}

bool WADParser::has_texture(const std::string &name) const {
	return textures.find(to_lower(name)) != textures.end();
}

const WADTexture *WADParser::get_texture(const std::string &name) const {
	auto it = textures.find(to_lower(name));
	if (it == textures.end()) return nullptr;
	return &it->second;
}

} // namespace goldsrc
