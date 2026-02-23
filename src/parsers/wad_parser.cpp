#include "wad_parser.h"
#include "texture_decode.h"
#include <cstring>

using namespace std;

namespace goldsrc {

bool WADParser::parse(const uint8_t *data, size_t size) {
	if (size < sizeof(WADHeader)) return false;

	const WADHeader *header = reinterpret_cast<const WADHeader *>(data);

	if (strncmp(header->identification, "WAD2", 4) != 0 &&
		strncmp(header->identification, "WAD3", 4) != 0) {
		return false;
	}

	int32_t numlumps = header->numlumps;
	int32_t infotableofs = header->infotableofs;

	if (numlumps <= 0 || infotableofs < 0) return false;
	if ((size_t)infotableofs + (size_t)numlumps * sizeof(WADLumpInfo) > size) {
		return false;
	}

	const WADLumpInfo *lumps = reinterpret_cast<const WADLumpInfo *>(data + infotableofs);

	for (int32_t i = 0; i < numlumps; i++) {
		const WADLumpInfo &lump = lumps[i];

		if (lump.filepos < 0 || lump.disksize <= 0) continue;
		if ((size_t)lump.filepos + lump.disksize > size) continue;

		// Type 0x43 = miptex (WAD3), Type 0x44 = miptex (WAD2 palette)
		if (lump.type != 0x43 && lump.type != 0x44) {
			continue;
		}

		if ((size_t)lump.disksize < sizeof(WADMipTex)) continue;

		const WADMipTex *miptex = reinterpret_cast<const WADMipTex *>(data + lump.filepos);

		if (miptex->width == 0 || miptex->height == 0 || miptex->offsets[0] == 0) {
			continue;
		}

		decode_texture(miptex, data + lump.filepos, lump.disksize);
	}

	return true;
}

void WADParser::decode_texture(const WADMipTex *miptex, const uint8_t *base, size_t lump_size) {
	uint32_t w = miptex->width;
	uint32_t h = miptex->height;

	if (w == 0 || h == 0 || w > 4096 || h > 4096 || miptex->offsets[0] == 0) return;

	// Palette is after all 4 mip levels + 2 byte count
	size_t pixels = (size_t)w * h;
	size_t datasize = pixels + (pixels / 4) + (pixels / 16) + (pixels / 64);

	size_t pixel_start = miptex->offsets[0];
	size_t palette_end = pixel_start + datasize + 2 + 256 * 3;
	if (palette_end > lump_size) return;

	const uint8_t *src = base + pixel_start;
	const uint8_t *palette = base + pixel_start + datasize + 2;

	bool has_transparency = (miptex->name[0] == '{');

	WADTexture tex;
	tex.name = string(miptex->name, strnlen(miptex->name, 16));
	tex.width = w;
	tex.height = h;
	tex.has_transparency = has_transparency;
	decode_palette_pixels(src, palette, pixels, has_transparency, tex.data);

	string lower_name = to_lower(tex.name);
	texture_names.push_back(tex.name);
	textures[lower_name] = std::move(tex);
}

bool WADParser::has_texture(const string &name) const {
	return textures.find(to_lower(name)) != textures.end();
}

const WADTexture *WADParser::get_texture(const string &name) const {
	auto it = textures.find(to_lower(name));
	if (it == textures.end()) return nullptr;
	return &it->second;
}

} // namespace goldsrc
