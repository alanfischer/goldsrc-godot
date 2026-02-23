#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace goldsrc {

// Decode palette-indexed pixels to RGBA. Index 255 is transparent when has_transparency is set.
inline void decode_palette_pixels(
	const uint8_t *pixel_data, const uint8_t *palette,
	size_t pixel_count, bool has_transparency,
	std::vector<uint8_t> &out_rgba) {

	out_rgba.resize(pixel_count * 4);
	for (size_t j = 0; j < pixel_count; j++) {
		uint8_t idx = pixel_data[j];
		uint8_t r = palette[idx * 3 + 0];
		uint8_t g = palette[idx * 3 + 1];
		uint8_t b = palette[idx * 3 + 2];

		if (has_transparency && idx == 255) {
			out_rgba[j * 4 + 0] = 0;
			out_rgba[j * 4 + 1] = 0;
			out_rgba[j * 4 + 2] = 0;
			out_rgba[j * 4 + 3] = 0;
		} else {
			out_rgba[j * 4 + 0] = r;
			out_rgba[j * 4 + 1] = g;
			out_rgba[j * 4 + 2] = b;
			out_rgba[j * 4 + 3] = 255;
		}
	}
}

} // namespace goldsrc
