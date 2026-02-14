#include "spr_parser.h"
#include <cstring>

using namespace std;

namespace goldsrc {

static void decode_spr_pixels(vector<uint8_t> &dest, const uint8_t *src,
	size_t pixel_count, const uint8_t *palette, SPRTextureFormat format) {
	dest.resize(pixel_count * 4);
	for (size_t p = 0; p < pixel_count; p++) {
		uint8_t idx = src[p];
		dest[p * 4 + 0] = palette[idx * 3 + 0];
		dest[p * 4 + 1] = palette[idx * 3 + 1];
		dest[p * 4 + 2] = palette[idx * 3 + 2];
		if (format == SPR_ALPHATEST) {
			dest[p * 4 + 3] = (idx == 255) ? 0 : 255;
		} else if (format == SPR_INDEXALPHA) {
			dest[p * 4 + 3] = idx;
		} else {
			dest[p * 4 + 3] = 255;
		}
	}
}

bool SPRParser::parse(const uint8_t *data, size_t size) {
	if (size < sizeof(SPRHeader)) return false;

	const SPRHeader *header = reinterpret_cast<const SPRHeader *>(data);

	if (header->magic != SPR_MAGIC || header->version != SPR_VERSION) {
		return false;
	}

	spr_data.type = static_cast<SPRType>(header->type);
	spr_data.texture_format = static_cast<SPRTextureFormat>(header->texture_format);
	spr_data.bounding_radius = header->bounding_radius;

	size_t offset = sizeof(SPRHeader);

	// Read palette (256 colors, 3 bytes each)
	if (offset + 2 > size) return false;
	uint16_t palette_size = *reinterpret_cast<const uint16_t *>(data + offset);
	offset += 2;

	if (palette_size != 256 || offset + palette_size * 3 > size) return false;

	uint8_t palette[256 * 3];
	memcpy(palette, data + offset, 256 * 3);
	offset += palette_size * 3;

	for (int32_t i = 0; i < header->num_frames; i++) {
		if (offset + sizeof(int32_t) > size) return false;

		int32_t frame_type = *reinterpret_cast<const int32_t *>(data + offset);
		offset += sizeof(int32_t);

		if (frame_type != 0) {
			// Group sprite — read count and intervals, then frames
			if (offset + sizeof(int32_t) > size) return false;
			int32_t group_count = *reinterpret_cast<const int32_t *>(data + offset);
			offset += sizeof(int32_t);

			if (group_count <= 0) continue;

			// Skip intervals (float per frame)
			size_t intervals_size = (size_t)group_count * sizeof(float);
			if (offset + intervals_size > size) return false;
			offset += intervals_size;

			for (int32_t g = 0; g < group_count; g++) {
				if (offset + 4 * sizeof(int32_t) > size) return false;

				int32_t origin_x = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;
				int32_t origin_y = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;
				int32_t width = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;
				int32_t height = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;

				if (width <= 0 || height <= 0 || width > 4096 || height > 4096) return false;
				size_t pixel_count = (size_t)width * height;
				if (offset + pixel_count > size) return false;

				SPRFrame frame;
				frame.origin_x = origin_x;
				frame.origin_y = origin_y;
				frame.width = width;
				frame.height = height;
				decode_spr_pixels(frame.data, data + offset, pixel_count,
					palette, spr_data.texture_format);
				offset += pixel_count;
				spr_data.frames.push_back(std::move(frame));
			}
		} else {
			// Single frame
			if (offset + 4 * sizeof(int32_t) > size) return false;

			int32_t origin_x = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;
			int32_t origin_y = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;
			int32_t width = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;
			int32_t height = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;

			if (width <= 0 || height <= 0 || width > 4096 || height > 4096) return false;
			size_t pixel_count = (size_t)width * height;
			if (offset + pixel_count > size) return false;

			SPRFrame frame;
			frame.origin_x = origin_x;
			frame.origin_y = origin_y;
			frame.width = width;
			frame.height = height;
			decode_spr_pixels(frame.data, data + offset, pixel_count,
				palette, spr_data.texture_format);
			offset += pixel_count;
			spr_data.frames.push_back(std::move(frame));
		}
	}

	return !spr_data.frames.empty();
}

} // namespace goldsrc
