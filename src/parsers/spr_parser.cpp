#include "spr_parser.h"
#include <cstring>

namespace goldsrc {

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
	std::memcpy(palette, data + offset, 256 * 3);
	offset += palette_size * 3;

	// Determine the last palette entry for alpha in indexalpha mode
	// Entry 255 is typically the transparent color

	for (int32_t i = 0; i < header->num_frames; i++) {
		if (offset + sizeof(int32_t) > size) return false;

		int32_t frame_type = *reinterpret_cast<const int32_t *>(data + offset);
		offset += sizeof(int32_t);

		if (frame_type != 0) {
			// Group sprite — read count and intervals, then frames
			if (offset + sizeof(int32_t) > size) return false;
			int32_t group_count = *reinterpret_cast<const int32_t *>(data + offset);
			offset += sizeof(int32_t);

			// Skip intervals (float per frame)
			offset += group_count * sizeof(float);

			for (int32_t g = 0; g < group_count; g++) {
				if (offset + 4 * sizeof(int32_t) > size) return false;

				int32_t origin_x = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;
				int32_t origin_y = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;
				int32_t width = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;
				int32_t height = *reinterpret_cast<const int32_t *>(data + offset); offset += 4;

				uint32_t pixel_count = width * height;
				if (offset + pixel_count > size) return false;

				SPRFrame frame;
				frame.origin_x = origin_x;
				frame.origin_y = origin_y;
				frame.width = width;
				frame.height = height;
				frame.data.resize(pixel_count * 4);

				const uint8_t *src = data + offset;
				for (uint32_t p = 0; p < pixel_count; p++) {
					uint8_t idx = src[p];
					frame.data[p * 4 + 0] = palette[idx * 3 + 0];
					frame.data[p * 4 + 1] = palette[idx * 3 + 1];
					frame.data[p * 4 + 2] = palette[idx * 3 + 2];

					if (spr_data.texture_format == SPR_ALPHATEST) {
						frame.data[p * 4 + 3] = (idx == 255) ? 0 : 255;
					} else if (spr_data.texture_format == SPR_INDEXALPHA) {
						frame.data[p * 4 + 3] = idx;
					} else {
						frame.data[p * 4 + 3] = 255;
					}
				}

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

			uint32_t pixel_count = width * height;
			if (offset + pixel_count > size) return false;

			SPRFrame frame;
			frame.origin_x = origin_x;
			frame.origin_y = origin_y;
			frame.width = width;
			frame.height = height;
			frame.data.resize(pixel_count * 4);

			const uint8_t *src = data + offset;
			for (uint32_t p = 0; p < pixel_count; p++) {
				uint8_t idx = src[p];
				frame.data[p * 4 + 0] = palette[idx * 3 + 0];
				frame.data[p * 4 + 1] = palette[idx * 3 + 1];
				frame.data[p * 4 + 2] = palette[idx * 3 + 2];

				if (spr_data.texture_format == SPR_ALPHATEST) {
					frame.data[p * 4 + 3] = (idx == 255) ? 0 : 255;
				} else if (spr_data.texture_format == SPR_INDEXALPHA) {
					frame.data[p * 4 + 3] = idx;
				} else {
					frame.data[p * 4 + 3] = 255;
				}
			}

			offset += pixel_count;
			spr_data.frames.push_back(std::move(frame));
		}
	}

	return !spr_data.frames.empty();
}

} // namespace goldsrc
