#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace goldsrc {

// SPR format constants
static const int SPR_MAGIC = 0x50534449; // "IDSP"
static const int SPR_VERSION = 2;

enum SPRType {
	SPR_VP_PARALLEL_UPRIGHT = 0,
	SPR_FACING_UPRIGHT = 1,
	SPR_VP_PARALLEL = 2,
	SPR_ORIENTED = 3,
	SPR_VP_PARALLEL_ORIENTED = 4,
};

enum SPRTextureFormat {
	SPR_NORMAL = 0,
	SPR_ADDITIVE = 1,
	SPR_INDEXALPHA = 2,
	SPR_ALPHATEST = 3,
};

#pragma pack(push, 1)
struct SPRHeader {
	int32_t magic;
	int32_t version;
	int32_t type;
	int32_t texture_format;
	float bounding_radius;
	int32_t max_width;
	int32_t max_height;
	int32_t num_frames;
	float beam_length;
	int32_t synch_type;
};

struct SPRFrameHeader {
	int32_t group;
	int32_t origin_x;
	int32_t origin_y;
	int32_t width;
	int32_t height;
};
#pragma pack(pop)

struct SPRFrame {
	int32_t origin_x, origin_y;
	uint32_t width, height;
	std::vector<uint8_t> data; // RGBA pixels
};

struct SPRData {
	SPRType type;
	SPRTextureFormat texture_format;
	float bounding_radius;
	std::vector<SPRFrame> frames;
};

class SPRParser {
public:
	bool parse(const uint8_t *data, size_t size);
	const SPRData &get_data() const { return spr_data; }

private:
	SPRData spr_data;
};

} // namespace goldsrc
