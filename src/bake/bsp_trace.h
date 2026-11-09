#pragma once

// Ray traces through a BSP's hull 0 tree, and the lightmap and sun lookups the bakers share.

#include "../parsers/bsp_parser.h"

#include <cstdint>
#include <vector>

namespace goldsrc {

// The closest face a ray hit, in parsed face indices.
struct RayHit {
	int face_index = -1;
	float t = 1e30f;
	float hit_pos[3] = {0, 0, 0};
};

// A ray "reaches the sky" if it hits nothing, or if its closest hit is a sky-textured face.
bool ray_reached_sky(const BSPData &bsp_data, const RayHit &hit);

// Walks the BSP node tree from node_index along origin + dir * t, t in [t_min, t_max], testing
// the faces on each node; best_hit keeps the closest. Only model_index's own faces count.
void trace_ray(const BSPData &bsp_data, int node_index, const float origin[3], const float dir[3],
	float t_min, float t_max, RayHit &best_hit, int model_index = 0);

// Style 0's lightmap colour (0..255) at a GoldSrc point on a face; out_luxel, if given, the
// luxel index within the face (-1 when the face has no lightmap there).
void sample_lightmap_at(const ParsedFace &face, const float hit_pos[3],
	const std::vector<uint8_t> &lighting, float &out_r, float &out_g, float &out_b,
	int *out_luxel = nullptr);

struct SunLight {
	bool present = false;
	float dir[3] = {0, 0, -1};   // TOWARD the sun; straight down until a light_environment says
	float color[3] = {0, 0, 0};  // 0..1, scaled by brightness / 200
	int style = 0;               // lightstyle hlrad baked the sun into
};

// The map's first light_environment, which is the one hlrad lit it with.
SunLight find_sun(const BSPData &bsp_data);

} // namespace goldsrc
