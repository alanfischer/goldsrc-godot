#include "surface_below_sky.h"
#include "bsp_trace.h"

#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>

using namespace godot;
using namespace std;

namespace goldsrc {

namespace {

constexpr float GAP_SEARCH = 160.0f;  // GoldSrc units: 4 m
constexpr float MIN_GAP = 12.0f;      // 0.3 m
constexpr float NO_GAP = -1e6f;

int world_contents(const BSPData &bsp, int node, float x, float y, float z) {
	while (node >= 0) {
		if ((size_t)node >= bsp.nodes.size()) return CONTENTS_SOLID;
		const auto &n = bsp.nodes[node];
		if (n.planenum < 0 || (size_t)n.planenum >= bsp.planes.size()) return CONTENTS_SOLID;
		const auto &pl = bsp.planes[n.planenum];
		node = pl.normal[0] * x + pl.normal[1] * y + pl.normal[2] * z >= pl.dist ? n.children[0] : n.children[1];
	}
	int leaf = -(node + 1);
	return leaf >= 0 && (size_t)leaf < bsp.leafs.size() ? bsp.leafs[leaf].contents : CONTENTS_SOLID;
}

} // anonymous namespace

Dictionary bake_surface_below_sky(const BSPData &bsp_data, float cell_size_gs, float scale_factor) {
	if (bsp_data.models.empty() || cell_size_gs <= 0.0f) return Dictionary();
	uint64_t t0 = Time::get_singleton()->get_ticks_msec();

	const auto &world = bsp_data.models[0];
	int head = world.headnode[0];
	int w = max(1, (int)ceilf((world.maxs[0] - world.mins[0]) / cell_size_gs));
	int h = max(1, (int)ceilf((world.maxs[1] - world.mins[1]) / cell_size_gs));
	float top = world.maxs[2] + 1.0f;
	float span = world.maxs[2] - world.mins[2] + 2.0f;
	static const float down[3] = {0, 0, -1};

	// Liquid stops the trace, and a func_water is a brush entity whose faces are in its own
	// model's tree, not the world's: without these a moat's floor reads as open to the sky.
	vector<int> liquid_models;
	for (const auto &ent : bsp_data.entities) {
		auto cn = ent.properties.find("classname");
		auto mdl = ent.properties.find("model");
		if (cn == ent.properties.end() || cn->second != "func_water" || mdl == ent.properties.end()
				|| mdl->second.size() < 2 || mdl->second[0] != '*') continue;
		int m = atoi(mdl->second.c_str() + 1);
		if (m > 0 && (size_t)m < bsp_data.models.size()) liquid_models.push_back(m);
	}

	// The air under the surface at x, y (GoldSrc units): (underside, ground), or none.
	const Vector2 no_gap(NO_GAP, NO_GAP);
	auto gap_under = [&](float x, float y, float landed) {
		float origin[3] = {x, y, top};
		float upper = landed;
		float t = top - landed + 0.5f;
		for (int i = 0; i < 32 && upper > landed - GAP_SEARCH; i++) {
			RayHit hit;
			trace_ray(bsp_data, head, origin, down, t, span, hit);
			float lower = hit.face_index >= 0 ? hit.hit_pos[2] : top - span;
			if (world_contents(bsp_data, head, x, y, (upper + lower) * 0.5f) == CONTENTS_EMPTY)
				return upper - lower >= MIN_GAP ? Vector2(upper, lower) : no_gap;
			if (hit.face_index < 0) break;
			upper = lower;
			t = hit.t + 0.5f;
		}
		return no_gap;
	};

	// The surface at x, y, and in `air` the open air under it. `capped`: solid just over the
	// surface (a ceiling met from below, the trace having come down through the solid over a
	// sealed room), so it is not open from there up.
	auto land = [&](float x, float y, Vector2 &air, bool &capped) {
		float origin[3] = {x, y, top};
		// A column that meets nothing is outside the sealed map: it stops at the top.
		float landed = top;
		float t = 0.0f;
		while (t < span) {
			RayHit hit;
			trace_ray(bsp_data, head, origin, down, t, span, hit);
			if (hit.face_index < 0) break;
			if (!is_sky_texture(bsp_data.faces[hit.face_index].texture_name)) {
				landed = hit.hit_pos[2];
				break;
			}
			t = hit.t + 0.5f;
		}
		air = landed < top ? gap_under(x, y, landed) : no_gap;
		capped = landed < top
			&& world_contents(bsp_data, head, x, y, landed + 1.0f) == CONTENTS_SOLID;
		if (landed < top) {
			for (int m : liquid_models) {
				const auto &liquid = bsp_data.models[m];
				if (x < liquid.mins[0] || x > liquid.maxs[0] || y < liquid.mins[1] || y > liquid.maxs[1])
					continue;
				RayHit hit;
				trace_ray(bsp_data, liquid.headnode[0], origin, down, 0.0f, span, hit, m);
				if (hit.face_index >= 0 && hit.hit_pos[2] > landed) {
					landed = hit.hit_pos[2];
					air = no_gap;  // the surface is liquid
				}
			}
		}
		return landed;
	};

	// A cell is one height, so it takes the highest of its centre and corners: a roof over any of
	// it covers all of it. A centre alone left every wall a cell straddled open to the sky.
	// Corners are shared, and nudged off the grid lines that walls are built on.
	const float nudge = 0.25f;
	vector<float> corners((size_t)(w + 1) * (h + 1));
	vector<Vector2> corner_air((size_t)(w + 1) * (h + 1));
	vector<char> corner_capped((size_t)(w + 1) * (h + 1));
	for (int iy = 0; iy <= h; iy++)
		for (int ix = 0; ix <= w; ix++) {
			size_t c = (size_t)iy * (w + 1) + ix;
			bool capped;
			corners[c] = land(world.mins[0] + ix * cell_size_gs + nudge,
				world.mins[1] + iy * cell_size_gs + nudge, corner_air[c], capped);
			corner_capped[c] = capped;
		}

	PackedByteArray data;
	data.resize((size_t)w * h * 4);
	float *heights = (float *)data.ptrw();
	PackedByteArray under_data;
	under_data.resize((size_t)w * h * 8);
	float *under = (float *)under_data.ptrw();
	int open = 0;
	int sheltered = 0;
	for (int iy = 0; iy < h; iy++) {
		for (int ix = 0; ix < w; ix++) {
			float tops[5];
			Vector2 airs[5];
			bool capped[5];
			tops[0] = land(world.mins[0] + (ix + 0.5f) * cell_size_gs,
				world.mins[1] + (iy + 0.5f) * cell_size_gs, airs[0], capped[0]);
			float landed = tops[0];
			for (int k = 0; k < 4; k++) {
				size_t c = (size_t)(iy + (k >> 1)) * (w + 1) + ix + (k & 1);
				tops[k + 1] = corners[c];
				airs[k + 1] = corner_air[c];
				capped[k + 1] = corner_capped[c];
				landed = fmaxf(landed, corners[c]);
			}
			if (landed >= top) open++;
			heights[(size_t)iy * w + ix] = landed * scale_factor;
			// As Overcast's rain floor reads it back: solid where the middle or two corners
			// are, else the air every open sample agrees on. One lower than the cell's top is out
			// from under the roof, open from its top up, unless it is capped: then it is in the
			// solid the cell's roof is part of.
			int solid = 0;
			float hi = 1e30f, lo = -1e30f;
			bool none = false;
			for (int k = 0; k < 5 && !none; k++) {
				if (tops[k] >= landed - 0.5f) {
					if (airs[k].x <= airs[k].y) {
						if (k == 0) none = true;
						solid++;
						continue;
					}
					hi = fminf(hi, airs[k].x);
					lo = fmaxf(lo, airs[k].y);
				} else if (capped[k]) {
					if (k == 0) none = true;
					solid++;
				} else {
					lo = fmaxf(lo, tops[k]);
				}
			}
			Vector2 cell_air = no_gap;
			if (!none && solid < 2 && hi < 1e30f && hi - lo >= MIN_GAP) {
				cell_air = Vector2(hi * scale_factor, lo * scale_factor);
				sheltered++;
			}
			under[((size_t)iy * w + ix) * 2] = cell_air.x;
			under[((size_t)iy * w + ix) * 2 + 1] = cell_air.y;
		}
	}

	Dictionary result;
	result["origin"] = Vector3(-world.mins[0] * scale_factor, 0.0f, world.mins[1] * scale_factor);
	result["size"] = Vector2(w * cell_size_gs * scale_factor, h * cell_size_gs * scale_factor);
	result["image"] = Image::create_from_data(w, h, false, Image::FORMAT_RF, data);
	result["under"] = Image::create_from_data(w, h, false, Image::FORMAT_RGF, under_data);
	result["top"] = top * scale_factor;
	UtilityFunctions::print("[GoldSrc] Surface below sky: ", w, "x", h, " columns (", open,
		" outside the map, ", sheltered, " with air under cover), ", (int64_t)(Time::get_singleton()->get_ticks_msec() - t0), "ms");
	return result;
}

} // namespace goldsrc
