#include "light_grid.h"
#include "bsp_trace.h"

#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

using namespace godot;
using namespace std;

namespace goldsrc {

Dictionary bake_light_grid(const BSPData &bsp_data, const SunFraction &face_sun_fraction,
		float cell_size_gs, float scale_factor) {
	if (cell_size_gs <= 0.0f) return Dictionary();
	// Bake reads the raw lighting lump + per-face lightmap offsets directly
	// (see sample_lightmap_at), not lm_atlases — so do NOT gate on lm_atlases.
	// In the shader-lightstyle path (shader_lightstyles = true, which the game
	// always uses) build_mesh populates layer_textures[] but never lm_atlases,
	// so gating on it here suppressed the grid on every map. A non-empty lighting
	// lump is the correct signal that lightmaps exist; sample_lightmap_at bounds-
	// checks each offset, so stale/mismatched offsets just read as unlit.
	if (bsp_data.models.empty() || bsp_data.lighting.empty())
		return Dictionary();

	uint64_t t0 = Time::get_singleton()->get_ticks_msec();

	// Grid bounds from model 0 (worldspawn), padded by 1 cell
	float mins[3] = {
		bsp_data.models[0].mins[0] - cell_size_gs,
		bsp_data.models[0].mins[1] - cell_size_gs,
		bsp_data.models[0].mins[2] - cell_size_gs
	};
	float maxs[3] = {
		bsp_data.models[0].maxs[0] + cell_size_gs,
		bsp_data.models[0].maxs[1] + cell_size_gs,
		bsp_data.models[0].maxs[2] + cell_size_gs
	};

	int dims[3];
	for (int i = 0; i < 3; i++) {
		dims[i] = (int)ceilf((maxs[i] - mins[i]) / cell_size_gs);
		if (dims[i] < 1) dims[i] = 1;
	}

	// GoldSrc directions for ambient cube:
	// Index 0: +X_gs, 1: -X_gs, 2: +Y_gs, 3: -Y_gs, 4: +Z_gs, 5: -Z_gs
	static const float directions[6][3] = {
		{ 1, 0, 0}, {-1, 0, 0},
		{ 0, 1, 0}, { 0,-1, 0},
		{ 0, 0, 1}, { 0, 0,-1}
	};

	// Rays that escape the map return the sky's light; one more ray toward the sun adds it.
	const SunLight sun = find_sun(bsp_data);
	const float *sky_color = sun.color;
	const float *sun_dir = sun.dir;
	const bool has_sky_light = sun.present;

	// 3D texture layout: each direction gets dims[2] slices (GoldSrc Z = Godot Y),
	// each slice is dims[0] x dims[1] (GoldSrc XY = Godot XZ), RGB8.
	int slice_w = dims[0];
	int slice_h = dims[1];
	int num_slices = dims[2];
	size_t slice_bytes = (size_t)slice_w * slice_h * 3;

	// Allocate 6 direction buffers, each with num_slices * slice_bytes
	size_t vol_bytes = slice_bytes * num_slices;
	vector<vector<uint8_t>> pixel_data(6, vector<uint8_t>(vol_bytes, 0));

	// Track which cells have valid data (non-solid, traced)
	size_t total_cells = (size_t)dims[0] * dims[1] * dims[2];
	vector<bool> cell_valid(total_cells, false);

	// The sky's share of each cell's light, weighted by brightness over the cube's faces: what
	// night turns down on models, the light grid's twin of the world's lm_sun.
	const bool has_sun_fraction = !face_sun_fraction.empty();
	vector<float> sky_weight(has_sun_fraction ? total_cells : 0, 0.0f);
	vector<float> all_weight(has_sun_fraction ? total_cells : 0, 0.0f);
	vector<uint8_t> sun_data(has_sun_fraction ? total_cells : 0, 0);

	float max_trace_dist = 8192.0f;
	int head_node = bsp_data.models[0].headnode[0];
	int cells_traced = 0;
	int cells_solid = 0;
	int cells_sunlit = 0;

	for (int gz = 0; gz < dims[2]; gz++) {
		for (int gy = 0; gy < dims[1]; gy++) {
			for (int gx = 0; gx < dims[0]; gx++) {
				float cx = mins[0] + (gx + 0.5f) * cell_size_gs;
				float cy = mins[1] + (gy + 0.5f) * cell_size_gs;
				float cz = mins[2] + (gz + 0.5f) * cell_size_gs;

				// Check if cell is in solid
				float gs_pos[3] = {cx, cy, cz};
				{
					int node_idx = head_node;
					while (node_idx >= 0) {
						if ((size_t)node_idx >= bsp_data.nodes.size()) break;
						const auto &node = bsp_data.nodes[node_idx];
						if (node.planenum < 0 || (size_t)node.planenum >= bsp_data.planes.size()) break;
						const auto &plane = bsp_data.planes[node.planenum];
						float d = plane.normal[0] * cx + plane.normal[1] * cy + plane.normal[2] * cz;
						node_idx = (d >= plane.dist) ? node.children[0] : node.children[1];
					}
					int leaf_idx = -(node_idx + 1);
					if (leaf_idx >= 0 && (size_t)leaf_idx < bsp_data.leafs.size()) {
						if (bsp_data.leafs[leaf_idx].contents == CONTENTS_SOLID) {
							cells_solid++;
							continue;
						}
					}
				}

				cells_traced++;
				size_t cell_idx = (size_t)gz * dims[0] * dims[1] + (size_t)gy * dims[0] + gx;
				cell_valid[cell_idx] = true;

				// Pixel offset within slice gz: row gy, column gx
				size_t px_ofs = (size_t)gz * slice_bytes + ((size_t)gy * slice_w + gx) * 3;

				// Trace the 6 axis-aligned directions
				for (int d = 0; d < 6; d++) {
					RayHit hit;
					trace_ray(bsp_data, head_node, gs_pos, directions[d],
						0.0f, max_trace_dist, hit);

					bool reached_sky = ray_reached_sky(bsp_data, hit);
					if (!reached_sky) {
						float r, g, b;
						int luxel;
						sample_lightmap_at(bsp_data.faces[hit.face_index],
							hit.hit_pos, bsp_data.lighting, r, g, b, &luxel);
						if (has_sun_fraction && luxel >= 0) {
							const auto &frac = face_sun_fraction[hit.face_index];
							float f = (size_t)luxel < frac.size() ? layer0_sun_share(bsp_data,
								bsp_data.faces[hit.face_index], luxel, frac[luxel] / 255.0f, sun.style) : 0.0f;
							sky_weight[cell_idx] += (r + g + b) * f;
							all_weight[cell_idx] += r + g + b;
						}
						pixel_data[d][px_ofs + 0] = (uint8_t)clamp(r, 0.0f, 255.0f);
						pixel_data[d][px_ofs + 1] = (uint8_t)clamp(g, 0.0f, 255.0f);
						pixel_data[d][px_ofs + 2] = (uint8_t)clamp(b, 0.0f, 255.0f);
					} else if (has_sky_light) {
						// Ray reached the sky — ambient sky fill for non-sun directions
						float ambient_sky = 0.15f;
						float r = sky_color[0] * ambient_sky * 255.0f;
						float g = sky_color[1] * ambient_sky * 255.0f;
						float b = sky_color[2] * ambient_sky * 255.0f;
						if (has_sun_fraction) {
							sky_weight[cell_idx] += r + g + b;
							all_weight[cell_idx] += r + g + b;
						}
						pixel_data[d][px_ofs + 0] = (uint8_t)clamp(r, 0.0f, 255.0f);
						pixel_data[d][px_ofs + 1] = (uint8_t)clamp(g, 0.0f, 255.0f);
						pixel_data[d][px_ofs + 2] = (uint8_t)clamp(b, 0.0f, 255.0f);
					}
				}

				// Trace one extra ray toward the sun. If it reaches the sky
				// (hits a sky brush or nothing), this cell has direct sunlight —
				// add sun color to each cube face weighted by alignment with the
				// sun direction.
				if (has_sky_light) {
					RayHit sun_hit;
					trace_ray(bsp_data, head_node, gs_pos, sun_dir,
						0.0f, max_trace_dist, sun_hit);

					if (ray_reached_sky(bsp_data, sun_hit)) {
						cells_sunlit++;
						// Cell has direct sunlight — distribute sun color to cube faces
						// weighted by alignment toward the sun
						for (int d = 0; d < 6; d++) {
							float dot = directions[d][0] * sun_dir[0]
							          + directions[d][1] * sun_dir[1]
							          + directions[d][2] * sun_dir[2];
							float weight = fmaxf(dot, 0.0f);
							if (weight <= 0.0f) continue;
							float sr = sky_color[0] * weight * 255.0f;
							float sg = sky_color[1] * weight * 255.0f;
							float sb = sky_color[2] * weight * 255.0f;
							if (has_sun_fraction) {
								sky_weight[cell_idx] += sr + sg + sb;
								all_weight[cell_idx] += sr + sg + sb;
							}
							pixel_data[d][px_ofs + 0] = (uint8_t)clamp(
								pixel_data[d][px_ofs + 0] + sr, 0.0f, 255.0f);
							pixel_data[d][px_ofs + 1] = (uint8_t)clamp(
								pixel_data[d][px_ofs + 1] + sg, 0.0f, 255.0f);
							pixel_data[d][px_ofs + 2] = (uint8_t)clamp(
								pixel_data[d][px_ofs + 2] + sb, 0.0f, 255.0f);
						}
					}
				}
			}
		}
	}

	for (size_t i = 0; i < sun_data.size(); i++)
		if (all_weight[i] > 0) sun_data[i] = (uint8_t)lroundf(sky_weight[i] / all_weight[i] * 255.0f);

	// Flood-fill invalid (solid) cells from nearest valid neighbors.
	// This prevents trilinear filtering from interpolating toward black at boundaries.
	{
		static const int offsets[6][3] = {
			{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}
		};
		int cells_filled = 0;
		// Iterate multiple passes until no more cells are filled
		for (int pass = 0; pass < 4; pass++) {
			int filled_this_pass = 0;
			for (int gz = 0; gz < dims[2]; gz++) {
				for (int gy = 0; gy < dims[1]; gy++) {
					for (int gx = 0; gx < dims[0]; gx++) {
						size_t idx = (size_t)gz * dims[0] * dims[1] + (size_t)gy * dims[0] + gx;
						if (cell_valid[idx]) continue;

						// Average all valid neighbors
						int count = 0;
						float accum[6][3] = {};
						float sun_accum = 0;
						for (int n = 0; n < 6; n++) {
							int nx = gx + offsets[n][0];
							int ny = gy + offsets[n][1];
							int nz = gz + offsets[n][2];
							if (nx < 0 || nx >= dims[0] || ny < 0 || ny >= dims[1] ||
								nz < 0 || nz >= dims[2]) continue;
							size_t nidx = (size_t)nz * dims[0] * dims[1] + (size_t)ny * dims[0] + nx;
							if (!cell_valid[nidx]) continue;
							count++;
							if (has_sun_fraction) sun_accum += sun_data[nidx];
							size_t npx = (size_t)nz * slice_bytes + ((size_t)ny * slice_w + nx) * 3;
							for (int d = 0; d < 6; d++) {
								accum[d][0] += pixel_data[d][npx + 0];
								accum[d][1] += pixel_data[d][npx + 1];
								accum[d][2] += pixel_data[d][npx + 2];
							}
						}
						if (count == 0) continue;

						float inv = 1.0f / count;
						size_t px_ofs = (size_t)gz * slice_bytes + ((size_t)gy * slice_w + gx) * 3;
						for (int d = 0; d < 6; d++) {
							pixel_data[d][px_ofs + 0] = (uint8_t)(accum[d][0] * inv);
							pixel_data[d][px_ofs + 1] = (uint8_t)(accum[d][1] * inv);
							pixel_data[d][px_ofs + 2] = (uint8_t)(accum[d][2] * inv);
						}
						if (has_sun_fraction) sun_data[idx] = (uint8_t)(sun_accum * inv);
						cell_valid[idx] = true;
						filled_this_pass++;
					}
				}
			}
			cells_filled += filled_this_pass;
			if (filled_this_pass == 0) break;
		}
		UtilityFunctions::print("[GoldSrc] Light grid flood-fill: ", (int64_t)cells_filled, " cells filled");
	}

	// Build 6 arrays of slice Images for Texture3D construction.
	// GoldSrc → Godot direction mapping:
	// GS +X → Godot -X,  GS -X → Godot +X
	// GS +Y → Godot +Z,  GS -Y → Godot -Z
	// GS +Z → Godot +Y,  GS -Z → Godot -Y
	// Godot order [+X, -X, +Y, -Y, +Z, -Z] = GS indices [1, 0, 4, 5, 2, 3]
	static const int gs_to_godot[6] = {1, 0, 4, 5, 2, 3};

	// Each direction produces an Array of num_slices Images (sliced along GoldSrc Z = Godot Y)
	Array dir_slices; // Array of 6 Arrays, each containing num_slices Images
	dir_slices.resize(6);
	for (int godot_dir = 0; godot_dir < 6; godot_dir++) {
		int gs_dir = gs_to_godot[godot_dir];
		Array slices;
		slices.resize(num_slices);
		for (int s = 0; s < num_slices; s++) {
			PackedByteArray pba;
			pba.resize(slice_bytes);
			memcpy(pba.ptrw(), pixel_data[gs_dir].data() + (size_t)s * slice_bytes, slice_bytes);
			slices[s] = Image::create_from_data(slice_w, slice_h, false, Image::FORMAT_RGB8, pba);
		}
		dir_slices[godot_dir] = slices;
	}

	// Grid origin in Godot coords: GS(mins) → Godot(-mins[0]*sf, mins[2]*sf, mins[1]*sf)
	Vector3 grid_origin(-mins[0] * scale_factor, mins[2] * scale_factor, mins[1] * scale_factor);
	// Grid dims in Godot coords: GS(x,y,z) → Godot(x, z, y)
	Vector3i grid_dims(dims[0], dims[2], dims[1]);

	Dictionary result;
	result["grid_origin"] = grid_origin;
	result["grid_dims"] = grid_dims;
	result["cell_size"] = (double)(cell_size_gs * scale_factor);
	result["dir_slices"] = dir_slices; // 6 arrays of Images for Texture3D
	if (has_sun_fraction) {
		size_t sun_slice_bytes = (size_t)slice_w * slice_h;
		Array sun_slices;
		sun_slices.resize(num_slices);
		for (int s = 0; s < num_slices; s++) {
			PackedByteArray pba;
			pba.resize(sun_slice_bytes);
			memcpy(pba.ptrw(), sun_data.data() + (size_t)s * sun_slice_bytes, sun_slice_bytes);
			sun_slices[s] = Image::create_from_data(slice_w, slice_h, false, Image::FORMAT_L8, pba);
		}
		result["sun_slices"] = sun_slices; // the sky's share of each cell's light
	}

	uint64_t elapsed = Time::get_singleton()->get_ticks_msec() - t0;
	UtilityFunctions::print("[GoldSrc] Light grid baked: ", dims[0], "x", dims[1], "x", dims[2],
		" (", (int64_t)cells_traced, " traced, ", (int64_t)cells_solid, " solid, ",
		(int64_t)cells_sunlit, " sunlit, ", (int64_t)elapsed, "ms)");

	return result;
}

} // namespace goldsrc
