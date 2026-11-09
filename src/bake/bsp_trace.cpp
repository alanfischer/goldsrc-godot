#include "bsp_trace.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace std;

namespace goldsrc {

// A ray "reaches the sky" if it hits nothing, or if its closest hit is a
// sky-textured brush face. GoldSrc maps are sealed by sky brushes, so a ray
// aimed upward/outward hits the sky face rather than escaping into the void —
// but that point still sees open sky (and the sun). Treating a sky-face hit as
// an occluder (the old behavior) left virtually no cell sunlit on sealed maps.
bool ray_reached_sky(
	const BSPData &bsp_data, const RayHit &hit)
{
	if (hit.face_index < 0) return true;
	if ((size_t)hit.face_index >= bsp_data.faces.size()) return false;
	return is_sky_texture(bsp_data.faces[hit.face_index].texture_name);
}

// Ray-polygon intersection: check if ray hits a convex polygon.
// Returns distance along ray, or -1 if no hit.
namespace {

float ray_polygon_intersect(
	const float origin[3], const float dir[3],
	const ParsedFace &face)
{
	// Plane: dot(normal, P) = dot(normal, v0)
	float denom = face.normal[0] * dir[0] + face.normal[1] * dir[1] + face.normal[2] * dir[2];
	if (fabsf(denom) < 1e-8f) return -1.0f; // parallel

	const auto &v0 = face.vertices[0];
	float plane_d = face.normal[0] * v0.pos[0] + face.normal[1] * v0.pos[1] + face.normal[2] * v0.pos[2];
	float origin_d = face.normal[0] * origin[0] + face.normal[1] * origin[1] + face.normal[2] * origin[2];
	float t = (plane_d - origin_d) / denom;
	if (t <= 0.001f) return -1.0f; // behind or at origin

	// Hit point on plane
	float hx = origin[0] + dir[0] * t;
	float hy = origin[1] + dir[1] * t;
	float hz = origin[2] + dir[2] * t;

	// Point-in-polygon via cross-product winding test.
	// Vertices may be wound CW or CCW relative to the stored normal (depends on
	// face.side in BSP), so check that all edges give the same sign, not a fixed sign.
	int n = (int)face.vertices.size();
	int pos_count = 0, neg_count = 0;
	for (int i = 0; i < n; i++) {
		const auto &a = face.vertices[i];
		const auto &b = face.vertices[(i + 1) % n];
		float ex = b.pos[0] - a.pos[0];
		float ey = b.pos[1] - a.pos[1];
		float ez = b.pos[2] - a.pos[2];
		float px = hx - a.pos[0];
		float py = hy - a.pos[1];
		float pz = hz - a.pos[2];
		float cx = ey * pz - ez * py;
		float cy = ez * px - ex * pz;
		float cz = ex * py - ey * px;
		float d = cx * face.normal[0] + cy * face.normal[1] + cz * face.normal[2];
		if (d > 0.1f) pos_count++;
		else if (d < -0.1f) neg_count++;
		if (pos_count > 0 && neg_count > 0) return -1.0f; // outside
	}

	return t;
}

} // anonymous namespace

// Sample lightmap color at a GoldSrc world position on a given face.
// Returns RGB [0..255] range.
void sample_lightmap_at(
	const ParsedFace &face,
	const float hit_pos[3],
	const vector<uint8_t> &lighting,
	float &out_r, float &out_g, float &out_b,
	int *out_luxel)
{
	out_r = out_g = out_b = 0;
	if (out_luxel) *out_luxel = -1;

	if (face.lightmap_offset < 0) return;
	if (face.lightmap_width <= 0 || face.lightmap_height <= 0) return;

	// Compute texture S/T at hit position
	float s = hit_pos[0] * face.s_axis[0] + hit_pos[1] * face.s_axis[1] + hit_pos[2] * face.s_axis[2] + face.s_offset;
	float t = hit_pos[0] * face.t_axis[0] + hit_pos[1] * face.t_axis[1] + hit_pos[2] * face.t_axis[2] + face.t_offset;

	// Convert to lightmap pixel coords
	float lm_u = s / 16.0f - face.lm_mins_s;
	float lm_v = t / 16.0f - face.lm_mins_t;

	int px = clamp((int)roundf(lm_u), 0, face.lightmap_width - 1);
	int py = clamp((int)roundf(lm_v), 0, face.lightmap_height - 1);

	// Sample style 0 only (static lighting)
	size_t pixel_count = (size_t)face.lightmap_width * face.lightmap_height;
	size_t ofs = (size_t)face.lightmap_offset + (size_t)(py * face.lightmap_width + px) * 3;
	if (ofs + 2 >= lighting.size()) return;
	if (out_luxel) *out_luxel = py * face.lightmap_width + px;

	out_r = (float)lighting[ofs + 0];
	out_g = (float)lighting[ofs + 1];
	out_b = (float)lighting[ofs + 2];
}

// BSP-accelerated ray trace: walk the BSP node tree with a ray,
// testing faces at each visited node.
void trace_ray(
	const BSPData &bsp_data,
	int node_index,
	const float origin[3], const float dir[3],
	float t_min, float t_max,
	RayHit &best_hit,
	int model_index)
{
	if (t_min >= t_max) return;
	if (node_index < 0) return; // leaf — hull 0 stores faces on nodes, not leaves

	if ((size_t)node_index >= bsp_data.nodes.size()) return;
	const auto &node = bsp_data.nodes[node_index];
	if (node.planenum < 0 || (size_t)node.planenum >= bsp_data.planes.size()) return;
	const auto &plane = bsp_data.planes[node.planenum];

	// Test this node's faces
	for (uint16_t i = 0; i < node.numfaces; i++) {
		int raw_idx = (int)node.firstface + i;
		if (raw_idx >= (int)bsp_data.raw_to_parsed.size()) continue;
		int parsed = bsp_data.raw_to_parsed[raw_idx];
		if (parsed < 0) continue;

		const auto &face = bsp_data.faces[parsed];
		if (face.model_index != model_index) continue; // the tree's own model only
		if (face.vertices.size() < 3) continue;

		float t = ray_polygon_intersect(origin, dir, face);
		if (t > 0 && t < best_hit.t && t >= t_min && t <= t_max) {
			best_hit.face_index = parsed;
			best_hit.t = t;
			best_hit.hit_pos[0] = origin[0] + dir[0] * t;
			best_hit.hit_pos[1] = origin[1] + dir[1] * t;
			best_hit.hit_pos[2] = origin[2] + dir[2] * t;
		}
	}

	// Classify ray endpoints against the plane
	float d_start = plane.normal[0] * (origin[0] + dir[0] * t_min) +
	                plane.normal[1] * (origin[1] + dir[1] * t_min) +
	                plane.normal[2] * (origin[2] + dir[2] * t_min) - plane.dist;
	float d_end   = plane.normal[0] * (origin[0] + dir[0] * t_max) +
	                plane.normal[1] * (origin[1] + dir[1] * t_max) +
	                plane.normal[2] * (origin[2] + dir[2] * t_max) - plane.dist;

	if (d_start >= 0 && d_end >= 0) {
		trace_ray(bsp_data, node.children[0], origin, dir, t_min, t_max, best_hit, model_index);
	} else if (d_start < 0 && d_end < 0) {
		trace_ray(bsp_data, node.children[1], origin, dir, t_min, t_max, best_hit, model_index);
	} else {
		// Ray crosses the plane — compute split point
		float denom = plane.normal[0] * dir[0] + plane.normal[1] * dir[1] + plane.normal[2] * dir[2];
		float t_split;
		if (fabsf(denom) < 1e-8f) {
			t_split = (t_min + t_max) * 0.5f;
		} else {
			float origin_d = plane.normal[0] * origin[0] + plane.normal[1] * origin[1] + plane.normal[2] * origin[2];
			t_split = (plane.dist - origin_d) / denom;
			t_split = clamp(t_split, t_min, t_max);
		}

		int near_child = d_start >= 0 ? 0 : 1;
		int far_child = 1 - near_child;
		trace_ray(bsp_data, node.children[near_child], origin, dir, t_min, t_split, best_hit, model_index);
		trace_ray(bsp_data, node.children[far_child], origin, dir, t_split, t_max, best_hit, model_index);
	}
}

// The map's first light_environment, which is the one hlrad lit it with. The angle/pitch
// name the direction light TRAVELS; dir is negated so a trace along it looks for the sun.
SunLight find_sun(const BSPData &bsp_data) {
	SunLight sun;
	for (const auto &ent : bsp_data.entities) {
		auto cn = ent.properties.find("classname");
		if (cn == ent.properties.end() || cn->second != "light_environment") continue;

		auto lp = ent.properties.find("_light");
		if (lp != ent.properties.end()) {
			float lr = 0, lg = 0, lb = 0, lbright = 200.0f;
			if (sscanf(lp->second.c_str(), "%f %f %f %f", &lr, &lg, &lb, &lbright) >= 3) {
				float peak = fmaxf(lr, fmaxf(lg, lb));
				if (peak > 0) {
					sun.color[0] = (lr / peak) * (lbright / 200.0f) * peak / 255.0f;
					sun.color[1] = (lg / peak) * (lbright / 200.0f) * peak / 255.0f;
					sun.color[2] = (lb / peak) * (lbright / 200.0f) * peak / 255.0f;
				}
			}
		}

		float yaw_deg = 0, pitch_deg = 0;
		float a0 = 0, a1 = 0;
		auto ap = ent.properties.find("angles");
		if (ap != ent.properties.end()) {
			if (sscanf(ap->second.c_str(), "%f %f", &a0, &a1) >= 2)
				yaw_deg = a1;
		} else {
			auto angp = ent.properties.find("angle");
			if (angp != ent.properties.end())
				yaw_deg = atof(angp->second.c_str());
		}
		// As hlrad: a missing or zero pitch falls back to angles[0].
		auto pp = ent.properties.find("pitch");
		if (pp != ent.properties.end())
			pitch_deg = atof(pp->second.c_str());
		if (pitch_deg == 0)
			pitch_deg = a0;

		float pitch_rad = pitch_deg * 3.14159265f / 180.0f;
		float yaw_rad = yaw_deg * 3.14159265f / 180.0f;
		sun.dir[0] = -(cosf(pitch_rad) * cosf(yaw_rad));
		sun.dir[1] = -(cosf(pitch_rad) * sinf(yaw_rad));
		sun.dir[2] = -sinf(pitch_rad);

		auto sp = ent.properties.find("style");
		if (sp != ent.properties.end())
			sun.style = atoi(sp->second.c_str());
		sun.present = true;
		break;
	}
	return sun;
}

} // namespace goldsrc
