#include "sun_fraction.h"
#include "bsp_trace.h"

#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/classes/worker_thread_pool.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace godot;
using namespace std;

namespace goldsrc {

namespace {

// hlrad writes lightmaps as pow(light, gamma); shares are only additive before that.
const float LIGHTMAP_GAMMA = 0.55f;
const int SKY_RAYS = 12;
const float SUN_TRACE_DIST = 16384.0f;
// Beyond this a point light's share is noise next to whatever else lights the luxel.
const float POINT_LIGHT_RANGE = 1024.0f;

struct PointLight {
	float pos[3];
	float lum;
	int leaf;
};

struct LuxelTerms {
	float lit;    // style-0 light, linear
	float other;  // every other style's light, linear (switchable lights, a styled sun)
	float sun, sky, point;
	bool valid;
};

struct SunFractionJob {
	const BSPData *bsp;
	const SunLight *sun;
	const vector<PointLight> *lights;
	const vector<vector<bool>> *light_pvs;
	const vector<array<float, 3>> *model_origins;
	int head_node;
	vector<vector<LuxelTerms>> *terms;
};

static inline float lin(uint8_t v) {
	return powf(v / 255.0f, 1.0f / LIGHTMAP_GAMMA);
}

static inline float luxel_lum(const vector<uint8_t> &lighting, size_t ofs) {
	return (lin(lighting[ofs]) + lin(lighting[ofs + 1]) + lin(lighting[ofs + 2])) / 3.0f;
}

static int leaf_at(const BSPData &bsp, int node, const float p[3]) {
	while (node >= 0) {
		if ((size_t)node >= bsp.nodes.size()) return -1;
		const auto &n = bsp.nodes[node];
		if (n.planenum < 0 || (size_t)n.planenum >= bsp.planes.size()) return -1;
		const auto &pl = bsp.planes[n.planenum];
		float d = pl.normal[0] * p[0] + pl.normal[1] * p[1] + pl.normal[2] * p[2];
		node = d >= pl.dist ? n.children[0] : n.children[1];
	}
	int leaf = -(node + 1);
	return (leaf >= 0 && (size_t)leaf < bsp.leafs.size()) ? leaf : -1;
}

static inline bool point_in_face(const ParsedFace &face, const float p[3]) {
	int n = (int)face.vertices.size();
	int pos = 0, neg = 0;
	for (int i = 0; i < n; i++) {
		const float *a = face.vertices[i].pos;
		const float *b = face.vertices[(i + 1) % n].pos;
		float e[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
		float q[3] = {p[0] - a[0], p[1] - a[1], p[2] - a[2]};
		float c[3] = {e[1] * q[2] - e[2] * q[1], e[2] * q[0] - e[0] * q[2], e[0] * q[1] - e[1] * q[0]};
		float d = c[0] * face.normal[0] + c[1] * face.normal[1] + c[2] * face.normal[2];
		if (d > 0) pos++;
		else if (d < 0) neg++;
		if (pos && neg) return false;
	}
	return true;
}

// Solve S·p + s_off = s, T·p + t_off = t, N·p = d for p (Cramer's rule).
static bool luxel_to_world(const ParsedFace &face, float s, float t, float out[3]) {
	const float *S = face.s_axis, *T = face.t_axis, *N = face.normal;
	const float *v0 = face.vertices[0].pos;
	float rhs[3] = {s - face.s_offset, t - face.t_offset,
		N[0] * v0[0] + N[1] * v0[1] + N[2] * v0[2]};
	auto det3 = [](const float a[3], const float b[3], const float c[3]) {
		return a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
			+ a[2] * (b[0] * c[1] - b[1] * c[0]);
	};
	float det = det3(S, T, N);
	if (fabsf(det) < 1e-8f) return false;
	for (int k = 0; k < 3; k++) {
		float r0[3] = {S[0], S[1], S[2]}, r1[3] = {T[0], T[1], T[2]}, r2[3] = {N[0], N[1], N[2]};
		r0[k] = rhs[0]; r1[k] = rhs[1]; r2[k] = rhs[2];
		out[k] = det3(r0, r1, r2) / det;
	}
	return true;
}

static bool reaches_sky(const BSPData &bsp, int head, const float p[3], const float dir[3]) {
	RayHit hit;
	trace_ray(bsp, head, p, dir, 0.0f, SUN_TRACE_DIST, hit);
	return ray_reached_sky(bsp, hit);
}

static void sun_fraction_face(void *userdata, uint32_t face_index) {
	const SunFractionJob &job = *(const SunFractionJob *)userdata;
	const auto &bsp = *job.bsp;
	const auto &face = bsp.faces[face_index];
	auto &out = (*job.terms)[face_index];
	if (face.lightmap_offset < 0 || face.vertices.size() < 3) return;
	if (face.model_index < 0 || (size_t)face.model_index >= job.model_origins->size()) return;
	const auto &model_origin = (*job.model_origins)[face.model_index];
	if (face.lightmap_width <= 0 || face.lightmap_height <= 0) return;

	int n_styles = 0;
	int sun_layer = -1;
	for (int s = 0; s < 4 && face.styles[s] != 255; s++) {
		if (job.sun->style != 0 && face.styles[s] == job.sun->style) sun_layer = s;
		n_styles++;
	}
	size_t pixels = (size_t)face.lightmap_width * face.lightmap_height;
	if (n_styles == 0 || (size_t)face.lightmap_offset + pixels * 3 * n_styles > bsp.lighting.size()) return;
	out.resize(pixels);

	const float *N = face.normal;
	float centroid[3] = {0, 0, 0};
	for (const auto &v : face.vertices)
		for (int k = 0; k < 3; k++) centroid[k] += v.pos[k];
	for (int k = 0; k < 3; k++) centroid[k] /= face.vertices.size();

	// Tangent frame for the hemisphere.
	float tx[3], ty[3];
	{
		float up[3] = {0, 0, 1};
		if (fabsf(N[2]) > 0.9f) { up[0] = 1; up[2] = 0; }
		tx[0] = up[1] * N[2] - up[2] * N[1];
		tx[1] = up[2] * N[0] - up[0] * N[2];
		tx[2] = up[0] * N[1] - up[1] * N[0];
		float l = sqrtf(tx[0] * tx[0] + tx[1] * tx[1] + tx[2] * tx[2]);
		for (int k = 0; k < 3; k++) tx[k] /= l;
		ty[0] = N[1] * tx[2] - N[2] * tx[1];
		ty[1] = N[2] * tx[0] - N[0] * tx[2];
		ty[2] = N[0] * tx[1] - N[1] * tx[0];
	}
	float sun_ndl = N[0] * job.sun->dir[0] + N[1] * job.sun->dir[1] + N[2] * job.sun->dir[2];

	for (int y = 0; y < face.lightmap_height; y++) {
		for (int x = 0; x < face.lightmap_width; x++) {
			size_t px = (size_t)y * face.lightmap_width + x;
			LuxelTerms &lt = out[px];
			lt = LuxelTerms{};
			lt.valid = true;
			float layer_lum[4] = {0, 0, 0, 0};
			for (int s = 0; s < n_styles; s++)
				layer_lum[s] = luxel_lum(bsp.lighting, (size_t)face.lightmap_offset + (s * pixels + px) * 3);
			for (int s = 0; s < n_styles; s++) {
				if (s == 0 && face.styles[0] == 0) lt.lit = layer_lum[s];
				else lt.other += layer_lum[s];
			}
			if (sun_layer >= 0) {
				// Exact: the sun has a layer of its own.
				lt.sun = layer_lum[sun_layer];
				continue;
			}
			if (job.sun->style != 0 || job.lights->empty()) continue;

			float p[3];
			if (!luxel_to_world(face, (face.lm_mins_s + x) * 16.0f, (face.lm_mins_t + y) * 16.0f, p))
				continue;
			// Edge luxels sit past the polygon, often inside the neighbouring wall.
			for (int i = 0; i < 4 && !point_in_face(face, p); i++)
				for (int k = 0; k < 3; k++) p[k] = (p[k] + centroid[k]) * 0.5f;
			for (int k = 0; k < 3; k++) p[k] += N[k] * 2.0f + model_origin[k];
			int leaf = leaf_at(bsp, job.head_node, p);
			if (leaf < 0 || bsp.leafs[leaf].contents == CONTENTS_SOLID) {
				lt.valid = false;
				continue;
			}

			if (sun_ndl > 0 && reaches_sky(bsp, job.head_node, p, job.sun->dir))
				lt.sun = sun_ndl;

			// Cosine-weighted spiral, turned per luxel so neighbours don't band.
			uint32_t h = (uint32_t)(face_index * 73856093u) ^ (uint32_t)(px * 19349663u);
			float turn = (h & 0xffff) / 65536.0f * 6.2831853f;
			int seen = 0;
			for (int r = 0; r < SKY_RAYS; r++) {
				float rad = sqrtf((r + 0.5f) / SKY_RAYS);
				float phi = r * 2.3999632f + turn;
				float a = rad * cosf(phi), b = rad * sinf(phi), c = sqrtf(fmaxf(0.0f, 1.0f - rad * rad));
				float dir[3] = {tx[0] * a + ty[0] * b + N[0] * c,
					tx[1] * a + ty[1] * b + N[1] * c,
					tx[2] * a + ty[2] * b + N[2] * c};
				if (reaches_sky(bsp, job.head_node, p, dir)) seen++;
			}
			lt.sky = (float)seen / SKY_RAYS;

			for (size_t li = 0; li < job.lights->size(); li++) {
				const PointLight &L = (*job.lights)[li];
				const auto &pvs = (*job.light_pvs)[li];
				if (!pvs.empty() && (size_t)leaf < pvs.size() && !pvs[leaf]) continue;
				float v[3] = {L.pos[0] - p[0], L.pos[1] - p[1], L.pos[2] - p[2]};
				float d2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
				if (d2 > POINT_LIGHT_RANGE * POINT_LIGHT_RANGE) continue;
				float d = sqrtf(fmaxf(d2, 1.0f));
				float ndl = (N[0] * v[0] + N[1] * v[1] + N[2] * v[2]) / d;
				if (ndl <= 0) continue;
				float dir[3] = {v[0] / d, v[1] / d, v[2] / d};
				RayHit hit;
				trace_ray(bsp, job.head_node, p, dir, 0.0f, d - 1.0f, hit);
				if (hit.face_index >= 0) continue;
				lt.point += L.lum * ndl / fmaxf(d2, 256.0f);
			}
		}
	}
}

// Non-negative least squares over three columns, by trying every subset of them: the
// unconstrained solve of the best subset whose weights are all >= 0.
static void fit_nonnegative(const double ata[3][3], const double atb[3], double w[3]) {
	double best = -1e300;
	w[0] = w[1] = w[2] = 0;
	for (int mask = 1; mask < 8; mask++) {
		int idx[3], n = 0;
		for (int i = 0; i < 3; i++) if (mask & (1 << i)) idx[n++] = i;
		double m[3][4] = {};
		for (int r = 0; r < n; r++) {
			for (int c = 0; c < n; c++) m[r][c] = ata[idx[r]][idx[c]];
			m[r][n] = atb[idx[r]];
		}
		bool ok = true;
		for (int c = 0; c < n && ok; c++) {
			int piv = c;
			for (int r = c + 1; r < n; r++) if (fabs(m[r][c]) > fabs(m[piv][c])) piv = r;
			if (fabs(m[piv][c]) < 1e-12) { ok = false; break; }
			for (int k = 0; k <= n; k++) std::swap(m[c][k], m[piv][k]);
			for (int r = 0; r < n; r++) {
				if (r == c) continue;
				double f = m[r][c] / m[c][c];
				for (int k = 0; k <= n; k++) m[r][k] -= f * m[c][k];
			}
		}
		if (!ok) continue;
		double x[3] = {0, 0, 0};
		for (int r = 0; r < n; r++) {
			x[idx[r]] = m[r][n] / m[r][r];
			if (x[idx[r]] < 0) ok = false;
		}
		if (!ok) continue;
		// Minimising |Ax-b|² is maximising 2x·Atb - x·AtA·x.
		double gain = 0;
		for (int i = 0; i < 3; i++) {
			gain += 2 * x[i] * atb[i];
			for (int j = 0; j < 3; j++) gain -= x[i] * ata[i][j] * x[j];
		}
		if (gain > best) {
			best = gain;
			for (int i = 0; i < 3; i++) w[i] = x[i];
		}
	}
}

} // anonymous namespace

float layer0_sun_share(const BSPData &bsp, const ParsedFace &face,
		int luxel, float fraction, int sun_style) {
	if (sun_style != 0) return face.styles[0] == sun_style ? 1.0f : 0.0f;
	size_t pixels = (size_t)face.lightmap_width * face.lightmap_height;
	float layer0 = 0, total = 0;
	for (int s = 0; s < 4 && face.styles[s] != 255; s++) {
		size_t ofs = (size_t)face.lightmap_offset + (s * pixels + luxel) * 3;
		if (ofs + 2 >= bsp.lighting.size()) break;
		float l = luxel_lum(bsp.lighting, ofs);
		if (s == 0) layer0 = l;
		total += l;
	}
	return layer0 > 0 ? fminf(1.0f, fraction * total / layer0) : 0.0f;
}

SunFraction compute_sun_fractions(const BSPData &bsp) {
	SunFraction fractions;
	const SunLight sun = find_sun(bsp);
	if (!sun.present || bsp.lighting.empty() || bsp.models.empty()) return fractions;
	uint64_t t0 = Time::get_singleton()->get_ticks_msec();
	int head = bsp.models[0].headnode[0];

	// Only lights that landed in style 0: a targeted or styled light has its own layer.
	vector<PointLight> lights;
	for (const auto &ent : bsp.entities) {
		auto cn = ent.properties.find("classname");
		if (cn == ent.properties.end()) continue;
		if (cn->second != "light" && cn->second != "light_spot") continue;
		auto st = ent.properties.find("style");
		if (st != ent.properties.end() && atoi(st->second.c_str()) != 0) continue;
		if (ent.properties.count("targetname")) continue;
		PointLight L;
		auto org = ent.properties.find("origin");
		if (org == ent.properties.end() ||
			sscanf(org->second.c_str(), "%f %f %f", &L.pos[0], &L.pos[1], &L.pos[2]) != 3) continue;
		float c[4] = {255, 255, 255, 200};
		auto lp = ent.properties.find("_light");
		int n = lp == ent.properties.end() ? 0 :
			sscanf(lp->second.c_str(), "%f %f %f %f", &c[0], &c[1], &c[2], &c[3]);
		if (n == 1) c[1] = c[2] = c[0], c[3] = 255;
		else if (n == 3) c[3] = 255;
		L.lum = (c[0] + c[1] + c[2]) / 3.0f * (c[3] / 255.0f);
		L.leaf = leaf_at(bsp, head, L.pos);
		if (L.lum > 0) lights.push_back(L);
	}
	vector<vector<bool>> light_pvs(lights.size());
	if (!bsp.visibility.empty())
		for (size_t i = 0; i < lights.size(); i++)
			if (lights[i].leaf > 0) light_pvs[i] = bsp.decompress_pvs(lights[i].leaf);

	// Brush entities are lit where they stand: an ORIGIN brush stores the faces local to it.
	vector<array<float, 3>> model_origins(bsp.models.size());
	for (size_t m = 0; m < bsp.models.size(); m++)
		for (int k = 0; k < 3; k++) model_origins[m][k] = bsp.models[m].origin[k];
	for (const auto &ent : bsp.entities) {
		auto mk = ent.properties.find("model");
		auto org = ent.properties.find("origin");
		if (mk == ent.properties.end() || org == ent.properties.end() || mk->second.size() < 2 ||
			mk->second[0] != '*') continue;
		int m = atoi(mk->second.c_str() + 1);
		if (m > 0 && (size_t)m < model_origins.size())
			sscanf(org->second.c_str(), "%f %f %f", &model_origins[m][0], &model_origins[m][1],
				&model_origins[m][2]);
	}

	vector<vector<LuxelTerms>> terms(bsp.faces.size());
	SunFractionJob job{&bsp, &sun, &lights, &light_pvs, &model_origins, head, &terms};
	WorkerThreadPool *pool = WorkerThreadPool::get_singleton();
	auto group = pool->add_native_group_task(&sun_fraction_face, &job, (int)bsp.faces.size(),
		-1, true, "sun fraction");
	pool->wait_for_group_task_completion(group);

	// Weights that best rebuild the style-0 lightmap from the three terms. Fitted on the world
	// alone: brush entities only take the result.
	double w[3] = {1, 0, 0};
	bool estimated = sun.style == 0 && !lights.empty();
	if (estimated) {
		double ata[3][3] = {}, atb[3] = {};
		for (size_t fi = 0; fi < terms.size(); fi++) {
			if (bsp.faces[fi].model_index != 0) continue;
			for (const auto &lt : terms[fi]) {
				if (!lt.valid || lt.lit <= lin(2) || lt.lit >= lin(250)) continue;
				double col[3] = {lt.sun, lt.sky, lt.point};
				for (int i = 0; i < 3; i++) {
					atb[i] += col[i] * lt.lit;
					for (int j = 0; j < 3; j++) ata[i][j] += col[i] * col[j];
				}
			}
		}
		fit_nonnegative(ata, atb, w);
	}

	// The map's overall sky share, for whatever the fit can't place.
	double sky_total = 0, all_total = 0;
	for (size_t fi = 0; fi < terms.size(); fi++) {
		if (bsp.faces[fi].model_index != 0) continue;
		for (const auto &lt : terms[fi]) {
			if (!lt.valid) continue;
			double s = w[0] * lt.sun + w[1] * lt.sky, p = w[2] * lt.point;
			sky_total += s;
			all_total += s + p;
		}
	}
	double sky_share = estimated ? (all_total > 0 ? sky_total / all_total : 1.0) : 1.0;

	fractions.resize(bsp.faces.size());
	int64_t luxels = 0;
	for (size_t fi = 0; fi < terms.size(); fi++) {
		const auto &ft = terms[fi];
		if (ft.empty()) continue;
		const auto &face = bsp.faces[fi];
		int W = face.lightmap_width, H = face.lightmap_height;
		vector<float> f(ft.size(), 0.0f);
		for (size_t px = 0; px < ft.size(); px++) {
			const LuxelTerms &lt = ft[px];
			float total = lt.lit + lt.other;
			if (total <= 0) continue;
			float sky_part;
			if (sun.style != 0) {
				sky_part = lt.sun;
			} else if (!estimated) {
				sky_part = lt.lit;
			} else {
				float s = (float)(w[0] * lt.sun + w[1] * lt.sky);
				float p = (float)(w[2] * lt.point);
				float excess = lt.valid ? fmaxf(0.0f, lt.lit - s - p) : lt.lit;
				float predicted = s + p;
				sky_part = (float)(excess * sky_share);
				if (predicted > 0) sky_part += (lt.lit - excess) * s / predicted;
			}
			f[px] = fminf(1.0f, sky_part / total);
		}
		// 3x3 blur inside the face: the hemisphere is a dozen rays.
		auto &dst = fractions[fi];
		dst.resize(ft.size());
		for (int y = 0; y < H; y++) {
			for (int x = 0; x < W; x++) {
				float sum = 0;
				int n = 0;
				for (int dy = -1; dy <= 1; dy++) {
					for (int dx = -1; dx <= 1; dx++) {
						int xx = x + dx, yy = y + dy;
						if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
						sum += f[(size_t)yy * W + xx];
						n++;
					}
				}
				dst[(size_t)y * W + x] = (uint8_t)lroundf(sum / n * 255.0f);
			}
		}
		luxels += (int64_t)ft.size();
	}

	UtilityFunctions::print("[GoldSrc] Sun fraction: ", luxels, " luxels, ",
		(int64_t)lights.size(), " lights, ",
		sun.style != 0 ? String("sun on style ") + String::num_int64(sun.style) :
			!estimated ? String("sky-lit only") :
			String("fit sun ") + String::num(w[0], 4) + " sky " + String::num(w[1], 4) +
			" point " + String::num(w[2], 6) + ", sky share " + String::num(sky_share, 3),
		", ", (int64_t)(Time::get_singleton()->get_ticks_msec() - t0), "ms");
	return fractions;
}

} // namespace goldsrc
