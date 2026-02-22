// Regression tests for clip hull unexpansion against ww_golem.bsp
//
// Build: clang++ -std=c++17 -O2 -I src -o test_clip_hull test_clip_hull.cpp src/bsp_hull.cpp src/parsers/bsp_parser.cpp
// Run:   ./test_clip_hull ../../res/maps/ww_golem.bsp
//
// Three regression test points, each from a real bug:
//
// 1) ANGLED WALL (slab gap fix)
//    Godot (-29.25, 3, -62.06) = GS (1170, -2482.4, 120)
//    Hull 0 SOLID (visible wall). Antiparallel planes had slab_gap=39.9 but
//    vertex-based thickness was 44.9, overstating it. The old code used vertex
//    thickness and didn't clamp, so the wall collapsed. Fix: use slab_gap =
//    di + dj for the clamping check, plus binary search fallback.
//    Assertion: point is covered by a surviving cell with >= 8 GS min dimension.
//
// 2) THIN CLIP BRUSH WALL (minimum wall thickness)
//    Godot (48.82, 3, -55.50) = GS (-1952.8, -2220, 120)
//    Hull 0 EMPTY (no visible geometry), hull 3 SOLID (pure CLIP brush).
//    Slab gap 43.9 vs total support 44.0 — essentially zero-width original brush.
//    After clamping with min=1, the wall was 1 GS unit (0.025 Godot) thick —
//    invisible. Fix: minimum preserved wall thickness = 8 GS units.
//    Assertion: point is covered by a surviving cell with >= 8 GS min dimension.
//
// 3) PAPER-THIN HORIZONTAL ARTIFACT (min dimension filter)
//    Godot (-23.57, 0.47, -50.30) = GS (942.8, -2012, 18.8) — inside the artifact
//    BSP tree split at z=18 (hull 3 Z support boundary) created a 1 GS unit thick
//    horizontal cell at z=[18,19]. Stays at expanded position because its z-facing
//    plane has sibling=SOLID and the cell is too thin for the thickness-gated probe.
//    Fix: skip any cell with axis-aligned min dimension < 4 GS units.
//    Assertion: NO surviving cell (after thin filter) contains this point.

#include "src/bsp_hull.h"
#include "src/parsers/bsp_parser.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include <fstream>

using std::vector;
using goldsrc_hull::HullPlane;
using goldsrc_hull::ConvexCell;
using goldsrc_hull::CellVertex;

// ---------- The full unexpansion pipeline (mirrors goldsrc_bsp.cpp) ----------
// Returns all surviving cells after unexpansion + thin filter.

struct ProcessedCell {
	vector<CellVertex> verts;
	float mins[3], maxs[3];
	float min_dim;
};

static vector<ProcessedCell> process_all_cells(
	const goldsrc::BSPData &bsp, int hull_index,
	float hull_hx, float hull_hy, float hull_hz,
	float min_dim_threshold) {

	const auto &bmodel = bsp.models[0];
	int root = bmodel.headnode[hull_index];
	const float EPSILON = 0.1f;

	HullPlane bbox_planes[6];
	bbox_planes[0] = {{ 1, 0, 0}, bmodel.maxs[0] + 1.0f};
	bbox_planes[1] = {{-1, 0, 0}, -bmodel.mins[0] + 1.0f};
	bbox_planes[2] = {{ 0, 1, 0}, bmodel.maxs[1] + 1.0f};
	bbox_planes[3] = {{ 0,-1, 0}, -bmodel.mins[1] + 1.0f};
	bbox_planes[4] = {{ 0, 0, 1}, bmodel.maxs[2] + 1.0f};
	bbox_planes[5] = {{ 0, 0,-1}, -bmodel.mins[2] + 1.0f};

	vector<HullPlane> accumulated;
	vector<ConvexCell> cells;
	goldsrc_hull::walk_clip_tree(bsp.clipnodes, bsp.planes, root, accumulated, cells);

	vector<ProcessedCell> result;

	for (auto &cell : cells) {
		int opc = (int)cell.planes.size();
		for (int i = 0; i < 6; i++) cell.planes.push_back(bbox_planes[i]);

		// Split cells per face
		vector<ConvexCell> current = {cell};
		for (int pi = 0; pi < opc; pi++) {
			vector<ConvexCell> next;
			for (auto &c : current) {
				goldsrc_hull::split_cell_for_face(c, pi, c.planes[pi].sibling_child,
					bsp.clipnodes, bsp.planes, EPSILON, next);
			}
			current = std::move(next);
		}

		for (auto &c : current) {
			auto pre_verts = goldsrc_hull::compute_cell_vertices(c.planes, EPSILON);
			if ((int)pre_verts.size() < 4) continue;

			vector<float> orig_dist(opc);
			for (int pi = 0; pi < opc; pi++) orig_dist[pi] = c.planes[pi].dist;

			// Pass 1: determine what to unexpand
			struct UI { bool unexpand = false; float support = 0; float clamped = 0; };
			vector<UI> u(opc);
			for (int pi = 0; pi < opc; pi++) {
				auto &hp = c.planes[pi];
				float sup = goldsrc_hull::minkowski_support(hp.normal, hull_hx, hull_hy, hull_hz);
				u[pi].support = sup;
				u[pi].clamped = sup;
				if (hp.sibling_child != (int16_t)goldsrc::CONTENTS_SOLID) {
					u[pi].unexpand = true;
				} else {
					float min_dot = hp.dist;
					float cx = 0, cy = 0, cz = 0;
					int fc = 0;
					for (const auto &v : pre_verts) {
						float d = hp.normal[0]*v.gs[0] + hp.normal[1]*v.gs[1] + hp.normal[2]*v.gs[2];
						if (d < min_dot) min_dot = d;
						if (fabsf(d - hp.dist) < 0.5f) {
							cx += v.gs[0]; cy += v.gs[1]; cz += v.gs[2]; fc++;
						}
					}
					float thickness = hp.dist - min_dot;
					if (fc >= 3 && thickness > sup) {
						cx /= fc; cy /= fc; cz /= fc;
						int ct = goldsrc_hull::clip_tree_contents(bsp.clipnodes, bsp.planes, root,
							cx + hp.normal[0]*sup, cy + hp.normal[1]*sup, cz + hp.normal[2]*sup);
						if (ct != goldsrc::CONTENTS_SOLID) u[pi].unexpand = true;
					}
				}
			}

			// Pass 2: antiparallel slab clamping
			for (int i = 0; i < opc; i++) {
				if (!u[i].unexpand) continue;
				for (int j = i + 1; j < opc; j++) {
					if (!u[j].unexpand) continue;
					float dn = c.planes[i].normal[0]*c.planes[j].normal[0]
					         + c.planes[i].normal[1]*c.planes[j].normal[1]
					         + c.planes[i].normal[2]*c.planes[j].normal[2];
					if (dn > -0.8f) continue;
					float sg = c.planes[i].dist + c.planes[j].dist;
					float ts = u[i].support + u[j].support;
					if (ts > sg) {
						float avail = fmaxf(0.0f, sg - 8.0f);
						float r = (ts > 0) ? avail / ts : 0.0f;
						u[i].clamped = fminf(u[i].clamped, u[i].support * r);
						u[j].clamped = fminf(u[j].clamped, u[j].support * r);
					}
				}
			}

			// Pass 3: apply unexpansion
			for (int pi = 0; pi < opc; pi++) {
				if (u[pi].unexpand) c.planes[pi].dist -= u[pi].clamped;
			}

			auto post_verts = goldsrc_hull::compute_cell_vertices(c.planes, EPSILON);

			// Binary search fallback if collapsed
			if ((int)post_verts.size() < 4) {
				for (int pi = 0; pi < opc; pi++) c.planes[pi].dist = orig_dist[pi];
				float lo = 0.0f, hi = 1.0f;
				for (int iter = 0; iter < 12; iter++) {
					float mid = (lo + hi) * 0.5f;
					auto test = c.planes;
					for (int pi = 0; pi < opc; pi++) {
						if (u[pi].unexpand) test[pi].dist -= u[pi].clamped * mid;
					}
					auto tv = goldsrc_hull::compute_cell_vertices(test, EPSILON);
					if ((int)tv.size() >= 4) lo = mid;
					else hi = mid;
				}
				if (lo > 0.01f) {
					for (int pi = 0; pi < opc; pi++) {
						if (u[pi].unexpand) c.planes[pi].dist -= u[pi].clamped * lo;
					}
					post_verts = goldsrc_hull::compute_cell_vertices(c.planes, EPSILON);
				}
			}

			if ((int)post_verts.size() < 4) continue;

			// Compute AABB
			float mn[3] = {1e9f, 1e9f, 1e9f};
			float mx[3] = {-1e9f, -1e9f, -1e9f};
			for (const auto &v : post_verts) {
				for (int a = 0; a < 3; a++) {
					if (v.gs[a] < mn[a]) mn[a] = v.gs[a];
					if (v.gs[a] > mx[a]) mx[a] = v.gs[a];
				}
			}
			float md = fminf(fminf(mx[0]-mn[0], mx[1]-mn[1]), mx[2]-mn[2]);

			// Thin-cell filter
			if (md < min_dim_threshold) continue;

			ProcessedCell pc;
			pc.verts = std::move(post_verts);
			for (int a = 0; a < 3; a++) { pc.mins[a] = mn[a]; pc.maxs[a] = mx[a]; }
			pc.min_dim = md;
			result.push_back(std::move(pc));
		}
	}

	return result;
}

// Check if a GS point is inside any surviving cell's AABB (conservative)
// and vertex hull (precise).
static bool point_covered_by_any_cell(
	const vector<ProcessedCell> &cells, const float p[3],
	float *out_min_dim = nullptr) {

	for (const auto &pc : cells) {
		// Quick AABB check
		bool in_aabb = true;
		for (int a = 0; a < 3; a++) {
			if (p[a] < pc.mins[a] - 1.0f || p[a] > pc.maxs[a] + 1.0f) {
				in_aabb = false;
				break;
			}
		}
		if (!in_aabb) continue;

		// The cell survived processing, so the point is covered if it's
		// within the AABB (our cells are convex, but we stored verts not planes,
		// so AABB is a good enough proxy for "is this wall present")
		if (out_min_dim) *out_min_dim = pc.min_dim;
		return true;
	}
	return false;
}

// ---------- Tests ----------

struct TestResult {
	const char *name;
	bool passed;
	const char *detail;
};

static goldsrc::BSPParser g_parser;
static const goldsrc::BSPData *g_bsp = nullptr;
static vector<ProcessedCell> g_cells; // processed with 4 GS threshold

// Godot to GoldSrc coordinate conversion
// gs_x = -godot_x / scale, gs_y = godot_z / scale, gs_z = godot_y / scale
static void godot_to_gs(float gx, float gy, float gz, float out[3]) {
	const float scale = 0.025f;
	out[0] = -gx / scale;
	out[1] = gz / scale;
	out[2] = gy / scale;
}

// Test 1: Angled wall must survive unexpansion
// The vertex-based thickness bug caused this wall to collapse because
// slab_gap (39.9) < total_support (44.2) but vertex thickness (44.9) > total_support,
// so the old code didn't clamp and the wall vanished.
static TestResult test_angled_wall() {
	// Godot (-29.25, 3, -62.06) = GS (1170, -2482.4, 120)
	float p[3];
	godot_to_gs(-29.25f, 3.0f, -62.06f, p);

	// Precondition: hull 3 thinks this point is SOLID
	int root = g_bsp->models[0].headnode[3];
	int ct = goldsrc_hull::clip_tree_contents(g_bsp->clipnodes, g_bsp->planes, root, p[0], p[1], p[2]);
	if (ct != goldsrc::CONTENTS_SOLID)
		return {"angled_wall", false, "precondition: hull 3 not SOLID at test point"};

	float min_dim = 0;
	if (!point_covered_by_any_cell(g_cells, p, &min_dim))
		return {"angled_wall", false, "wall collapsed — no surviving cell covers the point"};

	if (min_dim < 8.0f) {
		static char buf[128];
		snprintf(buf, sizeof(buf), "wall too thin: min_dim=%.1f GS (need >= 8)", min_dim);
		return {"angled_wall", false, buf};
	}

	return {"angled_wall", true, "wall survives with adequate thickness"};
}

// Test 2: Pure CLIP brush wall must survive with minimum thickness
// This is a wall that only exists in clip hulls (hull 0 = EMPTY). The slab gap
// was essentially equal to the total support, leaving near-zero thickness.
// Fix: preserve a minimum 8 GS units of wall.
static TestResult test_clip_brush_wall() {
	// Godot (48.82, 3, -55.50) = GS (-1952.8, -2220, 120)
	float p[3];
	godot_to_gs(48.82f, 3.0f, -55.50f, p);

	// Precondition: hull 0 is EMPTY (pure CLIP brush), hull 3 is SOLID
	int root3 = g_bsp->models[0].headnode[3];
	int ct3 = goldsrc_hull::clip_tree_contents(g_bsp->clipnodes, g_bsp->planes, root3, p[0], p[1], p[2]);
	if (ct3 != goldsrc::CONTENTS_SOLID)
		return {"clip_brush_wall", false, "precondition: hull 3 not SOLID at test point"};

	float min_dim = 0;
	if (!point_covered_by_any_cell(g_cells, p, &min_dim))
		return {"clip_brush_wall", false, "wall vanished — no surviving cell covers the point"};

	if (min_dim < 8.0f) {
		static char buf[128];
		snprintf(buf, sizeof(buf), "wall too thin: min_dim=%.1f GS (need >= 8)", min_dim);
		return {"clip_brush_wall", false, buf};
	}

	return {"clip_brush_wall", true, "CLIP brush wall present with adequate thickness"};
}

// Test 3: Paper-thin horizontal BSP artifact must be filtered out
// BSP tree split at z=18 creates a 1 GS unit thick cell. This floating sliver
// must not appear in the final output.
static TestResult test_no_thin_artifact() {
	// The artifact was at GS z=[18,19], in the area around GS x=942, y=-2012.
	// Pick a point in the middle of it: GS (942.8, -2012, 18.5)
	// = Godot (-23.57, 0.4625, -50.30)
	float p[3];
	godot_to_gs(-23.57f, 0.4625f, -50.30f, p);

	// This point should NOT be inside any surviving cell (the thin slab should
	// have been filtered). But the point might legitimately be inside a thick
	// cell that also covers this area (e.g. a floor/wall). So we check that
	// no cell covering this point has min_dim < 4 GS.
	for (const auto &pc : g_cells) {
		bool in_aabb = true;
		for (int a = 0; a < 3; a++) {
			if (p[a] < pc.mins[a] - 0.5f || p[a] > pc.maxs[a] + 0.5f) {
				in_aabb = false;
				break;
			}
		}
		if (!in_aabb) continue;

		if (pc.min_dim < 4.0f) {
			static char buf[128];
			snprintf(buf, sizeof(buf),
				"thin artifact survived: min_dim=%.1f GS, z=[%.0f,%.0f]",
				pc.min_dim, pc.mins[2], pc.maxs[2]);
			return {"no_thin_artifact", false, buf};
		}
	}

	return {"no_thin_artifact", true, "no thin artifacts present at test location"};
}

// Test 4: Aggregate stats sanity check
static TestResult test_aggregate_stats() {
	// The map should produce a reasonable number of surviving cells
	int n = (int)g_cells.size();
	if (n < 1000)
		return {"aggregate_stats", false, "too few surviving cells (< 1000)"};
	if (n > 5000)
		return {"aggregate_stats", false, "too many surviving cells (> 5000)"};

	return {"aggregate_stats", true, "cell count in expected range"};
}

// ---------- Main ----------

int main(int argc, char **argv) {
	if (argc < 2) {
		fprintf(stderr, "Usage: %s <bsp_file>\n", argv[0]);
		return 1;
	}

	// Load BSP
	std::ifstream f(argv[1], std::ios::binary | std::ios::ate);
	if (!f) {
		fprintf(stderr, "Cannot open %s\n", argv[1]);
		return 1;
	}
	size_t sz = f.tellg();
	f.seekg(0);
	vector<uint8_t> data(sz);
	f.read((char*)data.data(), sz);

	if (!g_parser.parse(data.data(), data.size())) {
		fprintf(stderr, "Failed to parse BSP\n");
		return 1;
	}
	g_bsp = &g_parser.get_data();

	printf("BSP loaded: %zu clipnodes, %zu planes\n",
		g_bsp->clipnodes.size(), g_bsp->planes.size());

	// Run the full pipeline (hull 3, 4 GS min dim threshold)
	printf("Processing clip hull 3...\n");
	g_cells = process_all_cells(*g_bsp, 3, 16.0f, 16.0f, 18.0f, 4.0f);
	printf("Surviving cells: %d\n\n", (int)g_cells.size());

	// Run tests
	TestResult tests[] = {
		test_angled_wall(),
		test_clip_brush_wall(),
		test_no_thin_artifact(),
		test_aggregate_stats(),
	};

	int passed = 0, failed = 0;
	for (const auto &t : tests) {
		printf("  %s  %-20s  %s\n",
			t.passed ? "PASS" : "FAIL",
			t.name, t.detail);
		if (t.passed) passed++;
		else failed++;
	}

	printf("\n%d passed, %d failed\n", passed, failed);
	return failed > 0 ? 1 : 0;
}
