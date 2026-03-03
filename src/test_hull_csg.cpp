// Standalone test: hull 1 clip brush extraction via un-expand + hull 0 clip
// Build: clang++ -std=c++17 -O2 -I. test_hull_csg.cpp parsers/bsp_parser.cpp bsp_hull.cpp -o test_hull_csg
// Run:   ./test_hull_csg (auto-discovers maps relative to binary dir)

#include "parsers/bsp_parser.h"
#include "bsp_hull.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <chrono>
#include <string>
#include <functional>

using namespace std;
using namespace goldsrc_hull;

// --- Test point definition ---
struct TestPoint {
	float gs[3];
	const char *label;
	bool should_be_inside;
};

// --- Per-map test data ---
struct MapTestData {
	const char *bsp_relative_path;
	const char *map_name;
	const TestPoint *points;
	size_t num_points;
};

// ww_golem test points
static const TestPoint ww_golem_points[] = {
	// Clip brush test point - AABB must overlap a cell
	{{1198.0f, -2468.0f, 20.0f}, "clip_brush", true},
	// Player-reported artifact cells that should not exist
	{{1046.5f, 109.9f, 191.2f}, "artifact_1", false},
	{{-1790.0f, 490.0f, 290.7f}, "artifact_2", false},
	{{-1828.0f, 490.0f, 58.6f}, "artifact_3", false},
	{{-1289.0f, 336.7f, 25.8f}, "artifact_4", false},
	// Player-reported missing clip hulls
	{{-890.7f, 724.0f, 766.6f}, "missing_1", true},
	{{-488.3f, 742.0f, 811.6f}, "missing_2", true},
	{{1690.0f, -1628.4f, 158.7f}, "missing_3", true},
	{{449.2f, 464.5f, 845.8f}, "missing_4", true},
	{{1769.4f, -1505.4f, 335.0f}, "missing_5", true},
	{{-1708.2f, -2407.1f, 291.5f}, "missing_6", true},
	{{1129.9f, -2504.3f, 246.1f}, "missing_7", true},
	{{-1821.0f, -2339.2f, 127.1f}, "missing_8", true},
	// Player-reported new artifacts (from hull 1 clip)
	{{479.9f, 464.0f, 384.4f}, "artifact_5", false},
	{{1803.5f, 464.0f, 335.7f}, "artifact_6", false},
	// New missing points
	{{1541.1f, 486.1f, 844.4f}, "missing_9", true},
	{{1639.3f, -984.8f, 844.1f}, "missing_10", true},
	{{1829.8f, -3616.8f, 140.2f}, "missing_11", true},
	// New artifact
	{{-1872.0f, 455.1f, 478.8f}, "artifact_7", false},
	// New missing points (batch 3)
	{{1621.4f, -1808.9f, 215.9f}, "missing_12", true},
	{{1198.4f, -2474.9f, 174.8f}, "missing_13", true},
	{{-1741.2f, -2386.1f, 243.1f}, "missing_14", true},
	// New missing point (batch 4)
	{{1741.3f, -1547.5f, 73.4f}, "missing_15", true},
	// New missing points (batch 5)
	{{2336.4f, -1760.2f, 563.5f}, "missing_16", true},
	{{1600.8f, -1842.9f, 458.9f}, "missing_17", true},
	// New missing point (batch 6)
	{{1142.2f, -2485.9f, 160.8f}, "missing_18", true},
	// All info_player_start / info_player_teamspawn entities
	{{529.0f, -2490.0f, 152.0f}, "info_player_start", false},
	{{-1872.0f, 624.0f, 736.0f}, "teamspawn_1", false},
	{{-528.0f, 624.0f, 736.0f}, "teamspawn_2", false},
	{{464.0f, 624.0f, 736.0f}, "teamspawn_3", false},
	{{1808.0f, 624.0f, 736.0f}, "teamspawn_4", false},
	{{1040.0f, -1920.0f, 48.0f}, "teamspawn_5", false},
	{{1192.0f, -1920.0f, 48.0f}, "teamspawn_6", false},
	{{1112.0f, -2008.0f, 52.0f}, "teamspawn_7", false},
	{{-160.0f, 736.0f, 64.0f}, "teamspawn_8", false},
	{{-224.0f, 736.0f, 64.0f}, "teamspawn_9", false},
	{{-288.0f, 736.0f, 64.0f}, "teamspawn_10", false},
	{{-160.0f, 656.0f, 64.0f}, "teamspawn_11", false},
	{{-224.0f, 656.0f, 64.0f}, "teamspawn_12", false},
	{{-288.0f, 656.0f, 64.0f}, "teamspawn_13", false},
	{{-288.0f, 576.0f, 64.0f}, "teamspawn_14", false},
	{{-160.0f, 576.0f, 64.0f}, "teamspawn_15", false},
	{{-224.0f, 576.0f, 64.0f}, "teamspawn_16", false},
	{{96.0f, 736.0f, 64.0f}, "teamspawn_17", false},
	{{160.0f, 736.0f, 64.0f}, "teamspawn_18", false},
	{{224.0f, 736.0f, 64.0f}, "teamspawn_19", false},
	{{160.0f, 656.0f, 64.0f}, "teamspawn_20", false},
	{{224.0f, 656.0f, 64.0f}, "teamspawn_21", false},
	{{96.0f, 656.0f, 64.0f}, "teamspawn_22", false},
	{{224.0f, 576.0f, 64.0f}, "teamspawn_23", false},
	{{160.0f, 576.0f, 64.0f}, "teamspawn_24", false},
	{{96.0f, 576.0f, 64.0f}, "teamspawn_25", false},
	{{-1352.0f, -1928.0f, 48.0f}, "teamspawn_26", false},
	{{-1456.0f, -1928.0f, 52.0f}, "teamspawn_27", false},
	{{-1400.0f, -1928.0f, 216.0f}, "teamspawn_28", false},
	{{949.9f, -16.1f, 845.0f}, "missing_12", true},
	{{-1670.6f, -2381.4f, 248.8f}, "missing_13", true},
};

// ww_hunt test points
static const TestPoint ww_hunt_points[] = {
	// Player-reported artifact cells (batch 1)
	{{-1871.2f, 528.0f, 256.5f}, "artifact_1", false},
	{{-1630.7f, 704.0f, 377.1f}, "artifact_2", false},
	{{-1529.5f, 104.9f, 117.5f}, "artifact_3", false},
	{{-499.3f, 16.0f, 199.1f}, "artifact_4", false},
	// Player-reported missing clip hulls
	{{-1039.5f, -560.5f, 369.8f}, "missing_1", true},
	// Player-reported artifact cells (batch 2)
	{{-2512.0f, 1471.2f, 378.9f}, "artifact_5", false},
	{{-1273.7f, 160.9f, 150.8f}, "artifact_6", false},
	{{-624.4f, 16.0f, 180.2f}, "artifact_7", false},
	{{1920.0f, 724.2f, 158.3f}, "artifact_8", false},
	// Player-reported artifact cells (batch 3)
	{{-3520.0f, 1838.9f, 431.6f}, "artifact_9", false},
	// Player-reported artifact cells (batch 4)
	{{1440.0f, 593.1f, 149.4f}, "artifact_10", false},
	{{286.3f, 995.0f, 124.3f}, "artifact_11", false},
	{{-2888.6f, 3376.0f, 578.3f}, "artifact_12", false},
	// Batch 5
	{{-179.5f, 141.3f, 636.1f}, "missing_2", true},
	{{152.6f, 1024.0f, 110.9f}, "artifact_13", false},
	// Batch 6
	{{250.1f, 1024.0f, 103.3f}, "artifact_14", false},
	{{1198.9f, -633.8f, 106.1f}, "missing_3", true},
};

// ww_2fort test points
static const TestPoint ww_2fort_points[] = {
	{{1042.2f, 2154.7f, 110.0f}, "artifact_1", false},
	{{462.2f, 2577.7f, -402.0f}, "artifact_2", false},
	{{-1103.9f, 1696.0f, -240.8f}, "artifact_3", false},
	{{491.2f, 2985.6f, -402.0f}, "artifact_4", false},
	{{-973.9f, 1699.4f, -227.5f}, "artifact_5", false},
	{{1132.5f, -1646.6f, -144.1f}, "artifact_6", false},
	{{955.9f, -1536.0f, -42.4f}, "artifact_7", false},
};

// ww_ravine2 test points
static const TestPoint ww_ravine2_points[] = {
	{{816.7f, 1905.8f, 541.6f}, "missing_1", true},
	{{883.0f, 2170.3f, 541.0f}, "missing_2", true},
	{{1074.1f, 1688.4f, 541.0f}, "missing_3", true},
	{{499.0f, 1714.6f, 540.7f}, "missing_4", true},
	{{1284.8f, 1715.6f, 540.4f}, "missing_5", true},
	{{903.8f, -1889.1f, 540.0f}, "missing_6", true},
};

// castle_rush test points
static const TestPoint castle_rush_points[] = {
	{{484.0f, -2880.1f, -38.9f}, "missing_1", true},
	{{376.0f, -1062.3f, 177.6f}, "missing_2", true},
};

static const MapTestData all_maps[] = {
	{"../../../res/maps/ww_golem.bsp", "ww_golem", ww_golem_points,
		sizeof(ww_golem_points)/sizeof(ww_golem_points[0])},
	{"../../../res/maps/ww_hunt.bsp", "ww_hunt", ww_hunt_points,
		sizeof(ww_hunt_points)/sizeof(ww_hunt_points[0])},
	{"../../../res/maps/ww_2fort.bsp", "ww_2fort", ww_2fort_points,
		sizeof(ww_2fort_points)/sizeof(ww_2fort_points[0])},
	{"../../../res/maps/ww_ravine2.bsp", "ww_ravine2", ww_ravine2_points,
		sizeof(ww_ravine2_points)/sizeof(ww_ravine2_points[0])},
	{"../../../res/maps/castle_rush.bsp", "castle_rush", castle_rush_points,
		sizeof(castle_rush_points)/sizeof(castle_rush_points[0])},
};

// --- Pipeline: run full clip brush extraction on a BSP ---
struct PipelineResult {
	vector<ConvexCell> final_cells;
	vector<ConvexCell> solid_cells;       // step 1
	vector<ConvexCell> unexpanded_cells;  // step 2
};

static PipelineResult run_pipeline(const goldsrc::BSPData &bsp, const char *map_name) {
	PipelineResult result;

	const auto &bmodel = bsp.models[0];
	int hull_index = 1;
	int root = bmodel.headnode[hull_index];
	int hull0_root = bmodel.headnode[0];
	printf("Hull %d root=%d, Hull 0 root=%d\n", hull_index, root, hull0_root);
	printf("clipnodes=%zu, nodes=%zu, leafs=%zu, planes=%zu\n",
		bsp.clipnodes.size(), bsp.nodes.size(), bsp.leafs.size(), bsp.planes.size());

	const float EPSILON = 0.1f;
	float he[3] = {16, 16, 36};

	HullPlane bbox_planes[6];
	bbox_planes[0] = {{ 1, 0, 0}, bmodel.maxs[0] + 1.0f};
	bbox_planes[1] = {{-1, 0, 0}, -bmodel.mins[0] + 1.0f};
	bbox_planes[2] = {{ 0, 1, 0}, bmodel.maxs[1] + 1.0f};
	bbox_planes[3] = {{ 0,-1, 0}, -bmodel.mins[1] + 1.0f};
	bbox_planes[4] = {{ 0, 0, 1}, bmodel.maxs[2] + 1.0f};
	bbox_planes[5] = {{ 0, 0,-1}, -bmodel.mins[2] + 1.0f};

	// Step 1: Walk expanded hull 1 tree
	printf("\n--- Step 1: Walk clip tree (expanded solid) ---\n");
	vector<HullPlane> accumulated;
	walk_clip_tree(bsp.clipnodes, bsp.planes, root, accumulated, result.solid_cells,
		goldsrc::CONTENTS_SOLID);
	printf("Expanded solid cells: %zu\n", result.solid_cells.size());

	for (auto &cell : result.solid_cells) {
		for (int i = 0; i < 6; i++) {
			HullPlane bp = bbox_planes[i];
			bp.from_hull1 = false;
			cell.planes.push_back(bp);
		}
	}

	// Step 2: Un-expand ALL hull 1 planes
	printf("\n--- Step 2: Un-expand all hull 1 planes ---\n");
	auto t0 = chrono::steady_clock::now();

	auto unexpand_cell_f = [&](const ConvexCell &cell, float factor) -> ConvexCell {
		ConvexCell ue;
		for (const auto &hp : cell.planes) {
			HullPlane p = hp;
			if (p.from_hull1) {
				float support = fabsf(p.normal[0]) * he[0]
				              + fabsf(p.normal[1]) * he[1]
				              + fabsf(p.normal[2]) * he[2];
				p.dist -= support * hp.unexpand_sign * factor;
			}
			ue.planes.push_back(p);
		}
		return ue;
	};
	auto unexpand_cell = [&](const ConvexCell &cell) -> ConvexCell {
		return unexpand_cell_f(cell, 1.0f);
	};

	size_t step2_degen = 0, step2_rescued = 0;
	for (auto &cell : result.solid_cells) {
		ConvexCell ue = unexpand_cell(cell);
		auto verts = compute_cell_vertices(ue.planes, EPSILON);
		if (verts.size() >= 4) {
			result.unexpanded_cells.push_back(std::move(ue));
			continue;
		}
		vector<ConvexCell> h0_fragments;
		clip_cell_by_hull0(cell, bsp.nodes, bsp.leafs, bsp.planes,
			hull0_root, EPSILON, h0_fragments);
		bool any_rescued = false;
		for (auto &frag : h0_fragments) {
			ConvexCell uf = unexpand_cell(frag);
			auto fv = compute_cell_vertices(uf.planes, EPSILON);
			if (fv.size() >= 4) {
				result.unexpanded_cells.push_back(std::move(uf));
				any_rescued = true;
			}
		}
		if (!any_rescued) {
			float lo = 0.0f, hi = 1.0f, best = -1.0f;
			ConvexCell best_cell;
			for (int iter = 0; iter < 10; iter++) {
				float mid = (lo + hi) * 0.5f;
				ConvexCell trial = unexpand_cell_f(cell, mid);
				auto tv = compute_cell_vertices(trial.planes, EPSILON);
				if (tv.size() >= 4) {
					best = mid;
					best_cell = std::move(trial);
					lo = mid;
				} else {
					hi = mid;
				}
			}
			if (best >= 0.0f) {
				result.unexpanded_cells.push_back(std::move(best_cell));
				any_rescued = true;
			}
		}
		if (any_rescued) step2_rescued++;
		else step2_degen++;
	}

	auto t1 = chrono::steady_clock::now();
	printf("Un-expanded cells: %zu (degenerate: %zu, rescued: %zu)  (%lldms)\n",
		result.unexpanded_cells.size(), step2_degen, step2_rescued,
		(long long)chrono::duration_cast<chrono::milliseconds>(t1 - t0).count());

	// Step 3: Clip un-expanded cells against hull 0 EMPTY
	printf("\n--- Step 3: Clip against hull 0 ---\n");
	auto t2 = chrono::steady_clock::now();

	vector<ConvexCell> clipped_cells;
	for (size_t i = 0; i < result.unexpanded_cells.size(); i++) {
		clip_cell_by_hull0(result.unexpanded_cells[i], bsp.nodes, bsp.leafs, bsp.planes,
			hull0_root, EPSILON, clipped_cells);
	}

	auto t3 = chrono::steady_clock::now();
	printf("Clipped cells (in hull0-empty): %zu  (%lldms)\n", clipped_cells.size(),
		(long long)chrono::duration_cast<chrono::milliseconds>(t3 - t2).count());

	auto classify_h1 = [&](const float p[3], float tolerance = 0.0f) -> int {
		int idx = root;
		while (idx >= 0) {
			if ((size_t)idx >= bsp.clipnodes.size()) return 0;
			const auto &node = bsp.clipnodes[idx];
			if (node.planenum < 0 || (size_t)node.planenum >= bsp.planes.size()) return 0;
			const auto &plane = bsp.planes[node.planenum];
			float dot = plane.normal[0]*p[0] + plane.normal[1]*p[1] + plane.normal[2]*p[2];
			idx = (dot >= plane.dist - tolerance) ? node.children[0] : node.children[1];
		}
		return idx;
	};

	auto vert_near_wall = [&](const float v[3]) -> bool {
		for (int sx = -1; sx <= 1; sx += 2) {
			for (int sy = -1; sy <= 1; sy += 2) {
				for (int sz = -1; sz <= 1; sz += 2) {
					float p[3] = {v[0]+sx*he[0], v[1]+sy*he[1], v[2]+sz*he[2]};
					int c = classify_hull0_tree(bsp.nodes, bsp.leafs, bsp.planes, hull0_root, p);
					if (c == goldsrc::CONTENTS_SOLID) return true;
				}
			}
		}
		return false;
	};

	// Step 4: Filter - h0 EMPTY + h1/ring combined check
	printf("\n--- Step 4: Vertex filter (h0 EMPTY + h1 SOLID) ---\n");
	auto t4 = chrono::steady_clock::now();

	vector<ConvexCell> result_cells;
	size_t degenerate = 0, h0_filtered = 0, h1_filtered = 0, h1_rescued = 0;
	for (auto &cell : clipped_cells) {
		auto verts = compute_cell_vertices(cell.planes, EPSILON);
		if (verts.size() < 4) { degenerate++; continue; }
		float cx = 0, cy = 0, cz = 0;
		for (const auto &v : verts) { cx += v.gs[0]; cy += v.gs[1]; cz += v.gs[2]; }
		cx /= verts.size(); cy /= verts.size(); cz /= verts.size();

		float cpt[3] = {cx, cy, cz};
		if (cx > -1200 && cx < -800 && cy > -700 && cy < -400 && cz > 200 && cz < 900) { float tmn[3]={1e9,1e9,1e9},tmx[3]={-1e9,-1e9,-1e9}; for(const auto &v:verts){for(int a=0;a<3;a++){if(v.gs[a]<tmn[a])tmn[a]=v.gs[a];if(v.gs[a]>tmx[a])tmx[a]=v.gs[a];}} printf("  TRACE cell: cpt=(%.1f,%.1f,%.1f) %zu verts dim=%.0fx%.0fx%.0f AABB=(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n", cx,cy,cz, verts.size(), tmx[0]-tmn[0],tmx[1]-tmn[1],tmx[2]-tmn[2], tmn[0],tmn[1],tmn[2],tmx[0],tmx[1],tmx[2]); }
		bool any_h0_solid = false;
		for (const auto &v : verts) {
			float nudged[3] = {v.gs[0], v.gs[1], v.gs[2]};
			float dx = cx - v.gs[0], dy = cy - v.gs[1], dz = cz - v.gs[2];
			float len = sqrtf(dx*dx + dy*dy + dz*dz);
			if (len > 0.01f) {
				float t = 0.1f / len;
				nudged[0] += dx * t; nudged[1] += dy * t; nudged[2] += dz * t;
			}
			int h0c = classify_hull0_tree(
				bsp.nodes, bsp.leafs, bsp.planes, hull0_root, nudged);
			if (h0c == goldsrc::CONTENTS_SOLID) { any_h0_solid = true; break; }
		}
		if (any_h0_solid) {
			int h0_cent = classify_hull0_tree(
				bsp.nodes, bsp.leafs, bsp.planes, hull0_root, cpt);
			if (h0_cent == goldsrc::CONTENTS_SOLID) {
				// Rescue: centroid h1 SOLID and not near wall = likely real clip brush
				int h1_cent = classify_h1(cpt);
				if (h1_cent != goldsrc::CONTENTS_SOLID || vert_near_wall(cpt)) {
					h0_filtered++; continue;
				}
			}
		}

		bool any_h1_empty = false;
		bool has_clip_indicator = false;
		int nw_count = 0;
		int h1_empty_count = 0;
		for (const auto &v : verts) {
			int h1c = classify_h1(v.gs);
			bool nw = vert_near_wall(v.gs);
			if (nw) nw_count++;
			if (h1c != goldsrc::CONTENTS_SOLID) {
				any_h1_empty = true;
				h1_empty_count++;
			} else if (!has_clip_indicator && !nw) {
				has_clip_indicator = true;
			}
		}

		// Compute cell dimensions (used by multiple filter paths)
		float mn[3]={1e9,1e9,1e9}, mx[3]={-1e9,-1e9,-1e9};
		for (const auto &v : verts) { for(int a=0;a<3;a++){if(v.gs[a]<mn[a])mn[a]=v.gs[a]; if(v.gs[a]>mx[a])mx[a]=v.gs[a];}}
		float dim[3] = {mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2]};
		float min_dim = fminf(dim[0], fminf(dim[1], dim[2]));
		float max_he = fmaxf(he[0], fmaxf(he[1], he[2]));
		bool big_per_axis = (dim[0] >= 2.0f*he[0]) &&
		                    (dim[1] >= 2.0f*he[1]) &&
		                    (dim[2] >= 2.0f*he[2]);
		bool big_global = min_dim >= 2.0f * max_he;
		bool big_per_he = (dim[0] >= he[0]) && (dim[1] >= he[1]) && (dim[2] >= he[2]);

		if (any_h1_empty && !has_clip_indicator) {
			// No clip indicator: cell needs rescue via centroid h1 check + size gate
			int ch1 = classify_h1(cpt);
			if (ch1 == goldsrc::CONTENTS_SOLID && !vert_near_wall(cpt)) {
				if (big_per_he) {
					// No clip indicator: require centroid h0 not solid
					int ch0 = classify_hull0_tree(
						bsp.nodes, bsp.leafs, bsp.planes, hull0_root, cpt);
					if (ch0 != goldsrc::CONTENTS_SOLID) {
						result_cells.push_back(std::move(cell));
						h1_rescued++;
					} else {
						h1_filtered++;
					}
				} else {
					h1_filtered++;
				}
			} else {
				bool rescued = false;
				if (ch1 != goldsrc::CONTENTS_SOLID) {
					int ch1_tol = classify_h1(cpt, 8.0f);
					if (ch1_tol == goldsrc::CONTENTS_SOLID && big_global) {
						result_cells.push_back(std::move(cell));
						h1_rescued++;
						rescued = true;
					}
				}
				if (!rescued) h1_filtered++;
			}
			continue;
		}

		// Cell has clip indicator (or all verts h1 SOLID).
		// Per-axis size gates:
		// 1) Any dim < he[i]: physically impossible as un-expanded brush. Always filter.
		// 2) Any dim < 2*he[i] with weak evidence: likely expansion artifact.
		bool impossibly_thin = verts.size() >= 6 && ((dim[0] < he[0]*0.5f) || (dim[1] < he[1]*0.5f) || (dim[2] < he[2]*0.5f));
		if (impossibly_thin) { h1_filtered++; continue; }
		// Degenerate small cells: 2+ dims < he and max dim too small to be real
		{	int degen_count = 0;
			float max_dim_val = fmaxf(dim[0], fmaxf(dim[1], dim[2]));
			for (int a = 0; a < 3; a++) { if (dim[a] < he[a]) degen_count++; }
			if (degen_count >= 2 && max_dim_val < 2.0f * max_he) { h1_filtered++; continue; }
		}
		// Cells with a small-axis dim at expansion boundary (un-expanded ~0) are artifacts
		if (any_h1_empty) {
			bool small_axis_narrow = false;
			for (int a = 0; a < 3; a++) {
				if (he[a] < max_he && dim[a] >= 2.0f * he[a] - 1.0f && dim[a] <= 2.0f * he[a] + 1.0f) {
					small_axis_narrow = true; break;
				}
			}
			if (small_axis_narrow) { h1_filtered++; continue; }
		}
		if (!big_per_he) {
			int cent_h1 = classify_h1(cpt);
			if (any_h1_empty || cent_h1 != goldsrc::CONTENTS_SOLID) {
				h1_filtered++;
				continue;
			}
			// Complex small cells with any near-wall verts are expansion artifacts
			if (verts.size() >= 6 && nw_count > 0) {
				h1_filtered++;
				continue;
			}
		} else if (!big_per_axis) {
			// Count axes where dim < 2*he (negative after un-expansion)
			int sub_axes = 0;
			for (int a = 0; a < 3; a++) { if (dim[a] < 2.0f * he[a]) sub_axes++; }
			// 2+ sub-expansion axes with any h1 empty = expansion remnant
			if (sub_axes >= 2 && any_h1_empty) {
				h1_filtered++;
				continue;
			}
			int cent_h1 = classify_h1(cpt);
			if (cent_h1 != goldsrc::CONTENTS_SOLID ||
				h1_empty_count >= (int)verts.size()/2) {
				h1_filtered++;
				continue;
			}
		} else if (any_h1_empty) {
			// Big cells still need centroid h1 SOLID + majority h1 SOLID
			int cent_h1 = classify_h1(cpt);
			if (cent_h1 != goldsrc::CONTENTS_SOLID ||
				h1_empty_count >= (int)verts.size()/2) {
				h1_filtered++;
				continue;
			}
		}

		result_cells.push_back(std::move(cell));
	}

	auto t5 = chrono::steady_clock::now();
	printf("After filter: %zu -> %zu result (degen: %zu, h0: %zu, h1: %zu, rescued: %zu)  (%lldms)\n",
		clipped_cells.size(), result_cells.size(), degenerate, h0_filtered, h1_filtered, h1_rescued,
		(long long)chrono::duration_cast<chrono::milliseconds>(t5 - t4).count());

	// Step 5: Expansion ring filter
	printf("\n--- Step 5: Expansion ring filter ---\n");
	auto t6 = chrono::steady_clock::now();

	size_t ring_filtered = 0;
	for (auto &cell : result_cells) {
		auto verts = compute_cell_vertices(cell.planes, EPSILON);
		if (verts.size() < 4) continue;
		bool all_near_wall = true;
		int nw_count = 0;
		for (const auto &v : verts) {
			if (vert_near_wall(v.gs)) nw_count++; else all_near_wall = false;
		}
		if (all_near_wall) {
			// Rescue: centroid h1 SOLID and not near wall = clip brush at wall
			float acpt[3]={0,0,0};
			for (const auto &v : verts) { acpt[0]+=v.gs[0]; acpt[1]+=v.gs[1]; acpt[2]+=v.gs[2]; }
			acpt[0]/=verts.size(); acpt[1]/=verts.size(); acpt[2]/=verts.size();
			int ah1 = classify_h1(acpt);
			if (ah1 != goldsrc::CONTENTS_SOLID || vert_near_wall(acpt)) {
				// Second rescue: all verts h1 SOLID + centroid h0 non-solid + large cell
				bool all_h1s_anw = true;
				for (const auto &v : verts) {
					if (classify_h1(v.gs) != goldsrc::CONTENTS_SOLID) { all_h1s_anw = false; break; }
				}
				int ch0_anw = classify_hull0_tree(bsp.nodes, bsp.leafs, bsp.planes, hull0_root, acpt);
				float anw_mn[3]={1e9,1e9,1e9}, anw_mx[3]={-1e9,-1e9,-1e9};
				for (const auto &v : verts) { for(int a=0;a<3;a++){if(v.gs[a]<anw_mn[a])anw_mn[a]=v.gs[a];if(v.gs[a]>anw_mx[a])anw_mx[a]=v.gs[a];} }
				float anw_max_dim = fmaxf(anw_mx[0]-anw_mn[0], fmaxf(anw_mx[1]-anw_mn[1], anw_mx[2]-anw_mn[2]));
				if (!(all_h1s_anw && ch0_anw != goldsrc::CONTENTS_SOLID && anw_max_dim >= 256.0f)) {
					ring_filtered++; continue;
				}
			}
		}
		// Additional: small cells (fail per-axis) with majority near-wall verts
		float rn[3]={1e9,1e9,1e9}, rx[3]={-1e9,-1e9,-1e9};
		for (const auto &v : verts) { for(int a=0;a<3;a++){if(v.gs[a]<rn[a])rn[a]=v.gs[a]; if(v.gs[a]>rx[a])rx[a]=v.gs[a];}}
		float rdim[3] = {rx[0]-rn[0], rx[1]-rn[1], rx[2]-rn[2]};
		bool r_big = (rdim[0] >= 2.0f*he[0]) && (rdim[1] >= 2.0f*he[1]) && (rdim[2] >= 2.0f*he[2]);
		// Centroid near wall = expansion ring artifact
		float rcpt[3];
		rcpt[0]=rcpt[1]=rcpt[2]=0;
		for (const auto &v : verts) { rcpt[0]+=v.gs[0]; rcpt[1]+=v.gs[1]; rcpt[2]+=v.gs[2]; }
		rcpt[0]/=verts.size(); rcpt[1]/=verts.size(); rcpt[2]/=verts.size();
		if (vert_near_wall(rcpt)) {
			bool any_h0s = false, all_h1s = true;
			for (const auto &v : verts) {
				int vh0 = classify_hull0_tree(bsp.nodes, bsp.leafs, bsp.planes, hull0_root, v.gs);
				if (vh0 == goldsrc::CONTENTS_SOLID) any_h0s = true;
				if (classify_h1(v.gs) != goldsrc::CONTENTS_SOLID) { all_h1s = false; break; }
			}
			if (!all_h1s) { ring_filtered++; continue; }
			if (any_h0s) {
				// Second rescue: per-axis 4*he big + centroid h0 non-solid
				bool big4 = (rdim[0] >= 4.0f*he[0]) && (rdim[1] >= 4.0f*he[1]) && (rdim[2] >= 4.0f*he[2]);
				int ch0_cnw = classify_hull0_tree(bsp.nodes, bsp.leafs, bsp.planes, hull0_root, rcpt);
				if (!(big4 && ch0_cnw != goldsrc::CONTENTS_SOLID)) {
					ring_filtered++; continue;
				}
			}
		}
		// Small cells with majority near-wall verts
		int big_axes = 0;
		for (int a = 0; a < 3; a++) { if (rdim[a] >= 2.0f * he[a]) big_axes++; }
		if (!r_big && nw_count > (int)verts.size()/2) {
			// Rescue: if any non-near-wall vert is h1 SOLID, this is a real clip brush
			bool has_nnw_solid = false;
			for (const auto &v : verts) {
				if (!vert_near_wall(v.gs) && classify_h1(v.gs) == goldsrc::CONTENTS_SOLID) {
					has_nnw_solid = true; break;
				}
			}
			float rsd[3]={rdim[0],rdim[1],rdim[2]};
			if(rsd[0]<rsd[1])swap(rsd[0],rsd[1]); if(rsd[1]<rsd[2])swap(rsd[1],rsd[2]); if(rsd[0]<rsd[1])swap(rsd[0],rsd[1]);
			if (big_axes >= 2 && has_nnw_solid && rsd[0] >= 256.0f) {
				/* rescued */
			} else { ring_filtered++; continue; }
		}
		// Expansion ring tube: 2+ dims at expansion width with majority near-wall
		int narrow_axes = 0;
		for (int a = 0; a < 3; a++) { if (rdim[a] <= 2.0f * he[a] + 0.5f) narrow_axes++; }
		if (narrow_axes >= 2 && nw_count > (int)verts.size()/2) { ring_filtered++; continue; }
		// Single narrow axis with majority near-wall: check h1 at tight tolerance
		if (narrow_axes >= 1 && nw_count > (int)verts.size()/2) {
			int rh1_tol = classify_h1(rcpt, 2.0f);
			if (rh1_tol != goldsrc::CONTENTS_SOLID) { ring_filtered++; continue; }
		}
		// Non-big cells with 6+ verts and significant near-wall fraction (>1/3)
		if (!r_big && verts.size() >= 6 && nw_count >= 3 && nw_count * 3 > (int)verts.size()) {
			bool has_nnw_solid2 = false;
			for (const auto &v : verts) {
				if (!vert_near_wall(v.gs) && classify_h1(v.gs) == goldsrc::CONTENTS_SOLID) {
					has_nnw_solid2 = true; break;
				}
			}
			float rsd2[3]={rdim[0],rdim[1],rdim[2]};
			if(rsd2[0]<rsd2[1])swap(rsd2[0],rsd2[1]); if(rsd2[1]<rsd2[2])swap(rsd2[1],rsd2[2]); if(rsd2[0]<rsd2[1])swap(rsd2[0],rsd2[1]);
			if (big_axes >= 2 && has_nnw_solid2 && rsd2[0] >= 256.0f) {
				/* rescued */
			} else { ring_filtered++; continue; }
		}
				result.final_cells.push_back(std::move(cell));
	}

	auto t7 = chrono::steady_clock::now();
	printf("After expansion ring filter: %zu -> %zu (removed: %zu)  (%lldms)\n",
		result_cells.size(), result.final_cells.size(), ring_filtered,
		(long long)chrono::duration_cast<chrono::milliseconds>(t7 - t6).count());

	return result;
}

// --- Run tests for a single map ---
static void run_tests(const PipelineResult &pipeline, const goldsrc::BSPData &bsp,
	const TestPoint *points, size_t num_points, const char *map_name,
	int &total_pass, int &total_fail) {

	const float EPSILON = 0.1f;
	float he[3] = {16, 16, 36};

	const auto &bmodel = bsp.models[0];
	int hull0_root = bmodel.headnode[0];
	int root = bmodel.headnode[1];

	auto aabb_overlaps_cell = [&](const float center[3], const float half_ext[3],
		const ConvexCell &cell, float eps) -> bool {
		for (const auto &hp : cell.planes) {
			float support = fabsf(hp.normal[0]) * half_ext[0]
			              + fabsf(hp.normal[1]) * half_ext[1]
			              + fabsf(hp.normal[2]) * half_ext[2];
			float dot = hp.normal[0] * center[0] + hp.normal[1] * center[1] + hp.normal[2] * center[2];
			if (dot > hp.dist + support + eps) return false;
		}
		return true;
	};

	auto classify_h1 = [&](const float p[3], float tolerance = 0.0f) -> int {
		int idx = root;
		while (idx >= 0) {
			if ((size_t)idx >= bsp.clipnodes.size()) return 0;
			const auto &node = bsp.clipnodes[idx];
			if (node.planenum < 0 || (size_t)node.planenum >= bsp.planes.size()) return 0;
			const auto &plane = bsp.planes[node.planenum];
			float dot = plane.normal[0]*p[0] + plane.normal[1]*p[1] + plane.normal[2]*p[2];
			idx = (dot >= plane.dist - tolerance) ? node.children[0] : node.children[1];
		}
		return idx;
	};

	auto vert_near_wall = [&](const float v[3]) -> bool {
		for (int sx = -1; sx <= 1; sx += 2) {
			for (int sy = -1; sy <= 1; sy += 2) {
				for (int sz = -1; sz <= 1; sz += 2) {
					float p[3] = {v[0]+sx*he[0], v[1]+sy*he[1], v[2]+sz*he[2]};
					int c = classify_hull0_tree(bsp.nodes, bsp.leafs, bsp.planes, hull0_root, p);
					if (c == goldsrc::CONTENTS_SOLID) return true;
				}
			}
		}
		return false;
	};

	printf("\n=== COLLISION TESTS [%s] ===\n", map_name);
	int pass = 0, fail = 0;
	for (size_t ti = 0; ti < num_points; ti++) {
		const auto &tp = points[ti];
		printf("\n%s  GS(%.0f, %.0f, %.0f):\n", tp.label, tp.gs[0], tp.gs[1], tp.gs[2]);

		if (tp.should_be_inside) {
			int overlaps = 0;
			for (size_t i = 0; i < pipeline.final_cells.size(); i++) {
				if (aabb_overlaps_cell(tp.gs, he, pipeline.final_cells[i], EPSILON))
					overlaps++;
			}
			bool ok = overlaps > 0;
			printf("  Player AABB overlaps %d result cells  %s\n", overlaps, ok ? "PASS" : "FAIL");
			if (ok) pass++; else fail++;
		} else {
			int overlaps = 0;
			for (size_t i = 0; i < pipeline.final_cells.size(); i++) {
				if (aabb_overlaps_cell(tp.gs, he, pipeline.final_cells[i], EPSILON)) {
					overlaps++;
					auto cverts = compute_cell_vertices(pipeline.final_cells[i].planes, EPSILON);
					float cmn[3]={1e9,1e9,1e9}, cmx[3]={-1e9,-1e9,-1e9};
					for (const auto &v : cverts) {
						for(int a=0;a<3;a++){if(v.gs[a]<cmn[a])cmn[a]=v.gs[a]; if(v.gs[a]>cmx[a])cmx[a]=v.gs[a];}
					}
					printf("  -> artifact overlap cell %zu: AABB=(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f) dims=%.0fx%.0fx%.0f\n",
						i, cmn[0],cmn[1],cmn[2], cmx[0],cmx[1],cmx[2],
						cmx[0]-cmn[0], cmx[1]-cmn[1], cmx[2]-cmn[2]);
					// Diagnostic: why did this cell survive?
					float ccx=0,ccy=0,ccz=0;
					for (const auto &v : cverts) { ccx+=v.gs[0]; ccy+=v.gs[1]; ccz+=v.gs[2]; }
					ccx/=cverts.size(); ccy/=cverts.size(); ccz/=cverts.size();
					float ccp[3]={ccx,ccy,ccz};
					int dch1_0 = classify_h1(ccp, 0);
					bool dany_h1e=false, dhas_clip=false;
					int dh1e_cnt=0;
					int dnw_cnt=0;
					for (const auto &v : cverts) {
						int h1c = classify_h1(v.gs);
						if (h1c != goldsrc::CONTENTS_SOLID) { dany_h1e=true; dh1e_cnt++; }
						if (h1c == goldsrc::CONTENTS_SOLID && !vert_near_wall(v.gs)) dhas_clip=true;
						if (vert_near_wall(v.gs)) dnw_cnt++;
					}
					bool cent_nw = vert_near_wall(ccp);
					int dch0 = goldsrc_hull::classify_hull0_tree(
						bsp.nodes, bsp.leafs, bsp.planes, hull0_root, ccp);
					printf("       centroid=(%.1f,%.1f,%.1f) h1=%d h0=%d h1e=%d/%zu clip=%d near=%d/%zu cnw=%d\n",
						ccx,ccy,ccz, dch1_0, dch0, dh1e_cnt, cverts.size(), dhas_clip, dnw_cnt, cverts.size(), cent_nw);
				}
			}
			bool ok = overlaps == 0;
			if (overlaps > 0)
				printf("  Player AABB overlaps %d cells  FAIL\n", overlaps);
			else
				printf("  Player AABB clear of all cells  PASS\n");
			if (ok) pass++; else fail++;
		}
	}
	printf("\n=== [%s] %d PASS, %d FAIL ===\n", map_name, pass, fail);
	printf("Total result cells: %zu\n", pipeline.final_cells.size());
	total_pass += pass;
	total_fail += fail;

	// Diagnostics for missing_ points
	printf("\n=== DIAGNOSTICS [%s] ===\n", map_name);
	for (size_t ti = 0; ti < num_points; ti++) {
		const auto &tp = points[ti];
		if (strncmp(tp.label, "missing_", 8) != 0) continue;
		printf("\n%s  GS(%.1f, %.1f, %.1f):\n", tp.label, tp.gs[0], tp.gs[1], tp.gs[2]);

		int h1_exp = classify_h1(tp.gs);
		int h0 = goldsrc_hull::classify_hull0_tree(
			bsp.nodes, bsp.leafs, bsp.planes, hull0_root, tp.gs);
		printf("  hull1_expanded=%d  hull0=%d\n", h1_exp, h0);

		int s1_count = 0, s2_count = 0;
		for (size_t ci = 0; ci < pipeline.solid_cells.size(); ci++)
			if (aabb_overlaps_cell(tp.gs, he, pipeline.solid_cells[ci], EPSILON)) s1_count++;
		for (size_t ci = 0; ci < pipeline.unexpanded_cells.size(); ci++)
			if (aabb_overlaps_cell(tp.gs, he, pipeline.unexpanded_cells[ci], EPSILON)) s2_count++;
		printf("  step1 (expanded): %d  step2 (unexpanded): %d\n", s1_count, s2_count);

		if (s1_count > 0 && s2_count == 0) {
			for (size_t ci = 0; ci < pipeline.solid_cells.size(); ci++) {
				if (!aabb_overlaps_cell(tp.gs, he, pipeline.solid_cells[ci], EPSILON)) continue;
				auto ev = compute_cell_vertices(pipeline.solid_cells[ci].planes, EPSILON);
				float emn[3]={1e9,1e9,1e9}, emx[3]={-1e9,-1e9,-1e9};
				for (const auto &v : ev) {
					for (int a=0;a<3;a++) { if(v.gs[a]<emn[a])emn[a]=v.gs[a]; if(v.gs[a]>emx[a])emx[a]=v.gs[a]; }
				}
				printf("  expanded_cell%zu: %zu verts, AABB=(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n",
					ci, ev.size(), emn[0],emn[1],emn[2], emx[0],emx[1],emx[2]);
				ConvexCell ue;
				for (const auto &hp : pipeline.solid_cells[ci].planes) {
					HullPlane p = hp;
					if (p.from_hull1) {
						float support = fabsf(p.normal[0])*he[0]+fabsf(p.normal[1])*he[1]+fabsf(p.normal[2])*he[2];
						p.dist -= support * hp.unexpand_sign;
					}
					ue.planes.push_back(p);
				}
				auto uv = compute_cell_vertices(ue.planes, EPSILON);
				if (uv.size() < 4) {
					printf("    -> un-expanded: DEGENERATE (%zu verts)\n", uv.size());
				} else {
					float umn[3]={1e9,1e9,1e9}, umx[3]={-1e9,-1e9,-1e9};
					for (const auto &v : uv) {
						for (int a=0;a<3;a++) { if(v.gs[a]<umn[a])umn[a]=v.gs[a]; if(v.gs[a]>umx[a])umx[a]=v.gs[a]; }
					}
					printf("    -> un-expanded: %zu verts, AABB=(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n",
						uv.size(), umn[0],umn[1],umn[2], umx[0],umx[1],umx[2]);
				}
			}
		}

		for (size_t ci = 0; ci < pipeline.unexpanded_cells.size(); ci++) {
			if (!aabb_overlaps_cell(tp.gs, he, pipeline.unexpanded_cells[ci], EPSILON)) continue;
			auto verts = goldsrc_hull::compute_cell_vertices(pipeline.unexpanded_cells[ci].planes, EPSILON);
			float fmn[3] = {1e9,1e9,1e9}, fmx[3] = {-1e9,-1e9,-1e9};
			for (const auto &v : verts) {
				for (int a = 0; a < 3; a++) {
					if (v.gs[a] < fmn[a]) fmn[a] = v.gs[a];
					if (v.gs[a] > fmx[a]) fmx[a] = v.gs[a];
				}
			}
			printf("  unexpanded_cell%zu: %zu verts, dims=%.1f x %.1f x %.1f\n",
				ci, verts.size(), fmx[0]-fmn[0], fmx[1]-fmn[1], fmx[2]-fmn[2]);

			vector<ConvexCell> fragments;
			goldsrc_hull::clip_cell_by_hull0(pipeline.unexpanded_cells[ci],
				bsp.nodes, bsp.leafs, bsp.planes, hull0_root, EPSILON, fragments);
			printf("    hull0 clip -> %zu fragments\n", fragments.size());

			int frag_overlap = 0;
			for (size_t fi = 0; fi < fragments.size(); fi++) {
				if (!aabb_overlaps_cell(tp.gs, he, fragments[fi], EPSILON)) continue;
				frag_overlap++;
				auto fv = goldsrc_hull::compute_cell_vertices(fragments[fi].planes, EPSILON);
				bool any_h0_solid = false, any_h1_empty = false;
				int near_wall_count = 0;
				for (const auto &v : fv) {
					int h0c = goldsrc_hull::classify_hull0_tree(
						bsp.nodes, bsp.leafs, bsp.planes, hull0_root, v.gs);
					if (h0c == goldsrc::CONTENTS_SOLID) any_h0_solid = true;
					int h1c = classify_h1(v.gs);
					if (h1c != goldsrc::CONTENTS_SOLID) any_h1_empty = true;
					if (vert_near_wall(v.gs)) near_wall_count++;
				}
				float fcx=0,fcy=0,fcz=0;
				for (const auto &v : fv) { fcx+=v.gs[0]; fcy+=v.gs[1]; fcz+=v.gs[2]; }
				fcx/=fv.size(); fcy/=fv.size(); fcz/=fv.size();
				float fcpt[3]={fcx,fcy,fcz};
				bool fcnw = vert_near_wall(fcpt);
				int fc_h1 = classify_h1(fcpt);
				int fc_h0 = goldsrc_hull::classify_hull0_tree(
					bsp.nodes, bsp.leafs, bsp.planes, hull0_root, fcpt);
				const char *fate = "SURVIVES";
				if (fv.size() < 4) fate = "killed:degenerate";
				else if (any_h0_solid) {
					if (fc_h0 == goldsrc::CONTENTS_SOLID) {
						if (fc_h1 != goldsrc::CONTENTS_SOLID || fcnw)
							fate = "killed:h0_cent_solid";
						else fate = "rescued:h0_cent_h1solid";
					} else fate = "h0verts_but_cent_empty";
				}
				else if (any_h1_empty) fate = "killed:h1_empty_vert";
				else if (near_wall_count == (int)fv.size()) fate = "killed:ring_filter";
				float fn[3]={1e9f,1e9f,1e9f},fx[3]={-1e9f,-1e9f,-1e9f};
				for(const auto&v:fv){for(int a=0;a<3;a++){if(v.gs[a]<fn[a])fn[a]=v.gs[a];if(v.gs[a]>fx[a])fx[a]=v.gs[a];}}
				float fd[3]={fx[0]-fn[0],fx[1]-fn[1],fx[2]-fn[2]};
				printf("    frag%zu: %zu verts, dims=%.0fx%.0fx%.0f h0s=%d h1e=%d near=%d/%zu cent_nw=%d ch0=%d ch1=%d -> %s\n",
					fi, fv.size(), fd[0],fd[1],fd[2], any_h0_solid, any_h1_empty,
					near_wall_count, fv.size(), fcnw, fc_h0, fc_h1, fate);
			}
			printf("    overlapping fragments: %d\n", frag_overlap);
		}
	}
}

// --- Main ---
int main(int argc, char **argv) {
	int total_pass = 0, total_fail = 0;

	for (const auto &map : all_maps) {
		printf("\n########################################\n");
		printf("# MAP: %s\n", map.map_name);
		printf("########################################\n");

		string bsp_path;
		if (argc >= 2) {
			bsp_path = string(argv[1]) + "/" + map.map_name + ".bsp";
		} else {
			bsp_path = map.bsp_relative_path;
		}

		FILE *f = fopen(bsp_path.c_str(), "rb");
		if (!f) {
			printf("Cannot open %s - SKIPPING\n", bsp_path.c_str());
			continue;
		}
		fseek(f, 0, SEEK_END);
		size_t sz = ftell(f);
		fseek(f, 0, SEEK_SET);
		vector<uint8_t> buf(sz);
		fread(buf.data(), 1, sz, f);
		fclose(f);

		goldsrc::BSPParser parser;
		if (!parser.parse(buf.data(), sz)) {
			fprintf(stderr, "Failed to parse %s\n", bsp_path.c_str());
			continue;
		}

		const auto &bsp = parser.get_data();
		if (bsp.models.empty()) {
			fprintf(stderr, "No models in %s\n", bsp_path.c_str());
			continue;
		}

		auto pipeline = run_pipeline(bsp, map.map_name);
		run_tests(pipeline, bsp, map.points, map.num_points, map.map_name,
			total_pass, total_fail);
	}

	printf("\n========================================\n");
	printf("=== GRAND TOTAL: %d PASS, %d FAIL ===\n", total_pass, total_fail);
	printf("========================================\n");

	return total_fail > 0 ? 1 : 0;
}

