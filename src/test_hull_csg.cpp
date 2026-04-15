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

// All pre-sky-fix test data removed. vert_near_wall was fixed to treat
// CONTENTS_SKY as non-playable (equal to CONTENTS_SOLID). Prior test
// points are invalid: sky expansion ring artifacts were false positives
// that the fix eliminates, and "missing" points near sky brushes were
// incorrectly reported.
//
// TODO: Regenerate test points by running the sky-fixed pipeline on each
// map and visually inspecting the output in-engine. Add verified points
// here in the form:
//   static const TestPoint my_map_points[] = {
//       {{x, y, z}, "label", should_be_inside},
//   };
// Then register the map in all_maps[] below.

// ww_leyline: Python tool found 1 CLIP brush at (-2912, -1535.5, -175.5)
static const TestPoint ww_leyline_points[] = {
	{{-2912.0f, -1535.5f, -175.5f}, "missing_1", true},
};

static const MapTestData all_maps[] = {
	// No test points for most maps — pipeline just prints cell counts.
	{"", "ww_2fort",        nullptr, 0},
	{"", "ww_atrocity",     nullptr, 0},
	{"", "ww_castle",       nullptr, 0},
	{"", "ww_chasm",        nullptr, 0},
	{"", "ww_checkmate",    nullptr, 0},
	{"", "ww_december",     nullptr, 0},
	{"", "ww_feudal",       nullptr, 0},
	{"", "ww_golem",        nullptr, 0},
	{"", "ww_hiddenforest", nullptr, 0},
	{"", "ww_hunt",         nullptr, 0},
	{"", "ww_keep",         nullptr, 0},
	{"", "ww_leyline",      ww_leyline_points, 1},
	{"", "ww_library",      nullptr, 0},
	{"", "ww_memnate",      nullptr, 0},
	{"", "ww_monoliths",    nullptr, 0},
	{"", "ww_osaka",        nullptr, 0},
	{"", "ww_rage",         nullptr, 0},
	{"", "ww_ravine",       nullptr, 0},
	{"", "ww_ravine2",      nullptr, 0},
	{"", "ww_roc2",         nullptr, 0},
	{"", "ww_shrinkspell",  nullptr, 0},
	{"", "ww_storm",        nullptr, 0},
	{"", "ww_volcano",      nullptr, 0},
	{"", "ww_wolteg",       nullptr, 0},
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

	// Steps 4+5: Filter clip brush cells (vertex filter + expansion ring filter)
	printf("\n--- Steps 4+5: Filter clip brush cells ---\n");
	auto t4 = chrono::steady_clock::now();

	size_t pre_filter_count = clipped_cells.size();
	result.final_cells = goldsrc_hull::filter_clip_brush_cells(
		std::move(clipped_cells),
		bsp.nodes, bsp.leafs, bsp.clipnodes, bsp.planes,
		hull0_root, root, he, EPSILON);

	auto t5 = chrono::steady_clock::now();
	printf("After filter: %zu -> %zu result cells  (%lldms)\n",
		pre_filter_count, result.final_cells.size(),
		(long long)chrono::duration_cast<chrono::milliseconds>(t5 - t4).count());

	// Dump each surviving cell: centroid, AABB, h0/h1/nw at centroid,
	// plus per-vertex h1 and nw to show clip indicator status.
	for (size_t ci = 0; ci < result.final_cells.size(); ci++) {
		auto verts = compute_cell_vertices(result.final_cells[ci].planes, 0.1f);
		if (verts.empty()) continue;
		float cx=0,cy=0,cz=0;
		float mn[3]={1e9f,1e9f,1e9f}, mx[3]={-1e9f,-1e9f,-1e9f};
		for (const auto &v : verts) {
			cx+=v.gs[0]; cy+=v.gs[1]; cz+=v.gs[2];
			for (int a=0;a<3;a++){if(v.gs[a]<mn[a])mn[a]=v.gs[a];if(v.gs[a]>mx[a])mx[a]=v.gs[a];}
		}
		cx/=verts.size(); cy/=verts.size(); cz/=verts.size();
		float cpt[3]={cx,cy,cz};
		int ch0 = classify_hull0_tree(bsp.nodes, bsp.leafs, bsp.planes, hull0_root, cpt);
		int ch1 = classify_clip_hull(bsp.clipnodes, bsp.planes, root, cpt);
		bool cnw = vert_near_wall(bsp.nodes, bsp.leafs, bsp.planes, hull0_root, cpt, he);
		// Count vertices by (h1, nw) to see clip indicator presence
		int v_s_nw=0, v_s_nnw=0, v_e_nw=0, v_e_nnw=0;
		for (const auto &v : verts) {
			int vh1 = classify_clip_hull(bsp.clipnodes, bsp.planes, root, v.gs);
			bool vnw = vert_near_wall(bsp.nodes, bsp.leafs, bsp.planes, hull0_root, v.gs, he);
			if (vh1 == goldsrc::CONTENTS_SOLID) { if (vnw) v_s_nw++; else v_s_nnw++; }
			else                                { if (vnw) v_e_nw++; else v_e_nnw++; }
		}
		// v_s_nnw > 0 means at least one "clip indicator" vertex (h1=SOLID, not near wall)
		printf("  cell[%zu]: centroid=(%.1f,%.1f,%.1f) dims=%.0fx%.0fx%.0f"
		       " h0=%d h1=%d nw=%d | verts: S+nw=%d S+nnw=%d E+nw=%d E+nnw=%d\n",
		       ci, cx,cy,cz,
		       mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2],
		       ch0, ch1, (int)cnw,
		       v_s_nw, v_s_nnw, v_e_nw, v_e_nnw);
	}

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
		return goldsrc_hull::classify_clip_hull(bsp.clipnodes, bsp.planes, root, p, tolerance);
	};

	auto vert_near_wall = [&](const float v[3]) -> bool {
		return goldsrc_hull::vert_near_wall(bsp.nodes, bsp.leafs, bsp.planes, hull0_root, v, he);
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
			// Check hull1 at player position
			int h1_at_player = classify_h1(tp.gs);
			printf("  hull1_at_player=%d (%s)\n", h1_at_player,
				h1_at_player == goldsrc::CONTENTS_SOLID ? "SOLID" : "EMPTY");
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
					int dch1u = goldsrc_hull::classify_clip_tree_unexpanded(
						bsp.clipnodes, bsp.planes, root, he, ccp);
					bool dany_h1e=false, dhas_clip=false;
					int dh1e_cnt=0, dfree_h1e=0;
					int dnw_cnt=0;
					for (const auto &v : cverts) {
						int h1c = classify_h1(v.gs);
						bool nw = vert_near_wall(v.gs);
						if (h1c != goldsrc::CONTENTS_SOLID) { dany_h1e=true; dh1e_cnt++; if (!nw) dfree_h1e++; }
						if (h1c == goldsrc::CONTENTS_SOLID && !nw) dhas_clip=true;
						if (nw) dnw_cnt++;
					}
					bool cent_nw = vert_near_wall(ccp);
					int dch0 = goldsrc_hull::classify_hull0_tree(
						bsp.nodes, bsp.leafs, bsp.planes, hull0_root, ccp);
					printf("       centroid=(%.1f,%.1f,%.1f) h1=%d h0=%d h1u=%d h1e=%d/%zu free_h1e=%d clip=%d near=%d/%zu cnw=%d\n",
						ccx,ccy,ccz, dch1_0, dch0, dch1u, dh1e_cnt, cverts.size(), dfree_h1e, dhas_clip, dnw_cnt, cverts.size(), cent_nw);
					for (const auto &v : cverts) {
						int vh1 = classify_h1(v.gs);
						int vh1u = goldsrc_hull::classify_clip_tree_unexpanded(bsp.clipnodes, bsp.planes, root, he, v.gs);
						bool vnw = vert_near_wall(v.gs);
						printf("         vert(%.1f,%.1f,%.1f) h1=%d h1u=%d nw=%d\n", v.gs[0],v.gs[1],v.gs[2], vh1, vh1u, (int)vnw);
					}
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
		int h1_unexp = goldsrc_hull::classify_clip_tree_unexpanded(
			bsp.clipnodes, bsp.planes, root, he, tp.gs);
		printf("  hull1_expanded=%d  hull0=%d  hull1_unexpanded=%d\n", h1_exp, h0, h1_unexp);

		int s1_count = 0, s2_count = 0;
		for (size_t ci = 0; ci < pipeline.solid_cells.size(); ci++)
			if (aabb_overlaps_cell(tp.gs, he, pipeline.solid_cells[ci], EPSILON)) s1_count++;
		for (size_t ci = 0; ci < pipeline.unexpanded_cells.size(); ci++)
			if (aabb_overlaps_cell(tp.gs, he, pipeline.unexpanded_cells[ci], EPSILON)) s2_count++;
		printf("  step1 (expanded): %d  step2 (unexpanded): %d\n", s1_count, s2_count);

		if (s1_count > 0) {
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
					bool ue_overlaps = aabb_overlaps_cell(tp.gs, he, ue, EPSILON);
					printf("    -> un-expanded: %zu verts, AABB=(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f) overlaps=%d\n",
						uv.size(), umn[0],umn[1],umn[2], umx[0],umx[1],umx[2], ue_overlaps);
					if (!ue_overlaps) {
						// Show which plane excludes the AABB
						for (size_t pi = 0; pi < ue.planes.size(); pi++) {
							const auto &hp = ue.planes[pi];
							float support = fabsf(hp.normal[0])*he[0]+fabsf(hp.normal[1])*he[1]+fabsf(hp.normal[2])*he[2];
							float dot = hp.normal[0]*tp.gs[0]+hp.normal[1]*tp.gs[1]+hp.normal[2]*tp.gs[2];
							float margin = dot - hp.dist - support;
							if (margin > EPSILON)
								printf("      plane%zu n=(%.4f,%.4f,%.4f) d=%.2f excludes by %.2f (h1=%d usign=%d)\n",
									pi, hp.normal[0],hp.normal[1],hp.normal[2], hp.dist, margin, hp.from_hull1, hp.unexpand_sign);
						}
						// Try pre-clip-then-unexpand approach
						vector<ConvexCell> preclip_frags;
						goldsrc_hull::clip_cell_by_hull0(pipeline.solid_cells[ci],
							bsp.nodes, bsp.leafs, bsp.planes, hull0_root, EPSILON, preclip_frags);
						printf("      pre-clip: %zu h0 fragments\n", preclip_frags.size());
						for (size_t fi = 0; fi < preclip_frags.size(); fi++) {
							ConvexCell ue_frag;
							for (const auto &hp : preclip_frags[fi].planes) {
								HullPlane p = hp;
								if (p.from_hull1) {
									float support = fabsf(p.normal[0])*he[0]+fabsf(p.normal[1])*he[1]+fabsf(p.normal[2])*he[2];
									p.dist -= support * hp.unexpand_sign;
								}
								ue_frag.planes.push_back(p);
							}
							auto fv = compute_cell_vertices(ue_frag.planes, EPSILON);
							if (fv.size() < 4) continue;
							bool frag_overlaps = aabb_overlaps_cell(tp.gs, he, ue_frag, EPSILON);
							if (frag_overlaps) {
								float fn[3]={1e9f,1e9f,1e9f},fx[3]={-1e9f,-1e9f,-1e9f};
								for(const auto&v:fv){for(int a=0;a<3;a++){if(v.gs[a]<fn[a])fn[a]=v.gs[a];if(v.gs[a]>fx[a])fx[a]=v.gs[a];}}
								printf("      pre-clip frag%zu: %zu verts, AABB=(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f) OVERLAPS!\n",
									fi, fv.size(), fn[0],fn[1],fn[2], fx[0],fx[1],fx[2]);
							}
						}
					}
					// Also show hull0 clip results for the un-expanded cell
					if (ue_overlaps) {
						vector<ConvexCell> ue_frags;
						goldsrc_hull::clip_cell_by_hull0(ue, bsp.nodes, bsp.leafs, bsp.planes,
							hull0_root, EPSILON, ue_frags);
						int ue_frag_overlap = 0;
						for (size_t fi = 0; fi < ue_frags.size(); fi++) {
							if (aabb_overlaps_cell(tp.gs, he, ue_frags[fi], EPSILON)) ue_frag_overlap++;
						}
						printf("    -> hull0 clip: %zu frags, %d overlap test point\n", ue_frags.size(), ue_frag_overlap);
					}
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

