// Standalone test: hull 1 clip brush extraction via un-expand + hull 0 clip
// Build: clang++ -std=c++17 -O2 -I. test_hull_csg.cpp parsers/bsp_parser.cpp bsp_hull.cpp -o test_hull_csg
// Run:   ./test_hull_csg ../../res/maps/ww_golem.bsp

#include "parsers/bsp_parser.h"
#include "bsp_hull.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <chrono>

using namespace std;
using namespace goldsrc_hull;

int main(int argc, char **argv) {
	if (argc < 2) {
		fprintf(stderr, "Usage: %s <bsp_file>\n", argv[0]);
		return 1;
	}

	// Load BSP file
	FILE *f = fopen(argv[1], "rb");
	if (!f) { fprintf(stderr, "Cannot open %s\n", argv[1]); return 1; }
	fseek(f, 0, SEEK_END);
	size_t sz = ftell(f);
	fseek(f, 0, SEEK_SET);
	vector<uint8_t> buf(sz);
	fread(buf.data(), 1, sz, f);
	fclose(f);

	goldsrc::BSPParser parser;
	if (!parser.parse(buf.data(), sz)) {
		fprintf(stderr, "Failed to parse BSP\n");
		return 1;
	}

	const auto &bsp = parser.get_data();
	if (bsp.models.empty()) { fprintf(stderr, "No models\n"); return 1; }

	const auto &bmodel = bsp.models[0];
	int hull_index = 1;
	int root = bmodel.headnode[hull_index];
	int hull0_root = bmodel.headnode[0];
	printf("Hull %d root=%d, Hull 0 root=%d\n", hull_index, root, hull0_root);
	printf("clipnodes=%zu, nodes=%zu, leafs=%zu, planes=%zu\n",
		bsp.clipnodes.size(), bsp.nodes.size(), bsp.leafs.size(), bsp.planes.size());

	const float EPSILON = 0.1f;
	float he[3] = {16, 16, 36}; // Hull 1 half-extents

	// Bounding box planes (tagged from_hull1=false)
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
	vector<ConvexCell> solid_cells;
	walk_clip_tree(bsp.clipnodes, bsp.planes, root, accumulated, solid_cells,
		goldsrc::CONTENTS_SOLID);
	printf("Expanded solid cells: %zu\n", solid_cells.size());

	// Cap with bbox
	for (auto &cell : solid_cells) {
		for (int i = 0; i < 6; i++) {
			HullPlane bp = bbox_planes[i];
			bp.from_hull1 = false;
			cell.planes.push_back(bp);
		}
	}

	// Step 2: Un-expand ALL hull 1 planes
	printf("\n--- Step 2: Un-expand all hull 1 planes ---\n");
	auto t0 = chrono::steady_clock::now();

	// Helper: un-expand hull 1 planes in a cell
	auto unexpand_cell = [&](const ConvexCell &cell) -> ConvexCell {
		ConvexCell ue;
		for (const auto &hp : cell.planes) {
			HullPlane p = hp;
			if (p.from_hull1) {
				float support = fabsf(p.normal[0]) * he[0]
				              + fabsf(p.normal[1]) * he[1]
				              + fabsf(p.normal[2]) * he[2];
				p.dist -= support * hp.unexpand_sign;
			}
			ue.planes.push_back(p);
		}
		return ue;
	};

	vector<ConvexCell> unexpanded_cells;
	size_t step2_degen = 0, step2_rescued = 0;
	for (auto &cell : solid_cells) {
		ConvexCell ue = unexpand_cell(cell);
		auto verts = compute_cell_vertices(ue.planes, EPSILON);
		if (verts.size() >= 4) {
			unexpanded_cells.push_back(std::move(ue));
			continue;
		}
		// Degenerate after un-expansion — try clipping against hull 0 first,
		// then un-expanding each fragment. Hull 0 split planes add non-hull1
		// constraints that help resolve inter-brush contradictions.
		vector<ConvexCell> h0_fragments;
		clip_cell_by_hull0(cell, bsp.nodes, bsp.leafs, bsp.planes,
			hull0_root, EPSILON, h0_fragments);
		bool any_rescued = false;
		for (auto &frag : h0_fragments) {
			ConvexCell uf = unexpand_cell(frag);
			auto fv = compute_cell_vertices(uf.planes, EPSILON);
			if (fv.size() >= 4) {
				unexpanded_cells.push_back(std::move(uf));
				any_rescued = true;
			}
		}
		if (any_rescued) step2_rescued++;
		else step2_degen++;
	}

	auto t1 = chrono::steady_clock::now();
	printf("Un-expanded cells: %zu (degenerate: %zu, rescued: %zu)  (%lldms)\n",
		unexpanded_cells.size(), step2_degen, step2_rescued,
		(long long)chrono::duration_cast<chrono::milliseconds>(t1 - t0).count());

	// Step 3: Clip un-expanded cells against hull 0 EMPTY
	printf("\n--- Step 3: Clip against hull 0 ---\n");
	auto t2 = chrono::steady_clock::now();

	vector<ConvexCell> clipped_cells;
	for (size_t i = 0; i < unexpanded_cells.size(); i++) {
		clip_cell_by_hull0(unexpanded_cells[i], bsp.nodes, bsp.leafs, bsp.planes,
			hull0_root, EPSILON, clipped_cells);
	}

	auto t3 = chrono::steady_clock::now();
	printf("Clipped cells (in hull0-empty): %zu  (%lldms)\n", clipped_cells.size(),
		(long long)chrono::duration_cast<chrono::milliseconds>(t3 - t2).count());

	// Helpers used by steps 4 and 5
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

	// Classify point against hull 1 expanded tree with optional tolerance.
	// Positive tolerance expands the "solid" region (makes it easier to be inside).
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

	// Step 4: Filter — h0 EMPTY + h1/ring combined check
	printf("\n--- Step 4: Vertex filter (h0 EMPTY + h1 SOLID) ---\n");
	auto t4 = chrono::steady_clock::now();

	// Use a tolerance when checking hull 1: vertices barely outside expanded
	// bounds (precision/BSP-splitting artifacts) are OK; vertices far outside
	// indicate growth artifacts.
	vector<ConvexCell> result_cells;
	size_t degenerate = 0, h0_filtered = 0, h1_filtered = 0;
	for (auto &cell : clipped_cells) {
		auto verts = compute_cell_vertices(cell.planes, EPSILON);
		if (verts.size() < 4) { degenerate++; continue; }
		// Compute centroid for nudging boundary vertices
		float cx = 0, cy = 0, cz = 0;
		for (const auto &v : verts) { cx += v.gs[0]; cy += v.gs[1]; cz += v.gs[2]; }
		cx /= verts.size(); cy /= verts.size(); cz /= verts.size();

		bool any_h0_solid = false;
		bool any_h1_empty = false;
		bool has_clip_indicator = false;  // vertex that's h1 SOLID and not near wall
		for (const auto &v : verts) {
			// Nudge vertex slightly toward centroid to avoid boundary precision issues
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
			int h1c = classify_h1(v.gs);
			if (h1c != goldsrc::CONTENTS_SOLID) {
				any_h1_empty = true;
			} else if (!has_clip_indicator && !vert_near_wall(v.gs)) {
				has_clip_indicator = true;
			}
		}
		if (any_h0_solid) { h0_filtered++; continue; }
		// If any vertex is outside hull 1, reject unless the cell has at least
		// one "clip brush indicator" — a vertex that's inside hull 1 AND in open
		// space (not near world geometry). Growth artifacts lack such vertices.
		if (any_h1_empty && !has_clip_indicator) { h1_filtered++; continue; }
		result_cells.push_back(std::move(cell));
	}

	auto t5 = chrono::steady_clock::now();
	printf("After filter: %zu -> %zu result (degen: %zu, h0: %zu, h1: %zu)  (%lldms)\n",
		clipped_cells.size(), result_cells.size(), degenerate, h0_filtered, h1_filtered,
		(long long)chrono::duration_cast<chrono::milliseconds>(t5 - t4).count());

	// Step 5: Expansion ring filter — remove cells where ALL vertices are
	// within hull expansion distance of hull 0 solid (i.e., every vertex's
	// player AABB touches hull 0 solid). These are expansion ring slivers,
	// not real clip brushes.
	printf("\n--- Step 5: Expansion ring filter ---\n");
	auto t6 = chrono::steady_clock::now();

	vector<ConvexCell> final_cells;
	size_t ring_filtered = 0;
	for (auto &cell : result_cells) {
		auto verts = compute_cell_vertices(cell.planes, EPSILON);
		if (verts.size() < 4) continue;
		bool all_near_wall = true;
		for (const auto &v : verts) {
			if (!vert_near_wall(v.gs)) { all_near_wall = false; break; }
		}
		if (all_near_wall) { ring_filtered++; continue; }
		final_cells.push_back(std::move(cell));
	}

	auto t7 = chrono::steady_clock::now();
	printf("After expansion ring filter: %zu -> %zu (removed: %zu)  (%lldms)\n",
		result_cells.size(), final_cells.size(), ring_filtered,
		(long long)chrono::duration_cast<chrono::milliseconds>(t7 - t6).count());

	// --- AABB collision tests ---
	struct TestPoint {
		float gs[3];
		const char *label;
		bool should_be_inside;
	};

	TestPoint test_points[] = {
		// Clip brush test point — AABB must overlap a cell
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
	};

	// AABB-vs-cell: expand each plane by support(normal, he), then point test center.
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

	printf("\n=== COLLISION TESTS ===\n");
	int pass = 0, fail = 0;
	for (const auto &tp : test_points) {
		printf("\n%s  GS(%.0f, %.0f, %.0f):\n", tp.label, tp.gs[0], tp.gs[1], tp.gs[2]);

		if (tp.should_be_inside) {
			int overlaps = 0;
			for (size_t i = 0; i < final_cells.size(); i++) {
				if (aabb_overlaps_cell(tp.gs, he, final_cells[i], EPSILON))
					overlaps++;
			}
			bool ok = overlaps > 0;
			printf("  Player AABB overlaps %d result cells  %s\n", overlaps, ok ? "PASS" : "FAIL");
			if (ok) pass++; else fail++;
		} else {
			int overlaps = 0;
			for (size_t i = 0; i < final_cells.size(); i++) {
				if (aabb_overlaps_cell(tp.gs, he, final_cells[i], EPSILON))
					overlaps++;
			}
			bool ok = overlaps == 0;
			if (overlaps > 0)
				printf("  Player AABB overlaps %d cells  FAIL\n", overlaps);
			else
				printf("  Player AABB clear of all cells  PASS\n");
			if (ok) pass++; else fail++;
		}
	}
	printf("\n=== RESULTS: %d PASS, %d FAIL ===\n", pass, fail);
	printf("Total result cells: %zu\n", final_cells.size());

	// Diagnostics for missing_ points — trace which step kills the cell
	printf("\n=== DIAGNOSTICS ===\n");
	for (const auto &tp : test_points) {
		if (strncmp(tp.label, "missing_", 8) != 0) continue;
		printf("\n%s  GS(%.1f, %.1f, %.1f):\n", tp.label, tp.gs[0], tp.gs[1], tp.gs[2]);

		int h1_exp = classify_h1(tp.gs);
		int h0 = goldsrc_hull::classify_hull0_tree(
			bsp.nodes, bsp.leafs, bsp.planes, hull0_root, tp.gs);
		printf("  hull1_expanded=%d  hull0=%d\n", h1_exp, h0);

		// Count overlaps at each pipeline step
		int s1_count = 0, s2_count = 0;
		for (size_t ci = 0; ci < solid_cells.size(); ci++)
			if (aabb_overlaps_cell(tp.gs, he, solid_cells[ci], EPSILON)) s1_count++;
		for (size_t ci = 0; ci < unexpanded_cells.size(); ci++)
			if (aabb_overlaps_cell(tp.gs, he, unexpanded_cells[ci], EPSILON)) s2_count++;
		printf("  step1 (expanded): %d  step2 (unexpanded): %d\n", s1_count, s2_count);

		// If present in step1 but not step2, trace the expanded cell's un-expansion
		if (s1_count > 0 && s2_count == 0) {
			for (size_t ci = 0; ci < solid_cells.size(); ci++) {
				if (!aabb_overlaps_cell(tp.gs, he, solid_cells[ci], EPSILON)) continue;
				auto ev = compute_cell_vertices(solid_cells[ci].planes, EPSILON);
				float mn[3]={1e9,1e9,1e9}, mx[3]={-1e9,-1e9,-1e9};
				for (const auto &v : ev) {
					for (int a=0;a<3;a++) { if(v.gs[a]<mn[a])mn[a]=v.gs[a]; if(v.gs[a]>mx[a])mx[a]=v.gs[a]; }
				}
				printf("  expanded_cell%zu: %zu verts, AABB=(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n",
					ci, ev.size(), mn[0],mn[1],mn[2], mx[0],mx[1],mx[2]);
				// Un-expand and check
				ConvexCell ue;
				for (const auto &hp : solid_cells[ci].planes) {
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
					printf("    planes: %zu (hull1: ", ue.planes.size());
					int h1_count = 0;
					for (const auto &p : ue.planes) if (p.from_hull1) h1_count++;
					printf("%d)\n", h1_count);
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

		// Find the unexpanded cell(s) containing this point and trace their fate
		for (size_t ci = 0; ci < unexpanded_cells.size(); ci++) {
			if (!aabb_overlaps_cell(tp.gs, he, unexpanded_cells[ci], EPSILON)) continue;

			auto verts = goldsrc_hull::compute_cell_vertices(unexpanded_cells[ci].planes, EPSILON);
			float mn[3] = {1e9,1e9,1e9}, mx[3] = {-1e9,-1e9,-1e9};
			for (const auto &v : verts) {
				for (int a = 0; a < 3; a++) {
					if (v.gs[a] < mn[a]) mn[a] = v.gs[a];
					if (v.gs[a] > mx[a]) mx[a] = v.gs[a];
				}
			}
			printf("  unexpanded_cell%zu: %zu verts, dims=%.1f x %.1f x %.1f\n",
				ci, verts.size(), mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2]);

			// Clip this cell against hull 0 and trace what happens
			vector<ConvexCell> fragments;
			goldsrc_hull::clip_cell_by_hull0(unexpanded_cells[ci],
				bsp.nodes, bsp.leafs, bsp.planes, hull0_root, EPSILON, fragments);
			printf("    hull0 clip -> %zu fragments\n", fragments.size());

			int frag_overlap = 0;
			for (size_t fi = 0; fi < fragments.size(); fi++) {
				if (!aabb_overlaps_cell(tp.gs, he, fragments[fi], EPSILON)) continue;
				frag_overlap++;
				auto fv = goldsrc_hull::compute_cell_vertices(fragments[fi].planes, EPSILON);

				// Step 4 checks: h0 EMPTY + h1 SOLID for all vertices
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
				const char *fate = "SURVIVES";
				if (fv.size() < 4) fate = "killed:degenerate";
				else if (any_h0_solid) fate = "killed:h0_solid_vert";
				else if (any_h1_empty) fate = "killed:h1_empty_vert";
				else if (near_wall_count == (int)fv.size()) fate = "killed:ring_filter";
				printf("    frag%zu: %zu verts, h0s=%d h1e=%d near=%d/%zu -> %s\n",
					fi, fv.size(), any_h0_solid, any_h1_empty,
					near_wall_count, fv.size(), fate);
				// Print each vertex's h1 classification at various tolerances
				for (size_t vi = 0; vi < fv.size(); vi++) {
					int h1_0 = classify_h1(fv[vi].gs, 0);
					int h1_4 = classify_h1(fv[vi].gs, 4);
					int h1_8 = classify_h1(fv[vi].gs, 8);
					int h1_16 = classify_h1(fv[vi].gs, 16);
					bool nw = vert_near_wall(fv[vi].gs);
					printf("      v%zu (%.1f,%.1f,%.1f): h1[0]=%d h1[4]=%d h1[8]=%d h1[16]=%d near=%d\n",
						vi, fv[vi].gs[0], fv[vi].gs[1], fv[vi].gs[2],
						h1_0, h1_4, h1_8, h1_16, nw);
				}
			}
			printf("    overlapping fragments: %d\n", frag_overlap);
		}
	}

	printf("\nDone.\n");
	return 0;
}
