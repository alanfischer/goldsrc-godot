#include "bsp_hull.h"

#include <algorithm>

using std::vector;

namespace goldsrc_hull {

void walk_bsp_tree(
	const vector<goldsrc::BSPNode> &nodes,
	const vector<goldsrc::BSPLeaf> &leafs,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	vector<HullPlane> &accumulated,
	vector<ConvexCell> &out_cells,
	int target_contents) {

	if (node_index < 0) {
		int leaf_index = -(node_index + 1);
		if (leaf_index < 0 || (size_t)leaf_index >= leafs.size()) return;
		if (leafs[leaf_index].contents == target_contents) {
			ConvexCell cell;
			cell.planes = accumulated;
			out_cells.push_back(std::move(cell));
		}
		return;
	}

	if ((size_t)node_index >= nodes.size()) return;

	const auto &node = nodes[node_index];
	if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return;

	const auto &plane = planes[node.planenum];

	HullPlane front_plane;
	front_plane.normal[0] = -plane.normal[0];
	front_plane.normal[1] = -plane.normal[1];
	front_plane.normal[2] = -plane.normal[2];
	front_plane.dist = -plane.dist;
	accumulated.push_back(front_plane);
	walk_bsp_tree(nodes, leafs, planes, node.children[0], accumulated, out_cells, target_contents);
	accumulated.pop_back();

	HullPlane back_plane;
	back_plane.normal[0] = plane.normal[0];
	back_plane.normal[1] = plane.normal[1];
	back_plane.normal[2] = plane.normal[2];
	back_plane.dist = plane.dist;
	accumulated.push_back(back_plane);
	walk_bsp_tree(nodes, leafs, planes, node.children[1], accumulated, out_cells, target_contents);
	accumulated.pop_back();
}

void walk_clip_tree(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	vector<HullPlane> &accumulated,
	vector<ConvexCell> &out_cells,
	int target_contents) {

	// Negative child = contents value directly (not a leaf index)
	if (node_index < 0) {
		if (node_index == target_contents) {
			ConvexCell cell;
			cell.planes = accumulated;
			out_cells.push_back(std::move(cell));
		}
		return;
	}

	if ((size_t)node_index >= clipnodes.size()) return;

	const auto &node = clipnodes[node_index];
	if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return;

	const auto &plane = planes[node.planenum];

	// Front child: behind the plane (negated normal)
	HullPlane front_plane;
	front_plane.normal[0] = -plane.normal[0];
	front_plane.normal[1] = -plane.normal[1];
	front_plane.normal[2] = -plane.normal[2];
	front_plane.dist = -plane.dist;
	front_plane.unexpand_sign = -1;
	accumulated.push_back(front_plane);
	walk_clip_tree(clipnodes, planes, node.children[0], accumulated, out_cells, target_contents);
	accumulated.pop_back();

	// Back child: in front of the plane (positive normal)
	HullPlane back_plane;
	back_plane.normal[0] = plane.normal[0];
	back_plane.normal[1] = plane.normal[1];
	back_plane.normal[2] = plane.normal[2];
	back_plane.dist = plane.dist;
	back_plane.unexpand_sign = 1;
	accumulated.push_back(back_plane);
	walk_clip_tree(clipnodes, planes, node.children[1], accumulated, out_cells, target_contents);
	accumulated.pop_back();
}

void walk_clip_tree_unexpanded(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	float half_extents[3],
	vector<HullPlane> &accumulated,
	vector<ConvexCell> &out_cells,
	int target_contents) {

	if (node_index < 0) {
		if (node_index == target_contents) {
			ConvexCell cell;
			cell.planes = accumulated;
			out_cells.push_back(std::move(cell));
		}
		return;
	}

	if ((size_t)node_index >= clipnodes.size()) return;

	const auto &node = clipnodes[node_index];
	if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return;

	const auto &plane = planes[node.planenum];

	float support = fabsf(plane.normal[0]) * half_extents[0]
	              + fabsf(plane.normal[1]) * half_extents[1]
	              + fabsf(plane.normal[2]) * half_extents[2];

	// Front child: behind the un-expanded plane (negated normal)
	HullPlane front_plane;
	front_plane.normal[0] = -plane.normal[0];
	front_plane.normal[1] = -plane.normal[1];
	front_plane.normal[2] = -plane.normal[2];
	front_plane.dist = -(plane.dist - support);
	accumulated.push_back(front_plane);
	walk_clip_tree_unexpanded(clipnodes, planes, node.children[0], half_extents, accumulated, out_cells, target_contents);
	accumulated.pop_back();

	// Back child: in front of the un-expanded plane (positive normal)
	HullPlane back_plane;
	back_plane.normal[0] = plane.normal[0];
	back_plane.normal[1] = plane.normal[1];
	back_plane.normal[2] = plane.normal[2];
	back_plane.dist = plane.dist - support;
	accumulated.push_back(back_plane);
	walk_clip_tree_unexpanded(clipnodes, planes, node.children[1], half_extents, accumulated, out_cells, target_contents);
	accumulated.pop_back();
}

void walk_clip_tree_nonsolid(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	vector<HullPlane> &accumulated,
	vector<ConvexCell> &out_cells) {

	if (node_index < 0) {
		if (node_index != goldsrc::CONTENTS_SOLID) {
			ConvexCell cell;
			cell.planes = accumulated;
			out_cells.push_back(std::move(cell));
		}
		return;
	}

	if ((size_t)node_index >= clipnodes.size()) return;

	const auto &node = clipnodes[node_index];
	if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return;

	const auto &plane = planes[node.planenum];

	HullPlane front_plane;
	front_plane.normal[0] = -plane.normal[0];
	front_plane.normal[1] = -plane.normal[1];
	front_plane.normal[2] = -plane.normal[2];
	front_plane.dist = -plane.dist;
	accumulated.push_back(front_plane);
	walk_clip_tree_nonsolid(clipnodes, planes, node.children[0], accumulated, out_cells);
	accumulated.pop_back();

	HullPlane back_plane;
	back_plane.normal[0] = plane.normal[0];
	back_plane.normal[1] = plane.normal[1];
	back_plane.normal[2] = plane.normal[2];
	back_plane.dist = plane.dist;
	accumulated.push_back(back_plane);
	walk_clip_tree_nonsolid(clipnodes, planes, node.children[1], accumulated, out_cells);
	accumulated.pop_back();
}

// Check if an entire clip subtree is all-solid (every leaf = CONTENTS_SOLID).
static bool is_all_solid(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	int node_index) {
	if (node_index < 0)
		return (node_index == goldsrc::CONTENTS_SOLID);
	if ((size_t)node_index >= clipnodes.size()) return false;
	const auto &node = clipnodes[node_index];
	return is_all_solid(clipnodes, node.children[0])
	    && is_all_solid(clipnodes, node.children[1]);
}

// Walk clip tree, merging adjacent solid cells by skipping internal splits.
// When both children of a node lead to all-solid subtrees, we don't push
// the splitting plane — we just continue to either child (same result).
static void walk_clip_tree_merged_impl(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	vector<HullPlane> &accumulated,
	vector<ConvexCell> &out_cells,
	int target_contents) {

	if (node_index < 0) {
		if (node_index == target_contents) {
			ConvexCell cell;
			cell.planes = accumulated;
			out_cells.push_back(std::move(cell));
		}
		return;
	}

	if ((size_t)node_index >= clipnodes.size()) return;
	const auto &node = clipnodes[node_index];
	if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return;

	// If both children are all-solid, skip the split plane entirely.
	// Just emit one cell from the accumulated planes.
	bool front_all_solid = is_all_solid(clipnodes, node.children[0]);
	bool back_all_solid = is_all_solid(clipnodes, node.children[1]);

	if (front_all_solid && back_all_solid) {
		// Both sides solid — this plane is internal, don't add it
		// Just recurse into either side (they'll emit the cell)
		// We recurse into front child since it'll reach a solid leaf
		walk_clip_tree_merged_impl(clipnodes, planes, node.children[0],
			accumulated, out_cells, target_contents);
		return;
	}

	const auto &plane = planes[node.planenum];

	// Front child: behind the plane (negated normal)
	HullPlane front_plane;
	front_plane.normal[0] = -plane.normal[0];
	front_plane.normal[1] = -plane.normal[1];
	front_plane.normal[2] = -plane.normal[2];
	front_plane.dist = -plane.dist;
	front_plane.unexpand_sign = -1;
	accumulated.push_back(front_plane);
	walk_clip_tree_merged_impl(clipnodes, planes, node.children[0],
		accumulated, out_cells, target_contents);
	accumulated.pop_back();

	// Back child: in front of the plane (positive normal)
	HullPlane back_plane;
	back_plane.normal[0] = plane.normal[0];
	back_plane.normal[1] = plane.normal[1];
	back_plane.normal[2] = plane.normal[2];
	back_plane.dist = plane.dist;
	back_plane.unexpand_sign = 1;
	accumulated.push_back(back_plane);
	walk_clip_tree_merged_impl(clipnodes, planes, node.children[1],
		accumulated, out_cells, target_contents);
	accumulated.pop_back();
}

void walk_clip_tree_merged(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	vector<HullPlane> &accumulated,
	vector<ConvexCell> &out_cells,
	int target_contents) {

	walk_clip_tree_merged_impl(clipnodes, planes, node_index,
		accumulated, out_cells, target_contents);
}

void clip_cell_by_hull0(
	const ConvexCell &cell,
	const vector<goldsrc::BSPNode> &nodes,
	const vector<goldsrc::BSPLeaf> &leafs,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	float epsilon,
	vector<ConvexCell> &out_cells) {

	if (node_index < 0) {
		int leaf_index = -(node_index + 1);
		if (leaf_index < 0 || (size_t)leaf_index >= leafs.size()) return;
		if (leafs[leaf_index].contents != goldsrc::CONTENTS_SOLID) {
			out_cells.push_back(cell);
		}
		return;
	}

	if ((size_t)node_index >= nodes.size()) return;
	const auto &node = nodes[node_index];
	if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return;
	const auto &plane = planes[node.planenum];

	auto verts = compute_cell_vertices(cell.planes, epsilon);
	if (verts.empty()) return;

	float min_dot = 1e30f, max_dot = -1e30f;
	for (const auto &v : verts) {
		float dot = plane.normal[0] * v.gs[0]
		          + plane.normal[1] * v.gs[1]
		          + plane.normal[2] * v.gs[2];
		if (dot < min_dot) min_dot = dot;
		if (dot > max_dot) max_dot = dot;
	}

	if (min_dot >= plane.dist - epsilon) {
		clip_cell_by_hull0(cell, nodes, leafs, planes, node.children[0], epsilon, out_cells);
		return;
	}
	if (max_dot <= plane.dist + epsilon) {
		clip_cell_by_hull0(cell, nodes, leafs, planes, node.children[1], epsilon, out_cells);
		return;
	}

	// Straddling — split. Tag split planes as from_hull1=false.
	ConvexCell front_cell = cell;
	HullPlane fc;
	fc.normal[0] = -plane.normal[0];
	fc.normal[1] = -plane.normal[1];
	fc.normal[2] = -plane.normal[2];
	fc.dist = -plane.dist;
	fc.from_hull1 = false;
	front_cell.planes.push_back(fc);
	clip_cell_by_hull0(front_cell, nodes, leafs, planes, node.children[0], epsilon, out_cells);

	ConvexCell back_cell = cell;
	HullPlane bc;
	bc.normal[0] = plane.normal[0];
	bc.normal[1] = plane.normal[1];
	bc.normal[2] = plane.normal[2];
	bc.dist = plane.dist;
	bc.from_hull1 = false;
	back_cell.planes.push_back(bc);
	clip_cell_by_hull0(back_cell, nodes, leafs, planes, node.children[1], epsilon, out_cells);
}

bool cell_touches_playable(
	const ConvexCell &cell,
	const vector<goldsrc::BSPNode> &nodes,
	const vector<goldsrc::BSPLeaf> &leafs,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	float epsilon) {

	if (node_index < 0) {
		int leaf_index = -(node_index + 1);
		if (leaf_index < 0 || (size_t)leaf_index >= leafs.size()) return false;
		int c = leafs[leaf_index].contents;
		// Playable: any content the player can physically occupy.
		// Excludes SOLID (already clipped away upstream) and SKY (unreachable).
		// Includes EMPTY, WATER, SLIME, LAVA — preserving clip brushes in liquid volumes.
		return c != goldsrc::CONTENTS_SOLID && c != goldsrc::CONTENTS_SKY;
	}

	if ((size_t)node_index >= nodes.size()) return false;
	const auto &node = nodes[node_index];
	if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return false;
	const auto &plane = planes[node.planenum];

	auto verts = compute_cell_vertices(cell.planes, epsilon);
	if (verts.empty()) return false;

	float min_dot = 1e30f, max_dot = -1e30f;
	for (const auto &v : verts) {
		float dot = plane.normal[0] * v.gs[0]
		          + plane.normal[1] * v.gs[1]
		          + plane.normal[2] * v.gs[2];
		if (dot < min_dot) min_dot = dot;
		if (dot > max_dot) max_dot = dot;
	}

	if (min_dot >= plane.dist - epsilon)
		return cell_touches_playable(cell, nodes, leafs, planes, node.children[0], epsilon);
	if (max_dot <= plane.dist + epsilon)
		return cell_touches_playable(cell, nodes, leafs, planes, node.children[1], epsilon);

	// Straddling — recurse both halves; short-circuit on first EMPTY hit.
	ConvexCell front_cell = cell;
	HullPlane fc;
	fc.normal[0] = -plane.normal[0]; fc.normal[1] = -plane.normal[1]; fc.normal[2] = -plane.normal[2];
	fc.dist = -plane.dist; fc.from_hull1 = false;
	front_cell.planes.push_back(fc);
	if (cell_touches_playable(front_cell, nodes, leafs, planes, node.children[0], epsilon))
		return true;

	ConvexCell back_cell = cell;
	HullPlane bc;
	bc.normal[0] = plane.normal[0]; bc.normal[1] = plane.normal[1]; bc.normal[2] = plane.normal[2];
	bc.dist = plane.dist; bc.from_hull1 = false;
	back_cell.planes.push_back(bc);
	return cell_touches_playable(back_cell, nodes, leafs, planes, node.children[1], epsilon);
}

void clip_cell_by_clip_tree(
	const ConvexCell &cell,
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	float epsilon,
	vector<ConvexCell> &out_cells,
	int keep_contents) {

	// Leaf: negative child = contents directly
	if (node_index < 0) {
		if (node_index == keep_contents) {
			out_cells.push_back(cell);
		}
		return;
	}

	if ((size_t)node_index >= clipnodes.size()) return;
	const auto &node = clipnodes[node_index];
	if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return;
	const auto &plane = planes[node.planenum];

	auto verts = compute_cell_vertices(cell.planes, epsilon);
	if (verts.empty()) return;

	float min_dot = 1e30f, max_dot = -1e30f;
	for (const auto &v : verts) {
		float dot = plane.normal[0] * v.gs[0]
		          + plane.normal[1] * v.gs[1]
		          + plane.normal[2] * v.gs[2];
		if (dot < min_dot) min_dot = dot;
		if (dot > max_dot) max_dot = dot;
	}

	if (min_dot >= plane.dist - epsilon) {
		clip_cell_by_clip_tree(cell, clipnodes, planes, node.children[0], epsilon, out_cells, keep_contents);
		return;
	}
	if (max_dot <= plane.dist + epsilon) {
		clip_cell_by_clip_tree(cell, clipnodes, planes, node.children[1], epsilon, out_cells, keep_contents);
		return;
	}

	// Straddling — split. Tag split planes as from_hull1=false.
	ConvexCell front_cell = cell;
	HullPlane fc;
	fc.normal[0] = -plane.normal[0];
	fc.normal[1] = -plane.normal[1];
	fc.normal[2] = -plane.normal[2];
	fc.dist = -plane.dist;
	fc.from_hull1 = false;
	front_cell.planes.push_back(fc);
	clip_cell_by_clip_tree(front_cell, clipnodes, planes, node.children[0], epsilon, out_cells, keep_contents);

	ConvexCell back_cell = cell;
	HullPlane bc;
	bc.normal[0] = plane.normal[0];
	bc.normal[1] = plane.normal[1];
	bc.normal[2] = plane.normal[2];
	bc.dist = plane.dist;
	bc.from_hull1 = false;
	back_cell.planes.push_back(bc);
	clip_cell_by_clip_tree(back_cell, clipnodes, planes, node.children[1], epsilon, out_cells, keep_contents);
}

// Classify a point against hull 0 BSP tree. Returns leaf contents.
static int classify_hull0(
	const vector<goldsrc::BSPNode> &nodes,
	const vector<goldsrc::BSPLeaf> &leafs,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	const float p[3]) {

	while (node_index >= 0) {
		if ((size_t)node_index >= nodes.size()) return 0;
		const auto &node = nodes[node_index];
		if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return 0;
		const auto &plane = planes[node.planenum];
		float dot = plane.normal[0] * p[0] + plane.normal[1] * p[1] + plane.normal[2] * p[2];
		if (dot >= plane.dist)
			node_index = node.children[0];
		else
			node_index = node.children[1];
	}
	int leaf_index = -(node_index + 1);
	if (leaf_index < 0 || (size_t)leaf_index >= leafs.size()) return 0;
	return leafs[leaf_index].contents;
}


// Classify a point against the hull 1 clip BSP tree with UN-EXPANDED planes.
// Points in the expansion ring (between hull 0 surface and hull 1 expanded surface)
// will classify as EMPTY here, while points in actual brush bodies remain SOLID.
static int classify_hull1_unexpanded(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	const float half_extents[3],
	const float p[3]) {

	while (node_index >= 0) {
		if ((size_t)node_index >= clipnodes.size()) return 0;
		const auto &node = clipnodes[node_index];
		if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return 0;
		const auto &plane = planes[node.planenum];
		float support = fabsf(plane.normal[0]) * half_extents[0]
		              + fabsf(plane.normal[1]) * half_extents[1]
		              + fabsf(plane.normal[2]) * half_extents[2];
		float dot = plane.normal[0] * p[0] + plane.normal[1] * p[1] + plane.normal[2] * p[2];
		if (dot >= plane.dist - support)
			node_index = node.children[0];
		else
			node_index = node.children[1];
	}
	return node_index;
}

int classify_hull0_tree(
	const vector<goldsrc::BSPNode> &nodes,
	const vector<goldsrc::BSPLeaf> &leafs,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	const float p[3]) {
	return classify_hull0(nodes, leafs, planes, node_index, p);
}

int classify_clip_tree_unexpanded(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	const float half_extents[3],
	const float p[3]) {
	return classify_hull1_unexpanded(clipnodes, planes, node_index, half_extents, p);
}

vector<ConvexCell> unexpand_hull1_planes(
	const vector<ConvexCell> &cells,
	float half_extents[3],
	float epsilon,
	const vector<goldsrc::BSPNode> &nodes,
	const vector<goldsrc::BSPLeaf> &leafs,
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &bsp_planes,
	int hull0_root,
	int hull1_root) {

	// Un-expand ALL hull 1 planes by subtracting the Minkowski support.
	// World brush planes collapse back to hull 0 surface positions.
	// Clip brush planes collapse to their original (pre-expansion) positions.
	// Degenerate cells (< 4 verts after un-expansion) are pruned.
	vector<ConvexCell> result;
	for (const auto &cell : cells) {
		ConvexCell ue;
		for (const auto &hp : cell.planes) {
			HullPlane p = hp;
			if (p.from_hull1) {
				float support = fabsf(p.normal[0]) * half_extents[0]
				              + fabsf(p.normal[1]) * half_extents[1]
				              + fabsf(p.normal[2]) * half_extents[2];
				p.dist -= support * hp.unexpand_sign;
			}
			ue.planes.push_back(p);
		}
		auto verts = compute_cell_vertices(ue.planes, epsilon);
		if (verts.size() < 4) continue;
		result.push_back(std::move(ue));
	}
	return result;
}

vector<CellVertex> compute_cell_vertices(
	const vector<HullPlane> &planes, float epsilon) {

	vector<CellVertex> verts;
	int np = (int)planes.size();

	// Use double precision for Cramer's rule to handle near-parallel planes
	// (e.g. thin wedge cells from BSP splitting where two planes are nearly
	// anti-parallel and the determinant is very small in float).
	for (int i = 0; i < np - 2; i++) {
		for (int j = i + 1; j < np - 1; j++) {
			for (int k = j + 1; k < np; k++) {
				const auto &p1 = planes[i];
				const auto &p2 = planes[j];
				const auto &p3 = planes[k];

				double n1[3] = {p1.normal[0], p1.normal[1], p1.normal[2]};
				double n2[3] = {p2.normal[0], p2.normal[1], p2.normal[2]};
				double n3[3] = {p3.normal[0], p3.normal[1], p3.normal[2]};
				double d1 = p1.dist, d2 = p2.dist, d3 = p3.dist;

				double cx = n2[1]*n3[2] - n2[2]*n3[1];
				double cy = n2[2]*n3[0] - n2[0]*n3[2];
				double cz = n2[0]*n3[1] - n2[1]*n3[0];

				double denom = n1[0]*cx + n1[1]*cy + n1[2]*cz;
				if (fabs(denom) < 1e-10) continue;

				double ax = n3[1]*n1[2] - n3[2]*n1[1];
				double ay = n3[2]*n1[0] - n3[0]*n1[2];
				double az = n3[0]*n1[1] - n3[1]*n1[0];

				double bx = n1[1]*n2[2] - n1[2]*n2[1];
				double by = n1[2]*n2[0] - n1[0]*n2[2];
				double bz = n1[0]*n2[1] - n1[1]*n2[0];

				double inv_denom = 1.0 / denom;
				float px = (float)((d1*cx + d2*ax + d3*bx) * inv_denom);
				float py = (float)((d1*cy + d2*ay + d3*by) * inv_denom);
				float pz = (float)((d1*cz + d2*az + d3*bz) * inv_denom);

				bool inside = true;
				for (int m = 0; m < np; m++) {
					if (m == i || m == j || m == k) continue;
					const auto &pp = planes[m];
					float dot = pp.normal[0] * px + pp.normal[1] * py + pp.normal[2] * pz;
					if (dot > pp.dist + epsilon) {
						inside = false;
						break;
					}
				}
				if (!inside) continue;

				bool duplicate = false;
				for (const auto &ev : verts) {
					float dx = ev.gs[0] - px, dy = ev.gs[1] - py, dz = ev.gs[2] - pz;
					if (sqrtf(dx * dx + dy * dy + dz * dz) < epsilon) {
						duplicate = true;
						break;
					}
				}
				if (!duplicate) {
					CellVertex cv;
					cv.gs[0] = px; cv.gs[1] = py; cv.gs[2] = pz;
					verts.push_back(cv);
				}
			}
		}
	}
	return verts;
}

bool compute_cell_aabb(const vector<HullPlane> &planes, float epsilon, CellAABB &out) {
	auto verts = compute_cell_vertices(planes, epsilon);
	if (verts.size() < 4) return false;

	out.mins[0] = out.maxs[0] = verts[0].gs[0];
	out.mins[1] = out.maxs[1] = verts[0].gs[1];
	out.mins[2] = out.maxs[2] = verts[0].gs[2];

	for (size_t i = 1; i < verts.size(); i++) {
		for (int a = 0; a < 3; a++) {
			if (verts[i].gs[a] < out.mins[a]) out.mins[a] = verts[i].gs[a];
			if (verts[i].gs[a] > out.maxs[a]) out.maxs[a] = verts[i].gs[a];
		}
	}
	return true;
}

bool aabb_overlap(const CellAABB &a, const CellAABB &b) {
	for (int i = 0; i < 3; i++) {
		if (a.maxs[i] < b.mins[i] || b.maxs[i] < a.mins[i]) return false;
	}
	return true;
}

// Remove planes that no vertex sits on (redundant constraints).
static ConvexCell prune_planes(const ConvexCell &cell, const vector<CellVertex> &verts, float epsilon) {
	ConvexCell pruned;
	for (const auto &hp : cell.planes) {
		bool contributes = false;
		for (const auto &v : verts) {
			float dot = hp.normal[0] * v.gs[0]
			          + hp.normal[1] * v.gs[1]
			          + hp.normal[2] * v.gs[2];
			if (fabsf(dot - hp.dist) < epsilon * 2.0f) {
				contributes = true;
				break;
			}
		}
		if (contributes) {
			pruned.planes.push_back(hp);
		}
	}
	return pruned;
}

vector<ConvexCell> subtract_convex(const ConvexCell &a, const ConvexCell &b, float epsilon) {
	// A - B = fragments of A that are outside at least one plane of B.
	// Sequential decomposition: for each plane of B, peel off the part of
	// the remaining volume that's OUTSIDE that plane (definitely not in B),
	// then restrict the remainder to INSIDE that plane (might still be in B).
	// What's left at the end is A ∩ B (discarded).

	vector<ConvexCell> fragments;
	ConvexCell remaining = a;

	for (const auto &bp : b.planes) {
		// Outside this plane of B: flip the constraint
		ConvexCell outside = remaining;
		HullPlane flipped;
		flipped.normal[0] = -bp.normal[0];
		flipped.normal[1] = -bp.normal[1];
		flipped.normal[2] = -bp.normal[2];
		flipped.dist = -bp.dist;
		outside.planes.push_back(flipped);

		// Check if this fragment is non-empty, prune redundant planes
		auto verts = compute_cell_vertices(outside.planes, epsilon);
		if (verts.size() >= 4) {
			fragments.push_back(prune_planes(outside, verts, epsilon));
		}

		// Remaining: inside this plane of B
		remaining.planes.push_back(bp);

		// Early out: if remaining is empty, no more fragments possible
		auto rem_verts = compute_cell_vertices(remaining.planes, epsilon);
		if (rem_verts.size() < 4) break;
		remaining = prune_planes(remaining, rem_verts, epsilon);
	}

	return fragments;
}

int classify_clip_hull(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int root, const float point[3], float tolerance) {

	int idx = root;
	while (idx >= 0) {
		if ((size_t)idx >= clipnodes.size()) return 0;
		const auto &node = clipnodes[idx];
		if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return 0;
		const auto &plane = planes[node.planenum];
		float dot = plane.normal[0]*point[0] + plane.normal[1]*point[1] + plane.normal[2]*point[2];
		idx = (dot >= plane.dist - tolerance) ? node.children[0] : node.children[1];
	}
	return idx;
}

bool vert_near_wall(
	const vector<goldsrc::BSPNode> &nodes,
	const vector<goldsrc::BSPLeaf> &leafs,
	const vector<goldsrc::BSPPlane> &planes,
	int hull0_root, const float v[3], const float he[3]) {

	for (int sx = -1; sx <= 1; sx += 2) {
		for (int sy = -1; sy <= 1; sy += 2) {
			for (int sz = -1; sz <= 1; sz += 2) {
				float p[3] = {v[0]+sx*he[0], v[1]+sy*he[1], v[2]+sz*he[2]};
				int c = classify_hull0_tree(nodes, leafs, planes, hull0_root, p);
				if (c == goldsrc::CONTENTS_SOLID) return true;
			}
		}
	}
	return false;
}

vector<ConvexCell> filter_clip_brush_cells(
	vector<ConvexCell> &&clipped_cells,
	const vector<goldsrc::BSPNode> &nodes,
	const vector<goldsrc::BSPLeaf> &leafs,
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int hull0_root, int hull1_root,
	const float he[3], float epsilon) {

	float max_he = fmaxf(he[0], fmaxf(he[1], he[2]));

	// Helper: check if a near-wall cell can be rescued as a real clip brush.
	// Requires a non-near-wall vertex that is h1 SOLID (proving clip brush
	// overlap), the cell is big on 2+ axes, and the largest dim >= 256.
	auto has_large_clip_rescue = [&](const vector<CellVertex> &verts,
		const float rdim[3], int big_axes) -> bool {
		bool has_nnw_solid = false;
		for (const auto &v : verts) {
			if (!vert_near_wall(nodes, leafs, planes, hull0_root, v.gs, he) &&
				classify_clip_hull(clipnodes, planes, hull1_root, v.gs) == goldsrc::CONTENTS_SOLID) {
				has_nnw_solid = true; break;
			}
		}
		float sorted[3] = {rdim[0], rdim[1], rdim[2]};
		if (sorted[0] < sorted[1]) std::swap(sorted[0], sorted[1]);
		if (sorted[1] < sorted[2]) std::swap(sorted[1], sorted[2]);
		if (sorted[0] < sorted[1]) std::swap(sorted[0], sorted[1]);
		return big_axes >= 2 && has_nnw_solid && sorted[0] >= 256.0f;
	};

	// ========================================================================
	// STAGE 1: Vertex filter
	//
	// Each un-expanded cell was clipped against hull 0 EMPTY in the previous
	// pipeline step, so every cell should lie in open space. However, the
	// Minkowski un-expansion creates two kinds of artifacts:
	//   (a) World brush remnants — cells from the expansion ring around world
	//       geometry that didn't fully collapse during un-expansion.
	//   (b) Growth artifacts — cells that grew beyond the original clip brush
	//       boundary because un-expansion pushed planes outward on some faces.
	//
	// This stage uses three complementary checks per cell:
	//   1. h0 solid vertex check — are any vertices actually inside world geo?
	//   2. h1 classification — do vertices/centroid classify as SOLID in the
	//      original expanded clip hull? (real clip brush cells should)
	//   3. Size gates — are cell dimensions physically plausible for an
	//      un-expanded brush?
	// ========================================================================
	vector<ConvexCell> result_cells;
	for (auto &cell : clipped_cells) {
		auto verts = compute_cell_vertices(cell.planes, epsilon);
		if (verts.size() < 4) continue;
		float cx = 0, cy = 0, cz = 0;
		for (const auto &v : verts) { cx += v.gs[0]; cy += v.gs[1]; cz += v.gs[2]; }
		cx /= verts.size(); cy /= verts.size(); cz /= verts.size();

		float cpt[3] = {cx, cy, cz};

		// --- 1a. h0 vertex check ---
		// Triple-plane intersection can land vertices exactly on the hull 0
		// boundary (BSP split plane). Nudge each vertex slightly toward the
		// centroid before classifying, so boundary vertices don't false-positive
		// as h0 SOLID.
		bool any_h0_solid = false;
		for (const auto &v : verts) {
			float nudged[3] = {v.gs[0], v.gs[1], v.gs[2]};
			float dx = cx - v.gs[0], dy = cy - v.gs[1], dz = cz - v.gs[2];
			float len = sqrtf(dx*dx + dy*dy + dz*dz);
			if (len > 0.01f) {
				float t = 0.1f / len;
				nudged[0] += dx * t; nudged[1] += dy * t; nudged[2] += dz * t;
			}
			int h0c = classify_hull0_tree(nodes, leafs, planes, hull0_root, nudged);
			if (h0c == goldsrc::CONTENTS_SOLID) { any_h0_solid = true; break; }
		}
		if (any_h0_solid) {
			// Some vertex is in world geometry. Filter unless the centroid proves
			// this is a real clip brush embedded in a wall: centroid must be h1
			// SOLID and not near any wall surface.
			int h0_cent = classify_hull0_tree(nodes, leafs, planes, hull0_root, cpt);
			if (h0_cent == goldsrc::CONTENTS_SOLID) {
				int h1_cent = classify_clip_hull(clipnodes, planes, hull1_root, cpt);
				if (h1_cent != goldsrc::CONTENTS_SOLID ||
					vert_near_wall(nodes, leafs, planes, hull0_root, cpt, he)) continue;
			}
		}

		// --- 1b. Per-vertex h1 classification ---
		// A "clip indicator" is a vertex that is h1 SOLID *and* not near a wall.
		// This is strong evidence the cell overlaps a real clip brush, not just
		// the expansion ring around world geometry.
		bool any_h1_empty = false;
		bool has_clip_indicator = false;
		int nw_count = 0;
		int h1_empty_count = 0;
		for (const auto &v : verts) {
			int h1c = classify_clip_hull(clipnodes, planes, hull1_root, v.gs);
			bool nw = vert_near_wall(nodes, leafs, planes, hull0_root, v.gs, he);
			if (nw) nw_count++;
			if (h1c != goldsrc::CONTENTS_SOLID) {
				any_h1_empty = true;
				h1_empty_count++;
			} else if (!has_clip_indicator && !nw) {
				has_clip_indicator = true;
			}
		}

		// --- 1c. Cell dimension analysis ---
		// big_per_he:   each dim >= he[i] (minimum for a non-degenerate brush)
		// big_per_axis: each dim >= 2*he[i] (full expansion width removed)
		// big_global:   smallest dim >= 2*max(he) (uniformly large)
		float mn[3]={1e9f,1e9f,1e9f}, mx[3]={-1e9f,-1e9f,-1e9f};
		for (const auto &v : verts) { for(int a=0;a<3;a++){if(v.gs[a]<mn[a])mn[a]=v.gs[a]; if(v.gs[a]>mx[a])mx[a]=v.gs[a];}}
		float dim[3] = {mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2]};
		float min_dim = fminf(dim[0], fminf(dim[1], dim[2]));
		bool big_per_axis = (dim[0] >= 2.0f*he[0]) &&
		                    (dim[1] >= 2.0f*he[1]) &&
		                    (dim[2] >= 2.0f*he[2]);
		bool big_global = min_dim >= 2.0f * max_he;
		bool big_per_he = (dim[0] >= he[0]) && (dim[1] >= he[1]) && (dim[2] >= he[2]);

		// --- 1d. No clip indicator path ---
		// All h1 SOLID verts are near walls, so no direct evidence of a clip
		// brush. The cell might still be a real clip brush if the centroid is
		// h1 SOLID, not near wall, big enough, and centroid h0 non-solid.
		// Secondary rescue: centroid barely outside h1 boundary (within 8 units
		// tolerance) + uniformly large cell = likely a ceiling clip brush whose
		// centroid straddles the h1 boundary.
		if (any_h1_empty && !has_clip_indicator) {
			int ch1 = classify_clip_hull(clipnodes, planes, hull1_root, cpt);
			if (ch1 == goldsrc::CONTENTS_SOLID &&
				!vert_near_wall(nodes, leafs, planes, hull0_root, cpt, he)) {
				if (big_per_he) {
					int ch0 = classify_hull0_tree(nodes, leafs, planes, hull0_root, cpt);
					if (ch0 != goldsrc::CONTENTS_SOLID) {
						result_cells.push_back(std::move(cell));
					}
				}
			} else {
				if (ch1 != goldsrc::CONTENTS_SOLID) {
					int ch1_tol = classify_clip_hull(clipnodes, planes, hull1_root, cpt, 8.0f);
					if (ch1_tol == goldsrc::CONTENTS_SOLID && big_global) {
						result_cells.push_back(std::move(cell));
					}
				}
			}
			continue;
		}

		// --- 1e. Size gate filters (cell has clip indicator or all h1 SOLID) ---
		// Impossibly thin: any dim < half the half-extent with 6+ verts means
		// the cell is a sliver from BSP splitting, not a real brush face.
		bool impossibly_thin = verts.size() >= 6 && ((dim[0] < he[0]*0.5f) || (dim[1] < he[1]*0.5f) || (dim[2] < he[2]*0.5f));
		if (impossibly_thin) continue;
		// Degenerate: 2+ dims smaller than he and max dim too small to be real
		{	int degen_count = 0;
			float max_dim_val = fmaxf(dim[0], fmaxf(dim[1], dim[2]));
			for (int a = 0; a < 3; a++) { if (dim[a] < he[a]) degen_count++; }
			if (degen_count >= 2 && max_dim_val < 2.0f * max_he) continue;
		}
		// Small-axis at expansion boundary: a dim of exactly 2*he on a short
		// axis (he < max_he) means un-expansion collapsed it to ~0. This is a
		// telltale expansion artifact on non-cubic hulls (e.g. hull 1: 16x16x36).
		{
			bool small_axis_narrow = false;
			for (int a = 0; a < 3; a++) {
				if (he[a] < max_he && dim[a] >= 2.0f * he[a] - 1.0f && dim[a] <= 2.0f * he[a] + 1.0f) {
					small_axis_narrow = true; break;
				}
			}
			if (small_axis_narrow) {
				if (any_h1_empty) continue;  // strong artifact signal
				// All verts h1 SOLID but at expansion boundary: only keep if
				// centroid is also h1 SOLID (real clip brush). If centroid is
				// h1 EMPTY, the cell spans across the h1 boundary — artifact.
				int cent_h1 = classify_clip_hull(clipnodes, planes, hull1_root, cpt);
				if (cent_h1 != goldsrc::CONTENTS_SOLID) continue;
			}
		}

		// --- 1f. Tiered size checks ---
		// Small cells (any dim < he): need centroid h1 SOLID and no h1 EMPTY
		//   verts. Complex small cells (6+ verts) near walls are artifacts.
		// Medium cells (some dim < 2*he): 2+ sub-expansion axes + any h1 EMPTY
		//   = expansion remnant. Otherwise need centroid h1 SOLID + majority.
		// Big cells: just need centroid h1 SOLID + majority h1 SOLID.
		if (!big_per_he) {
			int cent_h1 = classify_clip_hull(clipnodes, planes, hull1_root, cpt);
			if (any_h1_empty || cent_h1 != goldsrc::CONTENTS_SOLID) continue;
			if (verts.size() >= 6 && nw_count > 0) {
				// Rescue: cell is big on 2+ axes with max dim >= 256 and
				// centroid h0 non-solid. Thin clip brushes (shorter than
				// player height) near walls are real if all h1 SOLID.
				int ba1f = 0;
				float md1f = fmaxf(dim[0], fmaxf(dim[1], dim[2]));
				for (int a = 0; a < 3; a++) { if (dim[a] >= 2.0f * he[a]) ba1f++; }
				int ch0_1f = classify_hull0_tree(nodes, leafs, planes, hull0_root, cpt);
				if (!(ba1f >= 2 && md1f >= 256.0f && ch0_1f != goldsrc::CONTENTS_SOLID)) continue;
			}
		} else if (!big_per_axis) {
			int sub_axes = 0;
			for (int a = 0; a < 3; a++) { if (dim[a] < 2.0f * he[a]) sub_axes++; }
			if (sub_axes >= 2 && any_h1_empty) continue;
			int cent_h1 = classify_clip_hull(clipnodes, planes, hull1_root, cpt);
			if (cent_h1 != goldsrc::CONTENTS_SOLID ||
				h1_empty_count >= (int)verts.size()/2) continue;
		} else if (any_h1_empty) {
			int cent_h1 = classify_clip_hull(clipnodes, planes, hull1_root, cpt);
			if (cent_h1 != goldsrc::CONTENTS_SOLID ||
				h1_empty_count >= (int)verts.size()/2) continue;
		}

		result_cells.push_back(std::move(cell));
	}

	// ========================================================================
	// STAGE 2: Expansion ring filter
	//
	// The Minkowski expansion of world brushes creates a "ring" of solid space
	// around walls/floors/ceilings in the clip hull. After un-expansion, some
	// ring fragments survive stage 1 because they have valid dimensions and
	// h1 SOLID vertices. This stage catches them by checking whether the cell
	// sits at a wall boundary (all or most vertices are "near wall" — i.e.,
	// an AABB corner is in hull 0 SOLID).
	//
	// Each sub-check has rescue paths for real clip brushes that happen to sit
	// near walls (e.g., a railing clip brush flush against a wall).
	// ========================================================================
	vector<ConvexCell> final_cells;
	for (auto &cell : result_cells) {
		auto verts = compute_cell_vertices(cell.planes, epsilon);
		if (verts.size() < 4) continue;
		bool all_near_wall = true;
		int nw_count = 0;
		for (const auto &v : verts) {
			if (vert_near_wall(nodes, leafs, planes, hull0_root, v.gs, he)) nw_count++; else all_near_wall = false;
		}

		// Compute AABB and centroid (shared by all stage 2 checks)
		float rn[3]={1e9f,1e9f,1e9f}, rx[3]={-1e9f,-1e9f,-1e9f};
		for (const auto &v : verts) { for(int a=0;a<3;a++){if(v.gs[a]<rn[a])rn[a]=v.gs[a]; if(v.gs[a]>rx[a])rx[a]=v.gs[a];}}
		float rdim[3] = {rx[0]-rn[0], rx[1]-rn[1], rx[2]-rn[2]};
		bool r_big = (rdim[0] >= 2.0f*he[0]) && (rdim[1] >= 2.0f*he[1]) && (rdim[2] >= 2.0f*he[2]);
		float rcpt[3] = {0, 0, 0};
		for (const auto &v : verts) { rcpt[0]+=v.gs[0]; rcpt[1]+=v.gs[1]; rcpt[2]+=v.gs[2]; }
		rcpt[0]/=verts.size(); rcpt[1]/=verts.size(); rcpt[2]/=verts.size();

		// --- 2pre. Post-clip validity checks ---
		// After hull 0 clipping, cells may have new vertices with different h1
		// classifications. Re-check h1 status and thin dimensions.
		{
			bool any_h0s = false;
			int h1e_count = 0;
			for (const auto &v : verts) {
				if (classify_hull0_tree(nodes, leafs, planes, hull0_root, v.gs) == goldsrc::CONTENTS_SOLID)
					any_h0s = true;
				if (classify_clip_hull(clipnodes, planes, hull1_root, v.gs) != goldsrc::CONTENTS_SOLID)
					h1e_count++;
			}
			int cent_h1 = classify_clip_hull(clipnodes, planes, hull1_root, rcpt);
			int cent_h0 = classify_hull0_tree(nodes, leafs, planes, hull0_root, rcpt);
			// Cell straddles world boundary (h0 SOLID verts) AND h1 boundary
			// (centroid h1 EMPTY + >= 50% h1 EMPTY verts), with centroid in
			// open space (h0 EMPTY). This is an expansion ring artifact at a
			// corner where the ring wraps around world geometry.
			if (any_h0s && cent_h1 != goldsrc::CONTENTS_SOLID &&
				cent_h0 != goldsrc::CONTENTS_SOLID &&
				h1e_count * 2 >= (int)verts.size()) continue;
			// No near-wall verts + h1 EMPTY verts: expansion ring artifact
			// in open space. Kill if any dimension matches expansion width
			// (2*he ±1) OR all dimensions < 256.
			if (h1e_count > 0 && nw_count == 0) {
				float max_dim = fmaxf(rdim[0], fmaxf(rdim[1], rdim[2]));
				if (max_dim < 256.0f) continue;
				for (int a = 0; a < 3; a++) {
					if (fabsf(rdim[a] - 2.0f * he[a]) <= 1.0f) goto kill_2pre;
				}
			}
			if (false) { kill_2pre: continue; }
			// Multiple h1 EMPTY verts + not big on all axes + small cell
			// (max dim < 256): expansion ring fragment at a corner. Real
			// clip brushes this small should be entirely within hull1 SOLID.
			if (h1e_count >= 2 && !r_big) {
				float max_dim = fmaxf(rdim[0], fmaxf(rdim[1], rdim[2]));
				if (max_dim < 256.0f) continue;
			}
			// Small cell (all dims < 3*he) with h1 EMPTY verts: entirely
			// within the expansion ring region. Too small to be a real
			// clip brush — real brushes extend well beyond expansion width.
			if (h1e_count > 0 &&
				rdim[0] < 3.0f * he[0] && rdim[1] < 3.0f * he[1] && rdim[2] < 3.0f * he[2])
				continue;
			// Very thin (< half-extent) with h0 SOLID verts and minority
			// near-wall: floating slab artifact from clipping.
			if (any_h0s && nw_count * 2 < (int)verts.size()) {
				bool too_thin = false;
				for (int a = 0; a < 3; a++) {
					if (rdim[a] < he[a]) { too_thin = true; break; }
				}
				if (too_thin) continue;
			}
		}

		// --- 2aa. Free h1-EMPTY vertex check ---
		// A cell with h1-EMPTY vertices that are NOT near any wall has grown
		// beyond the clip brush boundary into open space. This is a growth
		// artifact from Minkowski un-expansion pushing planes outward. Real
		// clip brush boundary vertices that land in h1-EMPTY space are at
		// wall-junction BSP cuts and would be near-wall. Rescue only if the
		// cell is large enough (max_dim >= 256) to plausibly be a real brush
		// whose far vertex was clipped to a free position.
		{
			int free_h1e = 0;
			for (const auto &v : verts) {
				if (classify_clip_hull(clipnodes, planes, hull1_root, v.gs) != goldsrc::CONTENTS_SOLID &&
					!vert_near_wall(nodes, leafs, planes, hull0_root, v.gs, he)) {
					free_h1e++;
				}
			}
			if (free_h1e > 0) {
				int ba_aa = 0;
				for (int a = 0; a < 3; a++) { if (rdim[a] >= 2.0f * he[a]) ba_aa++; }
				bool rescued = has_large_clip_rescue(verts, rdim, ba_aa);
				// Post-rescue check for floor-to-ceiling expansion ring slabs:
				// a cell exactly player-height tall (Z == 2*he[2]) sandwiched
				// between a floor and a ceiling is rescued by has_large_clip_rescue
				// because top-face verts are h1-SOLID from the ceiling's expansion
				// ring, not from a real clip brush. The signature: ALL non-near-wall
				// h1-SOLID (rescuing) verts are on the top face (Z_max) AND at
				// least one near-wall bottom vert has solid ceiling above it at
				// Z_max + he[2] + 1 (one unit past the expansion boundary).
				// Use the near-wall vert XY to probe the ceiling, since the centroid
				// may be over open space (floor only exists near the walls).
				if (rescued && rdim[2] <= 2.0f * he[2] + 1.0f) {
					// Require all rescuing verts to be on the top face.
					bool all_nnw_solid_at_zmax = true;

					for (const auto &v : verts) {
						if (!vert_near_wall(nodes, leafs, planes, hull0_root, v.gs, he) &&
							classify_clip_hull(clipnodes, planes, hull1_root, v.gs) == goldsrc::CONTENTS_SOLID &&
							v.gs[2] < rx[2] - 1.0f) {
							all_nnw_solid_at_zmax = false; break;
						}
					}
					if (all_nnw_solid_at_zmax) {
						// Check for ceiling at Z_max + he[2] + 1 above each near-wall
						// bottom vert (XY from the near-wall vert, Z probes past ceiling).
						for (const auto &v : verts) {
							if (!vert_near_wall(nodes, leafs, planes, hull0_root, v.gs, he)) continue;
							if (v.gs[2] > rn[2] + 1.0f) continue;
							float test_above[3] = {v.gs[0], v.gs[1], rx[2] + he[2] + 1.0f};
							int h0_above = classify_hull0_tree(
								nodes, leafs, planes, hull0_root, test_above);
							// Accept any non-empty world content (SOLID, SKY, WATER, etc.)
							if (h0_above != goldsrc::CONTENTS_EMPTY && h0_above != 0) { rescued = false; break; }
						}
					}
				}
				// Additional rescue for horizontal clip brush slabs (floor/ceiling
				// clip brushes): thin in Z (≤ 2*he[2]) and no non-near-wall
				// h1-SOLID vertex (all SOLID verts flush with world geometry).
				// These have free h1-EMPTY vertices at their open bottom/top face,
				// which is valid — the face opens toward the room.
				if (!rescued && rdim[2] <= 2.0f * he[2] + 1.0f) {
					bool has_nnw_solid = false;
					for (const auto &v : verts) {
						if (!vert_near_wall(nodes, leafs, planes, hull0_root, v.gs, he) &&
							classify_clip_hull(clipnodes, planes, hull1_root, v.gs) == goldsrc::CONTENTS_SOLID) {
							has_nnw_solid = true; break;
						}
					}
					if (!has_nnw_solid) {
						// Could be a real floor/ceiling clip brush or a pure expansion
						// ring artifact — both have h1-SOLID verts flush with the world
						// surface. Distinguish via the unexpanded clip hull: expansion
						// ring artifacts un-expand to the world surface (centroid EMPTY),
						// while real clip brushes remain SOLID at their centroid.
						int uh1 = classify_clip_tree_unexpanded(
							clipnodes, planes, hull1_root, he, rcpt);
						if (uh1 == goldsrc::CONTENTS_SOLID) rescued = true;
					}
				}
				if (!rescued) continue;
			}
		}

		// --- 2a. All-near-wall check ---
		// Every vertex has an AABB corner in world geometry. Almost certainly
		// an expansion ring artifact. Rescue if centroid is h1 SOLID and not
		// near wall (clip brush flush with wall). Second rescue for very large
		// cells (>= 256 units) where all verts are h1 SOLID and centroid is
		// h0 non-solid.
		if (all_near_wall) {
			int ah1 = classify_clip_hull(clipnodes, planes, hull1_root, rcpt);
			if (ah1 != goldsrc::CONTENTS_SOLID ||
				vert_near_wall(nodes, leafs, planes, hull0_root, rcpt, he)) {
				bool all_h1s_anw = true;
				for (const auto &v : verts) {
					if (classify_clip_hull(clipnodes, planes, hull1_root, v.gs) != goldsrc::CONTENTS_SOLID) { all_h1s_anw = false; break; }
				}
				int ch0_anw = classify_hull0_tree(nodes, leafs, planes, hull0_root, rcpt);
				float anw_max_dim = fmaxf(rdim[0], fmaxf(rdim[1], rdim[2]));
				float min_he = fminf(he[0], fminf(he[1], he[2]));
				bool anw_sub_he = false;
				for (int a = 0; a < 3; a++) { if (rdim[a] < min_he) { anw_sub_he = true; break; } }
				// Also kill if centroid h0 is EMPTY (not CLIP) — expansion ring
				// in open space, not overlapping a real clip brush.
				bool h0_empty = (ch0_anw == goldsrc::CONTENTS_EMPTY);
				if (!(all_h1s_anw && ch0_anw != goldsrc::CONTENTS_SOLID && anw_max_dim >= 256.0f && !anw_sub_he && !h0_empty)) continue;
			}
		}

		// --- 2b. Centroid near wall ---
		// If the centroid's AABB touches world geometry, this cell is likely in
		// the expansion ring. Requires all verts h1 SOLID to survive (otherwise
		// definitely an artifact). Cells with any h0 SOLID vert need to be very
		// large (4x half-extents per axis) with centroid h0 non-solid to be
		// rescued as real clip brushes near corners.
		if (vert_near_wall(nodes, leafs, planes, hull0_root, rcpt, he)) {
			bool any_h0s = false;
			int h1e_cnt_2b = 0;
			for (const auto &v : verts) {
				int vh0 = classify_hull0_tree(nodes, leafs, planes, hull0_root, v.gs);
				if (vh0 == goldsrc::CONTENTS_SOLID) any_h0s = true;
				if (classify_clip_hull(clipnodes, planes, hull1_root, v.gs) != goldsrc::CONTENTS_SOLID) h1e_cnt_2b++;
			}
			bool all_h1s = (h1e_cnt_2b == 0);
			if (!all_h1s) {
				// Rescue: allow at most 1 h1 EMPTY vert if the cell is big
				// on 2+ axes, centroid solidly inside h1, and no h0 SOLID
				// verts. Large real clip brushes near walls can have a single
				// vertex land just outside the h1 boundary due to clipping.
				if (h1e_cnt_2b > 1 || any_h0s) continue;
				int rh1_rescue = classify_clip_hull(clipnodes, planes, hull1_root, rcpt, 4.0f);
				if (rh1_rescue != goldsrc::CONTENTS_SOLID) continue;
				int ba_2b = 0;
				for (int a = 0; a < 3; a++) { if (rdim[a] >= 2.0f * he[a]) ba_2b++; }
				if (ba_2b < 2) continue;
			}
			// Centroid near wall + all h1 SOLID: could be expansion ring or
			// real clip brush flush with wall. Require centroid solidly inside
			// h1 (4-unit tolerance) to survive — expansion ring centroids are
			// barely inside h1 at the ring boundary.
			int rh1_cnw = classify_clip_hull(clipnodes, planes, hull1_root, rcpt, 4.0f);
			if (any_h0s) {
				bool big4 = (rdim[0] >= 4.0f*he[0]) && (rdim[1] >= 4.0f*he[1]) && (rdim[2] >= 4.0f*he[2]);
				int ch0_cnw = classify_hull0_tree(nodes, leafs, planes, hull0_root, rcpt);
				if (!(big4 && ch0_cnw != goldsrc::CONTENTS_SOLID && rh1_cnw == goldsrc::CONTENTS_SOLID)) {
					// Secondary rescue: cell is big on 2+ axes with max dim >= 256,
					// centroid h0 non-solid, AND centroid solidly inside h1.
					int ba = 0;
					float md = fmaxf(rdim[0], fmaxf(rdim[1], rdim[2]));
					for (int a = 0; a < 3; a++) { if (rdim[a] >= 2.0f * he[a]) ba++; }
					if (!(ba >= 2 && md >= 256.0f && ch0_cnw != goldsrc::CONTENTS_SOLID
						&& ch0_cnw != goldsrc::CONTENTS_EMPTY && rh1_cnw == goldsrc::CONTENTS_SOLID)) continue;
				}
			} else {
				// No h0 SOLID verts: still require centroid solidly inside h1.
				if (rh1_cnw != goldsrc::CONTENTS_SOLID) continue;
				// Expansion ring slab: one dimension is exactly the expansion width
				// (2*he ±1) and the centroid is near wall. With no h0-SOLID verts
				// this cell sits flush in the Minkowski ring of a wall. Real clip
				// brushes at exact expansion width with centroid near wall are
				// indistinguishable from artifacts and do not appear in practice.
				for (int a = 0; a < 3; a++) {
					if (rdim[a] <= 2.0f * he[a] + 0.5f && fabsf(rdim[a] - 2.0f * he[a]) <= 1.0f)
						goto kill_2b_exp_ring;
				}
				goto skip_2b_exp_ring;
				kill_2b_exp_ring: continue;
				skip_2b_exp_ring:;
			}
		}

		// --- 2c. Small cells with majority near-wall verts ---
		// Not big on all axes but >50% near-wall verts. Rescued only if the
		// cell has a non-near-wall h1 SOLID vert (proving clip brush overlap),
		// is big on 2+ axes, and largest dim >= 256 (real brushes are big).
		int big_axes = 0;
		for (int a = 0; a < 3; a++) { if (rdim[a] >= 2.0f * he[a]) big_axes++; }
		if (!r_big && nw_count > (int)verts.size()/2) {
			// Kill thin-slab artifacts: any dim < he means the brush is thinner
			// than the hull half-extent. Exception: cells with h0 SOLID vertices
			// are real clip brushes embedded in world geometry.
			bool any_sub_he_2c = false;
			for (int a = 0; a < 3; a++) { if (rdim[a] < he[a]) { any_sub_he_2c = true; break; } }
			if (any_sub_he_2c) {
				bool any_h0s_2c = false;
				for (const auto &v : verts) {
					if (classify_hull0_tree(nodes, leafs, planes, hull0_root, v.gs) == goldsrc::CONTENTS_SOLID) {
						any_h0s_2c = true; break;
					}
				}
				if (!any_h0s_2c) continue;
				// h0-SOLID verts present but centroid in open air (EMPTY):
				// expansion ring artifact floating just above floor/wall geometry.
				// Real thin clip brushes embedded in world have non-EMPTY centroid
				// content (e.g. CONTENTS_SKY=-6 for floor-embedded clips).
				int ch0_sub_cpt = classify_hull0_tree(nodes, leafs, planes, hull0_root, rcpt);
				if (ch0_sub_cpt == goldsrc::CONTENTS_EMPTY) continue;
			}
			int ch0_2c = classify_hull0_tree(nodes, leafs, planes, hull0_root, rcpt);
			if (!has_large_clip_rescue(verts, rdim, big_axes)) {
				// Secondary rescue for all-near-wall cells: all h1 SOLID,
				// centroid h0 non-solid, big on 2+ axes, max dim >= 256.
				bool all_h1s_2c = true;
				for (const auto &v : verts) {
					if (classify_clip_hull(clipnodes, planes, hull1_root, v.gs) != goldsrc::CONTENTS_SOLID) { all_h1s_2c = false; break; }
				}
				float md_2c = fmaxf(rdim[0], fmaxf(rdim[1], rdim[2]));
				if (!(all_h1s_2c && ch0_2c != goldsrc::CONTENTS_SOLID && big_axes >= 2 && md_2c >= 256.0f)) continue;
			}
		}

		// --- 2d. Expansion ring tubes ---
		// Cells with 2+ dimensions at or below the expansion width (2*he+0.5)
		// and majority near-wall verts are thin tubes left from the ring.
		int narrow_axes = 0;
		for (int a = 0; a < 3; a++) { if (rdim[a] <= 2.0f * he[a] + 0.5f) narrow_axes++; }
		if (narrow_axes >= 2 && (nw_count > (int)verts.size()/2 || nw_count == 0)) continue;
		// Narrow axis thinner than half-extent with no near-wall verts:
		// thin expansion ring slab floating in open space. Real clip brushes
		// this thin would be adjacent to world geometry.
		if (narrow_axes >= 1 && nw_count == 0) {
			bool sub_he = false;
			for (int a = 0; a < 3; a++) {
				if (rdim[a] <= 2.0f * he[a] + 0.5f && rdim[a] < he[a] * 0.6f) { sub_he = true; break; }
			}
			if (sub_he) continue;
		}
		// Single narrow axis with majority near-wall: check centroid h1 at
		// tight tolerance (2 units). If centroid barely outside h1, it's a ring
		// artifact; if solidly inside, it's a real narrow clip brush.
		// Exception: if the narrow dimension is exactly at the expansion width
		// (2*he ±1), the cell sits flush on a surface and needs stronger evidence.
		if (narrow_axes >= 1 && nw_count > (int)verts.size()/2) {
			bool has_exact_expansion_dim = false;
			for (int a = 0; a < 3; a++) {
				if (rdim[a] <= 2.0f * he[a] + 0.5f && fabsf(rdim[a] - 2.0f * he[a]) <= 1.0f) {
					has_exact_expansion_dim = true; break;
				}
			}
			// Expansion-width dim + centroid near wall: the cell sits flush at
			// the expansion ring boundary against a wall. Kill — a real clip
			// brush at expansion width would not have its centroid near wall.
			if (has_exact_expansion_dim &&
				vert_near_wall(nodes, leafs, planes, hull0_root, rcpt, he)) continue;
			if (has_exact_expansion_dim && nw_count * 5 >= (int)verts.size() * 4) {
				// Very high near-wall ratio (>= 80%) with expansion-width dimension:
				// need >= 2 non-near-wall h1 SOLID verts to prove clip brush overlap.
				// A single free vert is not enough — expansion ring edges often have
				// one outlier vert.
				int free_solid_count = 0;
				for (const auto &v : verts) {
					if (!vert_near_wall(nodes, leafs, planes, hull0_root, v.gs, he) &&
						classify_clip_hull(clipnodes, planes, hull1_root, v.gs) == goldsrc::CONTENTS_SOLID) {
						free_solid_count++;
					}
				}
				if (free_solid_count < 2) continue;
			} else {
				int rh1_tol = classify_clip_hull(clipnodes, planes, hull1_root, rcpt, 2.0f);
				if (rh1_tol != goldsrc::CONTENTS_SOLID) continue;
			}
		}

		// --- 2e. Complex non-big cells with significant near-wall fraction ---
		// 6+ verts, >1/3 near wall, not big on all axes. Same rescue as 2c:
		// need a non-near-wall h1 SOLID vert + big on 2+ axes + largest >= 256.
		// Additionally, cells with h1 EMPTY verts are killed — they indicate
		// expansion ring artifacts that extend past the clip hull boundary.
		if (!r_big && verts.size() >= 6 && nw_count >= 3 && nw_count * 3 > (int)verts.size()) {
			// Count h1 EMPTY verts — multiple h1 EMPTY verts indicate
			// expansion ring artifacts extending past the clip hull boundary.
			// A single h1 EMPTY vert can occur on legitimate clip brushes
			// due to vertex placement near the hull1 boundary.
			int h1e_2e = 0;
			for (const auto &v : verts) {
				if (classify_clip_hull(clipnodes, planes, hull1_root, v.gs) != goldsrc::CONTENTS_SOLID)
					h1e_2e++;
			}
			if (h1e_2e >= 2) continue;
			if (!has_large_clip_rescue(verts, rdim, big_axes)) {
				// Secondary rescue (same as 2c): all h1 SOLID, centroid h0
				// non-solid, big on 2+ axes, max dim >= 256.
				bool all_h1s_2e = (h1e_2e == 0);
				int ch0_2e = classify_hull0_tree(nodes, leafs, planes, hull0_root, rcpt);
				float md_2e = fmaxf(rdim[0], fmaxf(rdim[1], rdim[2]));
				if (!(all_h1s_2e && ch0_2e != goldsrc::CONTENTS_SOLID && big_axes >= 2 && md_2e >= 256.0f)) continue;
			}
		}
		// --- 2f. r_big with near-wall half or majority ---
		// r_big cells where half or more vertices are near walls but the cell
		// is not large enough (max dim < 256) to be a real clip brush. These
		// are expansion ring artifacts from large world brushes.
		// --- 2f. r_big with near-wall half or majority, all h1 SOLID ---
		// r_big cells where half or more vertices are near walls and all
		// vertices are h1 SOLID but the cell is not large enough (max dim
		// < 256) to be a real clip brush. Cells with h1-EMPTY verts are
		// already handled by stage 2aa. Cells with h0 SOLID verts are
		// real clip brushes embedded in world geometry.
		{
			int h1e_2f = 0;
			for (const auto &v : verts) {
				if (classify_clip_hull(clipnodes, planes, hull1_root, v.gs) != goldsrc::CONTENTS_SOLID)
					h1e_2f++;
			}
			if (r_big && nw_count >= (int)verts.size()/2 && h1e_2f == 0) {
				bool any_h0s_2f = false;
				for (const auto &v : verts) {
					if (classify_hull0_tree(nodes, leafs, planes, hull0_root, v.gs) == goldsrc::CONTENTS_SOLID) {
						any_h0s_2f = true; break;
					}
				}
				bool hlcr2f = has_large_clip_rescue(verts, rdim, big_axes);
				if (hlcr2f) goto keep_2f;
				{
					// Kill if max dim < 256 (too small to be a real clip brush)
					// even if h0-SOLID verts present. Real clip brushes with h0
					// SOLID verts and max dim < 256 can't be distinguished from
					// expansion ring artifacts near wall corners.
					float md_2f = fmaxf(rdim[0], fmaxf(rdim[1], rdim[2]));
					if (md_2f < 256.0f) {
						// Rescue: centroid in non-EMPTY, non-SOLID content (e.g.
						// CONTENTS_SKY=-6, WATER=-3) strongly indicates a real
						// clip brush adjacent to special world geometry. Expansion
						// ring artifacts in this scenario would have centroid in
						// EMPTY space, not in special map content.
						int ch0_2f_val = classify_hull0_tree(
							nodes, leafs, planes, hull0_root, rcpt);
						if (ch0_2f_val == goldsrc::CONTENTS_EMPTY ||
							ch0_2f_val == goldsrc::CONTENTS_SOLID) continue;
						goto keep_2f;
					}
					if (!any_h0s_2f) continue;
				}
				keep_2f:;
			}
		}
		final_cells.push_back(std::move(cell));
	}

	return final_cells;
}

} // namespace goldsrc_hull
