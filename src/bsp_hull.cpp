#include "bsp_hull.h"

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

// Classify a point against the hull 1 clip BSP tree. Returns contents value.
static int classify_hull1(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int node_index,
	const float p[3]) {

	while (node_index >= 0) {
		if ((size_t)node_index >= clipnodes.size()) return 0;
		const auto &node = clipnodes[node_index];
		if (node.planenum < 0 || (size_t)node.planenum >= planes.size()) return 0;
		const auto &plane = planes[node.planenum];
		float dot = plane.normal[0] * p[0] + plane.normal[1] * p[1] + plane.normal[2] * p[2];
		if (dot >= plane.dist)
			node_index = node.children[0];
		else
			node_index = node.children[1];
	}
	return node_index; // negative = contents
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

	for (int i = 0; i < np - 2; i++) {
		for (int j = i + 1; j < np - 1; j++) {
			for (int k = j + 1; k < np; k++) {
				const auto &p1 = planes[i];
				const auto &p2 = planes[j];
				const auto &p3 = planes[k];

				float cx = p2.normal[1] * p3.normal[2] - p2.normal[2] * p3.normal[1];
				float cy = p2.normal[2] * p3.normal[0] - p2.normal[0] * p3.normal[2];
				float cz = p2.normal[0] * p3.normal[1] - p2.normal[1] * p3.normal[0];

				float denom = p1.normal[0] * cx + p1.normal[1] * cy + p1.normal[2] * cz;
				if (fabsf(denom) < 1e-6f) continue;

				float ax = p3.normal[1] * p1.normal[2] - p3.normal[2] * p1.normal[1];
				float ay = p3.normal[2] * p1.normal[0] - p3.normal[0] * p1.normal[2];
				float az = p3.normal[0] * p1.normal[1] - p3.normal[1] * p1.normal[0];

				float bx = p1.normal[1] * p2.normal[2] - p1.normal[2] * p2.normal[1];
				float by = p1.normal[2] * p2.normal[0] - p1.normal[0] * p2.normal[2];
				float bz = p1.normal[0] * p2.normal[1] - p1.normal[1] * p2.normal[0];

				float inv_denom = 1.0f / denom;
				float px = (p1.dist * cx + p2.dist * ax + p3.dist * bx) * inv_denom;
				float py = (p1.dist * cy + p2.dist * ay + p3.dist * by) * inv_denom;
				float pz = (p1.dist * cz + p2.dist * az + p3.dist * bz) * inv_denom;

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

} // namespace goldsrc_hull
