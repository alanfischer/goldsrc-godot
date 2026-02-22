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

	if (node_index < 0) {
		if (node_index == target_contents) {
			ConvexCell cell;
			cell.planes = accumulated;
			out_cells.push_back(std::move(cell));
		}
		return;
	}

	if ((size_t)node_index >= clipnodes.size()) return;

	const auto &cn = clipnodes[node_index];
	if (cn.planenum < 0 || (size_t)cn.planenum >= planes.size()) return;

	const auto &plane = planes[cn.planenum];

	HullPlane front_plane;
	front_plane.normal[0] = -plane.normal[0];
	front_plane.normal[1] = -plane.normal[1];
	front_plane.normal[2] = -plane.normal[2];
	front_plane.dist = -plane.dist;
	front_plane.sibling_child = cn.children[1];
	accumulated.push_back(front_plane);
	walk_clip_tree(clipnodes, planes, cn.children[0], accumulated, out_cells, target_contents);
	accumulated.pop_back();

	HullPlane back_plane;
	back_plane.normal[0] = plane.normal[0];
	back_plane.normal[1] = plane.normal[1];
	back_plane.normal[2] = plane.normal[2];
	back_plane.dist = plane.dist;
	back_plane.sibling_child = cn.children[0];
	accumulated.push_back(back_plane);
	walk_clip_tree(clipnodes, planes, cn.children[1], accumulated, out_cells, target_contents);
	accumulated.pop_back();
}

float minkowski_support(const float normal[3], float hx, float hy, float hz) {
	return fabsf(normal[0]) * hx + fabsf(normal[1]) * hy + fabsf(normal[2]) * hz;
}

int clip_tree_contents(
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &planes,
	int root, float x, float y, float z) {

	int node_index = root;
	while (node_index >= 0) {
		if ((size_t)node_index >= clipnodes.size()) return goldsrc::CONTENTS_EMPTY;
		const auto &cn = clipnodes[node_index];
		if (cn.planenum < 0 || (size_t)cn.planenum >= planes.size())
			return goldsrc::CONTENTS_EMPTY;
		const auto &plane = planes[cn.planenum];
		float dot = plane.normal[0] * x + plane.normal[1] * y + plane.normal[2] * z;
		if (dot >= plane.dist)
			node_index = cn.children[0];
		else
			node_index = cn.children[1];
	}
	return node_index;
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

void split_cell_for_face(
	ConvexCell cell,
	int face_index,
	int sibling_node,
	const vector<goldsrc::BSPClipNode> &clipnodes,
	const vector<goldsrc::BSPPlane> &bsp_planes,
	float epsilon,
	vector<ConvexCell> &output) {

	if (sibling_node < 0) {
		cell.planes[face_index].sibling_child = (int16_t)sibling_node;
		output.push_back(std::move(cell));
		return;
	}
	if ((size_t)sibling_node >= clipnodes.size()) {
		output.push_back(std::move(cell));
		return;
	}

	const auto &cn = clipnodes[sibling_node];
	if (cn.planenum < 0 || (size_t)cn.planenum >= bsp_planes.size()) {
		output.push_back(std::move(cell));
		return;
	}

	auto verts = compute_cell_vertices(cell.planes, epsilon);
	if ((int)verts.size() < 4) {
		output.push_back(std::move(cell));
		return;
	}

	const auto &fp = cell.planes[face_index];
	vector<int> face_vert_idx;
	for (int i = 0; i < (int)verts.size(); i++) {
		float dot = fp.normal[0] * verts[i].gs[0]
		          + fp.normal[1] * verts[i].gs[1]
		          + fp.normal[2] * verts[i].gs[2];
		if (fabsf(dot - fp.dist) < 0.5f) {
			face_vert_idx.push_back(i);
		}
	}
	if ((int)face_vert_idx.size() < 3) {
		output.push_back(std::move(cell));
		return;
	}

	const auto &splane = bsp_planes[cn.planenum];
	bool any_front = false, any_back = false;
	for (int idx : face_vert_idx) {
		float dot = splane.normal[0] * verts[idx].gs[0]
		          + splane.normal[1] * verts[idx].gs[1]
		          + splane.normal[2] * verts[idx].gs[2];
		float side = dot - splane.dist;
		if (side > 0.1f) any_front = true;
		if (side < -0.1f) any_back = true;
	}

	if (!any_back) {
		split_cell_for_face(std::move(cell), face_index, cn.children[0],
			clipnodes, bsp_planes, epsilon, output);
		return;
	}
	if (!any_front) {
		split_cell_for_face(std::move(cell), face_index, cn.children[1],
			clipnodes, bsp_planes, epsilon, output);
		return;
	}

	ConvexCell cell_front = cell;
	HullPlane hp_front;
	hp_front.normal[0] = -splane.normal[0];
	hp_front.normal[1] = -splane.normal[1];
	hp_front.normal[2] = -splane.normal[2];
	hp_front.dist = -splane.dist;
	hp_front.sibling_child = (int16_t)goldsrc::CONTENTS_SOLID;
	cell_front.planes.push_back(hp_front);

	ConvexCell cell_back = cell;
	HullPlane hp_back;
	hp_back.normal[0] = splane.normal[0];
	hp_back.normal[1] = splane.normal[1];
	hp_back.normal[2] = splane.normal[2];
	hp_back.dist = splane.dist;
	hp_back.sibling_child = (int16_t)goldsrc::CONTENTS_SOLID;
	cell_back.planes.push_back(hp_back);

	split_cell_for_face(std::move(cell_front), face_index, cn.children[0],
		clipnodes, bsp_planes, epsilon, output);
	split_cell_for_face(std::move(cell_back), face_index, cn.children[1],
		clipnodes, bsp_planes, epsilon, output);
}

bool point_inside(const vector<HullPlane> &planes, const float p[3], float tolerance) {
	for (const auto &hp : planes) {
		float dot = hp.normal[0] * p[0] + hp.normal[1] * p[1] + hp.normal[2] * p[2];
		if (dot > hp.dist + tolerance) return false;
	}
	return true;
}

} // namespace goldsrc_hull
