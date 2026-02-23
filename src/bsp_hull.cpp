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
		if (fabsf(dot - fp.dist) < FACE_VERTEX_TOLERANCE) {
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

vector<ProcessedCell> unexpand_clip_hull(
	const goldsrc::BSPData &bsp, int hull_index,
	float hull_hx, float hull_hy, float hull_hz,
	float min_dim_threshold) {

	if (bsp.models.empty()) return {};
	if (hull_index < 1 || hull_index > 3) return {};
	if (bsp.clipnodes.empty()) return {};

	const auto &bmodel = bsp.models[0];
	int root = bmodel.headnode[hull_index];
	if (root < 0 || (size_t)root >= bsp.clipnodes.size()) return {};

	// Bounding box planes to cap exterior cells
	HullPlane bbox_planes[6];
	bbox_planes[0] = {{ 1, 0, 0}, bmodel.maxs[0] + 1.0f};
	bbox_planes[1] = {{-1, 0, 0}, -bmodel.mins[0] + 1.0f};
	bbox_planes[2] = {{ 0, 1, 0}, bmodel.maxs[1] + 1.0f};
	bbox_planes[3] = {{ 0,-1, 0}, -bmodel.mins[1] + 1.0f};
	bbox_planes[4] = {{ 0, 0, 1}, bmodel.maxs[2] + 1.0f};
	bbox_planes[5] = {{ 0, 0,-1}, -bmodel.mins[2] + 1.0f};

	// Walk clip tree collecting CONTENTS_SOLID cells
	vector<HullPlane> accumulated;
	vector<ConvexCell> cells;
	walk_clip_tree(bsp.clipnodes, bsp.planes, root, accumulated, cells, goldsrc::CONTENTS_SOLID);

	if (cells.empty()) return {};

	vector<ProcessedCell> result;

	for (auto &cell : cells) {
		int original_plane_count = (int)cell.planes.size();

		for (int i = 0; i < 6; i++) {
			cell.planes.push_back(bbox_planes[i]);
		}

		// Split cells per face for sibling resolution
		vector<ConvexCell> current = {cell};
		for (int pi = 0; pi < original_plane_count; pi++) {
			vector<ConvexCell> next;
			for (auto &c : current) {
				split_cell_for_face(c, pi, c.planes[pi].sibling_child,
					bsp.clipnodes, bsp.planes, CELL_EPSILON, next);
			}
			current = std::move(next);
		}

		for (auto &c : current) {
			auto pre_verts = compute_cell_vertices(c.planes, CELL_EPSILON);
			if ((int)pre_verts.size() < 4) continue;

			// Save original distances for binary search fallback
			vector<float> orig_dist(original_plane_count);
			for (int pi = 0; pi < original_plane_count; pi++)
				orig_dist[pi] = c.planes[pi].dist;

			// First pass: determine which planes to unexpand and their support
			struct UnexpandInfo {
				bool should_unexpand = false;
				float support = 0;
				float clamped_support = 0;
			};
			vector<UnexpandInfo> uinfo(original_plane_count);

			for (int pi = 0; pi < original_plane_count; pi++) {
				auto &hp = c.planes[pi];
				float support = minkowski_support(hp.normal, hull_hx, hull_hy, hull_hz);
				uinfo[pi].support = support;
				uinfo[pi].clamped_support = support;

				if (hp.sibling_child != (int16_t)goldsrc::CONTENTS_SOLID) {
					uinfo[pi].should_unexpand = true;
				} else {
					// Probe check: for thick cells, probe one support distance
					// outside the face to detect thin BSP artifact layers
					float min_dot = hp.dist;
					float cx = 0, cy = 0, cz = 0;
					int face_count = 0;
					for (const auto &v : pre_verts) {
						float dot = hp.normal[0] * v.gs[0]
						          + hp.normal[1] * v.gs[1]
						          + hp.normal[2] * v.gs[2];
						if (dot < min_dot) min_dot = dot;
						if (fabsf(dot - hp.dist) < FACE_VERTEX_TOLERANCE) {
							cx += v.gs[0]; cy += v.gs[1]; cz += v.gs[2];
							face_count++;
						}
					}
					float thickness = hp.dist - min_dot;
					if (face_count >= 3 && thickness > support) {
						cx /= face_count; cy /= face_count; cz /= face_count;
						float probe_x = cx + hp.normal[0] * support;
						float probe_y = cy + hp.normal[1] * support;
						float probe_z = cz + hp.normal[2] * support;
						int contents = clip_tree_contents(
							bsp.clipnodes, bsp.planes,
							root, probe_x, probe_y, probe_z);
						if (contents != goldsrc::CONTENTS_SOLID) {
							uinfo[pi].should_unexpand = true;
						}
					}
				}
			}

			// Second pass: antiparallel slab clamping
			for (int i = 0; i < original_plane_count; i++) {
				if (!uinfo[i].should_unexpand) continue;
				const auto &pi_plane = c.planes[i];

				for (int j = i + 1; j < original_plane_count; j++) {
					if (!uinfo[j].should_unexpand) continue;
					const auto &pj_plane = c.planes[j];

					float dot_normals =
						pi_plane.normal[0] * pj_plane.normal[0] +
						pi_plane.normal[1] * pj_plane.normal[1] +
						pi_plane.normal[2] * pj_plane.normal[2];
					if (dot_normals > ANTIPARALLEL_THRESHOLD) continue;

					float slab_gap = pi_plane.dist + pj_plane.dist;
					float total_support = uinfo[i].support + uinfo[j].support;
					if (total_support > slab_gap) {
						float available = fmaxf(0.0f, slab_gap - MIN_WALL_PRESERVE);
						float ratio = (total_support > 0) ?
							available / total_support : 0.0f;
						uinfo[i].clamped_support = fminf(
							uinfo[i].clamped_support,
							uinfo[i].support * ratio);
						uinfo[j].clamped_support = fminf(
							uinfo[j].clamped_support,
							uinfo[j].support * ratio);
					}
				}
			}

			// Third pass: apply (clamped) unexpansion
			for (int pi = 0; pi < original_plane_count; pi++) {
				if (uinfo[pi].should_unexpand) {
					c.planes[pi].dist -= uinfo[pi].clamped_support;
				}
			}

			vector<CellVertex> verts = compute_cell_vertices(c.planes, CELL_EPSILON);

			// Fallback: if the cell collapsed, binary search for the maximum
			// unexpansion scale that produces valid geometry
			if ((int)verts.size() < 4) {
				for (int pi = 0; pi < original_plane_count; pi++)
					c.planes[pi].dist = orig_dist[pi];

				float lo = 0.0f, hi = 1.0f;
				for (int iter = 0; iter < BINARY_SEARCH_ITERS; iter++) {
					float mid = (lo + hi) * 0.5f;
					auto test = c.planes;
					for (int pi = 0; pi < original_plane_count; pi++) {
						if (uinfo[pi].should_unexpand)
							test[pi].dist -= uinfo[pi].clamped_support * mid;
					}
					auto tv = compute_cell_vertices(test, CELL_EPSILON);
					if ((int)tv.size() >= 4) lo = mid;
					else hi = mid;
				}

				if (lo > BINARY_SEARCH_MIN) {
					for (int pi = 0; pi < original_plane_count; pi++) {
						if (uinfo[pi].should_unexpand)
							c.planes[pi].dist -= uinfo[pi].clamped_support * lo;
					}
					verts = compute_cell_vertices(c.planes, CELL_EPSILON);
				}
			}

			if ((int)verts.size() < 4) continue;

			// Compute AABB and min dimension
			float mn[3] = {1e9f, 1e9f, 1e9f};
			float mx[3] = {-1e9f, -1e9f, -1e9f};
			for (const auto &v : verts) {
				for (int a = 0; a < 3; a++) {
					if (v.gs[a] < mn[a]) mn[a] = v.gs[a];
					if (v.gs[a] > mx[a]) mx[a] = v.gs[a];
				}
			}
			float min_dim = fminf(fminf(mx[0]-mn[0], mx[1]-mn[1]), mx[2]-mn[2]);

			// Skip BSP splitting artifacts: cells thinner than threshold
			if (min_dim < min_dim_threshold) continue;

			ProcessedCell pc;
			pc.verts = std::move(verts);
			for (int a = 0; a < 3; a++) {
				pc.mins[a] = mn[a];
				pc.maxs[a] = mx[a];
			}
			pc.min_dim = min_dim;
			result.push_back(std::move(pc));
		}
	}

	return result;
}

} // namespace goldsrc_hull
