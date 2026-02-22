#pragma once

#include "parsers/bsp_parser.h"
#include <cmath>
#include <cstdint>
#include <vector>

namespace goldsrc_hull {

struct HullPlane {
	float normal[3];
	float dist;
	int16_t sibling_child = 0;
};

struct ConvexCell {
	std::vector<HullPlane> planes;
};

struct CellVertex {
	float gs[3]; // GoldSrc coordinates only
};

// Walk the BSP node tree (hull 0) collecting cells with the given contents.
void walk_bsp_tree(
	const std::vector<goldsrc::BSPNode> &nodes,
	const std::vector<goldsrc::BSPLeaf> &leafs,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	std::vector<HullPlane> &accumulated,
	std::vector<ConvexCell> &out_cells,
	int target_contents = goldsrc::CONTENTS_SOLID);

// Walk a clip hull BSP tree (hulls 1-3) collecting cells with the given contents.
void walk_clip_tree(
	const std::vector<goldsrc::BSPClipNode> &clipnodes,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	std::vector<HullPlane> &accumulated,
	std::vector<ConvexCell> &out_cells,
	int target_contents = goldsrc::CONTENTS_SOLID);

// Minkowski support function for an AABB with half-extents (hx, hy, hz).
float minkowski_support(const float normal[3], float hx, float hy, float hz);

// Point query against a clip hull BSP tree.
// Returns contents at the given GoldSrc point (-1=empty, -2=solid, etc.)
int clip_tree_contents(
	const std::vector<goldsrc::BSPClipNode> &clipnodes,
	const std::vector<goldsrc::BSPPlane> &planes,
	int root, float x, float y, float z);

// Compute convex hull vertices from half-space planes via triple-plane
// intersection (Cramer's rule). Returns vertices in GoldSrc coordinates only.
std::vector<CellVertex> compute_cell_vertices(
	const std::vector<HullPlane> &planes, float epsilon);

// Walk the sibling subtree for a cell face, splitting the cell when the face
// spans a sibling splitting plane. Appends resulting sub-cells to output.
void split_cell_for_face(
	ConvexCell cell,
	int face_index,
	int sibling_node,
	const std::vector<goldsrc::BSPClipNode> &clipnodes,
	const std::vector<goldsrc::BSPPlane> &bsp_planes,
	float epsilon,
	std::vector<ConvexCell> &output);

// Check if a point is inside all half-space planes (within tolerance).
bool point_inside(const std::vector<HullPlane> &planes, const float p[3], float tolerance = 1.0f);

} // namespace goldsrc_hull
