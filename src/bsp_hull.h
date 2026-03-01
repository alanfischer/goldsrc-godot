#pragma once

#include "parsers/bsp_parser.h"
#include <cmath>
#include <cstdint>
#include <vector>

namespace goldsrc_hull {

constexpr float CELL_EPSILON = 0.1f;

struct HullPlane {
	float normal[3];
	float dist;
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

// Compute convex hull vertices from half-space planes via triple-plane
// intersection (Cramer's rule). Returns vertices in GoldSrc coordinates only.
std::vector<CellVertex> compute_cell_vertices(
	const std::vector<HullPlane> &planes, float epsilon);

} // namespace goldsrc_hull
