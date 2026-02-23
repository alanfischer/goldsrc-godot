#pragma once

#include "parsers/bsp_parser.h"
#include <cmath>
#include <cstdint>
#include <vector>

namespace goldsrc_hull {

// --- GoldSrc hull half-extents (maxs) in GoldSrc units ---
// Hull 0 = point (no expansion), Hull 1 = standing, Hull 2 = large, Hull 3 = crouching
struct HullExtents { float hx, hy, hz; };
constexpr HullExtents HULL_EXTENTS[4] = {
	{  0.0f,  0.0f,  0.0f },  // hull 0: point hull (no Minkowski expansion)
	{ 16.0f, 16.0f, 36.0f },  // hull 1: standing (32x32x72)
	{ 32.0f, 32.0f, 32.0f },  // hull 2: large (64x64x64)
	{ 16.0f, 16.0f, 18.0f },  // hull 3: crouching (32x32x36)
};

// --- Tuning constants for clip hull unexpansion ---
constexpr float CELL_EPSILON = 0.1f;           // tolerance for vertex computation & dedup
constexpr float FACE_VERTEX_TOLERANCE = 0.5f;   // how close a vertex must be to a face plane
constexpr float ANTIPARALLEL_THRESHOLD = -0.8f;  // normal dot product for antiparallel detection
constexpr float MIN_WALL_PRESERVE = 8.0f;        // minimum GS units preserved in thin slabs
constexpr float BINARY_SEARCH_MIN = 0.01f;       // minimum useful binary search result
constexpr int   BINARY_SEARCH_ITERS = 12;        // iterations for collapse recovery

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

// Result of the full unexpansion pipeline for a single convex cell.
struct ProcessedCell {
	std::vector<CellVertex> verts;
	float mins[3], maxs[3];
	float min_dim;
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

// Full clip hull unexpansion pipeline: walk clip tree, split cells per face,
// apply 3-pass Minkowski un-expansion with antiparallel clamping, binary search
// fallback for collapsed cells, and thin-cell filtering.
// Returns surviving ProcessedCells with vertices and AABB bounds.
std::vector<ProcessedCell> unexpand_clip_hull(
	const goldsrc::BSPData &bsp, int hull_index,
	float hull_hx, float hull_hy, float hull_hz,
	float min_dim_threshold);

} // namespace goldsrc_hull
