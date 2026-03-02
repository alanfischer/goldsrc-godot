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
	bool from_hull1 = true; // true = from clip hull tree walk (should un-expand), false = from hull 0 split
	int8_t unexpand_sign = 1; // +1 for back child (d -= support), -1 for front child (d += support)
	bool is_clip_brush = false; // set during un-expansion for planes that survive world matching
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

// Walk the clip BSP tree (hulls 1-3) collecting cells with the given contents.
// Clip nodes use int16_t children where negative = contents directly (not leaf index).
void walk_clip_tree(
	const std::vector<goldsrc::BSPClipNode> &clipnodes,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	std::vector<HullPlane> &accumulated,
	std::vector<ConvexCell> &out_cells,
	int target_contents = goldsrc::CONTENTS_SOLID);

// Walk clip tree, merging adjacent solid cells. When both children of a node
// lead to all-solid subtrees, the splitting plane is skipped (it's internal
// to the solid volume). This produces fewer, larger cells that can be
// un-expanded without creating gaps at shared BSP planes.
void walk_clip_tree_merged(
	const std::vector<goldsrc::BSPClipNode> &clipnodes,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	std::vector<HullPlane> &accumulated,
	std::vector<ConvexCell> &out_cells,
	int target_contents = goldsrc::CONTENTS_SOLID);

// Walk the clip BSP tree, un-expanding each plane by the AABB support function
// during traversal. This shrinks solid cells to their original (pre-Minkowski)
// geometry without creating gaps between adjacent cells.
void walk_clip_tree_unexpanded(
	const std::vector<goldsrc::BSPClipNode> &clipnodes,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	float half_extents[3],
	std::vector<HullPlane> &accumulated,
	std::vector<ConvexCell> &out_cells,
	int target_contents = goldsrc::CONTENTS_SOLID);

// Walk the clip BSP tree collecting cells at any leaf that is NOT CONTENTS_SOLID.
void walk_clip_tree_nonsolid(
	const std::vector<goldsrc::BSPClipNode> &clipnodes,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	std::vector<HullPlane> &accumulated,
	std::vector<ConvexCell> &out_cells);

struct CellAABB {
	float mins[3];
	float maxs[3];
};

// Clip a convex cell against the hull 0 BSP tree. Keep parts in non-solid
// hull 0 leaves. Split planes are tagged from_hull1=false so they won't
// be un-expanded later.
void clip_cell_by_hull0(
	const ConvexCell &cell,
	const std::vector<goldsrc::BSPNode> &nodes,
	const std::vector<goldsrc::BSPLeaf> &leafs,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	float epsilon,
	std::vector<ConvexCell> &out_cells);

// Un-expand only planes tagged from_hull1=true by subtracting support(n).
// Removes world surface expansion planes (matching hull 0 planes globally).
// Validates cells by classifying vertices against hull 1 — discards cells
// with any vertex in hull 1 EMPTY (world expansion ring artifacts).
// Prunes degenerate cells (< 4 vertices). Returns surviving cells.
std::vector<ConvexCell> unexpand_hull1_planes(
	const std::vector<ConvexCell> &cells,
	float half_extents[3],
	float epsilon,
	const std::vector<goldsrc::BSPNode> &nodes,
	const std::vector<goldsrc::BSPLeaf> &leafs,
	const std::vector<goldsrc::BSPClipNode> &clipnodes,
	const std::vector<goldsrc::BSPPlane> &bsp_planes,
	int hull0_root,
	int hull1_root);

// Clip a convex cell against a clip BSP tree (hulls 1-3). Keep parts in leaves
// matching keep_contents. Split planes are tagged from_hull1=false so they
// won't be un-expanded. Used to trim oversized cells back to hull 1 solid.
void clip_cell_by_clip_tree(
	const ConvexCell &cell,
	const std::vector<goldsrc::BSPClipNode> &clipnodes,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	float epsilon,
	std::vector<ConvexCell> &out_cells,
	int keep_contents = goldsrc::CONTENTS_SOLID);

// Classify a point against hull 0 BSP tree. Returns leaf contents.
int classify_hull0_tree(
	const std::vector<goldsrc::BSPNode> &nodes,
	const std::vector<goldsrc::BSPLeaf> &leafs,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	const float p[3]);

// Classify a point against the clip BSP tree with un-expanded planes.
// Points in the Minkowski expansion ring classify as EMPTY.
int classify_clip_tree_unexpanded(
	const std::vector<goldsrc::BSPClipNode> &clipnodes,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	const float half_extents[3],
	const float p[3]);

// Compute convex hull vertices from half-space planes via triple-plane
// intersection (Cramer's rule). Returns vertices in GoldSrc coordinates only.
std::vector<CellVertex> compute_cell_vertices(
	const std::vector<HullPlane> &planes, float epsilon);

// Compute AABB from cell vertices. Returns false if cell is degenerate (< 4 verts).
bool compute_cell_aabb(const std::vector<HullPlane> &planes, float epsilon, CellAABB &out);

// Test if two AABBs overlap.
bool aabb_overlap(const CellAABB &a, const CellAABB &b);

// Subtract convex cell B from convex cell A. Returns convex fragments of A
// that are outside B. Empty fragments are pruned via vertex computation.
std::vector<ConvexCell> subtract_convex(
	const ConvexCell &a, const ConvexCell &b, float epsilon);

} // namespace goldsrc_hull
