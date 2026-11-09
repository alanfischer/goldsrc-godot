#pragma once

#include "../parsers/bsp_parser.h"

#include <godot_cpp/variant/dictionary.hpp>

namespace goldsrc {

// Per column of the map, the first surface a trace from the top meets going straight down. Sky
// faces are where the open sky is, not a surface, so the trace carries on through them; the
// first face that is not sky is the surface. Faces are tested from both sides, so a sealed room's
// ceiling is met from above even though it only ever draws from below. Worldspawn and func_water
// count: a door or a platform overhead does not. A column outside the map stops at the top.
//
// A cell takes the highest of its centre and corners. Returns {origin: Vector3 (the max-X/min-Z
// corner in Godot coords, as the light grid's), size: Vector2 (Godot X/Z extent), image: Image
// FORMAT_RF of Godot Y heights, one texel per column (X = GS X, Y = GS Y), top: float, the Godot
// Y the trace starts from, which a column outside the map holds, under: Image FORMAT_RGF laid out
// as image}. "under" is the open air beneath the surface: the underside of the roof (R) and the
// ground below it (G), or -1e6 in both where there is none (open ground, a wall). Found by
// walking the same trace on down, face by face, to the first stretch of empty leaf at least
// 12 units (0.3 m) tall within 160 (4 m) of the surface.
godot::Dictionary bake_surface_below_sky(const BSPData &bsp, float cell_size_gs, float scale_factor);

} // namespace goldsrc
