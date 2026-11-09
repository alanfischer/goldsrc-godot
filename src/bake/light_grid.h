#pragma once

#include "sun_fraction.h"

#include <godot_cpp/variant/dictionary.hpp>

namespace goldsrc {

// An ambient cube per cell of a grid over the world: six axis rays from each cell's centre,
// each taking the lightmap where it lands, or the sky's light where it reaches the sky, plus the
// sun where a ray toward it does. Solid cells take their neighbours'. Returns {grid_origin:
// Vector3, grid_dims: Vector3i, cell_size: float (Godot units), dir_slices: 6 Arrays of RGB8
// slice Images in Godot order +X -X +Y -Y +Z -Z}, and with a sun fraction, sun_slices: L8
// slices of the sky's share of each cell's light. Empty without lightmaps.
godot::Dictionary bake_light_grid(const BSPData &bsp, const SunFraction &sun_fraction,
	float cell_size_gs, float scale_factor);

} // namespace goldsrc
