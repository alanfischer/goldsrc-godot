#pragma once

#include "../parsers/bsp_parser.h"

#include <cstdint>
#include <vector>

namespace goldsrc {

// Per parsed face, per luxel, 0..255: the share of its light that came from the sky, sun
// included (`sky`), and the sun's direct share alone (`direct`), which a game lighting the sun
// itself takes back out. Empty when the map has no light_environment.
struct SunFraction {
	std::vector<std::vector<uint8_t>> sky;
	std::vector<std::vector<uint8_t>> direct;
	// The sun's light on a luxel facing it square, linear lightmap units (0..1 is 0..255 before
	// hlrad's gamma), as the fit found it: the brightness to light the sun with at runtime.
	float sun_light = 0.0f;
	bool empty() const { return sky.empty(); }
	void clear() { sky.clear(); direct.clear(); sun_light = 0.0f; }
};

// How much of each luxel's light came from the sky.
//
// hlrad bakes the sun and the map's own lights into the same style-0 lightmap, so nothing in
// the BSP says which is which. Night needs to know: it turns the sky's share down and leaves
// the torches alone. When the mapper put the sun on a style of its own (ww_chasm) the split is
// exact, one layer over the sum. Otherwise each luxel is re-lit with the three things hlrad
// lit it with — a sun trace, a hemisphere of sky traces, the point lights — and the weights
// that best reproduce the real lightmap say how much of what it holds was which. What none of
// the three explains (bounce, texture lights) is shared out in the map's overall proportion.
SunFraction compute_sun_fractions(const BSPData &bsp);

// A sun fraction is the sky's share of every style a luxel has; this is its share of the first
// layer alone, the one the light grid samples.
float layer0_sun_share(const BSPData &bsp, const ParsedFace &face, int luxel, float fraction,
	int sun_style);

} // namespace goldsrc
