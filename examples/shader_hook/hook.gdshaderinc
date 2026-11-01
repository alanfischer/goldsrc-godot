// An example shader hook: night, wet ground and a lightning flash, none of which the extension
// knows about. The BSP importer's "shader_hook" option points a map at this file; every shader the
// map is built with then includes it, with one of GOLDSRC_HOOK_WORLD, _LIQUID or _SKY defined, and
// calls the functions that kind needs. demo.gd sets the uniforms.

#ifdef GOLDSRC_HOOK_WORLD
// A linear-light multiplier on the sky's part of each luxel: (1,1,1) is the map as compiled, a dim
// blue is moonlight. The map's own lights are not the sky's, so they stay lit.
uniform vec3 sky_light = vec3(1.0);
// 0 dry, 1 soaked. Only ground under open sky gets wet: the rain floor says where that is.
uniform float wetness = 0.0;
uniform sampler2D rain_floor : filter_nearest;
uniform vec3 rain_floor_origin;  // the rain floor's "origin": its max-X/min-Z corner
uniform vec2 rain_floor_size = vec2(1.0);
uniform bool has_rain_floor = false;

// lm is the styled lightmap as stored (gamma); sun_share is how much of it came from the sky.
vec3 goldsrc_lightmap(vec3 lm, float sun_share) {
	// The lightmap holds pow(light, 0.55), so the multiplier goes through the same curve.
	return lm * pow(max(mix(vec3(1.0), sky_light, sun_share), vec3(0.0)), vec3(0.55));
}

bool open_to_sky(vec3 p) {
	if (!has_rain_floor) {
		return true;
	}
	vec2 uv = vec2((rain_floor_origin.x - p.x) / rain_floor_size.x, (p.z - rain_floor_origin.z) / rain_floor_size.y);
	return uv.x >= 0.0 && uv.y >= 0.0 && uv.x < 1.0 && uv.y < 1.0
		&& p.y > textureLod(rain_floor, uv, 0.0).r - 0.1;
}

// lit_srgb is texture x lightmap (gamma, what the surface shows); albedo is what real-time lights
// see. Normals and view are the fragment's own, in view space.
void goldsrc_surface(inout vec3 lit_srgb, inout vec3 albedo, vec3 world_pos, vec3 world_normal,
		vec3 normal_view, vec3 view) {
	if (wetness <= 0.0 || world_normal.y < 0.7 || !open_to_sky(world_pos)) {
		return;
	}
	// Wet is darker, and reflects the sky at grazing angles (water's Fresnel).
	float fresnel = 0.02 + 0.98 * pow(1.0 - clamp(dot(normal_view, view), 0.0, 1.0), 5.0);
	float dark = mix(1.0, 0.6, wetness);
	lit_srgb = min(lit_srgb * dark + vec3(0.6) * fresnel * wetness, vec3(1.0));
	albedo *= dark;
}
#endif

#ifdef GOLDSRC_HOOK_LIQUID
uniform vec3 sky_light = vec3(1.0);

// Liquid is fullbright, so night dims it whole; lava (liquid_type 2) keeps its glow.
vec3 goldsrc_liquid(vec3 albedo, int liquid_type) {
	return liquid_type == 2 ? albedo : albedo * pow(max(sky_light, vec3(0.0)), vec3(0.55));
}
#endif

#ifdef GOLDSRC_HOOK_SKY
uniform vec3 sky_light = vec3(1.0);
uniform vec3 flash_color = vec3(0.0);

// color is the sky cubemap along eye_dir (world space).
vec3 goldsrc_sky(vec3 color, vec3 eye_dir) {
	return color * sky_light + flash_color * (0.3 + 0.7 * max(eye_dir.y, 0.0));
}
#endif
