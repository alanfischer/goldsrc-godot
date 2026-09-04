#include "goldsrc_bsp.h"

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/gd_script.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/classes/static_body3d.hpp>
#include <godot_cpp/classes/animatable_body3d.hpp>
#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/concave_polygon_shape3d.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/time.hpp>

#include <algorithm>
#include <cstring>
#include <godot_cpp/classes/geometry2d.hpp>
#include <map>
#include <set>
#include <tuple>
#include <numeric>
#ifdef _MSC_VER
#include <intrin.h>
static inline int popcount64(uint64_t x) { return (int)__popcnt64(x); }
#else
static inline int popcount64(uint64_t x) { return __builtin_popcountll(x); }
#endif

using namespace godot;
using namespace std;

// Minimal structs and helpers for convex cell collision (brush entities / water).
// These were previously in bsp_hull.h; inlined here to avoid that dependency.
struct HullPlane {
	float normal[3];
	float dist;
};

struct ConvexCell {
	std::vector<HullPlane> planes;
};

struct CellVertex {
	float gs[3];
};

static void walk_bsp_tree(
	const std::vector<goldsrc::BSPNode> &nodes,
	const std::vector<goldsrc::BSPLeaf> &leafs,
	const std::vector<goldsrc::BSPPlane> &planes,
	int node_index,
	std::vector<HullPlane> &accumulated,
	std::vector<ConvexCell> &out_cells,
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

static std::vector<CellVertex> compute_cell_vertices(
	const std::vector<HullPlane> &planes, float epsilon) {

	std::vector<CellVertex> verts;
	int np = (int)planes.size();
	for (int i = 0; i < np - 2; i++) {
		for (int j = i + 1; j < np - 1; j++) {
			for (int k = j + 1; k < np; k++) {
				const auto &p1 = planes[i];
				const auto &p2 = planes[j];
				const auto &p3 = planes[k];
				double n1[3] = {p1.normal[0], p1.normal[1], p1.normal[2]};
				double n2[3] = {p2.normal[0], p2.normal[1], p2.normal[2]};
				double n3[3] = {p3.normal[0], p3.normal[1], p3.normal[2]};
				double d1 = p1.dist, d2 = p2.dist, d3 = p3.dist;
				double cx = n2[1]*n3[2] - n2[2]*n3[1];
				double cy = n2[2]*n3[0] - n2[0]*n3[2];
				double cz = n2[0]*n3[1] - n2[1]*n3[0];
				double denom = n1[0]*cx + n1[1]*cy + n1[2]*cz;
				if (fabs(denom) < 1e-10) continue;
				double ax = n3[1]*n1[2] - n3[2]*n1[1];
				double ay = n3[2]*n1[0] - n3[0]*n1[2];
				double az = n3[0]*n1[1] - n3[1]*n1[0];
				double bx = n1[1]*n2[2] - n1[2]*n2[1];
				double by = n1[2]*n2[0] - n1[0]*n2[2];
				double bz = n1[0]*n2[1] - n1[1]*n2[0];
				double inv_denom = 1.0 / denom;
				float px = (float)((d1*cx + d2*ax + d3*bx) * inv_denom);
				float py = (float)((d1*cy + d2*ay + d3*by) * inv_denom);
				float pz = (float)((d1*cz + d2*az + d3*bz) * inv_denom);
				bool inside = true;
				for (int m = 0; m < np; m++) {
					if (m == i || m == j || m == k) continue;
					const auto &pp = planes[m];
					float dot = pp.normal[0]*px + pp.normal[1]*py + pp.normal[2]*pz;
					if (dot > pp.dist + epsilon) { inside = false; break; }
				}
				if (!inside) continue;
				bool duplicate = false;
				for (const auto &ev : verts) {
					float dx = ev.gs[0]-px, dy = ev.gs[1]-py, dz = ev.gs[2]-pz;
					if (sqrtf(dx*dx + dy*dy + dz*dz) < epsilon) { duplicate = true; break; }
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

namespace {

string str_to_lower(const string &s) {
	string result = s;
	transform(result.begin(), result.end(), result.begin(),
		[](unsigned char c) { return tolower(c); });
	return result;
}

// Shader source for lightstyle blending (opaque variant)
static const char *LIGHTSTYLE_SHADER_CODE = R"(
shader_type spatial;
render_mode ambient_light_disabled, specular_disabled;

uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic;
uniform sampler2D lm_layer0 : filter_linear;
uniform sampler2D lm_layer1 : filter_linear;
uniform sampler2D lm_layer2 : filter_linear;
uniform sampler2D lm_layer3 : filter_linear;
uniform sampler2D lightstyle_tex : filter_nearest;
uniform float overbright = 2.0;
// GoldSrc composited texture x lightmap on 8-bit gamma values. The RD renderers
// (forward+/mobile) shade in linear space and encode on output, gl_compatibility
// shades in sRGB and does not, so the same expression lands in a different space
// on each. Do the multiply in gamma space either way and convert once at the end.
uniform bool linear_output = false;

vec3 gs_to_srgb(vec3 c) {
	return mix(c * 12.92, 1.055 * pow(max(c, vec3(0.0)), vec3(1.0 / 2.4)) - 0.055, step(0.0031308, c));
}

vec3 gs_to_linear(vec3 c) {
	return mix(c / 12.92, pow((max(c, vec3(0.0)) + 0.055) / 1.055, vec3(2.4)), step(0.04045, c));
}

varying vec4 style_coords;

void vertex() {
	style_coords = COLOR;
}

void fragment() {
	vec4 albedo = texture(albedo_texture, UV);

	float b0 = texture(lightstyle_tex, vec2(style_coords.r, 0.5)).r;
	float b1 = texture(lightstyle_tex, vec2(style_coords.g, 0.5)).r;
	float b2 = texture(lightstyle_tex, vec2(style_coords.b, 0.5)).r;
	float b3 = texture(lightstyle_tex, vec2(style_coords.a, 0.5)).r;

	vec3 lm = texture(lm_layer0, UV2).rgb * b0
	         + texture(lm_layer1, UV2).rgb * b1
	         + texture(lm_layer2, UV2).rgb * b2
	         + texture(lm_layer3, UV2).rgb * b3;

	// ALBEDO stays in the renderer's native space so real-time lights stay correct;
	// only the lightmap term needs the gamma-space treatment. min() before the
	// conversion reproduces GoldSrc's clamp, which clips differently in linear.
	vec3 base_srgb = linear_output ? gs_to_srgb(albedo.rgb) : albedo.rgb;
	vec3 lit_srgb = min(base_srgb * lm * overbright, vec3(1.0));
	ALBEDO = albedo.rgb;
	EMISSION = linear_output ? gs_to_linear(lit_srgb) : lit_srgb;
	ROUGHNESS = 1.0;
}
)";

// Alpha-scissor variant for '{' textures (fences, grates)
static const char *LIGHTSTYLE_SHADER_ALPHA_CODE = R"(
shader_type spatial;
render_mode ambient_light_disabled, specular_disabled;

uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic;
uniform sampler2D lm_layer0 : filter_linear;
uniform sampler2D lm_layer1 : filter_linear;
uniform sampler2D lm_layer2 : filter_linear;
uniform sampler2D lm_layer3 : filter_linear;
uniform sampler2D lightstyle_tex : filter_nearest;
uniform float overbright = 2.0;
// GoldSrc composited texture x lightmap on 8-bit gamma values. The RD renderers
// (forward+/mobile) shade in linear space and encode on output, gl_compatibility
// shades in sRGB and does not, so the same expression lands in a different space
// on each. Do the multiply in gamma space either way and convert once at the end.
uniform bool linear_output = false;

vec3 gs_to_srgb(vec3 c) {
	return mix(c * 12.92, 1.055 * pow(max(c, vec3(0.0)), vec3(1.0 / 2.4)) - 0.055, step(0.0031308, c));
}

vec3 gs_to_linear(vec3 c) {
	return mix(c / 12.92, pow((max(c, vec3(0.0)) + 0.055) / 1.055, vec3(2.4)), step(0.04045, c));
}
uniform float alpha_scissor_threshold : hint_range(0, 1) = 0.5;

varying vec4 style_coords;

void vertex() {
	style_coords = COLOR;
}

void fragment() {
	vec4 albedo = texture(albedo_texture, UV);

	float b0 = texture(lightstyle_tex, vec2(style_coords.r, 0.5)).r;
	float b1 = texture(lightstyle_tex, vec2(style_coords.g, 0.5)).r;
	float b2 = texture(lightstyle_tex, vec2(style_coords.b, 0.5)).r;
	float b3 = texture(lightstyle_tex, vec2(style_coords.a, 0.5)).r;

	vec3 lm = texture(lm_layer0, UV2).rgb * b0
	         + texture(lm_layer1, UV2).rgb * b1
	         + texture(lm_layer2, UV2).rgb * b2
	         + texture(lm_layer3, UV2).rgb * b3;

	// ALBEDO stays in the renderer's native space so real-time lights stay correct;
	// only the lightmap term needs the gamma-space treatment. min() before the
	// conversion reproduces GoldSrc's clamp, which clips differently in linear.
	vec3 base_srgb = linear_output ? gs_to_srgb(albedo.rgb) : albedo.rgb;
	vec3 lit_srgb = min(base_srgb * lm * overbright, vec3(1.0));
	ALBEDO = albedo.rgb;
	EMISSION = linear_output ? gs_to_linear(lit_srgb) : lit_srgb;
	ROUGHNESS = 1.0;
	ALPHA = albedo.a;
	ALPHA_SCISSOR_THRESHOLD = alpha_scissor_threshold;
}
)";

// Sky surface shader: samples sky cubemap by view direction. Rendered first
// (render_priority -1) with no depth write, so it acts as a true background —
// walls occlude it normally and projectiles beyond it remain visible.
// sky_cubemap is set at runtime by GDScript.
static const char *SKY_SURFACE_SHADER_CODE = R"(
shader_type spatial;
render_mode unshaded, shadows_disabled, ambient_light_disabled, depth_draw_never;

uniform samplerCube sky_cubemap : source_color, hint_default_black;

void fragment() {
	// Reconstruct world-space eye direction matching sky_cubemap.gdshader EYEDIR.
	// VIEW points from fragment toward camera (view space); negate and rotate to world.
	vec3 eye_dir = normalize((INV_VIEW_MATRIX * vec4(-VIEW, 0.0)).xyz);
	ALBEDO = texture(sky_cubemap, vec3(-eye_dir.x, eye_dir.y, eye_dir.z)).rgb;
}
)";

// Which liquid a surface is, as the shader's liquid_type. GoldSrc kept this in one place per
// kind of brush and never in the texture name: a brush entity carries it in its "skin" key
// (CFuncDoor::KeyValue stores it straight into pev->skin), and a worldspawn surface takes it
// from the leaf it borders, which the compiler had already resolved from the name at build
// time. Both are the same CONTENTS_* value, so both come through here.
//
// Reading the name instead would be re-deriving what the compiler already decided, and it gets
// it wrong: ww_golem's lava-textured brush is deliberately skin -3, plain water that does not
// burn, and any texture whose name merely contains "lava" or "rice" would be miscalled.
enum LiquidKind { LIQUID_WATER = 0, LIQUID_SLIME = 1, LIQUID_LAVA = 2 };

static LiquidKind liquid_kind_for_contents(int contents) {
	switch (contents) {
		case goldsrc::CONTENTS_SLIME: return LIQUID_SLIME;
		case goldsrc::CONTENTS_LAVA: return LIQUID_LAVA;
		default: return LIQUID_WATER;
	}
}

// Vertical wave displacement, shared verbatim by the water surface and its depth-prepass twin.
// The two MUST move together: the prepass exists to stamp liquid depth ahead of the colour pass,
// and a silhouette that no longer matches is a surface that rejects itself.
//
// The phase is a function of WORLD position, never of UV or model space, so that two brushes
// meeting at an edge agree on where the crest is. Subdivision follows world-aligned planes for
// the same reason (see subdivide_water_polygon): shared vertices land on the same grid and
// therefore displace identically, which is what keeps the seams closed.
#define GOLDSRC_WATER_WAVE_GLSL \
"uniform float wave_height = 0.0;  // Godot units of crest, 0 = flat (worldspawn liquid)\n" \
"const float WAVE_FREQ = 0.49;     // radians per unit — a ~12.8 unit (512 GoldSrc) swell\n" \
"const float WAVE_SPEED = 1.1;\n" \
"vec3 goldsrc_water_wave(vec3 vertex, mat4 model) {\n" \
"	if (wave_height <= 0.0) { return vertex; }\n" \
"	vec3 w = (model * vec4(vertex, 1.0)).xyz;\n" \
"	float phase = TIME * WAVE_SPEED;\n" \
"	vertex.y += wave_height * sin(w.x * WAVE_FREQ + phase) * cos(w.z * WAVE_FREQ + phase);\n" \
"	return vertex;\n" \
"}\n"

// Quarter of the wave period (512 GoldSrc units), in GoldSrc units. Larger reads as folds;
// smaller buys nothing a sine can use. Only brushes that actually wave are cut this fine.
static const float WATER_SUBDIVIDE = 128.0f;

// GoldSrc's water waves need geometry to bend, and a BSP face is one flat polygon. Split it on
// world-aligned planes at multiples of `size`, exactly as Quake's GL_SubdivideSurface does, and
// for its reason: two faces that share an edge are cut by the same planes at the same places, so
// their vertices coincide and the wave cannot tear them apart. Splitting by longest-edge instead
// would be simpler and would leave T-junctions that crack open the moment anything moves.
static void subdivide_water_polygon(const std::vector<goldsrc::ParsedVertex> &poly, float size,
		std::vector<std::vector<goldsrc::ParsedVertex>> &out, int depth = 0) {
	if (poly.size() < 3) return;
	if (depth > 12) { out.push_back(poly); return; }  // pathological face; stop splitting

	for (int axis = 0; axis < 3; axis++) {
		float mn = 1e30f, mx = -1e30f;
		for (const auto &v : poly) { mn = std::min(mn, v.pos[axis]); mx = std::max(mx, v.pos[axis]); }
		if (mx - mn <= size) continue;

		// Cut on the grid line nearest the middle, so the pieces stay aligned map-wide.
		float mid = floorf(((mn + mx) * 0.5f) / size + 0.5f) * size;
		if (mid <= mn + 0.1f || mid >= mx - 0.1f) continue;  // nothing useful to cut

		std::vector<goldsrc::ParsedVertex> front, back;
		for (size_t i = 0; i < poly.size(); i++) {
			const auto &a = poly[i];
			const auto &b = poly[(i + 1) % poly.size()];
			float da = a.pos[axis] - mid;
			float db = b.pos[axis] - mid;
			if (da >= 0) front.push_back(a);
			if (da <= 0) back.push_back(a);
			if ((da > 0) == (db > 0) || da == 0 || db == 0) continue;

			goldsrc::ParsedVertex cut;
			float f = da / (da - db);
			for (int k = 0; k < 3; k++) {
				cut.pos[k] = a.pos[k] + f * (b.pos[k] - a.pos[k]);
				cut.normal[k] = a.normal[k];
			}
			for (int k = 0; k < 2; k++) {
				cut.uv[k] = a.uv[k] + f * (b.uv[k] - a.uv[k]);
				cut.lightmap_uv[k] = a.lightmap_uv[k] + f * (b.lightmap_uv[k] - a.lightmap_uv[k]);
			}
			cut.pos[axis] = mid;  // exact, so both sides share the vertex to the bit
			front.push_back(cut);
			back.push_back(cut);
		}
		subdivide_water_polygon(front, size, out, depth + 1);
		subdivide_water_polygon(back, size, out, depth + 1);
		return;
	}
	out.push_back(poly);
}

// Water surface shader: turbulent UV warp to simulate GoldSrc liquid surfaces.
// Applied to faces whose texture name starts with '!' or '*', and to every face of a
// func_water brush whatever its texture is called (see ent_is_liquid in build_mesh).
// GoldSrc water is translucent and visible from both sides (you see the surface from
// above and, when submerged, from below), so the material writes ALPHA and disables
// backface culling. cull_back would hide the inner faces and leave opaque water slabs.
//
// Writing ALPHA also puts the material in the transparent queue, which Godot sorts per
// MeshInstance3D by AABB-center distance and does not sort within a surface at all. Water is
// batched per (texture, atlas page), so a whole map's liquid is often ONE instance with a
// map-spanning AABB: far faces then blend over near ones and a pool reads inside-out. That
// is what WATER_DEPTH_SHADER_CODE below fixes — see it for the mechanism. This half only has
// to stay out of the way: depth_draw_never, so the color pass tests against the prepass's
// depth without stamping its own on top of it.
static const char *WATER_SHADER_CODE = R"(
shader_type spatial;
render_mode unshaded, shadows_disabled, ambient_light_disabled, cull_disabled, depth_draw_never;

uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic;
uniform float wave_amplitude : hint_range(0.0, 0.1) = 0.025;
uniform float wave_frequency : hint_range(1.0, 20.0) = 8.0;
uniform float wave_speed : hint_range(0.1, 5.0) = 1.6;
uniform float water_alpha : hint_range(0.0, 1.0) = 0.6;
// 0 water, 1 slime, 2 lava — the brush's CONTENTS_*, see liquid_kind_for_contents. Water is
// left at 1.0/1.0 so every map that already had liquid keeps exactly the surface it had.
//
// GoldSrc ran one warp for all three; separating them is ours. It stops at what the map cannot
// say for itself: how still a surface is belongs to WaveHeight, which the mapper sets per brush
// (ww_matrox has an ice brush at 5 and another at 0), so nothing here may override it.
uniform int liquid_type : hint_range(0, 2) = 0;
)" GOLDSRC_WATER_WAVE_GLSL R"(
void vertex() {
	VERTEX = goldsrc_water_wave(VERTEX, MODEL_MATRIX);
}

void fragment() {
	float speed_mul = 1.0;
	float amp_mul = 1.0;
	if (liquid_type == 1) { speed_mul = 0.6; amp_mul = 1.2; }        // slime: thicker, slower
	else if (liquid_type == 2) { speed_mul = 0.25; amp_mul = 1.6; }  // lava: slow, wide roll

	vec2 uv = UV;
	float phase = TIME * wave_speed * speed_mul;
	float amp = wave_amplitude * amp_mul;
	uv.x += sin(uv.y * wave_frequency + phase) * amp;
	uv.y += sin(uv.x * wave_frequency + phase) * amp;
	ALBEDO = texture(albedo_texture, uv).rgb;
	ALPHA = water_alpha;
	ROUGHNESS = 1.0;
}
)";

// Depth-only companion pass for water, drawn from a twin MeshInstance3D over the same mesh
// (see build_mesh). Together the two passes are a hand-rolled depth prepass, which is what
// makes water immune to draw order: this pass stamps liquid depth into the buffer first
// (render_priority -1), so the color pass above only survives where it is the NEAREST liquid
// fragment. Everything behind another water face is depth-rejected and never blends.
//
// Godot has a render mode for this — depth_prepass_alpha — but the Compatibility renderer
// (rendering_method.pc, and what this game ships on PC) parses it and then ignores it, so it
// buys nothing. Hence doing it by hand.
//
// Two details make it work:
//   ALPHA = 0.0    keeps the pass depth-only. It is still a transparent-queue draw, but
//                  src*0 + dst*1 leaves the color buffer exactly as it found it, while
//                  depth_draw_always writes depth regardless of alpha.
//   POSITION       pushes the stamped depth 0.2% FARTHER from the camera. The Compatibility
//                  renderer's depth func is GL_LESS, not LEQUAL, so a prepass at the exact
//                  surface depth would reject the color pass and water would vanish outright.
//                  Scaling the whole view-space position scales along the ray from the eye,
//                  so the silhouette projects identically — only depth moves. The bias is
//                  relative to view distance, so it holds at any scale; the only faces it
//                  fails to separate are ones within 0.2% of each other in depth, i.e. the
//                  near-coincident case, where either order looks the same anyway.
static const char *WATER_DEPTH_SHADER_CODE = R"(
shader_type spatial;
render_mode unshaded, shadows_disabled, ambient_light_disabled, cull_disabled, depth_draw_always;
)" GOLDSRC_WATER_WAVE_GLSL R"(
void vertex() {
	VERTEX = goldsrc_water_wave(VERTEX, MODEL_MATRIX);
	vec4 view_pos = MODELVIEW_MATRIX * vec4(VERTEX, 1.0);
	view_pos.xyz *= 1.002;
	POSITION = PROJECTION_MATRIX * view_pos;
}

void fragment() {
	ALPHA = 0.0;
}
)";

// Inline GDScript for animated texture cycling. Embedded into the scene by
// build_mesh() so it self-starts on load with no external file dependency.
// Named TextureAnimationPlayer to mirror the SPR SpriteAnimationPlayer convention.
// Exposes a subset of the AnimationPlayer API (play/pause/stop/seek/speed_scale)
// so BSP and SPR animations can be driven identically from game code.
static const char *TEXTURE_ANIMATION_PLAYER_SCRIPT =
	"extends Node\n"
	"@export var fps: float = 10.0\n"
	"var speed_scale: float = 1.0\n"
	"var _playing: bool = false\n"
	"var _animated := []\n"
	"var _tick := 0.0\n"
	"var _last_frame_idx := -1\n"
	"\n"
	"func _ready() -> void:\n"
	"\t_find_animated(get_parent())\n"
	"\tif not _animated.is_empty():\n"
	"\t\tplay()\n"
	"\n"
	"func play(_anim_name: String = \"\", _blend: float = -1.0, _speed: float = 1.0, _from_end: bool = false) -> void:\n"
	"\t_playing = true\n"
	"\tset_process(true)\n"
	"\n"
	"func pause() -> void:\n"
	"\t_playing = false\n"
	"\tset_process(false)\n"
	"\n"
	"func stop(keep_state: bool = true) -> void:\n"
	"\t_playing = false\n"
	"\tset_process(false)\n"
	"\tif not keep_state:\n"
	"\t\t_tick = 0.0\n"
	"\t\t_last_frame_idx = -1\n"
	"\n"
	"func is_playing() -> bool:\n"
	"\treturn _playing\n"
	"\n"
	"func seek(seconds: float, update: bool = false) -> void:\n"
	"\t_tick = seconds\n"
	"\t_last_frame_idx = -1\n"
	"\tif update:\n"
	"\t\t_tick_frame()\n"
	"\n"
	"func _find_animated(node: Node) -> void:\n"
	"\tfor child in node.get_children():\n"
	"\t\tif child == self:\n"
	"\t\t\tcontinue\n"
	"\t\tif child is MeshInstance3D and child.has_meta(\"tex_anim_frames\"):\n"
	"\t\t\t_animated.append({mi = child, frames = child.get_meta(\"tex_anim_frames\")})\n"
	"\t\t_find_animated(child)\n"
	"\n"
	"func _process(delta: float) -> void:\n"
	"\t_tick += delta * speed_scale\n"
	"\t_tick_frame()\n"
	"\n"
	"func _tick_frame() -> void:\n"
	"\tvar frame_idx := int(_tick * fps)\n"
	"\tif frame_idx == _last_frame_idx:\n"
	"\t\treturn\n"
	"\t_last_frame_idx = frame_idx\n"
	"\tfor entry in _animated:\n"
	"\t\t_apply_frame(entry.mi, entry.frames[frame_idx % entry.frames.size()])\n"
	"\n"
	"func _apply_frame(mi: MeshInstance3D, tex: ImageTexture) -> void:\n"
	"\tfor s in mi.mesh.get_surface_count():\n"
	"\t\tvar mat := mi.get_active_material(s)\n"
	"\t\tif mat is ShaderMaterial:\n"
	"\t\t\tmat.set_shader_parameter(\"albedo_texture\", tex)\n"
	"\t\telif mat is StandardMaterial3D:\n"
	"\t\t\tmat.set_texture(BaseMaterial3D.TEXTURE_ALBEDO, tex)\n";

// Write raw (unweighted) lightmap data for a single layer into an atlas
static void write_layer_pixels(
	int layer_index, const uint8_t styles[4],
	int lightmap_offset, int lm_width, int lm_height,
	int atlas_x, int atlas_y, int atlas_width, int atlas_height,
	uint8_t *dest, const vector<uint8_t> &lighting) {

	// Find which style slot this layer corresponds to
	if (layer_index >= 4 || styles[layer_index] >= 64) return;

	size_t pixel_count = (size_t)lm_width * lm_height;
	size_t layer_ofs = (size_t)lightmap_offset + (size_t)layer_index * pixel_count * 3;

	for (size_t p = 0; p < pixel_count; p++) {
		size_t src = layer_ofs + p * 3;
		if (src + 2 >= lighting.size()) break;
		int px = atlas_x + (int)(p % lm_width);
		int py = atlas_y + (int)(p / lm_width);
		if (px >= atlas_width || py < 0 || py >= atlas_height) continue;
		size_t dst = ((size_t)py * atlas_width + px) * 3;
		dest[dst + 0] = lighting[src + 0];
		dest[dst + 1] = lighting[src + 1];
		dest[dst + 2] = lighting[src + 2];
	}
}

// Recursively collect all parsed face indices reachable from a BSP node subtree
static void collect_faces_recursive(
	const std::vector<goldsrc::BSPNode> &nodes,
	const std::vector<int> &raw_to_parsed,
	int node_index,
	std::vector<int> &out_faces)
{
	if (node_index < 0) return; // leaf — no faces stored on leaves
	if ((size_t)node_index >= nodes.size()) return;

	const auto &node = nodes[node_index];

	// Collect this node's own faces
	for (uint16_t i = 0; i < node.numfaces; i++) {
		int raw_idx = (int)node.firstface + i;
		if (raw_idx < (int)raw_to_parsed.size()) {
			int parsed = raw_to_parsed[raw_idx];
			if (parsed >= 0) {
				out_faces.push_back(parsed);
			}
		}
	}

	// Recurse into children (positive = node, negative = leaf)
	for (int c = 0; c < 2; c++) {
		collect_faces_recursive(nodes, raw_to_parsed, node.children[c], out_faces);
	}
}

// Collect all BSP leaf indices reachable from a subtree.
static void collect_leaves_recursive(
	const std::vector<goldsrc::BSPNode> &nodes,
	const std::vector<goldsrc::BSPLeaf> &leafs,
	int node_index,
	std::vector<int> &out_leaves)
{
	if (node_index < 0) {
		int leaf_idx = -(node_index + 1);
		if ((size_t)leaf_idx < leafs.size())
			out_leaves.push_back(leaf_idx);
		return;
	}
	if ((size_t)node_index >= nodes.size()) return;
	const auto &node = nodes[node_index];
	collect_leaves_recursive(nodes, leafs, node.children[0], out_leaves);
	collect_leaves_recursive(nodes, leafs, node.children[1], out_leaves);
}

// Walk BSP tree to target_depth, collecting face groups and leaf sets from subtrees.
static void collect_spatial_groups(
	const std::vector<goldsrc::BSPNode> &nodes,
	const std::vector<goldsrc::BSPLeaf> &leafs,
	const std::vector<int> &raw_to_parsed,
	int node_index, int current_depth, int target_depth,
	std::vector<std::vector<int>> &groups,
	std::vector<std::vector<int>> &leaf_groups)
{
	if (node_index < 0) return; // leaf — handled by collect_leaves_recursive
	if ((size_t)node_index >= nodes.size()) return;

	const auto &node = nodes[node_index];

	if (current_depth >= target_depth) {
		// This subtree becomes one spatial group
		groups.emplace_back();
		collect_faces_recursive(nodes, raw_to_parsed, node_index, groups.back());
		leaf_groups.emplace_back();
		collect_leaves_recursive(nodes, leafs, node_index, leaf_groups.back());
		return;
	}

	// Faces on this node's splitting plane get their own group with pvs_leaves =
	// all descendant leaves, so they participate in PVS culling like any other face.
	if (node.numfaces > 0) {
		std::vector<int> node_faces;
		for (uint16_t i = 0; i < node.numfaces; i++) {
			int raw_idx = (int)node.firstface + i;
			if (raw_idx < (int)raw_to_parsed.size()) {
				int parsed = raw_to_parsed[raw_idx];
				if (parsed >= 0)
					node_faces.push_back(parsed);
			}
		}
		if (!node_faces.empty()) {
			groups.push_back(std::move(node_faces));
			std::vector<int> subtree_leaves;
			collect_leaves_recursive(nodes, leafs, node_index, subtree_leaves);
			leaf_groups.push_back(std::move(subtree_leaves));
		}
	}

	// Recurse into children
	for (int c = 0; c < 2; c++) {
		collect_spatial_groups(nodes, leafs, raw_to_parsed, node.children[c],
			current_depth + 1, target_depth, groups, leaf_groups);
	}
}

} // anonymous namespace

GoldSrcBSP::GoldSrcBSP() {
	for (int i = 0; i < 64; i++) {
		lightstyle_values[i] = 1.0f;
	}
}

void GoldSrcBSP::_bind_methods() {
	ClassDB::bind_method(D_METHOD("load_bsp", "path"), &GoldSrcBSP::load_bsp);
	ClassDB::bind_method(D_METHOD("load_bsp_from_data", "data"), &GoldSrcBSP::load_bsp_from_data);
	ClassDB::bind_method(D_METHOD("get_bsp_blob"), &GoldSrcBSP::get_bsp_blob);
	ClassDB::bind_method(D_METHOD("get_pvs_blob"), &GoldSrcBSP::get_pvs_blob);
	ClassDB::bind_method(D_METHOD("set_wad", "wad"), &GoldSrcBSP::set_wad);
	ClassDB::bind_method(D_METHOD("add_wad", "wad"), &GoldSrcBSP::add_wad);
	ClassDB::bind_method(D_METHOD("get_entities"), &GoldSrcBSP::get_entities);
	ClassDB::bind_method(D_METHOD("build_mesh"), &GoldSrcBSP::build_mesh);
	ClassDB::bind_method(D_METHOD("set_scale_factor", "scale"), &GoldSrcBSP::set_scale_factor);
	ClassDB::bind_method(D_METHOD("get_scale_factor"), &GoldSrcBSP::get_scale_factor);

	ClassDB::bind_method(D_METHOD("set_lightstyle", "index", "brightness"), &GoldSrcBSP::set_lightstyle);
	ClassDB::bind_method(D_METHOD("get_lightstyle", "index"), &GoldSrcBSP::get_lightstyle);
	ClassDB::bind_method(D_METHOD("get_lightstyle_image"), &GoldSrcBSP::get_lightstyle_image);
	ClassDB::bind_method(D_METHOD("get_lightstyle_texture"), &GoldSrcBSP::get_lightstyle_texture);
	ClassDB::bind_method(D_METHOD("point_contents", "position"), &GoldSrcBSP::point_contents);
	ClassDB::bind_method(D_METHOD("point_to_leaf", "position"), &GoldSrcBSP::point_to_leaf);
	ClassDB::bind_method(D_METHOD("get_leaf_count"), &GoldSrcBSP::get_leaf_count);
	ClassDB::bind_method(D_METHOD("get_leaf_pvs", "leaf_index"), &GoldSrcBSP::get_leaf_pvs);
	ClassDB::bind_method(D_METHOD("get_leaves_in_aabb", "aabb"), &GoldSrcBSP::get_leaves_in_aabb);
	ClassDB::bind_method(D_METHOD("get_texture", "name"), &GoldSrcBSP::get_texture);
	ClassDB::bind_method(D_METHOD("get_face_axes", "position", "normal"), &GoldSrcBSP::get_face_axes);
	ClassDB::bind_method(D_METHOD("set_shader_lightstyles", "enabled"), &GoldSrcBSP::set_shader_lightstyles);
	ClassDB::bind_method(D_METHOD("get_shader_lightstyles"), &GoldSrcBSP::get_shader_lightstyles);
	ClassDB::bind_method(D_METHOD("bake_light_grid", "cell_size"), &GoldSrcBSP::bake_light_grid);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "scale_factor"), "set_scale_factor", "get_scale_factor");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "shader_lightstyles"), "set_shader_lightstyles", "get_shader_lightstyles");
}

Error GoldSrcBSP::load_bsp(const String &path) {
	Ref<FileAccess> file = FileAccess::open(path, FileAccess::READ);
	if (file.is_null()) {
		UtilityFunctions::printerr("[GoldSrc] Cannot open BSP file: ", path);
		return ERR_FILE_NOT_FOUND;
	}

	int64_t len = file->get_length();
	PackedByteArray data = file->get_buffer(len);

	parser = make_unique<goldsrc::BSPParser>();
	if (!parser->parse(data.ptr(), data.size())) {
		UtilityFunctions::printerr("[GoldSrc] Failed to parse BSP file: ", path);
		parser.reset();
		return ERR_PARSE_ERROR;
	}

	mesh_built = false;
	UtilityFunctions::print("[GoldSrc] Loaded BSP: ", path,
		" (", (int64_t)parser->get_data().faces.size(), " faces, ",
		(int64_t)parser->get_data().entities.size(), " entities)");
	return OK;
}

Error GoldSrcBSP::load_bsp_from_data(const PackedByteArray &data) {
	if (data.is_empty()) return ERR_INVALID_DATA;
	parser = make_unique<goldsrc::BSPParser>();
	if (!parser->parse(data.ptr(), data.size())) {
		parser.reset();
		return ERR_PARSE_ERROR;
	}
	mesh_built = false;
	return OK;
}

PackedByteArray GoldSrcBSP::get_bsp_blob() const {
	if (!parser) return PackedByteArray();
	const auto &d = parser->get_data();
	if (d.nodes.empty() || d.leafs.empty()) return PackedByteArray();

	size_t planes_sz = d.planes.size()     * sizeof(goldsrc::BSPPlane);
	size_t vis_sz    = d.visibility.size();
	size_t nodes_sz  = d.nodes.size()      * sizeof(goldsrc::BSPNode);
	size_t clips_sz  = d.clipnodes.size()  * sizeof(goldsrc::BSPClipNode);
	size_t leafs_sz  = d.leafs.size()      * sizeof(goldsrc::BSPLeaf);
	size_t models_sz = d.models.size()     * sizeof(goldsrc::BSPModel);
	size_t hdr_sz    = sizeof(goldsrc::BSPHeader);
	size_t total     = hdr_sz + planes_sz + vis_sz + nodes_sz + clips_sz + leafs_sz + models_sz;

	PackedByteArray out;
	out.resize((int64_t)total);
	uint8_t *p = out.ptrw();
	memset(p, 0, total);

	auto *hdr = reinterpret_cast<goldsrc::BSPHeader *>(p);
	hdr->version = goldsrc::HLBSP_VERSION;

	size_t cur = hdr_sz;
	auto write_lump = [&](int idx, const void *data, size_t sz) {
		if (sz == 0) return;
		hdr->lumps[idx].fileofs = (int32_t)cur;
		hdr->lumps[idx].filelen = (int32_t)sz;
		memcpy(p + cur, data, sz);
		cur += sz;
	};

	write_lump(goldsrc::LUMP_PLANES,     d.planes.data(),     planes_sz);
	write_lump(goldsrc::LUMP_VISIBILITY, d.visibility.data(), vis_sz);
	write_lump(goldsrc::LUMP_NODES,      d.nodes.data(),      nodes_sz);
	write_lump(goldsrc::LUMP_CLIPNODES,  d.clipnodes.data(),  clips_sz);
	write_lump(goldsrc::LUMP_LEAFS,      d.leafs.data(),      leafs_sz);
	write_lump(goldsrc::LUMP_MODELS,     d.models.data(),     models_sz);

	return out;
}

void GoldSrcBSP::set_wad(const Ref<GoldSrcWAD> &wad) {
	wads.clear();
	if (wad.is_valid()) {
		wads.push_back(wad);
	}
}

void GoldSrcBSP::add_wad(const Ref<GoldSrcWAD> &wad) {
	if (wad.is_valid()) {
		wads.push_back(wad);
	}
}

void GoldSrcBSP::set_scale_factor(float scale) {
	scale_factor = scale;
}

float GoldSrcBSP::get_scale_factor() const {
	return scale_factor;
}

void GoldSrcBSP::set_shader_lightstyles(bool enabled) {
	shader_lightstyles = enabled;
}

bool GoldSrcBSP::get_shader_lightstyles() const {
	return shader_lightstyles;
}

int GoldSrcBSP::point_contents(Vector3 godot_pos) const {
	if (!parser) return goldsrc::CONTENTS_EMPTY;
	const auto &bsp_data = parser->get_data();
	if (bsp_data.nodes.empty() || bsp_data.leafs.empty()) return goldsrc::CONTENTS_EMPTY;

	// Convert Godot coords back to GoldSrc: godot=(-x*s, z*s, y*s)
	float gs_x = -godot_pos.x / scale_factor;
	float gs_y = godot_pos.z / scale_factor;
	float gs_z = godot_pos.y / scale_factor;

	// Walk hull 0 BSP node tree
	int node_index = bsp_data.models[0].headnode[0];
	while (node_index >= 0) {
		if ((size_t)node_index >= bsp_data.nodes.size()) return goldsrc::CONTENTS_EMPTY;
		const auto &node = bsp_data.nodes[node_index];
		if (node.planenum < 0 || (size_t)node.planenum >= bsp_data.planes.size())
			return goldsrc::CONTENTS_EMPTY;
		const auto &plane = bsp_data.planes[node.planenum];
		float dot = plane.normal[0] * gs_x + plane.normal[1] * gs_y + plane.normal[2] * gs_z;
		if (dot >= plane.dist)
			node_index = node.children[0];
		else
			node_index = node.children[1];
	}
	int leaf_index = -(node_index + 1);
	if (leaf_index < 0 || (size_t)leaf_index >= bsp_data.leafs.size())
		return goldsrc::CONTENTS_EMPTY;
	return bsp_data.leafs[leaf_index].contents;
}

int GoldSrcBSP::point_to_leaf(Vector3 godot_pos) const {
	if (!parser) return -1;
	const auto &bsp_data = parser->get_data();
	if (bsp_data.nodes.empty() || bsp_data.leafs.empty()) return -1;

	float gs_x = -godot_pos.x / scale_factor;
	float gs_y =  godot_pos.z / scale_factor;
	float gs_z =  godot_pos.y / scale_factor;

	int node_index = bsp_data.models[0].headnode[0];
	while (node_index >= 0) {
		if ((size_t)node_index >= bsp_data.nodes.size()) return -1;
		const auto &node = bsp_data.nodes[node_index];
		const auto &plane = bsp_data.planes[node.planenum];
		float dot = plane.normal[0]*gs_x + plane.normal[1]*gs_y + plane.normal[2]*gs_z;
		node_index = node.children[dot < plane.dist ? 1 : 0];
	}
	int leaf = -(node_index + 1);
	return ((size_t)leaf < bsp_data.leafs.size()) ? leaf : -1;
}

int GoldSrcBSP::get_leaf_count() const {
	if (!parser) return 0;
	return (int)parser->get_data().leafs.size();
}

PackedInt32Array GoldSrcBSP::get_leaf_pvs(int leaf_index) const {
	PackedInt32Array result;
	if (!parser) return result;
	const auto &bsp_data = parser->get_data();
	const auto pvs = bsp_data.decompress_pvs(leaf_index);
	for (int i = 0; i < (int)pvs.size(); i++)
		if (pvs[i]) result.append(i);
	return result;
}

void GoldSrcBSP::_collect_leaves_in_aabb(int node_idx, const float gs_min[3], const float gs_max[3],
		PackedInt32Array &result) const {
	if (node_idx < 0) {
		// Leaf node: index = ~node_idx (GoldSrc uses -(idx+1))
		int leaf = -(node_idx + 1);
		const auto &bsp_data = parser->get_data();
		if (leaf >= 0 && leaf < (int)bsp_data.leafs.size())
			result.append(leaf);
		return;
	}
	const auto &bsp_data = parser->get_data();
	if (node_idx >= (int)bsp_data.nodes.size()) return;
	const auto &node = bsp_data.nodes[node_idx];
	const auto &plane = bsp_data.planes[node.planenum];

	// Compute the projection interval [d_near, d_far] of the AABB onto the plane normal.
	// d_near is the minimum projection (back-side extreme), d_far is the maximum (front-side extreme).
	float d_near = 0.0f, d_far = 0.0f;
	for (int i = 0; i < 3; i++) {
		if (plane.normal[i] >= 0.0f) {
			d_near += plane.normal[i] * gs_min[i];
			d_far  += plane.normal[i] * gs_max[i];
		} else {
			d_near += plane.normal[i] * gs_max[i];
			d_far  += plane.normal[i] * gs_min[i];
		}
	}

	// children[0] = front (dot >= dist), children[1] = back (dot < dist)
	if (d_far < plane.dist)
		_collect_leaves_in_aabb(node.children[1], gs_min, gs_max, result);
	else if (d_near >= plane.dist)
		_collect_leaves_in_aabb(node.children[0], gs_min, gs_max, result);
	else {
		_collect_leaves_in_aabb(node.children[0], gs_min, gs_max, result);
		_collect_leaves_in_aabb(node.children[1], gs_min, gs_max, result);
	}
}

PackedInt32Array GoldSrcBSP::get_leaves_in_aabb(AABB godot_aabb) const {
	PackedInt32Array result;
	if (!parser) return result;
	const auto &bsp_data = parser->get_data();
	if (bsp_data.nodes.empty() || bsp_data.leafs.empty()) return result;

	// Convert Godot AABB to GoldSrc space.
	// Godot → GoldSrc: gs_x = -godot_x/sf, gs_y = godot_z/sf, gs_z = godot_y/sf
	// The x-axis negation swaps min/max for that axis.
	Vector3 g_min = godot_aabb.position;
	Vector3 g_max = godot_aabb.position + godot_aabb.size;

	float gs_min[3] = {
		-g_max.x / scale_factor,   // gs X: negated, so godot max_x → gs min_x
		 g_min.z / scale_factor,   // gs Y = godot Z
		 g_min.y / scale_factor,   // gs Z = godot Y
	};
	float gs_max[3] = {
		-g_min.x / scale_factor,
		 g_max.z / scale_factor,
		 g_max.y / scale_factor,
	};

	_collect_leaves_in_aabb(bsp_data.models[0].headnode[0], gs_min, gs_max, result);
	return result;
}

Ref<ImageTexture> GoldSrcBSP::find_texture(const string &name) const {
	string lower_name = str_to_lower(name);

	// Check cache first
	auto it = texture_cache.find(lower_name);
	if (it != texture_cache.end()) {
		return it->second;
	}

	// Check BSP embedded textures (case-insensitive, matching GoldSrc behavior)
	const auto &bsp_data = parser->get_data();
	for (const auto &tex : bsp_data.textures) {
		if (tex.has_data && str_to_lower(tex.name) == lower_name) {
			PackedByteArray pixels;
			pixels.resize(tex.data.size());
			memcpy(pixels.ptrw(), tex.data.data(), tex.data.size());
			Ref<Image> img = Image::create_from_data(tex.width, tex.height,
				false, Image::FORMAT_RGBA8, pixels);
			img->generate_mipmaps();
			Ref<ImageTexture> result = ImageTexture::create_from_image(img);
			texture_cache[lower_name] = result;
			return result;
		}
	}

	// Then check WADs
	String gname = String(name.c_str());
	for (const auto &wad : wads) {
		if (wad->has_texture(gname)) {
			Ref<ImageTexture> result = wad->get_texture(gname);
			texture_cache[lower_name] = result;
			return result;
		}
	}

	texture_cache[lower_name] = Ref<ImageTexture>();
	return Ref<ImageTexture>();
}

Ref<ImageTexture> GoldSrcBSP::get_texture(const String &name) const {
	if (!parser) return Ref<ImageTexture>();
	return find_texture(string(name.utf8().get_data()));
}

Array GoldSrcBSP::get_face_axes(Vector3 godot_pos, Vector3 godot_normal) const {
	if (!parser) return Array();

	const auto &faces = parser->get_data().faces;

	// Convert Godot position/normal back to GoldSrc coords
	// Forward: GS(x,y,z) -> Godot(-x*sf, z*sf, y*sf)
	// Inverse: Godot(x,y,z) -> GS(-x/sf, z/sf, y/sf)
	float sf = scale_factor;
	float gx = -godot_pos.x / sf;
	float gy = godot_pos.z / sf;
	float gz = godot_pos.y / sf;
	float gnx = -godot_normal.x;
	float gny = godot_normal.z;
	float gnz = godot_normal.y;

	// Find the worldspawn face closest to this position with matching normal
	float best_dist = 1e30f;
	const goldsrc::ParsedFace *best_face = nullptr;

	for (const auto &face : faces) {
		if (face.model_index != 0) continue; // worldspawn only

		// Check normal match (dot product > 0.9)
		float dot = face.normal[0] * gnx + face.normal[1] * gny + face.normal[2] * gnz;
		if (dot < 0.9f) continue;

		// Find closest point on face to our query point
		float min_dist = 1e30f;
		for (const auto &v : face.vertices) {
			float dx = v.pos[0] - gx;
			float dy = v.pos[1] - gy;
			float dz = v.pos[2] - gz;
			float d = dx*dx + dy*dy + dz*dz;
			if (d < min_dist) min_dist = d;
		}
		if (min_dist < best_dist) {
			best_dist = min_dist;
			best_face = &face;
		}
	}

	if (!best_face) return Array();

	// GoldSrc decals use the surface's texinfo S/T axes (normalized, no scale).
	// Convert from GoldSrc (x,y,z) to Godot (-x, z, y) for directions.
	Vector3 s_godot(-best_face->s_axis[0], best_face->s_axis[2], best_face->s_axis[1]);
	Vector3 t_godot(-best_face->t_axis[0], best_face->t_axis[2], best_face->t_axis[1]);

	Array result;
	result.push_back(s_godot.normalized());
	result.push_back(t_godot.normalized());
	return result;
}

Array GoldSrcBSP::get_entities() const {
	Array result;
	if (!parser) return result;

	for (const auto &ent : parser->get_data().entities) {
		Dictionary dict;
		for (const auto &kv : ent.properties) {
			dict[String(kv.first.c_str())] = String(kv.second.c_str());
		}
		result.push_back(dict);
	}
	return result;
}

// Convert a GoldSrc position to Godot (Z-up → Y-up, flip X for handedness)
Vector3 GoldSrcBSP::goldsrc_to_godot(float x, float y, float z) const {
	return Vector3(-x * scale_factor, z * scale_factor, y * scale_factor);
}

// --- Shelf-based lightmap atlas packer ---

namespace {

struct ShelfPacker {
	int width, height;
	int shelf_x, shelf_y, shelf_height;
	static const int PADDING = 1;
	static const int MAX_SIZE = 4096;

	ShelfPacker(int initial_size = 512) :
		width(initial_size), height(initial_size),
		shelf_x(0), shelf_y(0), shelf_height(0) {}

	bool pack(int w, int h, int &out_x, int &out_y) {
		int pw = w + PADDING;
		int ph = h + PADDING;

		// Try current shelf
		if (shelf_x + pw <= width && shelf_y + ph <= height) {
			out_x = shelf_x;
			out_y = shelf_y;
			shelf_x += pw;
			if (ph > shelf_height) shelf_height = ph;
			return true;
		}

		// Start new shelf
		int new_y = shelf_y + shelf_height;
		if (new_y + ph <= height && pw <= width) {
			shelf_x = pw;
			shelf_y = new_y;
			shelf_height = ph;
			out_x = 0;
			out_y = new_y;
			return true;
		}

		// Grow atlas
		while (width < MAX_SIZE || height < MAX_SIZE) {
			if (width <= height && width < MAX_SIZE) {
				width *= 2;
			} else if (height < MAX_SIZE) {
				height *= 2;
			} else {
				break;
			}

			// Retry new shelf in grown atlas
			new_y = shelf_y + shelf_height;
			if (new_y + ph <= height && pw <= width) {
				shelf_x = pw;
				shelf_y = new_y;
				shelf_height = ph;
				out_x = 0;
				out_y = new_y;
				return true;
			}
		}

		return false; // won't fit
	}

	int used_height() const {
		return shelf_y + shelf_height;
	}
};

static void bake_lightmap_pixels(
	const uint8_t styles[4], int lightmap_offset, int lm_width, int lm_height,
	int atlas_x, int atlas_y, int atlas_width, int atlas_height,
	uint8_t *dest, const vector<uint8_t> &lighting, const float *lightstyle_values) {

	size_t pixel_count = (size_t)lm_width * lm_height;
	for (size_t p = 0; p < pixel_count; p++) {
		float r = 0, g = 0, b = 0;
		for (int s = 0; s < 4; s++) {
			if (styles[s] >= 64) break;
			float brightness = lightstyle_values[styles[s]];
			size_t ofs = (size_t)lightmap_offset + (size_t)s * pixel_count * 3 + p * 3;
			if (ofs + 2 >= lighting.size()) break;
			r += lighting[ofs + 0] * brightness;
			g += lighting[ofs + 1] * brightness;
			b += lighting[ofs + 2] * brightness;
		}
		int px = atlas_x + (int)(p % lm_width);
		int py = atlas_y + (int)(p / lm_width);
		if (px >= atlas_width || py < 0 || py >= atlas_height) continue;
		size_t dst = ((size_t)py * atlas_width + px) * 3;
		dest[dst + 0] = (uint8_t)clamp(r, 0.0f, 255.0f);
		dest[dst + 1] = (uint8_t)clamp(g, 0.0f, 255.0f);
		dest[dst + 2] = (uint8_t)clamp(b, 0.0f, 255.0f);
	}
}

} // anonymous namespace

void GoldSrcBSP::build_mesh() {
	if (!parser) return;
	if (mesh_built) return;
	mesh_built = true;

	// The stripped BSP rides on the tree root, once, for both the PVS reader and
	// the hull trace. Bodies carry only a model index (see mark_bsp_carrier); a
	// consumer walks up from a body to find this. Set before the model loop so
	// nothing has to remember to do it afterwards.
	set_meta("bsp_data", get_bsp_blob());

	const auto &bsp_data = parser->get_data();
	uint64_t t0 = Time::get_singleton()->get_ticks_msec();
	uint64_t t_last = t0;
	auto log_timing = [&](const char *label) {
		uint64_t now = Time::get_singleton()->get_ticks_msec();
		UtilityFunctions::print("[GoldSrc] ", label, ": ",
			(int64_t)(now - t_last), "ms (total ",
			(int64_t)(now - t0), "ms)");
		t_last = now;
	};

	// Initialize face_lm_info for legacy path
	if (!shader_lightstyles) {
		face_lm_info.resize(bsp_data.faces.size());
	}

	// Count faces that actually receive a lightmap, independent of which path
	// bakes them. lm_atlases is only populated in the legacy path, so it can't
	// be used to report lightmap coverage when shader_lightstyles is on.
	int lit_face_count = 0;

	// Sky surface shader: always created, used for all 'sky*' texture faces
	Ref<Shader> sky_shader;
	sky_shader.instantiate();
	sky_shader->set_code(SKY_SURFACE_SHADER_CODE);

	// Water surface shader: turbulent warp for '!' and '*' texture faces
	Ref<Shader> water_shader;
	water_shader.instantiate();
	water_shader->set_code(WATER_SHADER_CODE);

	// Water's depth-only prepass. Carries no per-texture state, so one material is shared by
	// every water twin in the model; render_priority -1 is what puts it ahead of the color
	// pass in the transparent queue, which is the whole point of it.
	Ref<ShaderMaterial> water_depth_material;
	{
		Ref<Shader> water_depth_shader;
		water_depth_shader.instantiate();
		water_depth_shader->set_code(WATER_DEPTH_SHADER_CODE);
		water_depth_material.instantiate();
		water_depth_material->set_shader(water_depth_shader);
		water_depth_material->set_render_priority(-1);
	}

	// Create shared lightstyle brightness texture for shader path
	Ref<Shader> opaque_shader;
	Ref<Shader> alpha_shader;
	Ref<ImageTexture> black_1x1;  // shared black texture for unused layers
	if (shader_lightstyles) {
		lightstyle_image = Image::create(64, 1, false, Image::FORMAT_RF);
		for (int i = 0; i < 64; i++) {
			lightstyle_image->set_pixel(i, 0, Color(lightstyle_values[i], 0, 0, 1));
		}
		lightstyle_texture = ImageTexture::create_from_image(lightstyle_image);

		opaque_shader.instantiate();
		opaque_shader->set_code(LIGHTSTYLE_SHADER_CODE);
		alpha_shader.instantiate();
		alpha_shader->set_code(LIGHTSTYLE_SHADER_ALPHA_CODE);

		Ref<Image> black_img = Image::create(1, 1, false, Image::FORMAT_RGB8);
		black_img->set_pixel(0, 0, Color(0, 0, 0));
		black_1x1 = ImageTexture::create_from_image(black_img);
		log_timing("shader setup");
	}

	// Build entity lookup: "*N" → entity properties
	map<string, const goldsrc::ParsedEntity *> model_entities;
	for (const auto &ent : bsp_data.entities) {
		auto it = ent.properties.find("model");
		if (it != ent.properties.end() && !it->second.empty() && it->second[0] == '*') {
			model_entities[it->second] = &ent;
		}
	}

	// Group faces by model index, tracking global face indices
	struct FaceRef {
		const goldsrc::ParsedFace *face;
		int global_index;
	};
	map<int, vector<FaceRef>> model_faces;
	for (int i = 0; i < (int)bsp_data.faces.size(); i++) {
		model_faces[bsp_data.faces[i].model_index].push_back({&bsp_data.faces[i], i});
	}

	// World-face plane index for liquid coincidence culling. A brush-entity liquid volume
	// (func_water) keeps every brush face, including the ones flush against the pool's world
	// walls/floor/ceiling — a translucent liquid face sitting exactly on a coincident world
	// face speckles (z-fight) because two coplanar faces from different nodes never agree on
	// depth. GoldSrc's engine draws only the leaf-boundary faces, so those coincident pairs
	// don't exist for it. We replicate that: skip a liquid face when a WORLD (model 0) face
	// lies on the same geometric plane AND overlaps it in space — the opaque world face is
	// already drawn there, so nothing is lost, and the water/air surface (no world face above
	// it) is kept. Plane key is sign-independent: coincident world/liquid faces face opposite
	// ways (solid points into the cavity, liquid points out), so we fold ±normal together.
	struct FaceAABB { float mn[3]; float mx[3]; };
	auto plane_key = [](const goldsrc::ParsedFace *fc) -> std::tuple<int,int,int,int> {
		float n[3] = { fc->normal[0], fc->normal[1], fc->normal[2] };
		float d = n[0]*fc->vertices[0].pos[0] + n[1]*fc->vertices[0].pos[1] + n[2]*fc->vertices[0].pos[2];
		float sign = 1.0f;                       // canonicalize: first significant comp positive
		for (int c = 0; c < 3; c++) { if (fabsf(n[c]) > 1e-4f) { sign = (n[c] < 0) ? -1.0f : 1.0f; break; } }
		return std::make_tuple((int)lroundf(n[0]*sign*100.0f), (int)lroundf(n[1]*sign*100.0f),
		                       (int)lroundf(n[2]*sign*100.0f), (int)lroundf(d*sign*2.0f));
	};
	auto face_aabb = [](const goldsrc::ParsedFace *fc) -> FaceAABB {
		FaceAABB b; for (int k = 0; k < 3; k++) { b.mn[k] = 1e30f; b.mx[k] = -1e30f; }
		for (const auto &v : fc->vertices)
			for (int k = 0; k < 3; k++) { b.mn[k] = std::min(b.mn[k], v.pos[k]); b.mx[k] = std::max(b.mx[k], v.pos[k]); }
		return b;
	};
	// Only OPAQUE worldspawn faces go in the map: liquid/sky world faces must not be a cull
	// target (worldspawn water is already compiler-deduped and is the real water surface — an
	// entity liquid must never be culled against it, nor a worldspawn liquid face against
	// itself, which would erase maps whose water is a plain worldspawn brush, e.g. ww_hunt).
	map<std::tuple<int,int,int,int>, vector<FaceAABB>> world_plane_aabbs;
	for (const auto &fc : bsp_data.faces) {
		if (fc.model_index != 0 || fc.vertices.empty()) continue;
		const std::string &tn = fc.texture_name;
		bool liquid_or_sky = (!tn.empty() && (tn[0] == '!' || tn[0] == '*')) || goldsrc::is_sky_texture(tn);
		if (liquid_or_sky) continue;
		world_plane_aabbs[plane_key(&fc)].push_back(face_aabb(&fc));
	}

	int num_models = (int)bsp_data.models.size();
	bool has_anim = false;

	for (int m = 0; m < num_models; m++) {
		auto mf_it = model_faces.find(m);
		if (mf_it == model_faces.end()) continue;

		const auto &faces_for_model = mf_it->second;
		if (faces_for_model.empty()) continue;

		// Determine entity info for brush models
		string classname = "worldspawn";
		string targetname;
		string model_key = (m > 0) ? string("*") + std::to_string(m) : "";
		if (m > 0) {
			auto ent_it = model_entities.find(model_key);
			if (ent_it != model_entities.end()) {
				auto cn = ent_it->second->properties.find("classname");
				if (cn != ent_it->second->properties.end()) classname = cn->second;
				auto tn = ent_it->second->properties.find("targetname");
				if (tn != ent_it->second->properties.end()) targetname = tn->second;
			}
		}

		// A func_water brush is liquid whatever its texture is called. Half the game's water
		// is on unprefixed names the '!'/'*' test below cannot see — water4b (ww_2fort,
		// ww_countryside, ww_ravine2), scrollwater1/2 (ww_osaka, ww_ravine2) — and those maps
		// rendered their pools as ordinary opaque brushwork with an alpha slapped on top.
		const bool ent_is_liquid = (classname == "func_water");
		// A brush entity names its liquid in "skin"; worldspawn liquid has no entity and is
		// read per face from the leaf it borders (ParsedFace::liquid_contents).
		int ent_skin = goldsrc::CONTENTS_WATER;
		// Crest height in GoldSrc units. Waves are an ENTITY feature — CFuncDoor::KeyValue puts
		// WaveHeight into pev->scale and the engine bends only that brush — so worldspawn liquid
		// stays flat and, more to the point, stays untessellated: ww_leyline's 655 faces and
		// ww_golem's 568 pay nothing for a feature their maps never asked for.
		float ent_wave = 0.0f;
		if (m > 0) {
			auto ent_it = model_entities.find(model_key);
			if (ent_it != model_entities.end()) {
				auto sk = ent_it->second->properties.find("skin");
				if (sk != ent_it->second->properties.end()) ent_skin = atoi(sk->second.c_str());
				auto wh = ent_it->second->properties.find("WaveHeight");
				if (wh != ent_it->second->properties.end()) ent_wave = (float)atof(wh->second.c_str());
			}
		}
		// A mapper's declared 0 means flat and must survive: ww_matrox has an ice brush at
		// WaveHeight 5 and another at 0, and ww_countryside freezes its ice the same way.
		const bool waves = ent_is_liquid && ent_wave > 0.0f;

		// Create a node for this model. Worldspawn is a plain container; brush
		// entities are AnimatableBody3D roots so their entity transform is the
		// physical/rendered brush transform.
		Node3D *model_node = nullptr;
		AnimatableBody3D *body_node = nullptr;
		if (m == 0) {
			model_node = memnew(Node3D);
		} else {
			body_node = memnew(AnimatableBody3D);
			body_node->set_collision_mask(0);
			model_node = body_node;
		}
		String node_name;
		if (m == 0) {
			node_name = "worldspawn";
		} else {
			node_name = String(classname.c_str());
			if (!targetname.empty()) {
				node_name = node_name + "_" + String(targetname.c_str());
			}
			node_name = node_name + "_" + String::num_int64(m);
		}
		model_node->set_name(node_name);

		// Set origin for brush entities. CSG rebases brush entities that contain
		// an ORIGIN brush: their geometry is stored local to the origin-brush
		// center and the world placement lives in the entity's "origin" key
		// (the model-lump origin field stays zero). Apply the key so the node
		// sits at its pivot and rotating/riding entities work in place.
		if (m > 0) {
			const auto &bmodel = bsp_data.models[m];
			float ox = bmodel.origin[0], oy = bmodel.origin[1], oz = bmodel.origin[2];
			auto ent_it = model_entities.find(model_key);
			if (ent_it != model_entities.end()) {
				auto o = ent_it->second->properties.find("origin");
				if (o != ent_it->second->properties.end()) {
					sscanf(o->second.c_str(), "%f %f %f", &ox, &oy, &oz);
				}
			}
			if (ox != 0 || oy != 0 || oz != 0) {
				model_node->set_position(goldsrc_to_godot(ox, oy, oz));
			}
		}

		// Store entity properties as metadata
		if (m == 0) {
			// Worldspawn: first entity in the lump
			if (!bsp_data.entities.empty()) {
				Dictionary meta;
				for (const auto &kv : bsp_data.entities[0].properties) {
					meta[String(kv.first.c_str())] = String(kv.second.c_str());
				}
				model_node->set_meta("entity", meta);
			}
		} else {
			auto ent_it = model_entities.find(model_key);
			if (ent_it != model_entities.end()) {
				Dictionary meta;
				for (const auto &kv : ent_it->second->properties) {
					meta[String(kv.first.c_str())] = String(kv.second.c_str());
				}
				model_node->set_meta("entity", meta);
			}
		}

		add_child(model_node);

		// --- Build spatial groups for worldspawn (m==0) ---
		// For worldspawn, split faces spatially using BSP tree subtrees,
		// then sub-group by texture. This gives each MeshInstance3D a tight
		// AABB that Godot's occluder culling can actually use.
		// For brush entities (m>0), use a single group with all faces.

		struct SpatialGroup {
			vector<FaceRef> face_refs;
			vector<int> pvs_leaves;
			string label;
		};
		vector<SpatialGroup> spatial_groups;

		if (m == 0 && !bsp_data.nodes.empty() && !bsp_data.raw_to_parsed.empty()) {
			const int TARGET_DEPTH = 5;
			int root_node = bsp_data.models[0].headnode[0];

			vector<vector<int>> bsp_groups;
			vector<vector<int>> bsp_leaf_groups;
			collect_spatial_groups(bsp_data.nodes, bsp_data.leafs, bsp_data.raw_to_parsed,
				root_node, 0, TARGET_DEPTH, bsp_groups, bsp_leaf_groups);

			// Convert parsed face indices to FaceRef vectors
			for (size_t gi = 0; gi < bsp_groups.size(); gi++) {
				SpatialGroup sg;
				sg.label = "spatial_" + std::to_string(gi);
				if (gi < bsp_leaf_groups.size())
					sg.pvs_leaves = bsp_leaf_groups[gi];
				for (int parsed_idx : bsp_groups[gi]) {
					if (parsed_idx >= 0 && parsed_idx < (int)bsp_data.faces.size()) {
						sg.face_refs.push_back({&bsp_data.faces[parsed_idx], parsed_idx});
					}
				}
				if (!sg.face_refs.empty()) {
					spatial_groups.push_back(std::move(sg));
				}
			}


			UtilityFunctions::print("[GoldSrc] Worldspawn spatial split: ",
				(int64_t)spatial_groups.size(), " groups from depth ",
				(int64_t)TARGET_DEPTH);
		} else {
			// Brush entities or fallback: single group with all faces
			SpatialGroup sg;
			sg.label = "";
			sg.face_refs = faces_for_model;
			spatial_groups.push_back(std::move(sg));
		}

		int total_mesh_instances = 0;
		// Sky collision body built alongside rendering — shapes added per sky ArrayMesh.
		StaticBody3D *sky_body = nullptr;
		if (m == 0) {
			sky_body = memnew(StaticBody3D);
			sky_body->set_name("WorldSkyCollision");
			sky_body->set_collision_layer(1 << 8); // layer 9 (MASK_WORLD_SKY)
			sky_body->set_collision_mask(0);
		}

		// For brush entities (m > 0), the entity root itself is the
		// AnimatableBody3D. If all faces of this brush use sky or invisible-wall
		// textures (e.g. func_wall sky dome, {blue chroma-keyed enclosure), put it
		// on MASK_WORLD_SKY so projectiles/hitscans pass through like worldspawn
		// sky. Players still collide (MASK_WORLD_AND_CLIP includes the sky bit).
		if (m > 0) {
			int sky_like_faces = 0, solid_faces = 0;
			for (const auto &fr : faces_for_model) {
				const std::string &tn = fr.face->texture_name;
				if (goldsrc::is_tool_texture(tn)) continue;
				if (goldsrc::is_sky_texture(tn) || goldsrc::is_invisible_wall_texture(tn)) sky_like_faces++;
				else solid_faces++;
			}
			bool sky_brush = sky_like_faces > 0 && solid_faces == 0;
			body_node->set_collision_layer(sky_brush ? (1u << 8) : 1u);
		}

		// --- Lightmap atlas pages for the whole model ---
		// The atlas is what a material's lm_layer* uniforms bind, so the granularity of the
		// atlas IS the granularity of the materials. Packing one atlas per (spatial group,
		// texture) — as this used to — meant the same wall texture in two groups got two
		// materials with identical shaders and identical uniforms but different atlas
		// textures, so nothing could share state and every draw rebound its own lightmap
		// (ww_2fort: 410 meshes, 384 materials, 76 tris per draw call).
		//
		// Packing per model instead lets every face that shares a texture share one material.
		// Pages exist only because ShelfPacker caps a sheet at 4096²; a map whose lighting
		// overflows that gets a second page and splits materials at that seam, which is still
		// far coarser than splitting per group. Typical maps use one page.
		struct FacePack {
			int page = -1;
			int atlas_x = 0, atlas_y = 0;
			bool has_lightmap = false;
			int n_styles = 0;
		};
		struct AtlasPage {
			ShelfPacker packer;
			Ref<ImageTexture> layer_textures[4];  // shader path: one atlas per lightstyle layer
			Ref<ImageTexture> blended;            // legacy path: single pre-blended atlas
			vector<int> face_indices;             // global indices packed into this page
			int w = 0, h = 0;
			int max_styles = 0;
			bool has_lightmap = false;
		};
		std::map<int, FacePack> face_pack_by_index;
		vector<AtlasPage> pages;
		pages.push_back(AtlasPage());

		for (const auto &fr : faces_for_model) {
			const auto *face = fr.face;
			if (goldsrc::is_tool_texture(face->texture_name)) continue;
			if (face->vertices.size() < 3) continue;

			FacePack fp;
			if (face->lightmap_offset >= 0 && face->lightmap_width > 0 &&
				face->lightmap_height > 0 && !bsp_data.lighting.empty()) {

				// Count active styles
				for (int s = 0; s < 4; s++) {
					if (face->styles[s] == 255) break;
					fp.n_styles++;
				}

				size_t lm_size = (size_t)face->lightmap_width * face->lightmap_height * 3;
				size_t total_data = (size_t)fp.n_styles * lm_size;

				if (fp.n_styles > 0 &&
					(size_t)face->lightmap_offset + total_data <= bsp_data.lighting.size()) {
					// Try the open page, then one fresh page. A face only fails the open page
					// once that page has grown to the 4096² cap, so the retry lands unless the
					// face's own lightmap is larger than a whole sheet (then it stays unlit,
					// exactly as it would have before).
					for (int attempt = 0; attempt < 2; attempt++) {
						AtlasPage &pg = pages.back();
						if (pg.packer.pack(face->lightmap_width, face->lightmap_height,
							fp.atlas_x, fp.atlas_y)) {
							fp.page = (int)pages.size() - 1;
							fp.has_lightmap = true;
							pg.has_lightmap = true;
							pg.face_indices.push_back(fr.global_index);
							if (fp.n_styles > pg.max_styles) pg.max_styles = fp.n_styles;
							lit_face_count++;
							break;
						}
						if (attempt == 0) pages.push_back(AtlasPage());
					}

					// Record placement for the legacy runtime-rebake path (shader path
					// re-blends on the GPU from the per-style layers instead).
					if (fp.has_lightmap && !shader_lightstyles) {
						auto &info = face_lm_info[fr.global_index];
						info.atlas_x = fp.atlas_x;
						info.atlas_y = fp.atlas_y;
						info.lm_width = face->lightmap_width;
						info.lm_height = face->lightmap_height;
						memcpy(info.styles, face->styles, 4);
						info.lightmap_offset = face->lightmap_offset;
					}
				}
			}

			face_pack_by_index[fr.global_index] = fp;
		}

		// Bake each page's atlas image(s)
		for (size_t pi = 0; pi < pages.size(); pi++) {
			AtlasPage &pg = pages[pi];
			if (!pg.has_lightmap) continue;
			pg.w = pg.packer.width;
			pg.h = pg.packer.used_height();
			if (pg.h < 1) pg.h = 1;

			if (shader_lightstyles) {
				// Shader path: one atlas per style layer, blended live by the shader
				PackedByteArray layer_pixels[4];
				for (int l = 0; l < pg.max_styles; l++) {
					layer_pixels[l].resize(pg.w * pg.h * 3);
					memset(layer_pixels[l].ptrw(), 0, layer_pixels[l].size());
				}

				for (int gi : pg.face_indices) {
					const FacePack &fp = face_pack_by_index[gi];
					const auto *face = &bsp_data.faces[gi];
					for (int l = 0; l < fp.n_styles; l++) {
						write_layer_pixels(l, face->styles,
							face->lightmap_offset,
							face->lightmap_width, face->lightmap_height,
							fp.atlas_x, fp.atlas_y, pg.w, pg.h,
							layer_pixels[l].ptrw(), bsp_data.lighting);
					}
				}

				for (int l = 0; l < pg.max_styles; l++) {
					Ref<Image> img = Image::create_from_data(
						pg.w, pg.h, false, Image::FORMAT_RGB8, layer_pixels[l]);
					pg.layer_textures[l] = ImageTexture::create_from_image(img);
				}
				// Unused layers stay null — they get black_1x1 at material build
			} else {
				// Legacy path: single blended atlas, re-baked on the CPU when a lightstyle moves
				PackedByteArray atlas_pixels;
				atlas_pixels.resize(pg.w * pg.h * 3);
				memset(atlas_pixels.ptrw(), 255, atlas_pixels.size());

				uint8_t *atlas_ptr = atlas_pixels.ptrw();
				for (int gi : pg.face_indices) {
					const FacePack &fp = face_pack_by_index[gi];
					const auto *face = &bsp_data.faces[gi];
					bake_lightmap_pixels(face->styles, face->lightmap_offset,
						face->lightmap_width, face->lightmap_height,
						fp.atlas_x, fp.atlas_y, pg.w, pg.h,
						atlas_ptr, bsp_data.lighting, lightstyle_values);
				}

				Ref<Image> atlas_img = Image::create_from_data(
					pg.w, pg.h, false, Image::FORMAT_RGB8, atlas_pixels);
				pg.blended = ImageTexture::create_from_image(atlas_img);

				int atlas_idx = (int)lm_atlases.size();
				LightmapAtlasState state;
				state.image = atlas_img;
				state.texture = pg.blended;
				state.width = pg.w;
				state.height = pg.h;
				lm_atlases.push_back(state);

				for (int gi : pg.face_indices) {
					face_lm_info[gi].atlas_index = atlas_idx;
				}
			}
		}

		// One material per (texture, atlas page) for the whole model. Every uniform a BSP
		// material carries is a pure function of those two, so this is a dedup, not a
		// behaviour change — and the runtime writers (skybox cubemap, light grid, overbright,
		// lightstyle texture) all push map-global values, while per-entity render modes go
		// through set_surface_override_material and never touch the shared base.
		std::map<std::pair<string, int>, Ref<Material>> material_cache;

		for (size_t sg_idx = 0; sg_idx < spatial_groups.size(); sg_idx++) {
			const auto &sg = spatial_groups[sg_idx];

			// For worldspawn spatial groups, create an intermediate Node3D
			// so each texture-group MeshInstance3D gets a localized AABB.
			// For brush entities, meshes go directly inside the AnimatableBody3D root.
			Node3D *group_parent = (m > 0) ? (Node3D *)body_node : model_node;
			if (m == 0 && spatial_groups.size() > 1) {
				group_parent = memnew(Node3D);
				String label = String(sg.label.c_str());
				if (label.is_empty()) {
					label = String("group_") + String::num_int64(sg_idx);
				}
				group_parent->set_name(label);
				model_node->add_child(group_parent);
				if (!sg.pvs_leaves.empty()) {
					PackedInt32Array leaves_arr;
					leaves_arr.resize((int64_t)sg.pvs_leaves.size());
					int32_t *w = leaves_arr.ptrw();
					for (size_t li = 0; li < sg.pvs_leaves.size(); li++)
						w[li] = sg.pvs_leaves[li];
					group_parent->set_meta("pvs_leaves", leaves_arr);
				}
			}

			// Group this spatial group's faces by (texture, atlas page). With a single page —
			// the normal case — this is just "by texture", and every face of a texture in this
			// group lands in one mesh sharing one material.
			//
			// A face with no lightmap has no page of its own, so it inherits the page of the
			// first lit face sharing its texture here (and -1, meaning black_1x1, when the
			// whole texture group is unlit). That reproduces exactly what the per-group atlas
			// did: an unlit face rendered against its group's atlas at UV2 (0,0), and a wholly
			// unlit group rendered against black_1x1.
			map<string, int> tex_page;
			for (const auto &fr : sg.face_refs) {
				const FacePack &fp = face_pack_by_index[fr.global_index];
				if (!fp.has_lightmap) continue;
				auto it = tex_page.find(fr.face->texture_name);
				if (it == tex_page.end()) tex_page[fr.face->texture_name] = fp.page;
			}
			map<std::pair<string, int>, vector<FaceRef>> tex_groups;
			for (const auto &fr : sg.face_refs) {
				const FacePack &fp = face_pack_by_index[fr.global_index];
				int page = fp.has_lightmap ? fp.page : -1;
				if (page < 0) {
					auto it = tex_page.find(fr.face->texture_name);
					if (it != tex_page.end()) page = it->second;
				}
				tex_groups[std::make_pair(fr.face->texture_name, page)].push_back(fr);
			}

		for (const auto &group : tex_groups) {
			const string &tex_name = group.first.first;
			const int page_idx = group.first.second;
			const auto &face_refs = group.second;

			// Skip tool textures that should never be rendered
			if (goldsrc::is_tool_texture(tex_name)) continue;

			int total_tris = 0;
			for (const auto &fr : face_refs) {
				if (fr.face->vertices.size() >= 3) {
					total_tris += (int)fr.face->vertices.size() - 2;
				}
			}
			if (total_tris == 0) continue;

			// Resolve this group's atlas page (see the model-wide packing above).
			const AtlasPage *page = (page_idx >= 0) ? &pages[page_idx] : nullptr;
			const bool has_any_lightmap = page != nullptr && page->has_lightmap;
			const int atlas_w = has_any_lightmap ? page->w : 0;
			const int atlas_h = has_any_lightmap ? page->h : 0;

			// --- Build mesh arrays with UV2 (and COLOR for shader path) ---
			PackedVector3Array vertices;
			PackedVector3Array normals;
			PackedVector2Array uvs;
			PackedVector2Array uv2s;
			PackedColorArray colors; // shader path: style indices
			PackedInt32Array indices;
			int vert_offset = 0;

			bool is_water_group = ent_is_liquid
					|| (!tex_name.empty() && (tex_name[0] == '!' || tex_name[0] == '*'));

			for (size_t fi = 0; fi < face_refs.size(); fi++) {
				const auto *face = face_refs[fi].face;
				const FacePack &fp = face_pack_by_index[face_refs[fi].global_index];
				int nv = (int)face->vertices.size();
				if (nv < 3) continue;

				// Liquid coincidence cull (see world_plane_aabbs above): a liquid face that lies
				// on the same geometric plane as a world face AND overlaps it is skipped — the
				// opaque world wall/floor is already drawn there, and rendering the coincident
				// translucent liquid face on top of it is exactly what z-fights. Faces with no
				// coincident world face (the water/air surface, liquid exposed to open space such
				// as lava/water falls) are kept untouched, so nothing is moved and no seam appears.
				// Only brush ENTITIES (m>0) get coincidence-culled; worldspawn liquid (m==0) is
				// already the compiler-deduped surface and must render as-is.
				if (m > 0 && is_water_group) {
					const auto &wa = world_plane_aabbs.find(plane_key(face));
					if (wa != world_plane_aabbs.end()) {
						FaceAABB fb = face_aabb(face);
						const float EPS = 0.5f;
						bool coincident = false;
						for (const auto &wb : wa->second) {
							if (fb.mn[0] <= wb.mx[0]+EPS && fb.mx[0] >= wb.mn[0]-EPS &&
							    fb.mn[1] <= wb.mx[1]+EPS && fb.mx[1] >= wb.mn[1]-EPS &&
							    fb.mn[2] <= wb.mx[2]+EPS && fb.mx[2] >= wb.mn[2]-EPS) { coincident = true; break; }
						}
						if (coincident) continue;
					}
				}

				// Compute style color for this face (same for all vertices)
				Color style_color(0, 0, 0, 0);
				if (shader_lightstyles) {
					for (int s = 0; s < 4; s++) {
						float val = (face->styles[s] < 64)
							? (face->styles[s] + 0.5f) / 64.0f
							: 0.0f; // unused slot: maps to style 0 (brightness 1.0), paired with black layer
						switch (s) {
							case 0: style_color.r = val; break;
							case 1: style_color.g = val; break;
							case 2: style_color.b = val; break;
							case 3: style_color.a = val; break;
						}
					}
				}

				// A flat polygon cannot bend, so a waving face is cut into a world-aligned grid
				// first. WATER_SUBDIVIDE is a quarter of the wave's own 512-unit period, which
				// is what a sine needs to read as a curve rather than a fold.
				std::vector<std::vector<goldsrc::ParsedVertex>> pieces;
				if (waves) {
					subdivide_water_polygon(face->vertices, WATER_SUBDIVIDE, pieces);
				} else {
					pieces.push_back(face->vertices);
				}

				for (const auto &piece : pieces) {
				const int pnv = (int)piece.size();
				for (int i = 2; i < pnv; i++) {
					const int tri_indices[3] = {0, i - 1, i};

					for (int t = 0; t < 3; t++) {
						const auto &v = piece[tri_indices[t]];

						vertices.push_back(goldsrc_to_godot(v.pos[0], v.pos[1], v.pos[2]));

						normals.push_back(Vector3(
							-v.normal[0], v.normal[2], v.normal[1]));

						uvs.push_back(Vector2(v.uv[0], v.uv[1]));

						// UV2: remap per-face [0,1] lightmap UV to atlas coordinates
						if (fp.has_lightmap && atlas_w > 0 && atlas_h > 0) {
							float lm_u = (fp.atlas_x + v.lightmap_uv[0] * face->lightmap_width) / atlas_w;
							float lm_v = (fp.atlas_y + v.lightmap_uv[1] * face->lightmap_height) / atlas_h;
							uv2s.push_back(Vector2(lm_u, lm_v));
						} else {
							uv2s.push_back(Vector2(0, 0));
						}

						if (shader_lightstyles) {
							colors.push_back(style_color);
						}

						indices.push_back(vert_offset++);
					}
				}
				}
			}

			// Every face in this texture group may have been culled (e.g. a liquid volume whose
			// faces are all coincident with world geometry). An empty surface is invalid, so skip
			// the group entirely rather than feeding zero-length arrays to the RenderingServer.
			if (vertices.is_empty())
				continue;

			Ref<ArrayMesh> arr_mesh;
			arr_mesh.instantiate();

			Array arrays;
			arrays.resize(ArrayMesh::ARRAY_MAX);
			arrays[ArrayMesh::ARRAY_VERTEX] = vertices;
			arrays[ArrayMesh::ARRAY_NORMAL] = normals;
			arrays[ArrayMesh::ARRAY_TEX_UV] = uvs;
			arrays[ArrayMesh::ARRAY_TEX_UV2] = uv2s;
			arrays[ArrayMesh::ARRAY_INDEX] = indices;
			if (shader_lightstyles) {
				arrays[ArrayMesh::ARRAY_COLOR] = colors;
			}

			arr_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);

			Ref<ImageTexture> texture = find_texture(tex_name);
			bool is_sky_surface = goldsrc::is_sky_texture(tex_name);
			bool is_water = !is_sky_surface && (ent_is_liquid
					|| (!tex_name.empty() && (tex_name[0] == '!' || tex_name[0] == '*')));
			bool is_alpha_scissor = !is_sky_surface && !is_water && !tex_name.empty() && tex_name[0] == '{';

			// Build the material once per (texture, page) and hand the same one to every mesh
			// that needs it — the whole point of packing the atlas per model above.
			auto cache_key = std::make_pair(tex_name, page_idx);
			auto cached = material_cache.find(cache_key);
			if (cached != material_cache.end()) {
				arr_mesh->surface_set_material(0, cached->second);
			} else if (is_sky_surface) {
				// Sky surface: samples sky cubemap by view direction; sky_cubemap
				// uniform is set at runtime by GDScript (systems/skybox.gd).
				Ref<ShaderMaterial> material;
				material.instantiate();
				material->set_shader(sky_shader);
				material->set_render_priority(-1); // draw before BSP so it's a true background
				arr_mesh->surface_set_material(0, material);
				material_cache[cache_key] = material;
			} else if (is_water) {
				// Water/liquid surface: turbulent UV warp, no lightmap.
				Ref<ShaderMaterial> material;
				material.instantiate();
				material->set_shader(water_shader);
				int liquid_contents = ent_skin;
				if (m == 0) {
					// Worldspawn: take the first face in the group that borders a liquid. A
					// texture group is one surface of one pool, so they agree.
					liquid_contents = goldsrc::CONTENTS_WATER;
					for (const auto &fr : face_refs) {
						if (fr.face->liquid_contents != 0) {
							liquid_contents = fr.face->liquid_contents;
							break;
						}
					}
				}
				material->set_shader_parameter("liquid_type",
						(int)liquid_kind_for_contents(liquid_contents));
				material->set_shader_parameter("wave_height", ent_wave * scale_factor);
				if (texture.is_valid()) {
					material->set_shader_parameter("albedo_texture", texture);
				}
				arr_mesh->surface_set_material(0, material);
				material_cache[cache_key] = material;
			} else if (shader_lightstyles) {
				// Shader path: ShaderMaterial with 4-layer lightmap blending
				Ref<ShaderMaterial> material;
				material.instantiate();
				material->set_shader(is_alpha_scissor ? alpha_shader : opaque_shader);

				if (texture.is_valid()) {
					material->set_shader_parameter("albedo_texture", texture);
				}

				// Assign layer textures (use black_1x1 for layers without data)
				for (int l = 0; l < 4; l++) {
					String param = String("lm_layer") + String::num_int64(l);
					if (has_any_lightmap && page->layer_textures[l].is_valid()) {
						material->set_shader_parameter(param, page->layer_textures[l]);
					} else {
						material->set_shader_parameter(param, black_1x1);
					}
				}

				material->set_shader_parameter("lightstyle_tex", lightstyle_texture);
				material->set_shader_parameter("overbright", 2.0f);

				arr_mesh->surface_set_material(0, material);
				material_cache[cache_key] = material;
			} else {
				// Legacy path: StandardMaterial3D with detail texture multiply
				Ref<StandardMaterial3D> material;
				material.instantiate();
				material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);

				if (texture.is_valid()) {
					material->set_texture(BaseMaterial3D::TEXTURE_ALBEDO, texture);
					material->set_texture_filter(BaseMaterial3D::TEXTURE_FILTER_LINEAR_WITH_MIPMAPS);
				} else {
					material->set_albedo(Color(0.5f, 0.5f, 0.5f));
				}

				if (is_alpha_scissor) {
					material->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA_SCISSOR);
				}

				if (has_any_lightmap && page->blended.is_valid()) {
					material->set_feature(BaseMaterial3D::FEATURE_DETAIL, true);
					material->set_texture(BaseMaterial3D::TEXTURE_DETAIL_ALBEDO, page->blended);
					material->set_detail_blend_mode(BaseMaterial3D::BLEND_MODE_MUL);
					material->set_detail_uv(BaseMaterial3D::DETAIL_UV_2);
				}

				arr_mesh->surface_set_material(0, material);
				material_cache[cache_key] = material;
			}

			// Sky collision rides the rendering mesh, so it is per-MESH, not per-material —
			// it must still happen on a cache hit, unlike the material construction above.
			if (is_sky_surface && sky_body) {
				Ref<ConcavePolygonShape3D> sky_shape = arr_mesh->create_trimesh_shape();
				if (sky_shape.is_valid()) {
					CollisionShape3D *sky_col = memnew(CollisionShape3D);
					sky_col->set_shape(sky_shape);
					sky_body->add_child(sky_col);
				}
			}

			MeshInstance3D *mesh_instance = memnew(MeshInstance3D);
			mesh_instance->set_name(tex_name.empty() ? String("unnamed") : String(tex_name.c_str()));
			mesh_instance->set_mesh(arr_mesh);
			mesh_instance->set_layer_mask(2); // Layer 2 only — map lights (layer 1) skip BSP, weapon lights (layers 1+2) hit it
			if (is_sky_surface) {
				mesh_instance->set_meta("is_sky_surface", true);
			}
			if (is_water) {
				mesh_instance->set_meta("is_water_surface", true);
			}

			// Animated texture: +0name–+9name (primary) or +aname–+jname (alternate).
			// Collect all frame textures and store as metadata so the runtime
			// TextureAnimator can cycle them without the WAD being loaded.
			if (tex_name.size() >= 2 && tex_name[0] == '+') {
				unsigned char c1 = (unsigned char)tex_name[1];
				bool is_primary = isdigit(c1) != 0;
				bool is_alt = (toupper(c1) >= 'A' && toupper(c1) <= 'J');
				if (is_primary || is_alt) {
					std::string base = tex_name.substr(2);
					if (!base.empty()) {
						Array frames;
						if (is_primary) {
							// Primary sequence: +0 through +9
							for (int n = 0; n < 10; n++) {
								std::string frame_name = "+" + std::to_string(n) + base;
								Ref<ImageTexture> frame_tex = find_texture(frame_name);
								if (frame_tex.is_valid()) {
									frames.push_back(frame_tex);
								} else {
									break;
								}
							}
						} else {
							// Alternate sequence: +a through +j (A=0, B=1, ..., J=9)
							for (int n = 0; n < 10; n++) {
								std::string frame_name = "+" + std::string(1, (char)('a' + n)) + base;
								Ref<ImageTexture> frame_tex = find_texture(frame_name);
								if (frame_tex.is_valid()) {
									frames.push_back(frame_tex);
								} else {
									break;
								}
							}
						}
						if (frames.size() >= 2) {
							mesh_instance->set_meta("tex_anim_frames", frames);
							has_anim = true;
						}
					}
				}
			}

			group_parent->add_child(mesh_instance);
			total_mesh_instances++;

			// Water's depth prepass twin: same mesh, depth-only material, drawn first. See
			// WATER_DEPTH_SHADER_CODE for why water needs it. The mesh resource is shared, so
			// this costs a draw call and a node, not geometry.
			if (is_water) {
				MeshInstance3D *depth_instance = memnew(MeshInstance3D);
				depth_instance->set_name(mesh_instance->get_name() + String("_depth"));
				depth_instance->set_mesh(arr_mesh);
				// A waving brush needs its OWN depth material: the shared one carries no
				// amplitude, and a prepass that stays flat under a surface that moves stamps
				// the wrong depth and punches holes in it.
				if (waves) {
					Ref<ShaderMaterial> wave_depth;
					wave_depth.instantiate();
					wave_depth->set_shader(water_depth_material->get_shader());
					wave_depth->set_render_priority(-1);
					wave_depth->set_shader_parameter("wave_height", ent_wave * scale_factor);
					depth_instance->set_material_override(wave_depth);
				} else {
					depth_instance->set_material_override(water_depth_material);
				}
				depth_instance->set_layer_mask(2);
				depth_instance->set_cast_shadows_setting(GeometryInstance3D::SHADOW_CASTING_SETTING_OFF);
				group_parent->add_child(depth_instance);
				total_mesh_instances++;
			}
		}
		} // end spatial groups loop

		if (m == 0 && spatial_groups.size() > 1) {
			// Page count is the thing to watch: >1 means the map's lighting overflowed a
			// 4096² sheet and its materials are split at that seam. Expect 1.
			UtilityFunctions::print("[GoldSrc] Worldspawn: ",
				(int64_t)total_mesh_instances, " MeshInstance3D nodes across ",
				(int64_t)spatial_groups.size(), " spatial groups, ",
				(int64_t)material_cache.size(), " materials, ",
				(int64_t)pages.size(), " lightmap page(s)");
		}

		// Build collision for this model
		if (m == 0) {
			log_timing("worldspawn meshes");
			// Hull 0: face-based collision (exact visible geometry, no sky)
			// Projectiles use layer 1 (MASK_WORLD) — sky excluded so they pass through.
			build_hull_collision(model_node, m, 0, "WorldCollision", 1);
			// Sky collision body — shapes were added directly from rendering ArrayMesh
			// objects above; no separate face iteration needed.
			if (sky_body && sky_body->get_child_count() > 0) {
				// Sky is passable in hull 0 (that's why projectiles fly through it),
				// so this body's hull view has to block on CONTENTS_SKY instead.
				mark_bsp_carrier(sky_body, m, 1 << (-goldsrc::CONTENTS_SKY));
				model_node->add_child(sky_body);
				UtilityFunctions::print("[GoldSrc] Sky collision: ",
					(int64_t)sky_body->get_child_count(), " shape(s) (reused from rendering mesh)");
			} else if (sky_body) {
				memdelete(sky_body);
			}
			log_timing("hull 0 collision");
			{
				Area3D *water = memnew(Area3D);
				water->set_name("WaterVolumes");
				water->set_collision_layer(1 << 2);
				water->set_collision_mask(0);
				water->set_monitorable(true);
				water->set_monitoring(false);
				build_brush_convex(water, 0, goldsrc::CONTENTS_WATER);
				if (water->get_child_count() > 0) {
					model_node->add_child(water);
				} else {
					memdelete(water);
				}
			}
			log_timing("water volumes");
		} else {
			// Brush entity collision: convex shapes from BSP leaf decomposition.
			// To revert to triangle-soup collision: change to build_brush_concave.
			build_brush_convex(body_node, m);
			if (body_node->get_child_count() > 0) mark_bsp_carrier(body_node, m);
			// Volume Area3D shares the same shape resources for containment detection.
			if (body_node->get_child_count() > 0) {
				Area3D *vol = memnew(Area3D);
				vol->set_name("Volume");
				vol->set_collision_layer(0);
				vol->set_collision_mask(0);
				vol->set_monitorable(false);
				vol->set_monitoring(false);
				for (int i = 0; i < body_node->get_child_count(); i++) {
					CollisionShape3D *src = Object::cast_to<CollisionShape3D>(body_node->get_child(i));
					if (!src) continue;
					CollisionShape3D *col = memnew(CollisionShape3D);
					col->set_name(src->get_name());
					col->set_shape(src->get_shape()); // shared Ref, not duplicated
					vol->add_child(col);
				}
				model_node->add_child(vol);
			}
		}
	}
	log_timing("brush entity meshes + collision");

	// Create Node3D for each point entity (no brush model).
	// Brush entities already have nodes from the model loop above.
	int point_entity_count = 0;
	for (const auto &ent : bsp_data.entities) {
		auto cn = ent.properties.find("classname");
		if (cn == ent.properties.end()) continue;
		if (cn->second == "worldspawn") continue;

		// Skip brush entities — they already have nodes
		auto model_it = ent.properties.find("model");
		if (model_it != ent.properties.end() && !model_it->second.empty()
				&& model_it->second[0] == '*') {
			continue;
		}

		Node3D *node = memnew(Node3D);

		// Set position from "origin" key
		auto origin_it = ent.properties.find("origin");
		if (origin_it != ent.properties.end()) {
			float x = 0, y = 0, z = 0;
			sscanf(origin_it->second.c_str(), "%f %f %f", &x, &y, &z);
			node->set_position(goldsrc_to_godot(x, y, z));
		}

		// Store entity properties as metadata
		Dictionary meta;
		for (const auto &kv : ent.properties) {
			meta[String(kv.first.c_str())] = String(kv.second.c_str());
		}
		node->set_meta("entity", meta);

		// Name: classname_targetname_N or classname_N (unique to avoid @ prefix)
		auto tn = ent.properties.find("targetname");
		String node_name = String(cn->second.c_str());
		if (tn != ent.properties.end() && !tn->second.empty()) {
			node_name = node_name + "_" + String(tn->second.c_str());
		}
		node_name = node_name + "_" + String::num_int64(point_entity_count);
		node->set_name(node_name);

		add_child(node);
		point_entity_count++;
	}
	log_timing("point entity nodes");

	UtilityFunctions::print("[GoldSrc] Built BSP: ",
		(int64_t)num_models, " models, ",
		(int64_t)point_entity_count, " point entities, ",
		(int64_t)bsp_data.faces.size(), " faces, ",
		(int64_t)lit_face_count, " lightmapped");

	// Add a self-contained texture animator node so the saved .scn animates
	// automatically on load with no external file or manual wiring required.
	if (has_anim) {
		Ref<GDScript> script;
		script.instantiate();
		script->set_source_code(String(TEXTURE_ANIMATION_PLAYER_SCRIPT));
		if (script->reload() == OK) {
			Node *animator = memnew(Node);
			animator->set_name("TextureAnimationPlayer");
			animator->set_script(script);
			add_child(animator);
		} else {
			UtilityFunctions::printerr("[GoldSrc] Failed to compile texture animator script");
		}
	}
}

// Tag a collision body with the BSP model its shapes stand for, so a physics
// engine that can trace the real hulls (hop's HopBspTraceable) knows which tree
// to descend and swaps the shapes out for it. Additive: the shapes are still
// there, and an engine that ignores the metadata — default Godot physics —
// collides against exactly what it always did.
//
// The hull blob itself is NOT copied per body (a big map's is megabytes, and
// there are hundreds of brush entities); it rides once on the scene root under
// "bsp_data", and the consumer walks up to find it.
void GoldSrcBSP::mark_bsp_carrier(Node3D *body, int model_index, int blocking_contents) {
	body->set_meta("bsp_model", model_index);
	body->set_meta("bsp_scale", scale_factor);
	if (blocking_contents != 0) body->set_meta("bsp_blocking", blocking_contents);
}

void GoldSrcBSP::build_hull_collision(Node3D *parent, int model_index,
	int hull_index, const String &body_name, uint32_t collision_layer) {
	const auto &bsp_data = parser->get_data();

	if (model_index < 0 || model_index >= (int)bsp_data.models.size()) return;
	if (hull_index != 0) return; // only hull 0 (face-based) is supported

	// Hull 0: use parsed face vertices directly. This gives only
	// outward-facing surfaces — no internal BSP cell boundaries that
	// would trap a capsule collider reaching through thin walls.
	PackedVector3Array all_tris;
	int face_count = 0;

	for (const auto &face : bsp_data.faces) {
		if (face.model_index != model_index) continue;
		// Skip water/slime/lava faces ('!' or old-format '*' prefix)
		if (!face.texture_name.empty() && (face.texture_name[0] == '!' || face.texture_name[0] == '*')) continue;
		// Skip sky faces — not solid, projectiles pass through
		if (goldsrc::is_sky_texture(face.texture_name)) continue;
		int nv = (int)face.vertices.size();
		if (nv < 3) continue;
		for (int i = 2; i < nv; i++) {
			const auto &v0 = face.vertices[0];
			const auto &v1 = face.vertices[i - 1];
			const auto &v2 = face.vertices[i];
			all_tris.push_back(goldsrc_to_godot(v0.pos[0], v0.pos[1], v0.pos[2]));
			all_tris.push_back(goldsrc_to_godot(v1.pos[0], v1.pos[1], v1.pos[2]));
			all_tris.push_back(goldsrc_to_godot(v2.pos[0], v2.pos[1], v2.pos[2]));
		}
		face_count++;
	}

	if (all_tris.is_empty()) return;

	StaticBody3D *body = memnew(StaticBody3D);
	body->set_name(body_name);
	body->set_collision_layer(collision_layer);
	mark_bsp_carrier(body, model_index);

	Ref<ConcavePolygonShape3D> shape;
	shape.instantiate();
	shape->set_faces(all_tris);

	CollisionShape3D *col = memnew(CollisionShape3D);
	col->set_name("CollisionShape3D");
	col->set_shape(shape);
	body->add_child(col);

	parent->add_child(body);
	UtilityFunctions::print("[GoldSrc] Hull 0 collision: ",
		(int64_t)face_count, " faces (",
		(int64_t)(all_tris.size() / 3), " tris) for model ",
		(int64_t)model_index);
}

void GoldSrcBSP::build_brush_concave(Node3D *parent, int model_index) {
	const auto &bsp_data = parser->get_data();

	if (model_index < 0 || model_index >= (int)bsp_data.models.size()) return;

	PackedVector3Array all_tris;
	int face_count = 0;

	for (const auto &face : bsp_data.faces) {
		if (face.model_index != model_index) continue;
		if (!face.texture_name.empty() && (face.texture_name[0] == '!' || face.texture_name[0] == '*')) continue;
		int nv = (int)face.vertices.size();
		if (nv < 3) continue;
		for (int i = 2; i < nv; i++) {
			const auto &v0 = face.vertices[0];
			const auto &v1 = face.vertices[i - 1];
			const auto &v2 = face.vertices[i];
			all_tris.push_back(goldsrc_to_godot(v0.pos[0], v0.pos[1], v0.pos[2]));
			all_tris.push_back(goldsrc_to_godot(v1.pos[0], v1.pos[1], v1.pos[2]));
			all_tris.push_back(goldsrc_to_godot(v2.pos[0], v2.pos[1], v2.pos[2]));
		}
		face_count++;
	}

	if (all_tris.is_empty()) return;

	Ref<ConcavePolygonShape3D> shape;
	shape.instantiate();
	shape->set_faces(all_tris);

	CollisionShape3D *col = memnew(CollisionShape3D);
	col->set_name("CollisionShape3D");
	col->set_shape(shape);
	parent->add_child(col);
}

void GoldSrcBSP::build_brush_convex(Node3D *parent, int model_index,
		int contents_filter) {
	const auto &bsp_data = parser->get_data();
	if (model_index < 0 || model_index >= (int)bsp_data.models.size()) return;

	const auto &mdl = bsp_data.models[model_index];

	// Bounding box planes from the model (padded slightly)
	HullPlane bbox_planes[6];
	bbox_planes[0] = {{ 1, 0, 0}, mdl.maxs[0] + 1.0f};
	bbox_planes[1] = {{-1, 0, 0}, -mdl.mins[0] + 1.0f};
	bbox_planes[2] = {{ 0, 1, 0}, mdl.maxs[1] + 1.0f};
	bbox_planes[3] = {{ 0,-1, 0}, -mdl.mins[1] + 1.0f};
	bbox_planes[4] = {{ 0, 0, 1}, mdl.maxs[2] + 1.0f};
	bbox_planes[5] = {{ 0, 0,-1}, -mdl.mins[2] + 1.0f};

	// Collect matching leaf content types.
	// contents_filter == 0: match all non-empty contents (SOLID, WATER, etc.)
	// Otherwise: match only the specified content type.
	vector<int> target_types;
	if (contents_filter == 0) {
		target_types = {goldsrc::CONTENTS_SOLID, goldsrc::CONTENTS_WATER};
	} else {
		target_types = {contents_filter};
	}

	vector<ConvexCell> all_cells;
	for (int tc : target_types) {
		vector<HullPlane> accumulated;
		vector<ConvexCell> cells;
		walk_bsp_tree(bsp_data.nodes, bsp_data.leafs, bsp_data.planes,
			mdl.headnode[0], accumulated, cells, tc);
		all_cells.insert(all_cells.end(),
			std::make_move_iterator(cells.begin()),
			std::make_move_iterator(cells.end()));
	}

	if (all_cells.empty()) return;

	const float EPSILON = 0.1f;
	int shape_count = 0;
	for (auto &cell : all_cells) {
		for (int i = 0; i < 6; i++) {
			cell.planes.push_back(bbox_planes[i]);
		}
		vector<CellVertex> verts = compute_cell_vertices(
			cell.planes, EPSILON);
		if ((int)verts.size() < 4) continue;

		PackedVector3Array points;
		for (const auto &v : verts) {
			points.push_back(goldsrc_to_godot(v.gs[0], v.gs[1], v.gs[2]));
		}

		Ref<ConvexPolygonShape3D> shape;
		shape.instantiate();
		shape->set_points(points);

		CollisionShape3D *col = memnew(CollisionShape3D);
		col->set_name(String("ConvexShape_") + String::num_int64(shape_count));
		col->set_shape(shape);
		parent->add_child(col);
		shape_count++;
	}
}

void GoldSrcBSP::set_lightstyle(int style_index, float brightness) {
	if (style_index < 0 || style_index >= 64) return;
	if (lightstyle_values[style_index] == brightness) return;
	lightstyle_values[style_index] = brightness;
	if (shader_lightstyles && lightstyle_image.is_valid()) {
		// Shader path: update one pixel in the 64×1 brightness texture
		lightstyle_image->set_pixel(style_index, 0, Color(brightness, 0, 0, 1));
		lightstyle_texture->update(lightstyle_image);
	} else {
		// Legacy path: CPU rebake
		rebake_lightstyle(style_index);
	}
}

float GoldSrcBSP::get_lightstyle(int style_index) const {
	if (style_index < 0 || style_index >= 64) return 0.0f;
	return lightstyle_values[style_index];
}

Ref<Image> GoldSrcBSP::get_lightstyle_image() const {
	return lightstyle_image;
}

Ref<ImageTexture> GoldSrcBSP::get_lightstyle_texture() const {
	return lightstyle_texture;
}

void GoldSrcBSP::rebake_lightstyle(int style_index) {
	if (!parser) return;

	// Collect faces per atlas that use this style
	map<int, vector<int>> atlas_faces;

	for (int i = 0; i < (int)face_lm_info.size(); i++) {
		const auto &info = face_lm_info[i];
		for (int s = 0; s < 4; s++) {
			if (info.styles[s] == 255) break;
			if (info.styles[s] == style_index) {
				atlas_faces[info.atlas_index].push_back(i);
				break;
			}
		}
	}

	const auto &bsp_data = parser->get_data();

	// Rebake one atlas at a time: one data copy per atlas
	for (auto &[atlas_idx, face_indices] : atlas_faces) {
		if (atlas_idx < 0 || atlas_idx >= (int)lm_atlases.size()) continue;
		auto &atlas = lm_atlases[atlas_idx];

		PackedByteArray img_data = atlas.image->get_data();
		uint8_t *ptr = img_data.ptrw();

		for (int fi : face_indices) {
			const auto &info = face_lm_info[fi];
			bake_lightmap_pixels(info.styles, info.lightmap_offset,
				info.lm_width, info.lm_height,
				info.atlas_x, info.atlas_y, atlas.width, atlas.height,
				ptr, bsp_data.lighting, lightstyle_values);
		}

		atlas.image = Image::create_from_data(atlas.width, atlas.height,
			false, Image::FORMAT_RGB8, img_data);
		atlas.texture->update(atlas.image);
	}
}

// --- Ambient cube light grid baking ---

namespace {

struct LightGridRayHit {
	int face_index = -1;
	float t = 1e30f;
	float hit_pos[3] = {0, 0, 0};
};

// A ray "reaches the sky" if it hits nothing, or if its closest hit is a
// sky-textured brush face. GoldSrc maps are sealed by sky brushes, so a ray
// aimed upward/outward hits the sky face rather than escaping into the void —
// but that point still sees open sky (and the sun). Treating a sky-face hit as
// an occluder (the old behavior) left virtually no cell sunlit on sealed maps.
static inline bool ray_reached_sky(
	const goldsrc::BSPData &bsp_data, const LightGridRayHit &hit)
{
	if (hit.face_index < 0) return true;
	if ((size_t)hit.face_index >= bsp_data.faces.size()) return false;
	return goldsrc::is_sky_texture(bsp_data.faces[hit.face_index].texture_name);
}

// Ray-polygon intersection: check if ray hits a convex polygon.
// Returns distance along ray, or -1 if no hit.
static float ray_polygon_intersect(
	const float origin[3], const float dir[3],
	const goldsrc::ParsedFace &face)
{
	// Plane: dot(normal, P) = dot(normal, v0)
	float denom = face.normal[0] * dir[0] + face.normal[1] * dir[1] + face.normal[2] * dir[2];
	if (fabsf(denom) < 1e-8f) return -1.0f; // parallel

	const auto &v0 = face.vertices[0];
	float plane_d = face.normal[0] * v0.pos[0] + face.normal[1] * v0.pos[1] + face.normal[2] * v0.pos[2];
	float origin_d = face.normal[0] * origin[0] + face.normal[1] * origin[1] + face.normal[2] * origin[2];
	float t = (plane_d - origin_d) / denom;
	if (t <= 0.001f) return -1.0f; // behind or at origin

	// Hit point on plane
	float hx = origin[0] + dir[0] * t;
	float hy = origin[1] + dir[1] * t;
	float hz = origin[2] + dir[2] * t;

	// Point-in-polygon via cross-product winding test.
	// Vertices may be wound CW or CCW relative to the stored normal (depends on
	// face.side in BSP), so check that all edges give the same sign, not a fixed sign.
	int n = (int)face.vertices.size();
	int pos_count = 0, neg_count = 0;
	for (int i = 0; i < n; i++) {
		const auto &a = face.vertices[i];
		const auto &b = face.vertices[(i + 1) % n];
		float ex = b.pos[0] - a.pos[0];
		float ey = b.pos[1] - a.pos[1];
		float ez = b.pos[2] - a.pos[2];
		float px = hx - a.pos[0];
		float py = hy - a.pos[1];
		float pz = hz - a.pos[2];
		float cx = ey * pz - ez * py;
		float cy = ez * px - ex * pz;
		float cz = ex * py - ey * px;
		float d = cx * face.normal[0] + cy * face.normal[1] + cz * face.normal[2];
		if (d > 0.1f) pos_count++;
		else if (d < -0.1f) neg_count++;
		if (pos_count > 0 && neg_count > 0) return -1.0f; // outside
	}

	return t;
}

// Sample lightmap color at a GoldSrc world position on a given face.
// Returns RGB [0..255] range.
static void sample_lightmap_at(
	const goldsrc::ParsedFace &face,
	const float hit_pos[3],
	const vector<uint8_t> &lighting,
	float &out_r, float &out_g, float &out_b)
{
	out_r = out_g = out_b = 0;

	if (face.lightmap_offset < 0) return;
	if (face.lightmap_width <= 0 || face.lightmap_height <= 0) return;

	// Compute texture S/T at hit position
	float s = hit_pos[0] * face.s_axis[0] + hit_pos[1] * face.s_axis[1] + hit_pos[2] * face.s_axis[2] + face.s_offset;
	float t = hit_pos[0] * face.t_axis[0] + hit_pos[1] * face.t_axis[1] + hit_pos[2] * face.t_axis[2] + face.t_offset;

	// Convert to lightmap pixel coords
	float lm_u = s / 16.0f - face.lm_mins_s;
	float lm_v = t / 16.0f - face.lm_mins_t;

	int px = clamp((int)roundf(lm_u), 0, face.lightmap_width - 1);
	int py = clamp((int)roundf(lm_v), 0, face.lightmap_height - 1);

	// Sample style 0 only (static lighting)
	size_t pixel_count = (size_t)face.lightmap_width * face.lightmap_height;
	size_t ofs = (size_t)face.lightmap_offset + (size_t)(py * face.lightmap_width + px) * 3;
	if (ofs + 2 >= lighting.size()) return;

	out_r = (float)lighting[ofs + 0];
	out_g = (float)lighting[ofs + 1];
	out_b = (float)lighting[ofs + 2];
}

// BSP-accelerated ray trace: walk the BSP node tree with a ray,
// testing faces at each visited node.
static void trace_ray_bsp(
	const goldsrc::BSPData &bsp_data,
	int node_index,
	const float origin[3], const float dir[3],
	float t_min, float t_max,
	LightGridRayHit &best_hit)
{
	if (t_min >= t_max) return;
	if (node_index < 0) return; // leaf — hull 0 stores faces on nodes, not leaves

	if ((size_t)node_index >= bsp_data.nodes.size()) return;
	const auto &node = bsp_data.nodes[node_index];
	if (node.planenum < 0 || (size_t)node.planenum >= bsp_data.planes.size()) return;
	const auto &plane = bsp_data.planes[node.planenum];

	// Test this node's faces
	for (uint16_t i = 0; i < node.numfaces; i++) {
		int raw_idx = (int)node.firstface + i;
		if (raw_idx >= (int)bsp_data.raw_to_parsed.size()) continue;
		int parsed = bsp_data.raw_to_parsed[raw_idx];
		if (parsed < 0) continue;

		const auto &face = bsp_data.faces[parsed];
		if (face.model_index != 0) continue; // worldspawn only
		if (face.vertices.size() < 3) continue;

		float t = ray_polygon_intersect(origin, dir, face);
		if (t > 0 && t < best_hit.t && t >= t_min && t <= t_max) {
			best_hit.face_index = parsed;
			best_hit.t = t;
			best_hit.hit_pos[0] = origin[0] + dir[0] * t;
			best_hit.hit_pos[1] = origin[1] + dir[1] * t;
			best_hit.hit_pos[2] = origin[2] + dir[2] * t;
		}
	}

	// Classify ray endpoints against the plane
	float d_start = plane.normal[0] * (origin[0] + dir[0] * t_min) +
	                plane.normal[1] * (origin[1] + dir[1] * t_min) +
	                plane.normal[2] * (origin[2] + dir[2] * t_min) - plane.dist;
	float d_end   = plane.normal[0] * (origin[0] + dir[0] * t_max) +
	                plane.normal[1] * (origin[1] + dir[1] * t_max) +
	                plane.normal[2] * (origin[2] + dir[2] * t_max) - plane.dist;

	if (d_start >= 0 && d_end >= 0) {
		trace_ray_bsp(bsp_data, node.children[0], origin, dir, t_min, t_max, best_hit);
	} else if (d_start < 0 && d_end < 0) {
		trace_ray_bsp(bsp_data, node.children[1], origin, dir, t_min, t_max, best_hit);
	} else {
		// Ray crosses the plane — compute split point
		float denom = plane.normal[0] * dir[0] + plane.normal[1] * dir[1] + plane.normal[2] * dir[2];
		float t_split;
		if (fabsf(denom) < 1e-8f) {
			t_split = (t_min + t_max) * 0.5f;
		} else {
			float origin_d = plane.normal[0] * origin[0] + plane.normal[1] * origin[1] + plane.normal[2] * origin[2];
			t_split = (plane.dist - origin_d) / denom;
			t_split = clamp(t_split, t_min, t_max);
		}

		int near_child = d_start >= 0 ? 0 : 1;
		int far_child = 1 - near_child;
		trace_ray_bsp(bsp_data, node.children[near_child], origin, dir, t_min, t_split, best_hit);
		trace_ray_bsp(bsp_data, node.children[far_child], origin, dir, t_split, t_max, best_hit);
	}
}

} // anonymous namespace

Dictionary GoldSrcBSP::bake_light_grid(float cell_size_gs) const {
	if (!parser) return Dictionary();
	const auto &bsp_data = parser->get_data();
	// Bake reads the raw lighting lump + per-face lightmap offsets directly
	// (see sample_lightmap_at), not lm_atlases — so do NOT gate on lm_atlases.
	// In the shader-lightstyle path (shader_lightstyles = true, which the game
	// always uses) build_mesh populates layer_textures[] but never lm_atlases,
	// so gating on it here suppressed the grid on every map. A non-empty lighting
	// lump is the correct signal that lightmaps exist; sample_lightmap_at bounds-
	// checks each offset, so stale/mismatched offsets just read as unlit.
	if (bsp_data.models.empty() || bsp_data.lighting.empty())
		return Dictionary();

	uint64_t t0 = Time::get_singleton()->get_ticks_msec();

	// Grid bounds from model 0 (worldspawn), padded by 1 cell
	float mins[3] = {
		bsp_data.models[0].mins[0] - cell_size_gs,
		bsp_data.models[0].mins[1] - cell_size_gs,
		bsp_data.models[0].mins[2] - cell_size_gs
	};
	float maxs[3] = {
		bsp_data.models[0].maxs[0] + cell_size_gs,
		bsp_data.models[0].maxs[1] + cell_size_gs,
		bsp_data.models[0].maxs[2] + cell_size_gs
	};

	int dims[3];
	for (int i = 0; i < 3; i++) {
		dims[i] = (int)ceilf((maxs[i] - mins[i]) / cell_size_gs);
		if (dims[i] < 1) dims[i] = 1;
	}

	// GoldSrc directions for ambient cube:
	// Index 0: +X_gs, 1: -X_gs, 2: +Y_gs, 3: -Y_gs, 4: +Z_gs, 5: -Z_gs
	static const float directions[6][3] = {
		{ 1, 0, 0}, {-1, 0, 0},
		{ 0, 1, 0}, { 0,-1, 0},
		{ 0, 0, 1}, { 0, 0,-1}
	};

	// Extract light_environment sky color and sun direction for rays that escape the map.
	// HLRAD uses this as the sky light when compiling lightmaps, so surfaces hit by sunlight
	// have bright values, but rays going up into the sky need to return the sky contribution.
	float sky_color[3] = {0, 0, 0};
	float sun_dir[3] = {0, 0, -1}; // default: straight down
	bool has_sky_light = false;
	for (const auto &ent : bsp_data.entities) {
		auto cn = ent.properties.find("classname");
		if (cn == ent.properties.end() || cn->second != "light_environment") continue;

		// Parse "_light" as "R G B [brightness]"
		auto lp = ent.properties.find("_light");
		if (lp != ent.properties.end()) {
			float lr = 0, lg = 0, lb = 0, lbright = 200.0f;
			if (sscanf(lp->second.c_str(), "%f %f %f %f", &lr, &lg, &lb, &lbright) >= 3) {
				// Normalize and scale by brightness (match light_environment.gd logic)
				float peak = fmaxf(lr, fmaxf(lg, lb));
				if (peak > 0) {
					sky_color[0] = (lr / peak) * (lbright / 200.0f) * peak / 255.0f;
					sky_color[1] = (lg / peak) * (lbright / 200.0f) * peak / 255.0f;
					sky_color[2] = (lb / peak) * (lbright / 200.0f) * peak / 255.0f;
				}
			}
		}

		// Parse sun direction from "angles" (yaw) and "pitch"
		float yaw_deg = 0, pitch_deg = 0;
		auto ap = ent.properties.find("angles");
		if (ap != ent.properties.end()) {
			float a0, a1;
			if (sscanf(ap->second.c_str(), "%f %f", &a0, &a1) >= 2)
				yaw_deg = a1;
		} else {
			auto angp = ent.properties.find("angle");
			if (angp != ent.properties.end())
				yaw_deg = atof(angp->second.c_str());
		}
		auto pp = ent.properties.find("pitch");
		if (pp != ent.properties.end())
			pitch_deg = atof(pp->second.c_str());

		// Convert to GoldSrc direction vector pointing TOWARD the sun.
		// The angle/pitch specify the direction light TRAVELS (from sun to ground).
		// We negate to get the direction FROM ground TOWARD the sun, for visibility checks.
		float pitch_rad = pitch_deg * 3.14159265f / 180.0f;
		float yaw_rad = yaw_deg * 3.14159265f / 180.0f;
		sun_dir[0] = -(cosf(pitch_rad) * cosf(yaw_rad));
		sun_dir[1] = -(cosf(pitch_rad) * sinf(yaw_rad));
		sun_dir[2] = -sinf(pitch_rad);
		has_sky_light = true;
		break; // use first light_environment only
	}

	// 3D texture layout: each direction gets dims[2] slices (GoldSrc Z = Godot Y),
	// each slice is dims[0] x dims[1] (GoldSrc XY = Godot XZ), RGB8.
	int slice_w = dims[0];
	int slice_h = dims[1];
	int num_slices = dims[2];
	size_t slice_bytes = (size_t)slice_w * slice_h * 3;

	// Allocate 6 direction buffers, each with num_slices * slice_bytes
	size_t vol_bytes = slice_bytes * num_slices;
	vector<vector<uint8_t>> pixel_data(6, vector<uint8_t>(vol_bytes, 0));

	// Track which cells have valid data (non-solid, traced)
	size_t total_cells = (size_t)dims[0] * dims[1] * dims[2];
	vector<bool> cell_valid(total_cells, false);

	float max_trace_dist = 8192.0f;
	int head_node = bsp_data.models[0].headnode[0];
	int cells_traced = 0;
	int cells_solid = 0;
	int cells_sunlit = 0;

	for (int gz = 0; gz < dims[2]; gz++) {
		for (int gy = 0; gy < dims[1]; gy++) {
			for (int gx = 0; gx < dims[0]; gx++) {
				float cx = mins[0] + (gx + 0.5f) * cell_size_gs;
				float cy = mins[1] + (gy + 0.5f) * cell_size_gs;
				float cz = mins[2] + (gz + 0.5f) * cell_size_gs;

				// Check if cell is in solid
				float gs_pos[3] = {cx, cy, cz};
				{
					int node_idx = head_node;
					while (node_idx >= 0) {
						if ((size_t)node_idx >= bsp_data.nodes.size()) break;
						const auto &node = bsp_data.nodes[node_idx];
						if (node.planenum < 0 || (size_t)node.planenum >= bsp_data.planes.size()) break;
						const auto &plane = bsp_data.planes[node.planenum];
						float d = plane.normal[0] * cx + plane.normal[1] * cy + plane.normal[2] * cz;
						node_idx = (d >= plane.dist) ? node.children[0] : node.children[1];
					}
					int leaf_idx = -(node_idx + 1);
					if (leaf_idx >= 0 && (size_t)leaf_idx < bsp_data.leafs.size()) {
						if (bsp_data.leafs[leaf_idx].contents == goldsrc::CONTENTS_SOLID) {
							cells_solid++;
							continue;
						}
					}
				}

				cells_traced++;
				size_t cell_idx = (size_t)gz * dims[0] * dims[1] + (size_t)gy * dims[0] + gx;
				cell_valid[cell_idx] = true;

				// Pixel offset within slice gz: row gy, column gx
				size_t px_ofs = (size_t)gz * slice_bytes + ((size_t)gy * slice_w + gx) * 3;

				// Trace the 6 axis-aligned directions
				for (int d = 0; d < 6; d++) {
					LightGridRayHit hit;
					trace_ray_bsp(bsp_data, head_node, gs_pos, directions[d],
						0.0f, max_trace_dist, hit);

					bool reached_sky = ray_reached_sky(bsp_data, hit);
					if (!reached_sky) {
						float r, g, b;
						sample_lightmap_at(bsp_data.faces[hit.face_index],
							hit.hit_pos, bsp_data.lighting, r, g, b);
						pixel_data[d][px_ofs + 0] = (uint8_t)clamp(r, 0.0f, 255.0f);
						pixel_data[d][px_ofs + 1] = (uint8_t)clamp(g, 0.0f, 255.0f);
						pixel_data[d][px_ofs + 2] = (uint8_t)clamp(b, 0.0f, 255.0f);
					} else if (has_sky_light) {
						// Ray reached the sky — ambient sky fill for non-sun directions
						float ambient_sky = 0.15f;
						float r = sky_color[0] * ambient_sky * 255.0f;
						float g = sky_color[1] * ambient_sky * 255.0f;
						float b = sky_color[2] * ambient_sky * 255.0f;
						pixel_data[d][px_ofs + 0] = (uint8_t)clamp(r, 0.0f, 255.0f);
						pixel_data[d][px_ofs + 1] = (uint8_t)clamp(g, 0.0f, 255.0f);
						pixel_data[d][px_ofs + 2] = (uint8_t)clamp(b, 0.0f, 255.0f);
					}
				}

				// Trace one extra ray toward the sun. If it reaches the sky
				// (hits a sky brush or nothing), this cell has direct sunlight —
				// add sun color to each cube face weighted by alignment with the
				// sun direction.
				if (has_sky_light) {
					LightGridRayHit sun_hit;
					trace_ray_bsp(bsp_data, head_node, gs_pos, sun_dir,
						0.0f, max_trace_dist, sun_hit);

					if (ray_reached_sky(bsp_data, sun_hit)) {
						cells_sunlit++;
						// Cell has direct sunlight — distribute sun color to cube faces
						// weighted by alignment toward the sun
						for (int d = 0; d < 6; d++) {
							float dot = directions[d][0] * sun_dir[0]
							          + directions[d][1] * sun_dir[1]
							          + directions[d][2] * sun_dir[2];
							float weight = fmaxf(dot, 0.0f);
							if (weight <= 0.0f) continue;
							float sr = sky_color[0] * weight * 255.0f;
							float sg = sky_color[1] * weight * 255.0f;
							float sb = sky_color[2] * weight * 255.0f;
							pixel_data[d][px_ofs + 0] = (uint8_t)clamp(
								pixel_data[d][px_ofs + 0] + sr, 0.0f, 255.0f);
							pixel_data[d][px_ofs + 1] = (uint8_t)clamp(
								pixel_data[d][px_ofs + 1] + sg, 0.0f, 255.0f);
							pixel_data[d][px_ofs + 2] = (uint8_t)clamp(
								pixel_data[d][px_ofs + 2] + sb, 0.0f, 255.0f);
						}
					}
				}
			}
		}
	}

	// Flood-fill invalid (solid) cells from nearest valid neighbors.
	// This prevents trilinear filtering from interpolating toward black at boundaries.
	{
		static const int offsets[6][3] = {
			{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}
		};
		int cells_filled = 0;
		// Iterate multiple passes until no more cells are filled
		for (int pass = 0; pass < 4; pass++) {
			int filled_this_pass = 0;
			for (int gz = 0; gz < dims[2]; gz++) {
				for (int gy = 0; gy < dims[1]; gy++) {
					for (int gx = 0; gx < dims[0]; gx++) {
						size_t idx = (size_t)gz * dims[0] * dims[1] + (size_t)gy * dims[0] + gx;
						if (cell_valid[idx]) continue;

						// Average all valid neighbors
						int count = 0;
						float accum[6][3] = {};
						for (int n = 0; n < 6; n++) {
							int nx = gx + offsets[n][0];
							int ny = gy + offsets[n][1];
							int nz = gz + offsets[n][2];
							if (nx < 0 || nx >= dims[0] || ny < 0 || ny >= dims[1] ||
								nz < 0 || nz >= dims[2]) continue;
							size_t nidx = (size_t)nz * dims[0] * dims[1] + (size_t)ny * dims[0] + nx;
							if (!cell_valid[nidx]) continue;
							count++;
							size_t npx = (size_t)nz * slice_bytes + ((size_t)ny * slice_w + nx) * 3;
							for (int d = 0; d < 6; d++) {
								accum[d][0] += pixel_data[d][npx + 0];
								accum[d][1] += pixel_data[d][npx + 1];
								accum[d][2] += pixel_data[d][npx + 2];
							}
						}
						if (count == 0) continue;

						float inv = 1.0f / count;
						size_t px_ofs = (size_t)gz * slice_bytes + ((size_t)gy * slice_w + gx) * 3;
						for (int d = 0; d < 6; d++) {
							pixel_data[d][px_ofs + 0] = (uint8_t)(accum[d][0] * inv);
							pixel_data[d][px_ofs + 1] = (uint8_t)(accum[d][1] * inv);
							pixel_data[d][px_ofs + 2] = (uint8_t)(accum[d][2] * inv);
						}
						cell_valid[idx] = true;
						filled_this_pass++;
					}
				}
			}
			cells_filled += filled_this_pass;
			if (filled_this_pass == 0) break;
		}
		UtilityFunctions::print("[GoldSrc] Light grid flood-fill: ", (int64_t)cells_filled, " cells filled");
	}

	// Build 6 arrays of slice Images for Texture3D construction.
	// GoldSrc → Godot direction mapping:
	// GS +X → Godot -X,  GS -X → Godot +X
	// GS +Y → Godot +Z,  GS -Y → Godot -Z
	// GS +Z → Godot +Y,  GS -Z → Godot -Y
	// Godot order [+X, -X, +Y, -Y, +Z, -Z] = GS indices [1, 0, 4, 5, 2, 3]
	static const int gs_to_godot[6] = {1, 0, 4, 5, 2, 3};

	// Each direction produces an Array of num_slices Images (sliced along GoldSrc Z = Godot Y)
	Array dir_slices; // Array of 6 Arrays, each containing num_slices Images
	dir_slices.resize(6);
	for (int godot_dir = 0; godot_dir < 6; godot_dir++) {
		int gs_dir = gs_to_godot[godot_dir];
		Array slices;
		slices.resize(num_slices);
		for (int s = 0; s < num_slices; s++) {
			PackedByteArray pba;
			pba.resize(slice_bytes);
			memcpy(pba.ptrw(), pixel_data[gs_dir].data() + (size_t)s * slice_bytes, slice_bytes);
			slices[s] = Image::create_from_data(slice_w, slice_h, false, Image::FORMAT_RGB8, pba);
		}
		dir_slices[godot_dir] = slices;
	}

	// Grid origin in Godot coords: GS(mins) → Godot(-mins[0]*sf, mins[2]*sf, mins[1]*sf)
	Vector3 grid_origin(-mins[0] * scale_factor, mins[2] * scale_factor, mins[1] * scale_factor);
	// Grid dims in Godot coords: GS(x,y,z) → Godot(x, z, y)
	Vector3i grid_dims(dims[0], dims[2], dims[1]);

	Dictionary result;
	result["grid_origin"] = grid_origin;
	result["grid_dims"] = grid_dims;
	result["cell_size"] = (double)(cell_size_gs * scale_factor);
	result["dir_slices"] = dir_slices; // 6 arrays of Images for Texture3D

	uint64_t elapsed = Time::get_singleton()->get_ticks_msec() - t0;
	UtilityFunctions::print("[GoldSrc] Light grid baked: ", dims[0], "x", dims[1], "x", dims[2],
		" (", (int64_t)cells_traced, " traced, ", (int64_t)cells_solid, " solid, ",
		(int64_t)cells_sunlit, " sunlit, ", (int64_t)elapsed, "ms)");

	return result;
}

