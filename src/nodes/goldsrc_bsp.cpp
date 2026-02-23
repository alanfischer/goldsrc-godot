#include "goldsrc_bsp.h"

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/classes/static_body3d.hpp>
#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/concave_polygon_shape3d.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/occluder_instance3d.hpp>
#include <godot_cpp/classes/polygon_occluder3d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/time.hpp>

#include "../bsp_hull.h"

#include <algorithm>
#include <cstring>
#include <map>

using namespace godot;
using namespace std;
using goldsrc_hull::HullPlane;
using goldsrc_hull::ConvexCell;
using goldsrc_hull::CellVertex;

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
render_mode unshaded;

uniform sampler2D albedo_texture : source_color;
uniform sampler2D lm_layer0 : filter_linear;
uniform sampler2D lm_layer1 : filter_linear;
uniform sampler2D lm_layer2 : filter_linear;
uniform sampler2D lm_layer3 : filter_linear;
uniform sampler2D lightstyle_tex : filter_nearest;
uniform float overbright = 2.0;

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

	ALBEDO = albedo.rgb * lm * overbright;
}
)";

// Alpha-scissor variant for '{' textures (fences, grates)
static const char *LIGHTSTYLE_SHADER_ALPHA_CODE = R"(
shader_type spatial;
render_mode unshaded;

uniform sampler2D albedo_texture : source_color;
uniform sampler2D lm_layer0 : filter_linear;
uniform sampler2D lm_layer1 : filter_linear;
uniform sampler2D lm_layer2 : filter_linear;
uniform sampler2D lm_layer3 : filter_linear;
uniform sampler2D lightstyle_tex : filter_nearest;
uniform float overbright = 2.0;
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

	ALBEDO = albedo.rgb * lm * overbright;
	ALPHA = albedo.a;
	ALPHA_SCISSOR_THRESHOLD = alpha_scissor_threshold;
}
)";

// Write raw (unweighted) lightmap data for a single layer into an atlas
static void write_layer_pixels(
	int layer_index, const uint8_t styles[4],
	int lightmap_offset, int lm_width, int lm_height,
	int atlas_x, int atlas_y, int atlas_width,
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
		if (px >= atlas_width || py < 0) continue;
		size_t dst = ((size_t)py * atlas_width + px) * 3;
		dest[dst + 0] = lighting[src + 0];
		dest[dst + 1] = lighting[src + 1];
		dest[dst + 2] = lighting[src + 2];
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
	ClassDB::bind_method(D_METHOD("get_texture", "name"), &GoldSrcBSP::get_texture);
	ClassDB::bind_method(D_METHOD("get_face_axes", "position", "normal"), &GoldSrcBSP::get_face_axes);
	ClassDB::bind_method(D_METHOD("set_shader_lightstyles", "enabled"), &GoldSrcBSP::set_shader_lightstyles);
	ClassDB::bind_method(D_METHOD("get_shader_lightstyles"), &GoldSrcBSP::get_shader_lightstyles);
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
	int atlas_x, int atlas_y, int atlas_width,
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
		if (px >= atlas_width || py < 0) continue;
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

	int num_models = (int)bsp_data.models.size();

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

		// Create a Node3D for this model
		Node3D *model_node = memnew(Node3D);
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

		// Set origin for brush entities
		if (m > 0) {
			const auto &bmodel = bsp_data.models[m];
			if (bmodel.origin[0] != 0 || bmodel.origin[1] != 0 || bmodel.origin[2] != 0) {
				model_node->set_position(goldsrc_to_godot(
					bmodel.origin[0], bmodel.origin[1], bmodel.origin[2]));
			}
		}

		// Store entity properties as metadata
		if (m > 0) {
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

		// Group this model's faces by texture, preserving global indices
		map<string, vector<FaceRef>> tex_groups;
		for (const auto &fr : faces_for_model) {
			tex_groups[fr.face->texture_name].push_back(fr);
		}

		for (const auto &group : tex_groups) {
			const string &tex_name = group.first;
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

			// --- Build lightmap atlas for this texture group ---
			ShelfPacker packer;
			bool has_any_lightmap = false;

			// First pass: pack all face lightmaps and record placement
			struct FacePack {
				int global_idx;
				int atlas_x, atlas_y;
				bool has_lightmap;
				int n_styles;
			};
			vector<FacePack> face_packs;
			face_packs.reserve(face_refs.size());

			for (const auto &fr : face_refs) {
				const auto *face = fr.face;
				FacePack fp;
				fp.global_idx = fr.global_index;
				fp.has_lightmap = false;
				fp.n_styles = 0;

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
						if (packer.pack(face->lightmap_width, face->lightmap_height,
							fp.atlas_x, fp.atlas_y)) {
							fp.has_lightmap = true;
							has_any_lightmap = true;

							// Record in face_lm_info for legacy rebaking path
							if (!shader_lightstyles) {
								auto &info = face_lm_info[fr.global_index];
								info.atlas_index = (int)lm_atlases.size();
								info.atlas_x = fp.atlas_x;
								info.atlas_y = fp.atlas_y;
								info.lm_width = face->lightmap_width;
								info.lm_height = face->lightmap_height;
								memcpy(info.styles, face->styles, 4);
								info.lightmap_offset = face->lightmap_offset;
							}
						}
					}
				}

				face_packs.push_back(fp);
			}

			// Create atlas images and bake lightmaps
			Ref<ImageTexture> atlas_texture;   // legacy: single blended atlas
			Ref<ImageTexture> layer_textures[4]; // shader: per-layer atlases
			int atlas_w = 0, atlas_h = 0;

			if (has_any_lightmap) {
				atlas_w = packer.width;
				atlas_h = packer.used_height();
				if (atlas_h < 1) atlas_h = 1;

				if (shader_lightstyles) {
					// Shader path: only create full atlas for layers that have data
					int max_styles = 0;
					for (const auto &fp : face_packs) {
						if (fp.has_lightmap && fp.n_styles > max_styles)
							max_styles = fp.n_styles;
					}

					PackedByteArray layer_pixels[4];
					for (int l = 0; l < max_styles; l++) {
						layer_pixels[l].resize(atlas_w * atlas_h * 3);
						memset(layer_pixels[l].ptrw(), 0, layer_pixels[l].size());
					}

					for (size_t fi = 0; fi < face_packs.size(); fi++) {
						const auto &fp = face_packs[fi];
						if (!fp.has_lightmap) continue;
						const auto *face = face_refs[fi].face;

						for (int l = 0; l < fp.n_styles; l++) {
							write_layer_pixels(l, face->styles,
								face->lightmap_offset,
								face->lightmap_width, face->lightmap_height,
								fp.atlas_x, fp.atlas_y, atlas_w,
								layer_pixels[l].ptrw(), bsp_data.lighting);
						}
					}

					for (int l = 0; l < max_styles; l++) {
						Ref<Image> img = Image::create_from_data(
							atlas_w, atlas_h, false, Image::FORMAT_RGB8, layer_pixels[l]);
						layer_textures[l] = ImageTexture::create_from_image(img);
					}
					// Unused layers stay null — will get black_1x1 below
				} else {
					// Legacy path: single blended atlas
					PackedByteArray atlas_pixels;
					atlas_pixels.resize(atlas_w * atlas_h * 3);
					memset(atlas_pixels.ptrw(), 255, atlas_pixels.size());

					uint8_t *atlas_ptr = atlas_pixels.ptrw();
					for (size_t fi = 0; fi < face_packs.size(); fi++) {
						const auto &fp = face_packs[fi];
						if (!fp.has_lightmap) continue;

						const auto *face = face_refs[fi].face;
						bake_lightmap_pixels(face->styles, face->lightmap_offset,
							face->lightmap_width, face->lightmap_height,
							fp.atlas_x, fp.atlas_y, atlas_w,
							atlas_ptr, bsp_data.lighting, lightstyle_values);
					}

					Ref<Image> atlas_img = Image::create_from_data(
						atlas_w, atlas_h, false, Image::FORMAT_RGB8, atlas_pixels);
					atlas_texture = ImageTexture::create_from_image(atlas_img);

					int atlas_idx = (int)lm_atlases.size();
					LightmapAtlasState state;
					state.image = atlas_img;
					state.texture = atlas_texture;
					state.width = atlas_w;
					state.height = atlas_h;
					lm_atlases.push_back(state);

					for (const auto &fp : face_packs) {
						if (fp.has_lightmap) {
							face_lm_info[fp.global_idx].atlas_index = atlas_idx;
						}
					}
				}
			}

			// --- Build mesh arrays with UV2 (and COLOR for shader path) ---
			PackedVector3Array vertices;
			PackedVector3Array normals;
			PackedFloat32Array tangents;
			PackedVector2Array uvs;
			PackedVector2Array uv2s;
			PackedColorArray colors; // shader path: style indices
			PackedInt32Array indices;
			int vert_offset = 0;

			for (size_t fi = 0; fi < face_refs.size(); fi++) {
				const auto *face = face_refs[fi].face;
				const auto &fp = face_packs[fi];
				int nv = (int)face->vertices.size();
				if (nv < 3) continue;

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

				for (int i = 2; i < nv; i++) {
					const int tri_indices[3] = {0, i - 1, i};

					for (int t = 0; t < 3; t++) {
						const auto &v = face->vertices[tri_indices[t]];

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

						tangents.push_back(1.0f);
						tangents.push_back(0.0f);
						tangents.push_back(0.0f);
						tangents.push_back(1.0f);

						indices.push_back(vert_offset++);
					}
				}
			}

			Ref<ArrayMesh> arr_mesh;
			arr_mesh.instantiate();

			Array arrays;
			arrays.resize(ArrayMesh::ARRAY_MAX);
			arrays[ArrayMesh::ARRAY_VERTEX] = vertices;
			arrays[ArrayMesh::ARRAY_NORMAL] = normals;
			arrays[ArrayMesh::ARRAY_TANGENT] = tangents;
			arrays[ArrayMesh::ARRAY_TEX_UV] = uvs;
			arrays[ArrayMesh::ARRAY_TEX_UV2] = uv2s;
			arrays[ArrayMesh::ARRAY_INDEX] = indices;
			if (shader_lightstyles) {
				arrays[ArrayMesh::ARRAY_COLOR] = colors;
			}

			arr_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);

			Ref<ImageTexture> texture = find_texture(tex_name);
			bool is_alpha_scissor = tex_name.size() > 0 && tex_name[0] == '{';

			if (shader_lightstyles) {
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
					if (has_any_lightmap && layer_textures[l].is_valid()) {
						material->set_shader_parameter(param, layer_textures[l]);
					} else {
						material->set_shader_parameter(param, black_1x1);
					}
				}

				material->set_shader_parameter("lightstyle_tex", lightstyle_texture);
				material->set_shader_parameter("overbright", 2.0f);

				arr_mesh->surface_set_material(0, material);
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

				if (has_any_lightmap && atlas_texture.is_valid()) {
					material->set_feature(BaseMaterial3D::FEATURE_DETAIL, true);
					material->set_texture(BaseMaterial3D::TEXTURE_DETAIL_ALBEDO, atlas_texture);
					material->set_detail_blend_mode(BaseMaterial3D::BLEND_MODE_MUL);
					material->set_detail_uv(BaseMaterial3D::DETAIL_UV_2);
				}

				arr_mesh->surface_set_material(0, material);
			}

			MeshInstance3D *mesh_instance = memnew(MeshInstance3D);
			mesh_instance->set_name(String(tex_name.c_str()));
			mesh_instance->set_mesh(arr_mesh);
			model_node->add_child(mesh_instance);
		}

		// Build collision for this model
		if (m == 0) {
			log_timing("worldspawn meshes");
			// Hull 0: face-based collision (exact visible geometry).
			// Hull 1: clip brushes, un-expanded with internal-split filtering
			// to reconstruct original brush geometry.
			build_hull_collision(model_node, m, 0, "WorldCollision", 1);
			log_timing("hull 0 collision");
			build_water_volumes(model_node);
			build_occluders(model_node);
			log_timing("water + occluders");
			build_clip_hull_debug(model_node, 3); // hull 3 = crouching
			log_timing("clip hull collision");
		} else {
			// Brush entities: hull 0 collision on layer 1 (GDScript converts
			// to Area3D for triggers/ladders by reparenting the CollisionShape3D)
			build_hull_collision(model_node, m, 0, "StaticBody3D", 1);
		}
	}
	log_timing("brush entity meshes + collision");

	UtilityFunctions::print("[GoldSrc] Built BSP: ",
		(int64_t)num_models, " models, ",
		(int64_t)bsp_data.faces.size(), " faces, ",
		(int64_t)lm_atlases.size(), " lightmap atlases");
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
		// Skip water/slime/lava faces (textures starting with '!')
		if (!face.texture_name.empty() && face.texture_name[0] == '!') continue;
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

void GoldSrcBSP::build_water_volumes(Node3D *parent) {
	const auto &bsp_data = parser->get_data();
	if (bsp_data.models.empty()) return;

	const auto &bmodel = bsp_data.models[0]; // worldspawn

	// Bounding box planes (padded)
	HullPlane bbox_planes[6];
	bbox_planes[0] = {{ 1, 0, 0}, bmodel.maxs[0] + 1.0f};
	bbox_planes[1] = {{-1, 0, 0}, -bmodel.mins[0] + 1.0f};
	bbox_planes[2] = {{ 0, 1, 0}, bmodel.maxs[1] + 1.0f};
	bbox_planes[3] = {{ 0,-1, 0}, -bmodel.mins[1] + 1.0f};
	bbox_planes[4] = {{ 0, 0, 1}, bmodel.maxs[2] + 1.0f};
	bbox_planes[5] = {{ 0, 0,-1}, -bmodel.mins[2] + 1.0f};

	const float EPSILON = 0.1f; // slightly larger epsilon for water volumes

	// Walk hull 0 BSP tree collecting CONTENTS_WATER leaves
	int root = bmodel.headnode[0];
	vector<HullPlane> accumulated;
	vector<ConvexCell> cells;
	goldsrc_hull::walk_bsp_tree(bsp_data.nodes, bsp_data.leafs, bsp_data.planes,
		root, accumulated, cells, goldsrc::CONTENTS_WATER);

	if (cells.empty()) return;

	// Create a single Area3D for all water volumes
	Area3D *water_area = memnew(Area3D);
	water_area->set_name("WaterVolumes");
	water_area->set_collision_layer(1 << 2); // layer 3
	water_area->set_collision_mask(0);
	water_area->set_monitorable(true);
	water_area->set_monitoring(false);

	int shape_count = 0;
	for (auto &cell : cells) {
		for (int i = 0; i < 6; i++) {
			cell.planes.push_back(bbox_planes[i]);
		}
		vector<CellVertex> verts = goldsrc_hull::compute_cell_vertices(
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
		col->set_name(String("WaterShape_") + String::num_int64(shape_count));
		col->set_shape(shape);
		water_area->add_child(col);
		shape_count++;
	}

	if (shape_count > 0) {
		parent->add_child(water_area);
		UtilityFunctions::print("[GoldSrc] Water volumes: ",
			(int64_t)shape_count, " convex shapes from ",
			(int64_t)cells.size(), " BSP leaves");
	} else {
		memdelete(water_area);
	}
}

void GoldSrcBSP::build_occluders(Node3D *parent) {
	const auto &bsp_data = parser->get_data();

	// Minimum face area in GoldSrc square units to qualify as an occluder.
	// 4096 ≈ a 64×64 unit face. Smaller faces cost CPU rasterization time
	// but occlude very little. Tune this up if profiling shows overhead.
	const float MIN_AREA_GS = 32768.0f;  // ~180x180 GoldSrc units

	int count = 0;

	for (const auto &face : bsp_data.faces) {
		if (face.model_index != 0) continue;  // worldspawn only

		// Skip textures that shouldn't occlude
		const auto &tn = face.texture_name;
		if (tn.empty()) continue;
		if (tn[0] == '{') continue;   // alpha-tested (fences, grates)
		if (tn[0] == '!') continue;   // water/slime/lava
		if (tn.compare(0, 3, "sky") == 0) continue;
		if (goldsrc::is_tool_texture(tn)) continue;

		int nv = (int)face.vertices.size();
		if (nv < 3) continue;

		// Compute face area in GoldSrc units (sum of triangle cross products)
		float area = 0.0f;
		for (int i = 2; i < nv; i++) {
			Vector3 a(face.vertices[i-1].pos[0] - face.vertices[0].pos[0],
			          face.vertices[i-1].pos[1] - face.vertices[0].pos[1],
			          face.vertices[i-1].pos[2] - face.vertices[0].pos[2]);
			Vector3 b(face.vertices[i].pos[0] - face.vertices[0].pos[0],
			          face.vertices[i].pos[1] - face.vertices[0].pos[1],
			          face.vertices[i].pos[2] - face.vertices[0].pos[2]);
			area += a.cross(b).length();
		}
		area *= 0.5f;
		if (area < MIN_AREA_GS) continue;

		// Convert vertices to Godot coords and compute centroid
		Vector3 centroid(0, 0, 0);
		vector<Vector3> gd_verts(nv);
		for (int i = 0; i < nv; i++) {
			gd_verts[i] = goldsrc_to_godot(
				face.vertices[i].pos[0],
				face.vertices[i].pos[1],
				face.vertices[i].pos[2]);
			centroid += gd_verts[i];
		}
		centroid /= (float)nv;

		// Face normal in Godot coords (same axis swap as goldsrc_to_godot, no scale)
		Vector3 gd_normal(-face.normal[0], face.normal[2], face.normal[1]);
		gd_normal.normalize();

		// Build orthonormal basis on the face plane in Godot space.
		// Pick a reference axis that isn't parallel to the normal.
		Vector3 ref = (gd_normal.y > -0.9f && gd_normal.y < 0.9f)
			? Vector3(0, 1, 0) : Vector3(1, 0, 0);
		Vector3 local_x = ref.cross(gd_normal).normalized();
		Vector3 local_y = gd_normal.cross(local_x).normalized();

		// Project vertices into 2D relative to centroid
		PackedVector2Array polygon;
		polygon.resize(nv);
		for (int i = 0; i < nv; i++) {
			Vector3 rel = gd_verts[i] - centroid;
			polygon[i] = Vector2(rel.dot(local_x), rel.dot(local_y));
		}

		// Build transform: local XY = face plane, Z = normal, origin = centroid
		Transform3D xform;
		xform.basis.set_column(0, local_x);
		xform.basis.set_column(1, local_y);
		xform.basis.set_column(2, gd_normal);
		xform.origin = centroid;

		Ref<PolygonOccluder3D> occluder;
		occluder.instantiate();
		occluder->set_polygon(polygon);

		OccluderInstance3D *inst = memnew(OccluderInstance3D);
		inst->set_occluder(occluder);
		inst->set_transform(xform);
		parent->add_child(inst);
		count++;
	}

	UtilityFunctions::print("[GoldSrc] Built ", (int64_t)count, " occluders");
}

void GoldSrcBSP::build_clip_hull_debug(Node3D *parent, int hull_index) {
	const auto &bsp_data = parser->get_data();
	if (bsp_data.models.empty()) return;
	if (hull_index < 1 || hull_index > 3) return;
	if (bsp_data.clipnodes.empty()) return;

	const auto &bmodel = bsp_data.models[0]; // worldspawn
	int root = bmodel.headnode[hull_index];
	if (root < 0 || (size_t)root >= bsp_data.clipnodes.size()) return;

	// Bounding box planes to cap exterior cells
	HullPlane bbox_planes[6];
	bbox_planes[0] = {{ 1, 0, 0}, bmodel.maxs[0] + 1.0f};
	bbox_planes[1] = {{-1, 0, 0}, -bmodel.mins[0] + 1.0f};
	bbox_planes[2] = {{ 0, 1, 0}, bmodel.maxs[1] + 1.0f};
	bbox_planes[3] = {{ 0,-1, 0}, -bmodel.mins[1] + 1.0f};
	bbox_planes[4] = {{ 0, 0, 1}, bmodel.maxs[2] + 1.0f};
	bbox_planes[5] = {{ 0, 0,-1}, -bmodel.mins[2] + 1.0f};

	const float EPSILON = 0.1f;

	// Hull 3 (crouching) half-extents in GoldSrc units
	const float HULL_HX = 16.0f, HULL_HY = 16.0f, HULL_HZ = 18.0f;

	// Walk clip tree collecting CONTENTS_SOLID cells
	vector<HullPlane> accumulated;
	vector<ConvexCell> cells;
	goldsrc_hull::walk_clip_tree(bsp_data.clipnodes, bsp_data.planes,
		root, accumulated, cells, goldsrc::CONTENTS_SOLID);

	if (cells.empty()) return;

	// Collision body for all clip hull shapes
	StaticBody3D *clip_body = memnew(StaticBody3D);
	clip_body->set_name(String("ClipHull") + String::num_int64(hull_index) + "_Collision");
	clip_body->set_collision_layer(1);
	clip_body->set_collision_mask(0);
	int shape_count = 0;

	for (auto &cell : cells) {
		int original_plane_count = (int)cell.planes.size();

		// Add bounding box planes to cap unbounded exterior cells
		for (int i = 0; i < 6; i++) {
			cell.planes.push_back(bbox_planes[i]);
		}

		// For each original plane, walk the sibling subtree to resolve
		// what's on the other side. When a face spans a sibling split,
		// slice the cell so each sub-cell's face borders a single leaf.
		vector<ConvexCell> current = {cell};
		for (int pi = 0; pi < original_plane_count; pi++) {
			vector<ConvexCell> next;
			for (auto &c : current) {
				goldsrc_hull::split_cell_for_face(c, pi, c.planes[pi].sibling_child,
					bsp_data.clipnodes, bsp_data.planes,
					EPSILON, next);
			}
			current = std::move(next);
		}

		// Apply unexpansion and triangulate each resulting sub-cell
		for (auto &c : current) {
			auto pre_verts = goldsrc_hull::compute_cell_vertices(c.planes, EPSILON);
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
				float support = goldsrc_hull::minkowski_support(hp.normal, HULL_HX, HULL_HY, HULL_HZ);
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
						if (fabsf(dot - hp.dist) < 0.5f) {
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
						int contents = goldsrc_hull::clip_tree_contents(
							bsp_data.clipnodes, bsp_data.planes,
							root, probe_x, probe_y, probe_z);
						if (contents != goldsrc::CONTENTS_SOLID) {
							uinfo[pi].should_unexpand = true;
						}
					}
				}
			}

			// Second pass: for nearly-antiparallel plane pairs that are both
			// being unexpanded, check if the slab between them is thinner than
			// the combined support. If so, clamp proportionally.
			// Slab gap = di + dj (for nj ≈ -ni), the direct plane-distance gap.
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
					if (dot_normals > -0.8f) continue;

					float slab_gap = pi_plane.dist + pj_plane.dist;
					float total_support = uinfo[i].support + uinfo[j].support;
					if (total_support > slab_gap) {
						float available = fmaxf(0.0f, slab_gap - 8.0f);
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

			vector<CellVertex> verts = goldsrc_hull::compute_cell_vertices(
				c.planes, EPSILON);

			// Fallback: if the cell collapsed, binary search for the maximum
			// unexpansion scale that produces valid geometry
			if ((int)verts.size() < 4) {
				for (int pi = 0; pi < original_plane_count; pi++)
					c.planes[pi].dist = orig_dist[pi];

				float lo = 0.0f, hi = 1.0f;
				for (int iter = 0; iter < 12; iter++) {
					float mid = (lo + hi) * 0.5f;
					auto test = c.planes;
					for (int pi = 0; pi < original_plane_count; pi++) {
						if (uinfo[pi].should_unexpand)
							test[pi].dist -= uinfo[pi].clamped_support * mid;
					}
					auto tv = goldsrc_hull::compute_cell_vertices(test, EPSILON);
					if ((int)tv.size() >= 4) lo = mid;
					else hi = mid;
				}

				if (lo > 0.01f) {
					for (int pi = 0; pi < original_plane_count; pi++) {
						if (uinfo[pi].should_unexpand)
							c.planes[pi].dist -= uinfo[pi].clamped_support * lo;
					}
					verts = goldsrc_hull::compute_cell_vertices(c.planes, EPSILON);
				}
			}

			if ((int)verts.size() < 4) continue;

			// Skip BSP splitting artifacts: cells thinner than 4 GS
			// units along any axis are degenerate slivers from tree splits
			{
				float mn[3] = {1e9f, 1e9f, 1e9f};
				float mx[3] = {-1e9f, -1e9f, -1e9f};
				for (const auto &v : verts) {
					for (int a = 0; a < 3; a++) {
						if (v.gs[a] < mn[a]) mn[a] = v.gs[a];
						if (v.gs[a] > mx[a]) mx[a] = v.gs[a];
					}
				}
				float min_dim = fminf(fminf(mx[0]-mn[0], mx[1]-mn[1]), mx[2]-mn[2]);
				if (min_dim < 4.0f) continue;
			}

			// Create collision shape from vertex point cloud
			{
				PackedVector3Array points;
				for (const auto &v : verts) {
					points.push_back(goldsrc_to_godot(v.gs[0], v.gs[1], v.gs[2]));
				}
				Ref<ConvexPolygonShape3D> shape;
				shape.instantiate();
				shape->set_points(points);

				CollisionShape3D *col = memnew(CollisionShape3D);
				col->set_name(String("ClipShape_") + String::num_int64(shape_count));
				col->set_shape(shape);
				clip_body->add_child(col);
				shape_count++;
			}
		}
	}

	if (shape_count > 0) {
		parent->add_child(clip_body);
		UtilityFunctions::print("[GoldSrc] Clip hull ", (int64_t)hull_index,
			": ", (int64_t)shape_count, " collision shapes");
	} else {
		memdelete(clip_body);
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
				info.atlas_x, info.atlas_y, atlas.width,
				ptr, bsp_data.lighting, lightstyle_values);
		}

		atlas.image = Image::create_from_data(atlas.width, atlas.height,
			false, Image::FORMAT_RGB8, img_data);
		atlas.texture->update(atlas.image);
	}
}

