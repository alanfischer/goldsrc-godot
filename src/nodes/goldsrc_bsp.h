#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/animatable_body3d.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "../parsers/bsp_parser.h"
#include "goldsrc_wad.h"
#include <map>
#include <memory>

class GoldSrcBSP : public godot::Node3D {
	GDCLASS(GoldSrcBSP, godot::Node3D)

protected:
	static void _bind_methods();

public:
	GoldSrcBSP();
	~GoldSrcBSP() = default;

	godot::Error load_bsp(const godot::String &path);
	void set_wad(const godot::Ref<GoldSrcWAD> &wad);
	void add_wad(const godot::Ref<GoldSrcWAD> &wad);
	godot::Array get_entities() const;
	void build_mesh();

	void set_scale_factor(float scale);
	float get_scale_factor() const;

	void set_shader_lightstyles(bool enabled);
	bool get_shader_lightstyles() const;

	void set_lightstyle(int style_index, float brightness);
	float get_lightstyle(int style_index) const;

	godot::Ref<godot::Image> get_lightstyle_image() const;
	godot::Ref<godot::ImageTexture> get_lightstyle_texture() const;

	// Debug: check what the BSP thinks a point is (returns contents: -1=empty, -2=solid, -3=water)
	int point_contents(godot::Vector3 godot_pos) const;

	// Classify a point against a clip hull (1-3). Returns contents (-1=empty, -2=solid, etc.)
	int classify_point_hull(godot::Vector3 godot_pos, int hull_index) const;

	// Debug: visualize hull solid/empty convex volumes as transparent meshes
	void build_debug_hull_meshes(int hull_index);

	// Look up a texture by name from BSP embedded textures or loaded WADs
	godot::Ref<godot::ImageTexture> get_texture(const godot::String &name) const;

	// Get texture S/T axes for the face at a given Godot position.
	// Returns Array [s_axis: Vector3, t_axis: Vector3] in Godot coords, or empty if not found.
	godot::Array get_face_axes(godot::Vector3 godot_pos, godot::Vector3 godot_normal) const;

	// Bake ambient cube light grid from BSP lightmaps.
	// Returns Dictionary {grid_origin: Vector3, grid_dims: Vector3i, cell_size: float, images: Array[Image]}
	// Images are 6 RGB8 images (one per axis: +X, -X, +Y, -Y, +Z, -Z in Godot coords).
	godot::Dictionary bake_light_grid(float cell_size_gs = 32.0f) const;

private:
	godot::Ref<godot::ImageTexture> find_texture(const std::string &name) const;
	godot::Vector3 goldsrc_to_godot(float x, float y, float z) const;
	void build_hull_collision(godot::Node3D *parent, int model_index,
		int hull_index, const godot::String &body_name,
		uint32_t collision_layer);
	// Brush entity collision: two approaches (swap at the call site to compare).
	// Concave: triangle soup from face geometry (ConcavePolygonShape3D).
	// Convex: solid cells from BSP leaf decomposition (ConvexPolygonShape3D).
	// Both add CollisionShape3D children directly to parent.
	void build_brush_concave(godot::Node3D *parent, int model_index);
	void build_brush_convex(godot::Node3D *parent, int model_index,
		int contents_filter = 0);
	void build_occluders(godot::Node3D *parent);
	void rebake_lightstyle(int style_index);

	void set_debug_occluders(bool enabled);
	bool get_debug_occluders() const;

	void set_occluder_min_area(float area);
	float get_occluder_min_area() const;
	void set_occluder_boundary_margin(float margin);
	float get_occluder_boundary_margin() const;
	void set_occluder_exclude_normal(godot::Vector3 normal);
	godot::Vector3 get_occluder_exclude_normal() const;
	void set_occluder_exclude_threshold(float threshold);
	float get_occluder_exclude_threshold() const;
	void set_occluder_max_count(int count);
	int get_occluder_max_count() const;

	// Per-face lightmap placement info (for rebaking)
	struct FaceLightmapInfo {
		int atlas_index = -1;  // which atlas this face is in (-1 = none)
		int atlas_x = 0, atlas_y = 0;
		int lm_width = 0, lm_height = 0;
		uint8_t styles[4] = {255, 255, 255, 255};
		int lightmap_offset = -1;
	};

	// Per-atlas state
	struct LightmapAtlasState {
		godot::Ref<godot::Image> image;
		godot::Ref<godot::ImageTexture> texture;
		int width, height;
	};

	std::vector<FaceLightmapInfo> face_lm_info;
	std::vector<LightmapAtlasState> lm_atlases;
	float lightstyle_values[64];

	// Shader-based lightstyle members
	bool shader_lightstyles = true;
	bool debug_occluders = false;
	godot::Ref<godot::Image> lightstyle_image;         // 64×1 FORMAT_RF
	godot::Ref<godot::ImageTexture> lightstyle_texture; // shared across all materials

	std::unique_ptr<goldsrc::BSPParser> parser;
	std::vector<godot::Ref<GoldSrcWAD>> wads;
	mutable std::map<std::string, godot::Ref<godot::ImageTexture>> texture_cache;
	float scale_factor = 0.025f; // GoldSrc units to Godot units
	float occluder_min_area = 65535.0f; // minimum face area (GS units²) to qualify as occluder (~256×256)
	float occluder_boundary_margin = 512.0f; // skip faces within this many GS units of the map bbox boundary
	godot::Vector3 occluder_exclude_normal = godot::Vector3(0, 1, 0); // Godot-space axis to exclude (Y-up = horizontal faces); zero threshold disables
	float occluder_exclude_threshold = 0.7f; // faces with abs(dot(normal, exclude_normal)) > threshold are excluded; 0 = disabled
	int occluder_max_count = 0; // max occluders to keep after sorting by area; 0 = unlimited
	bool mesh_built = false;
};
