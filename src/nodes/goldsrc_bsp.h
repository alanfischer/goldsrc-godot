#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/array.hpp>

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

	void set_lightstyle(int style_index, float brightness);
	float get_lightstyle(int style_index) const;

	// Debug: check what the BSP thinks a point is (returns contents: -1=empty, -2=solid, -3=water)
	int point_contents(godot::Vector3 godot_pos) const;

	// Look up a texture by name from BSP embedded textures or loaded WADs
	godot::Ref<godot::ImageTexture> get_texture(const godot::String &name) const;

	// Get texture S/T axes for the face at a given Godot position.
	// Returns Array [s_axis: Vector3, t_axis: Vector3] in Godot coords, or empty if not found.
	godot::Array get_face_axes(godot::Vector3 godot_pos, godot::Vector3 godot_normal) const;

private:
	godot::Ref<godot::ImageTexture> find_texture(const std::string &name) const;
	godot::Vector3 goldsrc_to_godot(float x, float y, float z) const;
	void build_hull_collision(godot::Node3D *parent, int model_index,
		int hull_index, const godot::String &body_name,
		uint32_t collision_layer);
	void build_water_volumes(godot::Node3D *parent);
	void build_occluders(godot::Node3D *parent);
	void rebake_lightstyle(int style_index);

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

	std::unique_ptr<goldsrc::BSPParser> parser;
	std::vector<godot::Ref<GoldSrcWAD>> wads;
	mutable std::map<std::string, godot::Ref<godot::ImageTexture>> texture_cache;
	float scale_factor = 0.025f; // GoldSrc units to Godot units
	bool mesh_built = false;
};
