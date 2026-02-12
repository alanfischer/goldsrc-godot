#ifndef GOLDSRC_BSP_H
#define GOLDSRC_BSP_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/classes/static_body3d.hpp>
#include <godot_cpp/classes/concave_polygon_shape3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "../parsers/bsp_parser.h"
#include "goldsrc_wad.h"
#include <memory>

using namespace godot;

class GoldSrcBSP : public Node3D {
	GDCLASS(GoldSrcBSP, Node3D)

protected:
	static void _bind_methods();

public:
	GoldSrcBSP();
	~GoldSrcBSP();

	Error load_bsp(const String &path);
	void set_wad(const Ref<GoldSrcWAD> &wad);
	void add_wad(const Ref<GoldSrcWAD> &wad);
	Array get_entities() const;
	void build_mesh();

	void set_scale_factor(float scale);
	float get_scale_factor() const;

	void set_lightstyle(int style_index, float brightness);
	float get_lightstyle(int style_index) const;

private:
	Ref<ImageTexture> find_texture(const std::string &name) const;
	Vector3 goldsrc_to_godot(float x, float y, float z) const;
	void build_collision(Node3D *parent,
		const std::vector<const goldsrc::ParsedFace *> &faces);
	void rebake_lightstyle(int style_index);
	void bake_face_lightmap(int face_idx);

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
		Ref<Image> image;
		Ref<ImageTexture> texture;
		int width, height;
	};

	std::vector<FaceLightmapInfo> face_lm_info;
	std::vector<LightmapAtlasState> lm_atlases;
	float lightstyle_values[64];

	std::unique_ptr<goldsrc::BSPParser> parser;
	std::vector<Ref<GoldSrcWAD>> wads;
	float scale_factor = 0.025f; // GoldSrc units to Godot units
	bool mesh_built = false;
};

#endif // GOLDSRC_BSP_H
