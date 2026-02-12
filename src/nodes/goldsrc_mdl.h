#ifndef GOLDSRC_MDL_H
#define GOLDSRC_MDL_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/classes/animation_player.hpp>
#include <godot_cpp/classes/animation.hpp>
#include <godot_cpp/classes/animation_library.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/quaternion.hpp>

#include "../parsers/mdl_parser.h"
#include <memory>
#include <vector>

using namespace godot;

class GoldSrcMDL : public Node3D {
	GDCLASS(GoldSrcMDL, Node3D)

protected:
	static void _bind_methods();

public:
	GoldSrcMDL();
	~GoldSrcMDL();

	Error load_mdl(const String &path);
	void build_model();
	int get_sequence_count() const;
	String get_sequence_name(int index) const;
	int get_bodypart_count() const;
	String get_bodypart_name(int index) const;
	int get_bone_count() const;

	void set_scale_factor(float scale);
	float get_scale_factor() const;

private:
	void build_skeleton();
	void build_meshes();
	void build_animations();

	Transform3D compute_bone_transform(int bone_idx, const goldsrc::ParsedAnimFrame &frame) const;
	std::vector<Transform3D> compute_bone_world_transforms(const goldsrc::ParsedAnimFrame &frame) const;
	static Quaternion euler_to_quat(float x, float y, float z);

	std::unique_ptr<goldsrc::MDLParser> parser;
	Skeleton3D *skeleton = nullptr;
	std::vector<Transform3D> rest_bone_world; // cached rest-pose world transforms
	float scale_factor = 0.025f;
	bool model_built = false;
};

#endif // GOLDSRC_MDL_H
