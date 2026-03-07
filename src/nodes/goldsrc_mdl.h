#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/quaternion.hpp>

#include "../parsers/mdl_parser.h"
#include <memory>
#include <vector>

class GoldSrcMDL : public godot::Node3D {
	GDCLASS(GoldSrcMDL, godot::Node3D)

protected:
	static void _bind_methods();

public:
	GoldSrcMDL() = default;
	~GoldSrcMDL() = default;

	godot::Error load_mdl(const godot::String &path);
	void build_model();
	int get_sequence_count() const;
	godot::String get_sequence_name(int index) const;
	float get_sequence_fps(int index) const;
	int get_sequence_num_frames(int index) const;
	int get_bodypart_count() const;
	godot::String get_bodypart_name(int index) const;
	int get_bone_count() const;
	int get_skin_count() const;
	void set_skin(int family);
	godot::String get_skin_info() const;

	void set_scale_factor(float scale);
	float get_scale_factor() const;

private:
	void build_skeleton();
	void build_meshes();
	void build_animations();

	godot::Transform3D compute_bone_transform(int bone_idx, const goldsrc::ParsedAnimFrame &frame) const;
	std::vector<godot::Transform3D> compute_bone_world_transforms(const goldsrc::ParsedAnimFrame &frame) const;
	static godot::Quaternion euler_to_quat(float x, float y, float z);

	std::unique_ptr<goldsrc::MDLParser> parser;
	godot::Skeleton3D *skeleton = nullptr;
	std::vector<godot::Transform3D> rest_bone_world; // cached rest-pose world transforms
	std::vector<godot::Ref<godot::StandardMaterial3D>> stored_materials;
	std::vector<std::pair<godot::MeshInstance3D*, int>> mesh_skin_refs; // mesh instance + skin_ref
	float scale_factor = 0.025f;
	bool model_built = false;
};
