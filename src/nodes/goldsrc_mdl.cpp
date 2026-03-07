#include "goldsrc_mdl.h"

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/animation_player.hpp>
#include <godot_cpp/classes/animation.hpp>
#include <godot_cpp/classes/animation_library.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/basis.hpp>

#include <cmath>
#include <cstring>

using namespace godot;
using namespace std;

void GoldSrcMDL::_bind_methods() {
	ClassDB::bind_method(D_METHOD("load_mdl", "path"), &GoldSrcMDL::load_mdl);
	ClassDB::bind_method(D_METHOD("build_model"), &GoldSrcMDL::build_model);
	ClassDB::bind_method(D_METHOD("get_sequence_count"), &GoldSrcMDL::get_sequence_count);
	ClassDB::bind_method(D_METHOD("get_sequence_name", "index"), &GoldSrcMDL::get_sequence_name);
	ClassDB::bind_method(D_METHOD("get_sequence_fps", "index"), &GoldSrcMDL::get_sequence_fps);
	ClassDB::bind_method(D_METHOD("get_sequence_num_frames", "index"), &GoldSrcMDL::get_sequence_num_frames);
	ClassDB::bind_method(D_METHOD("get_bodypart_count"), &GoldSrcMDL::get_bodypart_count);
	ClassDB::bind_method(D_METHOD("get_bodypart_name", "index"), &GoldSrcMDL::get_bodypart_name);
	ClassDB::bind_method(D_METHOD("get_bone_count"), &GoldSrcMDL::get_bone_count);
	ClassDB::bind_method(D_METHOD("get_skin_count"), &GoldSrcMDL::get_skin_count);
	ClassDB::bind_method(D_METHOD("set_skin", "family"), &GoldSrcMDL::set_skin);
	ClassDB::bind_method(D_METHOD("get_skin_info"), &GoldSrcMDL::get_skin_info);
	ClassDB::bind_method(D_METHOD("set_scale_factor", "scale"), &GoldSrcMDL::set_scale_factor);
	ClassDB::bind_method(D_METHOD("get_scale_factor"), &GoldSrcMDL::get_scale_factor);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "scale_factor"), "set_scale_factor", "get_scale_factor");
}

Error GoldSrcMDL::load_mdl(const String &path) {
	Ref<FileAccess> file = FileAccess::open(path, FileAccess::READ);
	if (file.is_null()) {
		UtilityFunctions::printerr("[GoldSrc] Cannot open MDL file: ", path);
		return ERR_FILE_NOT_FOUND;
	}

	int64_t len = file->get_length();
	PackedByteArray data = file->get_buffer(len);

	parser = make_unique<goldsrc::MDLParser>();
	if (!parser->parse(data.ptr(), data.size())) {
		UtilityFunctions::printerr("[GoldSrc] Failed to parse MDL file: ", path);
		parser.reset();
		return ERR_PARSE_ERROR;
	}

	model_built = false;
	const auto &mdl = parser->get_data();
	UtilityFunctions::print("[GoldSrc] Loaded MDL: ", path,
		" (", (int64_t)mdl.bones.size(), " bones, ",
		(int64_t)mdl.sequences.size(), " sequences, ",
		(int64_t)mdl.bodyparts.size(), " bodyparts)");
	return OK;
}

void GoldSrcMDL::set_scale_factor(float scale) {
	scale_factor = scale;
}

float GoldSrcMDL::get_scale_factor() const {
	return scale_factor;
}

int GoldSrcMDL::get_sequence_count() const {
	if (!parser) return 0;
	return (int)parser->get_data().sequences.size();
}

String GoldSrcMDL::get_sequence_name(int index) const {
	if (!parser || index < 0 || index >= get_sequence_count()) return "";
	return String(parser->get_data().sequences[index].name.c_str());
}

float GoldSrcMDL::get_sequence_fps(int index) const {
	if (!parser || index < 0 || index >= get_sequence_count()) return 0.0f;
	return parser->get_data().sequences[index].fps;
}

int GoldSrcMDL::get_sequence_num_frames(int index) const {
	if (!parser || index < 0 || index >= get_sequence_count()) return 0;
	return parser->get_data().sequences[index].num_frames;
}

int GoldSrcMDL::get_bodypart_count() const {
	if (!parser) return 0;
	return (int)parser->get_data().bodyparts.size();
}

String GoldSrcMDL::get_bodypart_name(int index) const {
	if (!parser || index < 0 || index >= get_bodypart_count()) return "";
	return String(parser->get_data().bodyparts[index].name.c_str());
}

int GoldSrcMDL::get_bone_count() const {
	if (!parser) return 0;
	return (int)parser->get_data().bones.size();
}

int GoldSrcMDL::get_skin_count() const {
	if (!parser) return 0;
	return parser->get_data().num_skin_families;
}

void GoldSrcMDL::set_skin(int family) {
	if (!parser) return;
	const auto &mdl = parser->get_data();
	if (family < 0 || family >= mdl.num_skin_families) return;

	for (auto &[mesh_inst, skin_ref] : mesh_skin_refs) {
		if (!mesh_inst || !UtilityFunctions::is_instance_valid(mesh_inst)) continue;
		int tex_idx = skin_ref;
		int table_idx = family * mdl.num_skin_ref + skin_ref;
		if (table_idx >= 0 && table_idx < (int)mdl.skin_table.size()) {
			tex_idx = mdl.skin_table[table_idx];
		}
		if (tex_idx >= 0 && tex_idx < (int)stored_materials.size()) {
			mesh_inst->set_surface_override_material(0, stored_materials[tex_idx]);
		}
	}
}

String GoldSrcMDL::get_skin_info() const {
	if (!parser) return "no parser";
	const auto &mdl = parser->get_data();
	String info = "Skin families: " + String::num_int64(mdl.num_skin_families) +
		", skin refs: " + String::num_int64(mdl.num_skin_ref) +
		", textures: " + String::num_int64(mdl.textures.size()) + "\n";
	// List texture names
	for (int i = 0; i < (int)mdl.textures.size(); i++) {
		info += "  tex[" + String::num_int64(i) + "] = " + String(mdl.textures[i].name.c_str()) +
			" (" + String::num_int64(mdl.textures[i].width) + "x" + String::num_int64(mdl.textures[i].height) + ")\n";
	}
	// Dump skin table
	for (int f = 0; f < mdl.num_skin_families; f++) {
		info += "  family " + String::num_int64(f) + ": [";
		for (int r = 0; r < mdl.num_skin_ref; r++) {
			int idx = f * mdl.num_skin_ref + r;
			if (idx < (int)mdl.skin_table.size()) {
				if (r > 0) info += ", ";
				info += String::num_int64(mdl.skin_table[idx]);
			}
		}
		info += "]\n";
	}
	return info;
}

Quaternion GoldSrcMDL::euler_to_quat(float x, float y, float z) {
	// GoldSrc XYZ Euler angles — matches HL SDK AngleQuaternion exactly.
	// angles[0]=X, angles[1]=Y, angles[2]=Z
	float cx = cos(x * 0.5f), sx = sin(x * 0.5f);
	float cy = cos(y * 0.5f), sy = sin(y * 0.5f);
	float cz = cos(z * 0.5f), sz = sin(z * 0.5f);

	return Quaternion(
		sx * cy * cz - cx * sy * sz,
		cx * sy * cz + sx * cy * sz,
		cx * cy * sz - sx * sy * cz,
		cx * cy * cz + sx * sy * sz
	);
}

// Compute a bone's LOCAL transform, converted from GoldSrc Z-up to Godot Y-up.
//
// Coordinate conversion: GoldSrc (X=right, Y=forward, Z=up) → Godot (X=right, Y=up, Z=back)
// This is a -90° rotation around the X axis.
//
// For positions:     (x, y, z) → (x, z, -y)
// For quaternions:   q_godot = q_C * q_gs * q_C^-1 where q_C = rot(-90°, X)
//                    which simplifies to (qx, qy, qz, qw) → (qx, qz, -qy, qw)
Transform3D GoldSrcMDL::compute_bone_transform(int bone_idx, const goldsrc::ParsedAnimFrame &frame) const {
	float px = frame.bone_pos[bone_idx * 3 + 0];
	float py = frame.bone_pos[bone_idx * 3 + 1];
	float pz = frame.bone_pos[bone_idx * 3 + 2];

	float rx = frame.bone_rot[bone_idx * 3 + 0];
	float ry = frame.bone_rot[bone_idx * 3 + 1];
	float rz = frame.bone_rot[bone_idx * 3 + 2];

	Quaternion q = euler_to_quat(rx, ry, rz);

	// Position: GoldSrc→Godot swizzle + scale
	Vector3 pos(
		px * scale_factor,
		pz * scale_factor,
		-py * scale_factor
	);

	// Quaternion: GoldSrc→Godot via conjugation by -90° X rotation
	Quaternion godot_q(q.x, q.z, -q.y, q.w);

	Transform3D t;
	t.basis = Basis(godot_q);
	t.origin = pos;
	return t;
}

// Compute world (model-space) transforms for all bones by walking the hierarchy.
// Result is in Godot coordinate space.
vector<Transform3D> GoldSrcMDL::compute_bone_world_transforms(const goldsrc::ParsedAnimFrame &frame) const {
	const auto &mdl = parser->get_data();
	int num_bones = (int)mdl.bones.size();
	vector<Transform3D> world(num_bones);

	for (int i = 0; i < num_bones; i++) {
		Transform3D local = compute_bone_transform(i, frame);
		if (mdl.bones[i].parent >= 0) {
			world[i] = world[mdl.bones[i].parent] * local;
		} else {
			world[i] = local;
		}
	}
	return world;
}

void GoldSrcMDL::build_model() {
	if (!parser || model_built) return;
	model_built = true;

	build_skeleton();
	build_meshes();
	build_animations();
}

void GoldSrcMDL::build_skeleton() {
	const auto &mdl = parser->get_data();
	if (mdl.bones.empty()) return;

	skeleton = memnew(Skeleton3D);
	skeleton->set_name("Skeleton3D");
	add_child(skeleton);

	// Add bones
	for (int i = 0; i < (int)mdl.bones.size(); i++) {
		const auto &bone = mdl.bones[i];
		skeleton->add_bone(String(bone.name.c_str()));
		if (bone.parent >= 0) {
			skeleton->set_bone_parent(i, bone.parent);
		}
	}

	// Build rest frame from bone DEFAULT values.
	// This is critical: vertices in MDL are stored relative to the bone defaults
	// (bone.value[]), NOT relative to any animation frame. The rest pose must
	// match what the vertices expect.
	goldsrc::ParsedAnimFrame rest_frame;
	rest_frame.bone_pos.resize(mdl.bones.size() * 3);
	rest_frame.bone_rot.resize(mdl.bones.size() * 3);
	for (int b = 0; b < (int)mdl.bones.size(); b++) {
		rest_frame.bone_pos[b * 3 + 0] = mdl.bones[b].pos[0];
		rest_frame.bone_pos[b * 3 + 1] = mdl.bones[b].pos[1];
		rest_frame.bone_pos[b * 3 + 2] = mdl.bones[b].pos[2];
		rest_frame.bone_rot[b * 3 + 0] = mdl.bones[b].rot[0];
		rest_frame.bone_rot[b * 3 + 1] = mdl.bones[b].rot[1];
		rest_frame.bone_rot[b * 3 + 2] = mdl.bones[b].rot[2];
	}

	// Set bone rest poses (local transforms, converted to Godot space)
	for (int i = 0; i < (int)mdl.bones.size(); i++) {
		Transform3D bone_rest = compute_bone_transform(i, rest_frame);
		skeleton->set_bone_rest(i, bone_rest);
	}

	// CRITICAL: In Godot 4, bone poses default to identity (NOT rest).
	// Without this call, skinning applies inverse(rest_global) to every vertex,
	// which destroys the mesh geometry.
	skeleton->reset_bone_poses();

	// Cache rest-pose world transforms for transforming vertices to model space
	rest_bone_world = compute_bone_world_transforms(rest_frame);
}

void GoldSrcMDL::build_meshes() {
	const auto &mdl = parser->get_data();

	// Create textures
	vector<Ref<ImageTexture>> godot_textures;
	vector<Ref<StandardMaterial3D>> materials;

	for (const auto &tex : mdl.textures) {
		PackedByteArray pixels;
		pixels.resize(tex.data.size());
		memcpy(pixels.ptrw(), tex.data.data(), tex.data.size());

		Ref<Image> img = Image::create_from_data(tex.width, tex.height,
			false, Image::FORMAT_RGBA8, pixels);
		Ref<ImageTexture> gtex = ImageTexture::create_from_image(img);
		godot_textures.push_back(gtex);

		Ref<StandardMaterial3D> mat;
		mat.instantiate();
		mat->set_texture(BaseMaterial3D::TEXTURE_ALBEDO, gtex);
		mat->set_texture_filter(BaseMaterial3D::TEXTURE_FILTER_NEAREST);
		mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_PER_PIXEL);

		// Chrome/additive flags
		if (tex.flags & 0x0002) { // STUDIO_NF_CHROME
			mat->set_metallic(0.8f);
			mat->set_specular(0.8f);
		}
		if (tex.flags & 0x0020) { // STUDIO_NF_ADDITIVE
			mat->set_blend_mode(BaseMaterial3D::BLEND_MODE_ADD);
		}

		materials.push_back(mat);
	}
	stored_materials = materials;

	// Build mesh for each bodypart (first model only for now)
	for (int bp = 0; bp < (int)mdl.bodyparts.size(); bp++) {
		const auto &bodypart = mdl.bodyparts[bp];
		if (bodypart.models.empty()) continue;

		const auto &submodel = bodypart.models[0]; // Default model

		for (int mi = 0; mi < (int)submodel.meshes.size(); mi++) {
			const auto &mesh = submodel.meshes[mi];
			if (mesh.triangle_verts.empty()) continue;

			PackedVector3Array vertices;
			PackedVector3Array normals_arr;
			PackedVector2Array uvs;
			PackedInt32Array bone_indices;
			PackedFloat32Array bone_weights;
			PackedInt32Array indices;

			for (int t = 0; t < (int)mesh.triangle_verts.size(); t++) {
				const auto &tv = mesh.triangle_verts[t];

				int vi = tv.vertex_index;
				int ni = tv.normal_index;

				if (vi * 3 + 2 >= (int)submodel.vertices.size()) continue;
				if (ni * 3 + 2 >= (int)submodel.normals.size()) continue;

				float vx = submodel.vertices[vi * 3 + 0];
				float vy = submodel.vertices[vi * 3 + 1];
				float vz = submodel.vertices[vi * 3 + 2];

				float nx = submodel.normals[ni * 3 + 0];
				float ny = submodel.normals[ni * 3 + 1];
				float nz = submodel.normals[ni * 3 + 2];

				int bone_idx = (vi < (int)submodel.vert_bone.size()) ? submodel.vert_bone[vi] : 0;
				int norm_bone_idx = (ni < (int)submodel.norm_bone.size()) ? submodel.norm_bone[ni] : 0;

				// Vertices are in bone-local GoldSrc space.
				// 1) Apply coordinate conversion: GoldSrc (x,y,z) → Godot (x,z,-y)
				// 2) Transform from bone-local to model space using rest-pose world transforms
				// This gives vertices in Godot model space, matching Godot's skinning expectations.
				Vector3 bone_local_pos(
					vx * scale_factor,
					vz * scale_factor,
					-vy * scale_factor
				);
				Vector3 bone_local_norm(nx, nz, -ny);

				Vector3 model_pos;
				Vector3 model_norm;

				if (bone_idx >= 0 && bone_idx < (int)rest_bone_world.size()) {
					model_pos = rest_bone_world[bone_idx].xform(bone_local_pos);
				} else {
					model_pos = bone_local_pos;
				}

				if (norm_bone_idx >= 0 && norm_bone_idx < (int)rest_bone_world.size()) {
					model_norm = rest_bone_world[norm_bone_idx].basis.xform(bone_local_norm).normalized();
				} else {
					model_norm = bone_local_norm;
				}

				vertices.push_back(model_pos);
				normals_arr.push_back(model_norm);
				uvs.push_back(Vector2(tv.s, tv.t));

				// Bone weights (single bone per vertex in GoldSrc)
				bone_indices.push_back(bone_idx);
				bone_indices.push_back(0);
				bone_indices.push_back(0);
				bone_indices.push_back(0);
				bone_weights.push_back(1.0f);
				bone_weights.push_back(0.0f);
				bone_weights.push_back(0.0f);
				bone_weights.push_back(0.0f);

				indices.push_back(t);
			}

			Ref<ArrayMesh> arr_mesh;
			arr_mesh.instantiate();

			Array arrays;
			arrays.resize(ArrayMesh::ARRAY_MAX);
			arrays[ArrayMesh::ARRAY_VERTEX] = vertices;
			arrays[ArrayMesh::ARRAY_NORMAL] = normals_arr;
			arrays[ArrayMesh::ARRAY_TEX_UV] = uvs;
			arrays[ArrayMesh::ARRAY_INDEX] = indices;

			if (skeleton) {
				arrays[ArrayMesh::ARRAY_BONES] = bone_indices;
				arrays[ArrayMesh::ARRAY_WEIGHTS] = bone_weights;
			}

			arr_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);

			// Apply material
			int skin_ref = mesh.skin_ref;
			int tex_idx = skin_ref;
			if (tex_idx >= 0 && tex_idx < (int)mdl.skin_table.size()) {
				tex_idx = mdl.skin_table[tex_idx];
			}
			if (tex_idx >= 0 && tex_idx < (int)materials.size()) {
				arr_mesh->surface_set_material(0, materials[tex_idx]);
			}

			MeshInstance3D *mesh_instance = memnew(MeshInstance3D);
			String mesh_name = String(bodypart.name.c_str()) + "_mesh" + String::num_int64(mi);
			mesh_instance->set_name(mesh_name);
			mesh_instance->set_mesh(arr_mesh);

			if (skeleton) {
				skeleton->add_child(mesh_instance);
				mesh_instance->set_skeleton_path(NodePath(".."));
			} else {
				add_child(mesh_instance);
			}

			mesh_skin_refs.push_back({mesh_instance, skin_ref});
		}
	}
}

void GoldSrcMDL::build_animations() {
	const auto &mdl = parser->get_data();
	if (mdl.sequences.empty() || !skeleton) return;

	AnimationPlayer *anim_player = memnew(AnimationPlayer);
	anim_player->set_name("AnimationPlayer");
	add_child(anim_player);

	Ref<AnimationLibrary> lib;
	lib.instantiate();

	for (int s = 0; s < (int)mdl.sequences.size(); s++) {
		const auto &seq = mdl.sequences[s];
		if (seq.frames.empty() || seq.fps <= 0) continue;

		Ref<Animation> anim;
		anim.instantiate();

		float duration = (float)seq.num_frames / seq.fps;
		if (duration <= 0) duration = 0.01f;
		anim->set_length(duration);

		String seq_name = String(seq.name.c_str());
		// STUDIO_LOOPING = 0x0001: respect MDL per-sequence loop flag
		if (seq.flags & 0x0001) {
			anim->set_loop_mode(Animation::LOOP_LINEAR);
		} else {
			anim->set_loop_mode(Animation::LOOP_NONE);
		}

		for (int b = 0; b < (int)mdl.bones.size(); b++) {
			String bone_name = String(mdl.bones[b].name.c_str());
			int bone_idx = skeleton->find_bone(bone_name);
			if (bone_idx < 0) continue;

			// Animation tracks use bone-LOCAL transforms in Godot space.
			// compute_bone_transform handles the GoldSrc→Godot conversion.
			// Animation values include the base bone value + delta (already
			// computed by the parser), so they directly replace the rest pose.
			NodePath skel_path = NodePath(String("Skeleton3D:") + bone_name);
			int pos_track = anim->add_track(Animation::TYPE_POSITION_3D);
			anim->track_set_path(pos_track, skel_path);

			int rot_track = anim->add_track(Animation::TYPE_ROTATION_3D);
			anim->track_set_path(rot_track, skel_path);

			for (int f = 0; f < (int)seq.frames.size(); f++) {
				float time = (float)f / seq.fps;
				Transform3D t = compute_bone_transform(b, seq.frames[f]);

				anim->position_track_insert_key(pos_track, time, t.origin);
				anim->rotation_track_insert_key(rot_track, time, t.basis.get_quaternion());
			}
		}

		// Sanitize name for AnimationLibrary (no special chars)
		String safe_name = seq_name.replace(" ", "_").replace("/", "_").to_lower();
		if (safe_name.is_empty()) {
			safe_name = "sequence_" + String::num_int64(s);
		}

		lib->add_animation(safe_name, anim);
	}

	anim_player->add_animation_library("", lib);
	UtilityFunctions::print("[GoldSrc] Built ", (int64_t)mdl.sequences.size(), " animations");
}
