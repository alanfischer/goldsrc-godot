#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace goldsrc {

inline constexpr int MDL_MAGIC = 0x54534449; // "IDST"
inline constexpr int MDL_VERSION = 10;

#pragma pack(push, 1)

struct MDLHeader {
	int32_t id;       // IDST
	int32_t version;  // 10

	char name[64];
	int32_t length;

	float eyeposition[3];
	float min[3];
	float max[3];
	float bbmin[3];
	float bbmax[3];
	int32_t flags;

	int32_t numbones;
	int32_t boneindex;

	int32_t numbonecontrollers;
	int32_t bonecontrollerindex;

	int32_t numhitboxes;
	int32_t hitboxindex;

	int32_t numseq;
	int32_t seqindex;

	int32_t numseqgroups;
	int32_t seqgroupindex;

	int32_t numtextures;
	int32_t textureindex;
	int32_t texturedataindex;

	int32_t numskinref;
	int32_t numskinfamilies;
	int32_t skinindex;

	int32_t numbodyparts;
	int32_t bodypartindex;

	int32_t numattachments;
	int32_t attachmentindex;

	int32_t soundtable;
	int32_t soundindex;
	int32_t soundgroups;
	int32_t soundgroupindex;

	int32_t numtransitions;
	int32_t transitionindex;
};

struct MDLBone {
	char name[32];
	int32_t parent;
	int32_t flags;
	int32_t bonecontroller[6];
	float value[6];     // pos xyz, rot xyz (euler)
	float scale[6];
};

struct MDLHitbox {
	int32_t bone;
	int32_t group;      // body part group — 1 = head in HL player models
	float bbmin[3];
	float bbmax[3];
};

struct MDLBoneController {
	int32_t bone;
	int32_t type;
	float start;
	float end;
	int32_t rest;
	int32_t index;
};

struct MDLSequenceDesc {
	char label[32];
	float fps;
	int32_t flags;
	int32_t activity;
	int32_t actweight;
	int32_t numevents;
	int32_t eventindex;
	int32_t numframes;
	int32_t numpivots;
	int32_t pivotindex;
	int32_t motiontype;
	int32_t motionbone;
	float linearmovement[3];
	int32_t automoveposindex;
	int32_t automoveangleindex;
	float bbmin[3];
	float bbmax[3];
	int32_t numblends;
	int32_t animindex;
	int32_t blendtype[2];
	float blendstart[2];
	float blendend[2];
	int32_t blendparent;
	int32_t seqgroup;
	int32_t entrynode;
	int32_t exitnode;
	int32_t nodeflags;
	int32_t nextseq;
};

struct MDLTexture {
	char name[64];
	int32_t flags;
	int32_t width;
	int32_t height;
	int32_t index; // offset to pixel data
};

struct MDLBodyPart {
	char name[64];
	int32_t nummodels;
	int32_t base;
	int32_t modelindex;
};

struct MDLModel {
	char name[64];
	int32_t type;
	float boundingradius;
	int32_t nummesh;
	int32_t meshindex;
	int32_t numverts;
	int32_t vertinfoindex;  // bone index per vertex
	int32_t vertindex;      // vertex positions
	int32_t numnorms;
	int32_t norminfoindex;  // bone index per normal
	int32_t normindex;      // normals
	int32_t numgroups;
	int32_t groupindex;
};

struct MDLMesh {
	int32_t numtris;
	int32_t triindex;
	int32_t skinref;
	int32_t numnorms;
	int32_t normindex;
};

struct MDLSeqGroup {
	char label[32];
	char name[64];
	int32_t unused1;
	int32_t unused2;
};

// Animation value union
union MDLAnimValue {
	struct {
		uint8_t valid;
		uint8_t total;
	} num;
	int16_t value;
};

struct MDLAnimation {
	uint16_t offset[6]; // offsets to MDLAnimValue arrays for each channel
};

#pragma pack(pop)

// High-level parsed structures

struct ParsedBone {
	std::string name;
	int parent;
	float pos[3];
	float rot[3]; // euler angles
};

struct ParsedHitbox {
	int bone;
	int group;      // 1 = head in HL player models
	float bbmin[3]; // in bone-local GoldSrc space, unscaled
	float bbmax[3];
};

struct ParsedMDLTexture {
	std::string name;
	int width, height;
	int flags;
	std::vector<uint8_t> data; // RGBA
};

struct ParsedTriVertex {
	int vertex_index;
	int normal_index;
	float s, t; // texture coords
};

struct ParsedMDLMesh {
	int skin_ref;
	std::vector<ParsedTriVertex> triangle_verts; // triplets of 3 = triangles
};

struct ParsedBodyPart {
	std::string name;
	struct SubModel {
		std::string name;
		std::vector<float> vertices;  // xyz interleaved
		std::vector<float> normals;   // xyz interleaved
		std::vector<uint8_t> vert_bone; // bone index per vertex
		std::vector<uint8_t> norm_bone; // bone index per normal
		std::vector<ParsedMDLMesh> meshes;
	};
	std::vector<SubModel> models;
};

struct ParsedAnimFrame {
	std::vector<float> bone_pos; // numbones * 3
	std::vector<float> bone_rot; // numbones * 3 (euler)
};

struct ParsedSequence {
	std::string name;
	float fps;
	int num_frames;
	int flags;
	float linear_movement[3];
	std::vector<ParsedAnimFrame> frames;
};

struct MDLData {
	std::vector<ParsedBone> bones;
	std::vector<ParsedHitbox> hitboxes;
	std::vector<ParsedMDLTexture> textures;
	std::vector<int16_t> skin_table; // numskinref * numskinfamilies
	int num_skin_ref;
	int num_skin_families;
	std::vector<ParsedBodyPart> bodyparts;
	std::vector<ParsedSequence> sequences;
};

class MDLParser {
public:
	bool parse(const uint8_t *data, size_t size);
	const MDLData &get_data() const { return mdl_data; }

private:
	void parse_bones(const uint8_t *data, size_t size);
	void parse_hitboxes(const uint8_t *data, size_t size);
	void parse_textures(const uint8_t *data, size_t size);
	void parse_skins(const uint8_t *data, size_t size);
	void parse_bodyparts(const uint8_t *data, size_t size);
	void parse_sequences(const uint8_t *data, size_t size);

	void decode_animation(const uint8_t *data, size_t size,
		const MDLSequenceDesc *seq, int blend,
		std::vector<ParsedAnimFrame> &frames);

	const MDLHeader *header = nullptr;
	MDLData mdl_data;
};

} // namespace goldsrc
