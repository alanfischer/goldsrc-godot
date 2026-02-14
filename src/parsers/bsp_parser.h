#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <map>

namespace goldsrc {

inline constexpr int HLBSP_VERSION = 30;

enum BSPLumpType {
	LUMP_ENTITIES = 0,
	LUMP_PLANES,
	LUMP_TEXTURES,
	LUMP_VERTEXES,
	LUMP_VISIBILITY,
	LUMP_NODES,
	LUMP_TEXINFO,
	LUMP_FACES,
	LUMP_LIGHTING,
	LUMP_CLIPNODES,
	LUMP_LEAFS,
	LUMP_MARKSURFACES,
	LUMP_EDGES,
	LUMP_SURFEDGES,
	LUMP_MODELS,
	MAX_BSP_LUMPS
};

#pragma pack(push, 1)

struct BSPLump {
	int32_t fileofs;
	int32_t filelen;
};

struct BSPHeader {
	int32_t version;
	BSPLump lumps[MAX_BSP_LUMPS];
};

struct BSPVertex {
	float point[3];
};

struct BSPPlane {
	float normal[3];
	float dist;
	int32_t type;
};

struct BSPEdge {
	uint16_t v[2];
};

struct BSPFace {
	int16_t planenum;
	int16_t side;
	int32_t firstedge;
	int16_t numedges;
	int16_t texinfo;
	uint8_t styles[4];
	int32_t lightofs;
};

struct BSPTexInfo {
	float vecs[2][4]; // [s/t][xyz offset]
	int32_t miptex;
	int32_t flags;
};

struct BSPMipTexLump {
	int32_t nummiptex;
	int32_t dataofs[1]; // variable size
};

struct BSPMipTex {
	char name[16];
	uint32_t width, height;
	uint32_t offsets[4];
};

struct BSPModel {
	float mins[3], maxs[3];
	float origin[3];
	int32_t headnode[4];
	int32_t visleafs;
	int32_t firstface, numfaces;
};

struct BSPNode {
	int32_t planenum;
	int16_t children[2];
	int16_t mins[3], maxs[3];
	uint16_t firstface;
	uint16_t numfaces;
};

struct BSPClipNode {
	int32_t planenum;
	int16_t children[2];
};

struct BSPLeaf {
	int32_t contents;
	int32_t visofs;
	int16_t mins[3], maxs[3];
	uint16_t firstmarksurface;
	uint16_t nummarksurfaces;
	uint8_t ambient_level[4];
};

#pragma pack(pop)

// Parsed face data ready for mesh building
struct ParsedVertex {
	float pos[3];
	float uv[2];      // texture UV
	float normal[3];
	float lightmap_uv[2]; // lightmap UV
};

struct ParsedFace {
	std::string texture_name;
	int texture_width;
	int texture_height;
	std::vector<ParsedVertex> vertices; // fan-order polygon
	int lightmap_width;
	int lightmap_height;
	int lightmap_offset; // into lighting lump, -1 if none
	uint8_t styles[4] = {255, 255, 255, 255}; // lightstyle indices
	int model_index; // which BSP model this face belongs to (0=worldspawn)
};

struct ParsedEntity {
	std::map<std::string, std::string> properties;
};

struct BSPTextureData {
	std::string name;
	uint32_t width, height;
	std::vector<uint8_t> data; // RGBA, empty if external (WAD) texture
	bool has_data;
};

struct BSPData {
	std::vector<ParsedFace> faces;
	std::vector<ParsedEntity> entities;
	std::vector<BSPTextureData> textures;
	std::vector<uint8_t> lighting; // raw RGB lightmap data
	std::vector<BSPModel> models;
};

class BSPParser {
public:
	bool parse(const uint8_t *data, size_t size);
	const BSPData &get_data() const { return bsp_data; }

private:
	void parse_faces(const uint8_t *data, size_t size);
	void parse_textures(const uint8_t *data, size_t size);
	void parse_entities(const uint8_t *data, size_t size);

	BSPHeader header;
	std::vector<BSPVertex> vertexes;
	std::vector<BSPPlane> planes;
	std::vector<BSPEdge> edges;
	std::vector<int32_t> surfedges;
	std::vector<BSPFace> raw_faces;
	std::vector<BSPTexInfo> texinfos;
	std::vector<BSPModel> raw_models;

	BSPData bsp_data;
};

} // namespace goldsrc
