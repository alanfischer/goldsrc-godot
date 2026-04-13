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

inline constexpr int CONTENTS_EMPTY = -1;
inline constexpr int CONTENTS_SOLID = -2;
inline constexpr int CONTENTS_WATER = -3;
inline constexpr int CONTENTS_SKY   = -6;

// Tool textures that have no visible geometry at runtime
inline bool is_tool_texture(const std::string &name) {
	if (name.empty()) return false;
	std::string lower = name;
	for (auto &c : lower) c = (char)tolower(c);
	if (lower.compare(0, 3, "aaa") == 0) return true;
	return lower == "clip" || lower == "null" || lower == "origin" ||
	       lower == "bevel" || lower == "hint" || lower == "skip";
}

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
	float normal[3] = {0, 0, 0}; // face normal (GoldSrc coords)
	float s_axis[3] = {1, 0, 0}; // texture S axis (GoldSrc coords)
	float t_axis[3] = {0, 1, 0}; // texture T axis (GoldSrc coords)
	float s_offset = 0; // texinfo vecs[0][3] — S axis offset
	float t_offset = 0; // texinfo vecs[1][3] — T axis offset
	int lm_mins_s = 0;  // floor(min_s / 16) — lightmap origin in texel space
	int lm_mins_t = 0;  // floor(min_t / 16)
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
	std::vector<BSPClipNode> clipnodes;
	std::vector<BSPPlane> planes;
	std::vector<BSPNode> nodes;
	std::vector<BSPLeaf> leafs;
	std::vector<int> raw_to_parsed; // raw_to_parsed[raw_index] = parsed_index, or -1 if filtered
	std::vector<uint8_t> visibility; // raw RLE-compressed PVS data
	std::vector<uint16_t> marksurfaces; // leaf -> face index mapping

	// Decompress PVS bitfield for a leaf. Returns a vector of bools, one per leaf.
	// pvs[i] == true means leaf i is potentially visible from the given leaf.
	std::vector<bool> decompress_pvs(int leaf_index) const;
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
	std::vector<BSPClipNode> clipnodes;
	std::vector<BSPNode> nodes;
	std::vector<BSPLeaf> leafs;

	BSPData bsp_data;
};

} // namespace goldsrc
