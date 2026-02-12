#include "bsp_parser.h"
#include <cstring>
#include <cmath>

namespace goldsrc {

template <typename T>
static std::vector<T> read_lump(const uint8_t *data, size_t file_size, const BSPLump &lump) {
	std::vector<T> result;
	if (lump.fileofs < 0 || lump.filelen <= 0) return result;
	if ((size_t)lump.fileofs + lump.filelen > file_size) return result;
	size_t count = lump.filelen / sizeof(T);
	result.resize(count);
	std::memcpy(result.data(), data + lump.fileofs, count * sizeof(T));
	return result;
}

bool BSPParser::parse(const uint8_t *data, size_t size) {
	if (size < sizeof(BSPHeader)) return false;

	std::memcpy(&header, data, sizeof(BSPHeader));

	if (header.version != HLBSP_VERSION) {
		return false;
	}

	vertexes = read_lump<BSPVertex>(data, size, header.lumps[LUMP_VERTEXES]);
	planes = read_lump<BSPPlane>(data, size, header.lumps[LUMP_PLANES]);
	edges = read_lump<BSPEdge>(data, size, header.lumps[LUMP_EDGES]);
	surfedges = read_lump<int32_t>(data, size, header.lumps[LUMP_SURFEDGES]);
	raw_faces = read_lump<BSPFace>(data, size, header.lumps[LUMP_FACES]);
	texinfos = read_lump<BSPTexInfo>(data, size, header.lumps[LUMP_TEXINFO]);
	raw_models = read_lump<BSPModel>(data, size, header.lumps[LUMP_MODELS]);

	// Lighting data (raw bytes)
	{
		const BSPLump &lump = header.lumps[LUMP_LIGHTING];
		if (lump.filelen > 0 && (size_t)lump.fileofs + lump.filelen <= size) {
			bsp_data.lighting.resize(lump.filelen);
			std::memcpy(bsp_data.lighting.data(), data + lump.fileofs, lump.filelen);
		}
	}

	bsp_data.models = raw_models;

	parse_textures(data, size);
	parse_faces(data, size);
	parse_entities(data, size);

	return true;
}

void BSPParser::parse_textures(const uint8_t *data, size_t size) {
	const BSPLump &lump = header.lumps[LUMP_TEXTURES];
	if (lump.filelen <= 0) return;

	const uint8_t *tex_base = data + lump.fileofs;
	const BSPMipTexLump *miptexlump = reinterpret_cast<const BSPMipTexLump *>(tex_base);

	int32_t count = miptexlump->nummiptex;
	// dataofs array starts at offset 4 in the lump
	const int32_t *offsets = reinterpret_cast<const int32_t *>(tex_base + 4);

	bsp_data.textures.resize(count);

	for (int32_t i = 0; i < count; i++) {
		int32_t ofs = offsets[i];
		if (ofs < 0) continue;

		const BSPMipTex *miptex = reinterpret_cast<const BSPMipTex *>(tex_base + ofs);

		BSPTextureData &tex = bsp_data.textures[i];
		tex.name = std::string(miptex->name, strnlen(miptex->name, 16));
		tex.width = miptex->width;
		tex.height = miptex->height;
		tex.has_data = false;

		// If offsets[0] > 0, texture data is embedded in the BSP
		if (miptex->offsets[0] > 0 && miptex->width > 0 && miptex->height > 0) {
			uint32_t pixels = miptex->width * miptex->height;
			uint32_t datasize = pixels + (pixels / 4) + (pixels / 16) + (pixels / 64);

			const uint8_t *pixel_data = reinterpret_cast<const uint8_t *>(miptex) + miptex->offsets[0];
			const uint8_t *palette = reinterpret_cast<const uint8_t *>(miptex) + miptex->offsets[0] + datasize + 2;

			bool has_transparency = (miptex->name[0] == '{');

			tex.data.resize(pixels * 4);
			for (uint32_t j = 0; j < pixels; j++) {
				uint8_t idx = pixel_data[j];
				uint8_t r = palette[idx * 3 + 0];
				uint8_t g = palette[idx * 3 + 1];
				uint8_t b = palette[idx * 3 + 2];

				if (has_transparency && r == 0 && g == 0 && b == 255) {
					tex.data[j * 4 + 0] = 0;
					tex.data[j * 4 + 1] = 0;
					tex.data[j * 4 + 2] = 0;
					tex.data[j * 4 + 3] = 0;
				} else {
					tex.data[j * 4 + 0] = r;
					tex.data[j * 4 + 1] = g;
					tex.data[j * 4 + 2] = b;
					tex.data[j * 4 + 3] = 255;
				}
			}
			tex.has_data = true;
		}
	}
}

void BSPParser::parse_faces(const uint8_t *data, size_t size) {
	// Build a face-index-to-model-index lookup
	std::vector<int> face_to_model(raw_faces.size(), -1);
	for (int m = 0; m < (int)raw_models.size(); m++) {
		int first = raw_models[m].firstface;
		int count = raw_models[m].numfaces;
		for (int i = first; i < first + count && i < (int)raw_faces.size(); i++) {
			face_to_model[i] = m;
		}
	}

	for (size_t f = 0; f < raw_faces.size(); f++) {
		const BSPFace &face = raw_faces[f];
		const BSPTexInfo &ti = texinfos[face.texinfo];

		int tex_idx = ti.miptex;
		if (tex_idx < 0 || tex_idx >= (int)bsp_data.textures.size()) continue;

		const BSPTextureData &tex = bsp_data.textures[tex_idx];

		// Skip sky and trigger textures
		if (tex.name.compare(0, 3, "sky") == 0) continue;
		if (tex.name == "aaatrigger" || tex.name == "clip" || tex.name == "origin" || tex.name == "null") continue;

		// Get face normal from plane
		float nx = planes[face.planenum].normal[0];
		float ny = planes[face.planenum].normal[1];
		float nz = planes[face.planenum].normal[2];
		if (face.side) {
			nx = -nx; ny = -ny; nz = -nz;
		}

		ParsedFace pface;
		pface.model_index = face_to_model[f];
		pface.texture_name = tex.name;
		pface.texture_width = tex.width > 0 ? tex.width : 1;
		pface.texture_height = tex.height > 0 ? tex.height : 1;
		pface.lightmap_offset = face.lightofs;
		std::memcpy(pface.styles, face.styles, 4);

		// Compute lightmap extents
		float min_s = 1e30f, min_t = 1e30f;
		float max_s = -1e30f, max_t = -1e30f;

		// First pass: collect vertices and compute texture bounds
		std::vector<ParsedVertex> verts;
		for (int e = 0; e < face.numedges; e++) {
			int32_t surfedge = surfedges[face.firstedge + e];
			uint16_t vi;
			if (surfedge >= 0) {
				vi = edges[surfedge].v[0];
			} else {
				vi = edges[-surfedge].v[1];
			}

			const BSPVertex &vertex = vertexes[vi];

			ParsedVertex pv;
			pv.pos[0] = vertex.point[0];
			pv.pos[1] = vertex.point[1];
			pv.pos[2] = vertex.point[2];

			// Texture UVs
			float s = vertex.point[0] * ti.vecs[0][0] +
					  vertex.point[1] * ti.vecs[0][1] +
					  vertex.point[2] * ti.vecs[0][2] +
					  ti.vecs[0][3];
			float t = vertex.point[0] * ti.vecs[1][0] +
					  vertex.point[1] * ti.vecs[1][1] +
					  vertex.point[2] * ti.vecs[1][2] +
					  ti.vecs[1][3];

			if (s < min_s) min_s = s;
			if (s > max_s) max_s = s;
			if (t < min_t) min_t = t;
			if (t > max_t) max_t = t;

			pv.uv[0] = s / pface.texture_width;
			pv.uv[1] = t / pface.texture_height;

			pv.normal[0] = nx;
			pv.normal[1] = ny;
			pv.normal[2] = nz;

			pv.lightmap_uv[0] = s; // raw, will be adjusted below
			pv.lightmap_uv[1] = t;

			verts.push_back(pv);
		}

		// Compute lightmap size for this face
		int lm_mins_s = (int)std::floor(min_s / 16.0f);
		int lm_mins_t = (int)std::floor(min_t / 16.0f);
		int lm_maxs_s = (int)std::ceil(max_s / 16.0f);
		int lm_maxs_t = (int)std::ceil(max_t / 16.0f);
		pface.lightmap_width = lm_maxs_s - lm_mins_s + 1;
		pface.lightmap_height = lm_maxs_t - lm_mins_t + 1;

		// Adjust lightmap UVs to [0,1] range within the per-face lightmap
		for (auto &v : verts) {
			float s = v.lightmap_uv[0];
			float t = v.lightmap_uv[1];
			v.lightmap_uv[0] = (s / 16.0f - lm_mins_s + 0.5f) / pface.lightmap_width;
			v.lightmap_uv[1] = (t / 16.0f - lm_mins_t + 0.5f) / pface.lightmap_height;
		}

		pface.vertices = std::move(verts);
		bsp_data.faces.push_back(std::move(pface));
	}
}

void BSPParser::parse_entities(const uint8_t *data, size_t size) {
	const BSPLump &lump = header.lumps[LUMP_ENTITIES];
	if (lump.filelen <= 0) return;

	std::string ent_str(reinterpret_cast<const char *>(data + lump.fileofs), lump.filelen);

	ParsedEntity current;
	bool in_entity = false;
	size_t pos = 0;

	while (pos < ent_str.size()) {
		char c = ent_str[pos];

		if (c == '{') {
			current.properties.clear();
			in_entity = true;
			pos++;
		} else if (c == '}') {
			if (in_entity) {
				bsp_data.entities.push_back(current);
			}
			in_entity = false;
			pos++;
		} else if (c == '"' && in_entity) {
			// Parse key
			pos++;
			size_t key_start = pos;
			while (pos < ent_str.size() && ent_str[pos] != '"') pos++;
			std::string key = ent_str.substr(key_start, pos - key_start);
			pos++; // skip closing quote

			// Skip whitespace
			while (pos < ent_str.size() && (ent_str[pos] == ' ' || ent_str[pos] == '\t')) pos++;

			// Parse value
			if (pos < ent_str.size() && ent_str[pos] == '"') {
				pos++;
				size_t val_start = pos;
				while (pos < ent_str.size() && ent_str[pos] != '"') pos++;
				std::string value = ent_str.substr(val_start, pos - val_start);
				pos++;
				current.properties[key] = value;
			}
		} else {
			pos++;
		}
	}
}

} // namespace goldsrc
