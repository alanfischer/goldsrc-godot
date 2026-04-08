#include "bsp_parser.h"
#include "texture_decode.h"
#include <cstring>
#include <cmath>
#include <set>

using namespace std;

namespace goldsrc {

template <typename T>
static vector<T> read_lump(const uint8_t *data, size_t file_size, const BSPLump &lump) {
	vector<T> result;
	if (lump.fileofs < 0 || lump.filelen <= 0) return result;
	if ((size_t)lump.fileofs + lump.filelen > file_size) return result;
	size_t count = lump.filelen / sizeof(T);
	result.resize(count);
	memcpy(result.data(), data + lump.fileofs, count * sizeof(T));
	return result;
}

bool BSPParser::parse(const uint8_t *data, size_t size) {
	if (size < sizeof(BSPHeader)) return false;

	memcpy(&header, data, sizeof(BSPHeader));

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
	clipnodes = read_lump<BSPClipNode>(data, size, header.lumps[LUMP_CLIPNODES]);
	nodes = read_lump<BSPNode>(data, size, header.lumps[LUMP_NODES]);
	leafs = read_lump<BSPLeaf>(data, size, header.lumps[LUMP_LEAFS]);

	// Lighting data (raw bytes)
	{
		const BSPLump &lump = header.lumps[LUMP_LIGHTING];
		if (lump.filelen > 0 && (size_t)lump.fileofs + lump.filelen <= size) {
			bsp_data.lighting.resize(lump.filelen);
			memcpy(bsp_data.lighting.data(), data + lump.fileofs, lump.filelen);
		}
	}

	bsp_data.models = raw_models;
	bsp_data.clipnodes = clipnodes;
	bsp_data.planes = planes;
	bsp_data.nodes = nodes;
	bsp_data.leafs = leafs;

	// Visibility data (raw RLE bytes)
	{
		const BSPLump &lump = header.lumps[LUMP_VISIBILITY];
		if (lump.filelen > 0 && (size_t)lump.fileofs + lump.filelen <= size) {
			bsp_data.visibility.resize(lump.filelen);
			memcpy(bsp_data.visibility.data(), data + lump.fileofs, lump.filelen);
		}
	}

	// Marksurfaces (leaf -> face index mapping)
	bsp_data.marksurfaces = read_lump<uint16_t>(data, size, header.lumps[LUMP_MARKSURFACES]);

	parse_textures(data, size);
	parse_faces(data, size);
	parse_entities(data, size);

	return true;
}

void BSPParser::parse_textures(const uint8_t *data, size_t size) {
	const BSPLump &lump = header.lumps[LUMP_TEXTURES];
	if (lump.filelen <= 0) return;
	if ((size_t)lump.fileofs + lump.filelen > size) return;

	const size_t lump_end = (size_t)lump.fileofs + lump.filelen;
	const uint8_t *tex_base = data + lump.fileofs;
	const size_t lump_size = lump.filelen;

	// Need at least 4 bytes for nummiptex
	if (lump_size < 4) return;

	int32_t count;
	memcpy(&count, tex_base, sizeof(int32_t));
	if (count <= 0) return;

	// Validate offset array fits: 4 + count * 4
	size_t offsets_end = 4 + (size_t)count * sizeof(int32_t);
	if (offsets_end > lump_size) return;

	const int32_t *offsets = reinterpret_cast<const int32_t *>(tex_base + 4);

	bsp_data.textures.resize(count);

	for (int32_t i = 0; i < count; i++) {
		int32_t ofs = offsets[i];
		if (ofs < 0) continue;

		// Validate miptex header fits within lump
		if ((size_t)ofs + sizeof(BSPMipTex) > lump_size) continue;

		const BSPMipTex *miptex = reinterpret_cast<const BSPMipTex *>(tex_base + ofs);

		BSPTextureData &tex = bsp_data.textures[i];
		tex.name = string(miptex->name, strnlen(miptex->name, 16));
		tex.width = miptex->width;
		tex.height = miptex->height;
		tex.has_data = false;

		// If offsets[0] > 0, texture data is embedded in the BSP
		if (miptex->offsets[0] > 0 && miptex->width > 0 && miptex->height > 0) {
			// Guard against overflow: cap dimensions to reasonable BSP limits
			if (miptex->width > 4096 || miptex->height > 4096) continue;

			uint32_t pixels = miptex->width * miptex->height;
			uint32_t datasize = pixels + (pixels / 4) + (pixels / 16) + (pixels / 64);

			// Validate pixel data + palette (2-byte count + 256*3 palette) fits within lump
			size_t pixel_start = (size_t)ofs + miptex->offsets[0];
			size_t palette_end = pixel_start + datasize + 2 + 256 * 3;
			if (palette_end > lump_size) continue;

			const uint8_t *pixel_data = tex_base + pixel_start;
			const uint8_t *palette = tex_base + pixel_start + datasize + 2;

			bool has_transparency = (miptex->name[0] == '{');
			decode_palette_pixels(pixel_data, palette, pixels, has_transparency, tex.data);
			tex.has_data = true;
		}
	}
}

void BSPParser::parse_faces(const uint8_t *data, size_t size) {
	// Build a face-index-to-model-index lookup
	vector<int> face_to_model(raw_faces.size(), -1);
	for (int m = 0; m < (int)raw_models.size(); m++) {
		int first = raw_models[m].firstface;
		int count = raw_models[m].numfaces;
		for (int i = first; i < first + count && i < (int)raw_faces.size(); i++) {
			face_to_model[i] = m;
		}
	}

	bsp_data.raw_to_parsed.assign(raw_faces.size(), -1);

	// Pre-identify which models are water brush entities by scanning face
	// textures. Used to detect internal faces between adjacent water volumes
	// (without culling exposed faces like waterfalls).
	set<int> water_model_set;
	for (size_t i = 0; i < raw_faces.size(); i++) {
		int mi = face_to_model[i];
		if (mi <= 0) continue;
		if (raw_faces[i].texinfo < 0 || (size_t)raw_faces[i].texinfo >= texinfos.size()) continue;
		int ti_idx = texinfos[raw_faces[i].texinfo].miptex;
		if (ti_idx < 0 || ti_idx >= (int)bsp_data.textures.size()) continue;
		const auto &tn = bsp_data.textures[ti_idx].name;
		if (tn.find("water") != string::npos ||
		    (!tn.empty() && (tn[0] == '!' || tn[0] == '*')))
			water_model_set.insert(mi);
	}

	int water_faces_culled = 0;
	for (size_t f = 0; f < raw_faces.size(); f++) {
		const BSPFace &face = raw_faces[f];

		// Bounds check texinfo index
		if (face.texinfo < 0 || (size_t)face.texinfo >= texinfos.size()) continue;
		const BSPTexInfo &ti = texinfos[face.texinfo];

		int tex_idx = ti.miptex;
		if (tex_idx < 0 || tex_idx >= (int)bsp_data.textures.size()) continue;

		const BSPTextureData &tex = bsp_data.textures[tex_idx];

		// Skip tool textures on worldspawn (they have no visible geometry or
		// face-based collision). Keep them on brush entities (e.g. func_ladder).
		if (is_tool_texture(tex.name) && face_to_model[f] == 0) continue;

		// Cull internal water faces. In GoldSrc, water brush entities only show
		// their top surface — side and bottom faces are hidden behind pool
		// geometry. For worldspawn water, check both sides against the BSP tree.
		if (face.planenum >= 0 && (size_t)face.planenum < planes.size()) {
			const auto &pl = planes[face.planenum];
			float nx = pl.normal[0], ny = pl.normal[1], nz = pl.normal[2];
			if (face.side) { nx = -nx; ny = -ny; nz = -nz; }

			// For water brush entity faces: cull if the outside of this face
			// is inside another water model's BSP volume (internal seam between
			// adjacent water volumes). Keep exposed faces (e.g. waterfalls).
			if (water_model_set.count(face_to_model[f])) {
				// Compute face center
				float wcx = 0, wcy = 0, wcz = 0;
				int wnv = 0;
				for (int e = 0; e < face.numedges; e++) {
					size_t se_idx = (size_t)face.firstedge + e;
					if (se_idx >= surfedges.size()) break;
					int32_t se = surfedges[se_idx];
					size_t ei = (size_t)(se >= 0 ? se : -se);
					if (ei >= edges.size()) break;
					uint16_t vi = se >= 0 ? edges[ei].v[0] : edges[ei].v[1];
					if ((size_t)vi >= vertexes.size()) break;
					wcx += vertexes[vi].point[0];
					wcy += vertexes[vi].point[1];
					wcz += vertexes[vi].point[2];
					wnv++;
				}
				if (wnv > 0) {
					wcx /= wnv; wcy /= wnv; wcz /= wnv;
					// Probe both sides of the face. For each direction, walk
					// hull 0 (exact BSP geometry) of every OTHER water model.
					// If the probe lands inside another water entity, this
					// face is an internal seam between adjacent water entities.
					// Note: hull 0 uses the exact brush geometry (nodes/leafs),
					// NOT hull 1 (clipnodes) which is expanded by player bbox
					// and would false-positive on nearby but non-overlapping
					// brushes (e.g. waterfall next to a pool).
					bool inside_other = false;
					for (int ps = 0; ps < 2 && !inside_other; ps++) {
						float sign = ps == 0 ? 1.0f : -1.0f;
						float probe[3] = {
							wcx + nx * sign * 2.0f,
							wcy + ny * sign * 2.0f,
							wcz + nz * sign * 2.0f
						};
						for (int wmi : water_model_set) {
							if (wmi == face_to_model[f]) continue;
							// Walk hull 0 (nodes/leafs) of the other water model
							int ni = raw_models[wmi].headnode[0];
							while (ni >= 0 && (size_t)ni < nodes.size()) {
								const auto &nd = nodes[ni];
								if (nd.planenum < 0 || (size_t)nd.planenum >= planes.size()) break;
								const auto &np = planes[nd.planenum];
								float d = np.normal[0] * probe[0] + np.normal[1] * probe[1]
								        + np.normal[2] * probe[2] - np.dist;
								ni = d >= 0 ? nd.children[0] : nd.children[1];
							}
							// Negative ni = leaf index: -(ni+1)
							int li = -(ni + 1);
							if (li >= 0 && (size_t)li < leafs.size() &&
							    leafs[li].contents == CONTENTS_SOLID) {
								inside_other = true;
								break;
							}
						}
					}
					if (inside_other) {
						water_faces_culled++;
						continue; // internal seam — skip
					}
				}
			}

			// For worldspawn faces, check BSP leaf contents on both sides
			if (face_to_model[f] == 0) {
				float cx = 0, cy = 0, cz = 0;
				int nv_center = 0;
				for (int e = 0; e < face.numedges; e++) {
					size_t se_idx = (size_t)face.firstedge + e;
					if (se_idx >= surfedges.size()) break;
					int32_t se = surfedges[se_idx];
					size_t ei = (size_t)(se >= 0 ? se : -se);
					if (ei >= edges.size()) break;
					uint16_t vi = se >= 0 ? edges[ei].v[0] : edges[ei].v[1];
					if ((size_t)vi >= vertexes.size()) break;
					cx += vertexes[vi].point[0];
					cy += vertexes[vi].point[1];
					cz += vertexes[vi].point[2];
					nv_center++;
				}
				if (nv_center > 0) {
					cx /= nv_center; cy /= nv_center; cz /= nv_center;
					int root_node = bsp_data.models[0].headnode[0];
					bool both_water = true;
					for (int s = 0; s < 2; s++) {
						float sign = s == 0 ? 1.0f : -1.0f;
						float probe[3] = {
							cx + nx * sign * 2.0f,
							cy + ny * sign * 2.0f,
							cz + nz * sign * 2.0f
						};
						int ni = root_node;
						while (ni >= 0 && (size_t)ni < nodes.size()) {
							const auto &nd = nodes[ni];
							if (nd.planenum < 0 || (size_t)nd.planenum >= planes.size()) break;
							const auto &np = planes[nd.planenum];
							float d = np.normal[0] * probe[0] + np.normal[1] * probe[1]
							        + np.normal[2] * probe[2] - np.dist;
							ni = d >= 0 ? nd.children[0] : nd.children[1];
						}
						int li = -(ni + 1);
						if (li < 0 || (size_t)li >= leafs.size() ||
						    leafs[li].contents != CONTENTS_WATER) {
							both_water = false;
							break;
						}
					}
					if (both_water) { water_faces_culled++; continue; }
				}
			}
		}

		// Bounds check plane index
		if (face.planenum < 0 || (size_t)face.planenum >= planes.size()) continue;

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
		memcpy(pface.styles, face.styles, 4);
		pface.normal[0] = nx; pface.normal[1] = ny; pface.normal[2] = nz;
		pface.s_axis[0] = ti.vecs[0][0]; pface.s_axis[1] = ti.vecs[0][1]; pface.s_axis[2] = ti.vecs[0][2];
		pface.t_axis[0] = ti.vecs[1][0]; pface.t_axis[1] = ti.vecs[1][1]; pface.t_axis[2] = ti.vecs[1][2];
		pface.s_offset = ti.vecs[0][3];
		pface.t_offset = ti.vecs[1][3];

		// Compute lightmap extents
		float min_s = 1e30f, min_t = 1e30f;
		float max_s = -1e30f, max_t = -1e30f;

		// First pass: collect vertices and compute texture bounds
		vector<ParsedVertex> verts;
		bool face_valid = true;
		for (int e = 0; e < face.numedges; e++) {
			// Bounds check surfedge index
			size_t se_idx = (size_t)face.firstedge + e;
			if (se_idx >= surfedges.size()) { face_valid = false; break; }

			int32_t surfedge = surfedges[se_idx];

			// Bounds check edge index
			size_t edge_idx = (size_t)(surfedge >= 0 ? surfedge : -surfedge);
			if (edge_idx >= edges.size()) { face_valid = false; break; }

			uint16_t vi;
			if (surfedge >= 0) {
				vi = edges[edge_idx].v[0];
			} else {
				vi = edges[edge_idx].v[1];
			}

			// Bounds check vertex index
			if ((size_t)vi >= vertexes.size()) { face_valid = false; break; }

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

		if (!face_valid) continue;

		// Compute lightmap size for this face
		int lm_mins_s = (int)floor(min_s / 16.0f);
		int lm_mins_t = (int)floor(min_t / 16.0f);
		int lm_maxs_s = (int)ceil(max_s / 16.0f);
		int lm_maxs_t = (int)ceil(max_t / 16.0f);
		pface.lightmap_width = lm_maxs_s - lm_mins_s + 1;
		pface.lightmap_height = lm_maxs_t - lm_mins_t + 1;
		pface.lm_mins_s = lm_mins_s;
		pface.lm_mins_t = lm_mins_t;

		// Adjust lightmap UVs to [0,1] range within the per-face lightmap
		for (auto &v : verts) {
			float s = v.lightmap_uv[0];
			float t = v.lightmap_uv[1];
			v.lightmap_uv[0] = (s / 16.0f - lm_mins_s + 0.5f) / pface.lightmap_width;
			v.lightmap_uv[1] = (t / 16.0f - lm_mins_t + 0.5f) / pface.lightmap_height;
		}

		pface.vertices = std::move(verts);
		bsp_data.raw_to_parsed[f] = (int)bsp_data.faces.size();
		bsp_data.faces.push_back(std::move(pface));
	}

	if (water_faces_culled > 0) {
		fprintf(stderr, "[BSP] Culled %d internal water faces\n", water_faces_culled);
	}
}

void BSPParser::parse_entities(const uint8_t *data, size_t size) {
	const BSPLump &lump = header.lumps[LUMP_ENTITIES];
	if (lump.filelen <= 0) return;
	if ((size_t)lump.fileofs + lump.filelen > size) return;

	string ent_str(reinterpret_cast<const char *>(data + lump.fileofs), lump.filelen);

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
			string key = ent_str.substr(key_start, pos - key_start);
			if (pos >= ent_str.size()) break;
			pos++; // skip closing quote

			// Skip whitespace
			while (pos < ent_str.size() && (ent_str[pos] == ' ' || ent_str[pos] == '\t')) pos++;

			// Parse value
			if (pos < ent_str.size() && ent_str[pos] == '"') {
				pos++;
				size_t val_start = pos;
				while (pos < ent_str.size() && ent_str[pos] != '"') pos++;
				string value = ent_str.substr(val_start, pos - val_start);
				if (pos < ent_str.size()) pos++; // skip closing quote
				current.properties[key] = value;
			}
		} else {
			pos++;
		}
	}
}

vector<bool> BSPData::decompress_pvs(int leaf_index) const {
	int num_leafs = (int)leafs.size();
	vector<bool> pvs(num_leafs, false);

	if (leaf_index < 0 || leaf_index >= num_leafs) return pvs;

	const auto &leaf = leafs[leaf_index];

	// Leaf 0 is the solid "outside" leaf — has no PVS
	if (leaf_index == 0) return pvs;

	// visofs == -1 means all leaves are visible from this leaf
	if (leaf.visofs < 0) {
		pvs.assign(num_leafs, true);
		return pvs;
	}

	if (visibility.empty() || (size_t)leaf.visofs >= visibility.size()) return pvs;

	// RLE decompression: each byte is either a literal (bits = visible leaves)
	// or 0x00 followed by a count of zero-bytes to skip.
	// The bitfield covers leafs 1..num_leafs-1 (leaf 0 is the void leaf).
	int num_vis_leafs = num_leafs - 1; // exclude leaf 0
	int num_bytes = (num_vis_leafs + 7) / 8;

	const uint8_t *src = visibility.data() + leaf.visofs;
	const uint8_t *src_end = visibility.data() + visibility.size();
	int byte_idx = 0;

	while (byte_idx < num_bytes && src < src_end) {
		if (*src == 0) {
			// Zero byte: next byte is count of zero-bytes to skip
			src++;
			if (src >= src_end) break;
			byte_idx += *src;
			src++;
		} else {
			// Literal byte: each bit is one leaf (bit 0 = lowest leaf index)
			uint8_t bits = *src;
			for (int bit = 0; bit < 8 && (byte_idx * 8 + bit) < num_vis_leafs; bit++) {
				if (bits & (1 << bit)) {
					// +1 because PVS bits start at leaf 1
					pvs[byte_idx * 8 + bit + 1] = true;
				}
			}
			byte_idx++;
			src++;
		}
	}

	// A leaf can always see itself
	pvs[leaf_index] = true;

	return pvs;
}

} // namespace goldsrc
