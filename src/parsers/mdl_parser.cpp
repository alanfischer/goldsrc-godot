#include "mdl_parser.h"
#include <cstring>
#include <cmath>

namespace goldsrc {

bool MDLParser::parse(const uint8_t *data, size_t size) {
	if (size < sizeof(MDLHeader)) return false;

	header = reinterpret_cast<const MDLHeader *>(data);

	if (header->id != MDL_MAGIC || header->version != MDL_VERSION) {
		return false;
	}

	parse_bones(data);
	parse_textures(data);
	parse_skins(data);
	parse_bodyparts(data, size);
	parse_sequences(data, size);

	return true;
}

void MDLParser::parse_bones(const uint8_t *data) {
	const MDLBone *bones = reinterpret_cast<const MDLBone *>(data + header->boneindex);

	mdl_data.bones.resize(header->numbones);
	for (int i = 0; i < header->numbones; i++) {
		ParsedBone &pb = mdl_data.bones[i];
		pb.name = std::string(bones[i].name, strnlen(bones[i].name, 32));
		pb.parent = bones[i].parent;
		pb.pos[0] = bones[i].value[0];
		pb.pos[1] = bones[i].value[1];
		pb.pos[2] = bones[i].value[2];
		pb.rot[0] = bones[i].value[3];
		pb.rot[1] = bones[i].value[4];
		pb.rot[2] = bones[i].value[5];
	}
}

void MDLParser::parse_textures(const uint8_t *data) {
	if (header->numtextures <= 0) return;

	const MDLTexture *textures = reinterpret_cast<const MDLTexture *>(data + header->textureindex);

	mdl_data.textures.resize(header->numtextures);
	for (int i = 0; i < header->numtextures; i++) {
		ParsedMDLTexture &pt = mdl_data.textures[i];
		pt.name = std::string(textures[i].name, strnlen(textures[i].name, 64));
		pt.width = textures[i].width;
		pt.height = textures[i].height;
		pt.flags = textures[i].flags;

		// Pixel data is indexed color followed by 256-entry RGB palette
		const uint8_t *pixel_data = data + textures[i].index;
		int pixel_count = pt.width * pt.height;
		const uint8_t *palette = pixel_data + pixel_count;

		pt.data.resize(pixel_count * 4);
		for (int j = 0; j < pixel_count; j++) {
			uint8_t idx = pixel_data[j];
			pt.data[j * 4 + 0] = palette[idx * 3 + 0];
			pt.data[j * 4 + 1] = palette[idx * 3 + 1];
			pt.data[j * 4 + 2] = palette[idx * 3 + 2];
			pt.data[j * 4 + 3] = 255;
		}
	}
}

void MDLParser::parse_skins(const uint8_t *data) {
	mdl_data.num_skin_ref = header->numskinref;
	mdl_data.num_skin_families = header->numskinfamilies;

	int total = header->numskinref * header->numskinfamilies;
	if (total <= 0) return;

	const int16_t *skin_data = reinterpret_cast<const int16_t *>(data + header->skinindex);
	mdl_data.skin_table.assign(skin_data, skin_data + total);
}

void MDLParser::parse_bodyparts(const uint8_t *data, size_t size) {
	const MDLBodyPart *bodyparts = reinterpret_cast<const MDLBodyPart *>(data + header->bodypartindex);

	mdl_data.bodyparts.resize(header->numbodyparts);
	for (int bp = 0; bp < header->numbodyparts; bp++) {
		ParsedBodyPart &pbp = mdl_data.bodyparts[bp];
		pbp.name = std::string(bodyparts[bp].name, strnlen(bodyparts[bp].name, 64));

		const MDLModel *models = reinterpret_cast<const MDLModel *>(data + bodyparts[bp].modelindex);
		pbp.models.resize(bodyparts[bp].nummodels);

		for (int m = 0; m < bodyparts[bp].nummodels; m++) {
			const MDLModel &model = models[m];
			ParsedBodyPart::SubModel &sm = pbp.models[m];
			sm.name = std::string(model.name, strnlen(model.name, 64));

			// Vertices
			const float *verts = reinterpret_cast<const float *>(data + model.vertindex);
			sm.vertices.assign(verts, verts + model.numverts * 3);

			// Normals
			const float *norms = reinterpret_cast<const float *>(data + model.normindex);
			sm.normals.assign(norms, norms + model.numnorms * 3);

			// Bone indices
			const uint8_t *vbone = data + model.vertinfoindex;
			sm.vert_bone.assign(vbone, vbone + model.numverts);

			const uint8_t *nbone = data + model.norminfoindex;
			sm.norm_bone.assign(nbone, nbone + model.numnorms);

			// Meshes
			const MDLMesh *meshes = reinterpret_cast<const MDLMesh *>(data + model.meshindex);
			sm.meshes.resize(model.nummesh);

			for (int mi = 0; mi < model.nummesh; mi++) {
				ParsedMDLMesh &pm = sm.meshes[mi];
				pm.skin_ref = meshes[mi].skinref;

				// Parse triangle strip/fan commands
				const int16_t *tricmds = reinterpret_cast<const int16_t *>(data + meshes[mi].triindex);

				while (true) {
					int16_t cmd = *tricmds++;
					if (cmd == 0) break;

					bool is_fan = (cmd < 0);
					int vert_count = is_fan ? -cmd : cmd;

					struct TmpVert {
						int vi, ni;
						float s, t;
					};
					std::vector<TmpVert> strip_verts(vert_count);

					for (int v = 0; v < vert_count; v++) {
						strip_verts[v].vi = tricmds[0];
						strip_verts[v].ni = tricmds[1];
						strip_verts[v].s = (float)tricmds[2];
						strip_verts[v].t = (float)tricmds[3];
						tricmds += 4;

						// Normalize UVs by texture size
						if (pm.skin_ref < (int)mdl_data.textures.size()) {
							const auto &tex = mdl_data.textures[pm.skin_ref];
							if (tex.width > 0) strip_verts[v].s /= tex.width;
							if (tex.height > 0) strip_verts[v].t /= tex.height;
						}
					}

					// Convert to triangles
					for (int v = 2; v < vert_count; v++) {
						ParsedTriVertex a, b, c;

						if (is_fan) {
							a = {strip_verts[0].vi, strip_verts[0].ni, strip_verts[0].s, strip_verts[0].t};
							b = {strip_verts[v - 1].vi, strip_verts[v - 1].ni, strip_verts[v - 1].s, strip_verts[v - 1].t};
							c = {strip_verts[v].vi, strip_verts[v].ni, strip_verts[v].s, strip_verts[v].t};
						} else {
							if ((v % 2) == 0) {
								a = {strip_verts[v - 2].vi, strip_verts[v - 2].ni, strip_verts[v - 2].s, strip_verts[v - 2].t};
								b = {strip_verts[v - 1].vi, strip_verts[v - 1].ni, strip_verts[v - 1].s, strip_verts[v - 1].t};
								c = {strip_verts[v].vi, strip_verts[v].ni, strip_verts[v].s, strip_verts[v].t};
							} else {
								a = {strip_verts[v - 1].vi, strip_verts[v - 1].ni, strip_verts[v - 1].s, strip_verts[v - 1].t};
								b = {strip_verts[v - 2].vi, strip_verts[v - 2].ni, strip_verts[v - 2].s, strip_verts[v - 2].t};
								c = {strip_verts[v].vi, strip_verts[v].ni, strip_verts[v].s, strip_verts[v].t};
							}
						}

						pm.triangle_verts.push_back(a);
						pm.triangle_verts.push_back(b);
						pm.triangle_verts.push_back(c);
					}
				}
			}
		}
	}
}

void MDLParser::parse_sequences(const uint8_t *data, size_t size) {
	if (header->numseq <= 0) return;

	const MDLSequenceDesc *seqs = reinterpret_cast<const MDLSequenceDesc *>(data + header->seqindex);

	mdl_data.sequences.resize(header->numseq);
	for (int i = 0; i < header->numseq; i++) {
		ParsedSequence &ps = mdl_data.sequences[i];
		ps.name = std::string(seqs[i].label, strnlen(seqs[i].label, 32));
		ps.fps = seqs[i].fps;
		ps.num_frames = seqs[i].numframes;
		ps.flags = seqs[i].flags;
		ps.linear_movement[0] = seqs[i].linearmovement[0];
		ps.linear_movement[1] = seqs[i].linearmovement[1];
		ps.linear_movement[2] = seqs[i].linearmovement[2];

		// Only decode animations from seqgroup 0 (embedded in main file)
		if (seqs[i].seqgroup == 0) {
			decode_animation(data, size, &seqs[i], 0, ps.frames);
		}
	}
}

void MDLParser::decode_animation(const uint8_t *data, size_t size,
	const MDLSequenceDesc *seq, int blend,
	std::vector<ParsedAnimFrame> &frames) {

	int num_bones = header->numbones;
	int num_frames = seq->numframes;
	if (num_frames <= 0 || num_bones <= 0) return;

	const MDLBone *bones = reinterpret_cast<const MDLBone *>(data + header->boneindex);

	// Animation data: seq->animindex points to array of MDLAnimation[numbones]
	const MDLAnimation *anims = reinterpret_cast<const MDLAnimation *>(
		data + seq->animindex + blend * num_bones * sizeof(MDLAnimation));

	frames.resize(num_frames);
	for (int f = 0; f < num_frames; f++) {
		frames[f].bone_pos.resize(num_bones * 3);
		frames[f].bone_rot.resize(num_bones * 3);
	}

	for (int b = 0; b < num_bones; b++) {
		const MDLAnimation &anim = anims[b];

		for (int channel = 0; channel < 6; channel++) {
			float base_value = bones[b].value[channel];
			float scale = bones[b].scale[channel];

			if (anim.offset[channel] == 0) {
				// No animation data for this channel, use rest pose
				for (int f = 0; f < num_frames; f++) {
					if (channel < 3) {
						frames[f].bone_pos[b * 3 + channel] = base_value;
					} else {
						frames[f].bone_rot[b * 3 + (channel - 3)] = base_value;
					}
				}
				continue;
			}

			const MDLAnimValue *anim_values = reinterpret_cast<const MDLAnimValue *>(
				reinterpret_cast<const uint8_t *>(&anim) + anim.offset[channel]);

			int frame = 0;
			int anim_idx = 0;

			while (frame < num_frames) {
				uint8_t valid = anim_values[anim_idx].num.valid;
				uint8_t total = anim_values[anim_idx].num.total;
				anim_idx++;

				for (int t = 0; t < (int)total && frame < num_frames; t++, frame++) {
					float value;
					if (t < (int)valid) {
						value = base_value + anim_values[anim_idx + t].value * scale;
					} else {
						value = base_value + anim_values[anim_idx + valid - 1].value * scale;
					}

					if (channel < 3) {
						frames[frame].bone_pos[b * 3 + channel] = value;
					} else {
						frames[frame].bone_rot[b * 3 + (channel - 3)] = value;
					}
				}

				anim_idx += valid;
			}
		}
	}
}

} // namespace goldsrc
