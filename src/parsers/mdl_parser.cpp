#include "mdl_parser.h"
#include "parse_utils.h"
#include <cstring>

using namespace std;

namespace goldsrc {

static bool check_range(size_t offset, size_t count, size_t stride, size_t size) {
	size_t total = count * stride;
	if (stride > 0 && total / stride != count) return false; // overflow
	return offset <= size && total <= size - offset;
}

bool MDLParser::parse(const uint8_t *data, size_t size) {
	if (size < sizeof(MDLHeader)) return false;

	header_data = read_from<MDLHeader>(data);

	if (header_data.id != MDL_MAGIC || header_data.version != MDL_VERSION) {
		return false;
	}

	parse_bones(data, size);
	parse_hitboxes(data, size);
	parse_textures(data, size);
	parse_skins(data, size);
	parse_bodyparts(data, size);
	parse_sequences(data, size);

	return true;
}

void MDLParser::parse_hitboxes(const uint8_t *data, size_t size) {
	if (header_data.numhitboxes <= 0) return;
	if (!check_range(header_data.hitboxindex, header_data.numhitboxes, sizeof(MDLHitbox), size)) return;

	auto boxes = read_arr<MDLHitbox>(data, header_data.hitboxindex, header_data.numhitboxes);

	mdl_data.hitboxes.resize(header_data.numhitboxes);
	for (int i = 0; i < header_data.numhitboxes; i++) {
		ParsedHitbox &ph = mdl_data.hitboxes[i];
		ph.bone = boxes[i].bone;
		ph.group = boxes[i].group;
		for (int j = 0; j < 3; j++) {
			ph.bbmin[j] = boxes[i].bbmin[j];
			ph.bbmax[j] = boxes[i].bbmax[j];
		}
	}
}

void MDLParser::parse_bones(const uint8_t *data, size_t size) {
	if (header_data.numbones <= 0) return;
	if (!check_range(header_data.boneindex, header_data.numbones, sizeof(MDLBone), size)) return;

	raw_bones = read_arr<MDLBone>(data, header_data.boneindex, header_data.numbones);

	mdl_data.bones.resize(header_data.numbones);
	for (int i = 0; i < header_data.numbones; i++) {
		ParsedBone &pb = mdl_data.bones[i];
		pb.name = string(raw_bones[i].name, strnlen(raw_bones[i].name, 32));
		pb.parent = raw_bones[i].parent;
		pb.pos[0] = raw_bones[i].value[0];
		pb.pos[1] = raw_bones[i].value[1];
		pb.pos[2] = raw_bones[i].value[2];
		pb.rot[0] = raw_bones[i].value[3];
		pb.rot[1] = raw_bones[i].value[4];
		pb.rot[2] = raw_bones[i].value[5];
	}
}

void MDLParser::parse_textures(const uint8_t *data, size_t size) {
	if (header_data.numtextures <= 0) return;
	if (!check_range(header_data.textureindex, header_data.numtextures, sizeof(MDLTexture), size)) return;

	auto textures = read_arr<MDLTexture>(data, header_data.textureindex, header_data.numtextures);

	mdl_data.textures.resize(header_data.numtextures);
	for (int i = 0; i < header_data.numtextures; i++) {
		ParsedMDLTexture &pt = mdl_data.textures[i];
		pt.name = string(textures[i].name, strnlen(textures[i].name, 64));
		pt.width = textures[i].width;
		pt.height = textures[i].height;
		pt.flags = textures[i].flags;

		if (pt.width <= 0 || pt.height <= 0 || pt.width > 4096 || pt.height > 4096) continue;

		size_t pixel_count = (size_t)pt.width * pt.height;
		size_t data_end = (size_t)textures[i].index + pixel_count + 256 * 3;
		if (textures[i].index < 0 || data_end > size) continue;

		const uint8_t *pixel_data = data + textures[i].index;
		const uint8_t *palette = pixel_data + pixel_count;

		pt.data.resize(pixel_count * 4);
		for (size_t j = 0; j < pixel_count; j++) {
			uint8_t idx = pixel_data[j];
			pt.data[j * 4 + 0] = palette[idx * 3 + 0];
			pt.data[j * 4 + 1] = palette[idx * 3 + 1];
			pt.data[j * 4 + 2] = palette[idx * 3 + 2];
			pt.data[j * 4 + 3] = 255;
		}
	}
}

void MDLParser::parse_skins(const uint8_t *data, size_t size) {
	mdl_data.num_skin_ref = header_data.numskinref;
	mdl_data.num_skin_families = header_data.numskinfamilies;

	if (header_data.numskinref <= 0 || header_data.numskinfamilies <= 0) return;
	int64_t total = (int64_t)header_data.numskinref * header_data.numskinfamilies;
	if (total > 65536) return; // sanity cap
	if (!check_range(header_data.skinindex, (size_t)total, sizeof(int16_t), size)) return;

	mdl_data.skin_table = read_arr<int16_t>(data, header_data.skinindex, (size_t)total);
}

void MDLParser::parse_bodyparts(const uint8_t *data, size_t size) {
	if (header_data.numbodyparts <= 0) return;
	if (!check_range(header_data.bodypartindex, header_data.numbodyparts, sizeof(MDLBodyPart), size)) return;

	auto bodyparts = read_arr<MDLBodyPart>(data, header_data.bodypartindex, header_data.numbodyparts);

	mdl_data.bodyparts.resize(header_data.numbodyparts);
	for (int bp = 0; bp < header_data.numbodyparts; bp++) {
		ParsedBodyPart &pbp = mdl_data.bodyparts[bp];
		pbp.name = string(bodyparts[bp].name, strnlen(bodyparts[bp].name, 64));

		if (bodyparts[bp].nummodels <= 0) continue;
		if (!check_range(bodyparts[bp].modelindex, bodyparts[bp].nummodels, sizeof(MDLModel), size)) continue;

		auto models = read_arr<MDLModel>(data, bodyparts[bp].modelindex, bodyparts[bp].nummodels);
		pbp.models.resize(bodyparts[bp].nummodels);

		for (int m = 0; m < bodyparts[bp].nummodels; m++) {
			const MDLModel &model = models[m];
			ParsedBodyPart::SubModel &sm = pbp.models[m];
			sm.name = string(model.name, strnlen(model.name, 64));

			if (model.numverts > 0 && check_range(model.vertindex, (size_t)model.numverts * 3, sizeof(float), size))
				sm.vertices = read_arr<float>(data, model.vertindex, (size_t)model.numverts * 3);

			if (model.numnorms > 0 && check_range(model.normindex, (size_t)model.numnorms * 3, sizeof(float), size))
				sm.normals = read_arr<float>(data, model.normindex, (size_t)model.numnorms * 3);

			if (model.numverts > 0 && check_range(model.vertinfoindex, model.numverts, 1, size))
				sm.vert_bone = read_arr<uint8_t>(data, model.vertinfoindex, model.numverts);

			if (model.numnorms > 0 && check_range(model.norminfoindex, model.numnorms, 1, size))
				sm.norm_bone = read_arr<uint8_t>(data, model.norminfoindex, model.numnorms);

			if (model.nummesh <= 0) continue;
			if (!check_range(model.meshindex, model.nummesh, sizeof(MDLMesh), size)) continue;

			auto meshes = read_arr<MDLMesh>(data, model.meshindex, model.nummesh);
			sm.meshes.resize(model.nummesh);

			struct TmpVert { int vi, ni; float s, t; };
			vector<TmpVert> strip_verts;

			for (int mi = 0; mi < model.nummesh; mi++) {
				ParsedMDLMesh &pm = sm.meshes[mi];
				pm.skin_ref = meshes[mi].skinref;

				if (meshes[mi].triindex < 0 || (size_t)meshes[mi].triindex >= size) continue;
				const uint8_t *tcp     = data + meshes[mi].triindex;
				const uint8_t *tcp_end = data + (size & ~size_t(1));
				if (tcp >= tcp_end) continue;

				while (tcp + 2 <= tcp_end) {
					int16_t cmd = read_from<int16_t>(tcp); tcp += 2;
					if (cmd == 0) break;

					bool is_fan = (cmd < 0);
					int vert_count = is_fan ? -cmd : cmd;

					if (vert_count <= 0 || (size_t)vert_count * 8 > (size_t)(tcp_end - tcp)) break;

					strip_verts.resize(vert_count);

					for (int v = 0; v < vert_count; v++) {
						strip_verts[v].vi = read_from<int16_t>(tcp + 0);
						strip_verts[v].ni = read_from<int16_t>(tcp + 2);
						strip_verts[v].s  = (float)read_from<int16_t>(tcp + 4);
						strip_verts[v].t  = (float)read_from<int16_t>(tcp + 6);
						tcp += 8;

						if (pm.skin_ref < (int)mdl_data.textures.size()) {
							const auto &tex = mdl_data.textures[pm.skin_ref];
							if (tex.width > 0) strip_verts[v].s /= tex.width;
							if (tex.height > 0) strip_verts[v].t /= tex.height;
						}
					}

					for (int v = 2; v < vert_count; v++) {
						ParsedTriVertex a, b, c;

						if (is_fan) {
							a = {strip_verts[0].vi,   strip_verts[0].ni,   strip_verts[0].s,   strip_verts[0].t};
							b = {strip_verts[v-1].vi, strip_verts[v-1].ni, strip_verts[v-1].s, strip_verts[v-1].t};
							c = {strip_verts[v].vi,   strip_verts[v].ni,   strip_verts[v].s,   strip_verts[v].t};
						} else {
							if ((v % 2) == 0) {
								a = {strip_verts[v-2].vi, strip_verts[v-2].ni, strip_verts[v-2].s, strip_verts[v-2].t};
								b = {strip_verts[v-1].vi, strip_verts[v-1].ni, strip_verts[v-1].s, strip_verts[v-1].t};
								c = {strip_verts[v].vi,   strip_verts[v].ni,   strip_verts[v].s,   strip_verts[v].t};
							} else {
								a = {strip_verts[v-1].vi, strip_verts[v-1].ni, strip_verts[v-1].s, strip_verts[v-1].t};
								b = {strip_verts[v-2].vi, strip_verts[v-2].ni, strip_verts[v-2].s, strip_verts[v-2].t};
								c = {strip_verts[v].vi,   strip_verts[v].ni,   strip_verts[v].s,   strip_verts[v].t};
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
	if (header_data.numseq <= 0) return;
	if (!check_range(header_data.seqindex, header_data.numseq, sizeof(MDLSequenceDesc), size)) return;

	auto seqs = read_arr<MDLSequenceDesc>(data, header_data.seqindex, header_data.numseq);

	mdl_data.sequences.resize(header_data.numseq);
	for (int i = 0; i < header_data.numseq; i++) {
		ParsedSequence &ps = mdl_data.sequences[i];
		ps.name = string(seqs[i].label, strnlen(seqs[i].label, 32));
		ps.fps = seqs[i].fps;
		ps.num_frames = seqs[i].numframes;
		ps.flags = seqs[i].flags;
		ps.linear_movement[0] = seqs[i].linearmovement[0];
		ps.linear_movement[1] = seqs[i].linearmovement[1];
		ps.linear_movement[2] = seqs[i].linearmovement[2];

		if (seqs[i].seqgroup == 0) {
			decode_animation(data, size, &seqs[i], 0, ps.frames);
		}
	}
}

void MDLParser::decode_animation(const uint8_t *data, size_t size,
	const MDLSequenceDesc *seq, int blend,
	vector<ParsedAnimFrame> &frames) {

	int num_bones = header_data.numbones;
	int num_frames = seq->numframes;
	if (num_frames <= 0 || num_bones <= 0 || (int)raw_bones.size() != num_bones) return;

	const auto &bones = raw_bones;

	size_t anim_offset = (size_t)seq->animindex + (size_t)blend * num_bones * sizeof(MDLAnimation);
	if (!check_range(anim_offset, num_bones, sizeof(MDLAnimation), size)) return;
	auto anims = read_arr<MDLAnimation>(data, anim_offset, num_bones);

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
				// No animation data for this channel — use rest pose
				for (int f = 0; f < num_frames; f++) {
					if (channel < 3)
						frames[f].bone_pos[b * 3 + channel] = base_value;
					else
						frames[f].bone_rot[b * 3 + (channel - 3)] = base_value;
				}
				continue;
			}

			size_t anim_entry_ofs = anim_offset + (size_t)b * sizeof(MDLAnimation) + anim.offset[channel];
			if (anim_entry_ofs >= size) break;
			const uint8_t *anim_base = data + anim_entry_ofs;
			size_t max_anim_entries = (size - anim_entry_ofs) / sizeof(MDLAnimValue);

			int frame = 0;
			size_t anim_idx = 0;

			while (frame < num_frames) {
				if (anim_idx >= max_anim_entries) break;

				MDLAnimValue av = read_from<MDLAnimValue>(anim_base + anim_idx * sizeof(MDLAnimValue));
				uint8_t valid = av.num.valid;
				uint8_t total = av.num.total;
				anim_idx++;

				if (total == 0) break;

				for (int t = 0; t < (int)total && frame < num_frames; t++, frame++) {
					float value;
					if (valid == 0) {
						value = base_value;
					} else {
						size_t read_idx = anim_idx + (t < (int)valid ? t : valid - 1);
						if (read_idx >= max_anim_entries) {
							value = base_value;
						} else {
							MDLAnimValue av2 = read_from<MDLAnimValue>(anim_base + read_idx * sizeof(MDLAnimValue));
							value = base_value + av2.value * scale;
						}
					}

					if (channel < 3)
						frames[frame].bone_pos[b * 3 + channel] = value;
					else
						frames[frame].bone_rot[b * 3 + (channel - 3)] = value;
				}

				anim_idx += valid;
			}
		}
	}
}

} // namespace goldsrc
