#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <algorithm>
#include <cstring>
#include <vector>

#include "parsers/bsp_parser.h"
#include "parsers/mdl_parser.h"
#include "parsers/spr_parser.h"
#include "parsers/texture_decode.h"
#include "parsers/wad_parser.h"

using namespace goldsrc;

// ============================================================
// Binary builder — helpers for constructing minimal test blobs
// ============================================================

struct BinaryBuilder {
    std::vector<uint8_t> data;

    void le32(int32_t v) {
        data.push_back(v & 0xFF);
        data.push_back((v >> 8) & 0xFF);
        data.push_back((v >> 16) & 0xFF);
        data.push_back((v >> 24) & 0xFF);
    }
    void u32(uint32_t v) { le32((int32_t)v); }
    void u16(uint16_t v) {
        data.push_back(v & 0xFF);
        data.push_back((v >> 8) & 0xFF);
    }
    void f32(float v) {
        uint32_t b;
        memcpy(&b, &v, 4);
        u32(b);
    }
    void str_padded(const char *s, size_t max_len) {
        size_t n = strlen(s);
        for (size_t i = 0; i < max_len; i++)
            data.push_back(i < n ? (uint8_t)s[i] : 0);
    }
    void zeros(size_t n) { data.insert(data.end(), n, 0); }
    void bytes(const uint8_t *p, size_t n) { data.insert(data.end(), p, p + n); }
    void patch_le32(size_t off, int32_t v) {
        data[off + 0] = v & 0xFF;
        data[off + 1] = (v >> 8) & 0xFF;
        data[off + 2] = (v >> 16) & 0xFF;
        data[off + 3] = (v >> 24) & 0xFF;
    }
    size_t size() const { return data.size(); }
};

// ============================================================
// Factories for minimal valid binary blobs
// ============================================================

// Produces a BSP blob with the given entity string and empty geometry lumps.
static std::vector<uint8_t> make_minimal_bsp(const std::string &entity_str) {
    BinaryBuilder b;
    b.le32(30); // HLBSP_VERSION

    // 15 lumps; patch entities lump (index 0) after we know the offset
    size_t lump0_offset = b.size();
    for (int i = 0; i < 15; i++) {
        b.le32(0);
        b.le32(0);
    }

    int32_t ent_offset = (int32_t)b.size();
    int32_t ent_len = (int32_t)entity_str.size();
    b.patch_le32(lump0_offset + 0, ent_offset);
    b.patch_le32(lump0_offset + 4, ent_len);

    for (char c : entity_str)
        b.data.push_back((uint8_t)c);

    return b.data;
}

// Produces a minimal WAD3 blob with one 2×2 opaque texture.
// WADMipTex layout: name[16] + width + height + offsets[4] = 40 bytes
static std::vector<uint8_t> make_minimal_wad3(const char *tex_name,
                                               uint32_t w, uint32_t h,
                                               const uint8_t *pixels,
                                               const uint8_t palette[256 * 3]) {
    BinaryBuilder b;

    // WADHeader (identification + numlumps + infotableofs placeholder)
    b.data.insert(b.data.end(), {'W', 'A', 'D', '3'});
    b.le32(1); // numlumps
    size_t infotableofs_pos = b.size();
    b.le32(0); // placeholder

    // Lump data starts here (offset 12)
    size_t lump_start = b.size();

    // WADMipTex header (40 bytes)
    b.str_padded(tex_name, 16);
    b.u32(w);
    b.u32(h);
    uint32_t miptex_hdr = 40; // offsets[0] = immediately after the header
    b.u32(miptex_hdr);
    b.u32(0);
    b.u32(0);
    b.u32(0); // offsets[1..3] unused

    // Pixel data (mip level 0) + remaining mip levels (filled with zeros)
    size_t npix = (size_t)w * h;
    b.bytes(pixels, npix);
    b.zeros(npix / 4); // mip1
    b.zeros(npix / 16); // mip2
    b.zeros(npix / 64); // mip3

    // 2-byte count + palette
    b.u16(256);
    b.bytes(palette, 256 * 3);

    size_t lump_size = b.size() - lump_start;

    // WADLumpInfo (32 bytes)
    size_t lump_info_offset = b.size();
    b.le32((int32_t)lump_start); // filepos
    b.le32((int32_t)lump_size);  // disksize
    b.le32((int32_t)lump_size);  // size (uncompressed)
    b.data.push_back(0x43);      // type: miptex WAD3
    b.data.push_back(0);         // compression
    b.data.push_back(0);
    b.data.push_back(0); // pad
    b.str_padded(tex_name, 16);

    b.patch_le32(infotableofs_pos, (int32_t)lump_info_offset);

    return b.data;
}

// Produces a minimal SPR blob with one single-frame sprite.
static std::vector<uint8_t> make_minimal_spr(SPRType type, SPRTextureFormat fmt,
                                              int32_t w, int32_t h,
                                              const uint8_t *pixels,
                                              const uint8_t palette[256 * 3]) {
    BinaryBuilder b;

    // SPRHeader (10 × int32/float = 40 bytes)
    b.le32(SPR_MAGIC);
    b.le32(SPR_VERSION);
    b.le32((int32_t)type);
    b.le32((int32_t)fmt);
    b.f32(10.0f); // bounding_radius
    b.le32(w);    // max_width
    b.le32(h);    // max_height
    b.le32(1);    // num_frames
    b.f32(0.0f);  // beam_length
    b.le32(0);    // synch_type

    // Palette
    b.u16(256);
    b.bytes(palette, 256 * 3);

    // Single frame (frame_type=0)
    b.le32(0); // frame_type: single frame
    b.le32(0); // origin_x
    b.le32(0); // origin_y
    b.le32(w);
    b.le32(h);
    b.bytes(pixels, (size_t)w * h);

    return b.data;
}

// Writes s into a fixed-width MDL name field, zero-padded. Deliberately not strncpy: MDL
// name fields are not required to be NUL-terminated, and the parser reads them with
// strnlen(name, cap), so a test needs to be able to fill the field edge to edge.
static void set_fixed(char *dst, size_t cap, const std::string &s) {
    memset(dst, 0, cap);
    memcpy(dst, s.data(), std::min(cap, s.size()));
}

// MDL has far too many header fields to lay out byte-by-byte the way the BSP/WAD/SPR
// factories above do. The on-disk structs are already #pragma pack(1), so this appends
// them directly and patches the header in at the end, once every section offset is known.
struct MDLBuilder {
    std::vector<uint8_t> blob;
    MDLHeader hdr{};

    MDLBuilder() {
        hdr.id = MDL_MAGIC;
        hdr.version = MDL_VERSION;
        blob.resize(sizeof(MDLHeader), 0); // reserved; filled by finish()
    }

    int32_t append_bytes(const void *p, size_t n) {
        int32_t off = (int32_t)blob.size();
        if (n > 0) {
            const uint8_t *b = static_cast<const uint8_t *>(p);
            blob.insert(blob.end(), b, b + n);
        }
        return off;
    }

    template <typename T>
    int32_t append(const std::vector<T> &items) {
        return append_bytes(items.data(), items.size() * sizeof(T));
    }

    // Lays down a one-bone MDLAnimation table and appends the encoded data for whichever
    // of the 6 channels are non-empty. MDLAnimation::offset is relative to the start of
    // that bone's own table entry, which is what makes this fiddly enough to centralise.
    // Returns the table offset, for MDLSequenceDesc::animindex.
    int32_t append_anim_1bone(const std::vector<uint8_t> channels[6]) {
        MDLAnimation anim{};
        int32_t table = append_bytes(&anim, sizeof(anim));
        for (int c = 0; c < 6; c++) {
            if (channels[c].empty()) continue;
            int32_t off = append_bytes(channels[c].data(), channels[c].size());
            anim.offset[c] = (uint16_t)(off - table);
        }
        memcpy(blob.data() + table, &anim, sizeof(anim));
        return table;
    }

    std::vector<uint8_t> finish() {
        std::vector<uint8_t> out = blob;
        memcpy(out.data(), &hdr, sizeof(MDLHeader));
        return out;
    }
};

static void push_i16(std::vector<uint8_t> &out, int16_t v) {
    out.push_back((uint8_t)(v & 0xFF));
    out.push_back((uint8_t)((v >> 8) & 0xFF));
}

// One run-length span of an animation channel: `total` frames drawn from `values`, which
// holds `valid` entries. Frames past `valid` repeat the last value — that clamp is the
// part of the encoding most likely to be got wrong, so tests drive it directly.
struct AnimSpan {
    uint8_t valid;
    uint8_t total;
    std::vector<int16_t> values;
};

static std::vector<uint8_t> encode_anim_channel(const std::vector<AnimSpan> &spans) {
    std::vector<uint8_t> out;
    for (const AnimSpan &s : spans) {
        out.push_back(s.valid);
        out.push_back(s.total);
        for (int16_t v : s.values)
            push_i16(out, v);
    }
    return out;
}

struct TriVertRaw {
    int16_t vi, ni, s, t;
};

// Encodes a single triangle command run plus its terminator. Positive cmd = strip,
// negative = fan; the magnitude is the vertex count.
static std::vector<uint8_t> encode_tri_cmds(int16_t cmd, const std::vector<TriVertRaw> &verts) {
    std::vector<uint8_t> out;
    push_i16(out, cmd);
    for (const TriVertRaw &v : verts) {
        push_i16(out, v.vi);
        push_i16(out, v.ni);
        push_i16(out, v.s);
        push_i16(out, v.t);
    }
    push_i16(out, 0); // command stream terminator
    return out;
}

// Builds a 1-bone model carrying a single sequence, for exercising decode_animation.
// The caller's spans must cover num_frames.
static std::vector<uint8_t> make_anim_mdl(const float value[6], const float scale[6],
                                          int num_frames,
                                          const std::vector<uint8_t> channels[6]) {
    MDLBuilder mb;

    MDLBone bone{};
    set_fixed(bone.name, sizeof(bone.name), "root");
    bone.parent = -1;
    for (int i = 0; i < 6; i++) {
        bone.value[i] = value[i];
        bone.scale[i] = scale[i];
    }
    mb.hdr.numbones = 1;
    mb.hdr.boneindex = mb.append(std::vector<MDLBone>{bone});

    int32_t animindex = mb.append_anim_1bone(channels);

    MDLSequenceDesc seq{};
    set_fixed(seq.label, sizeof(seq.label), "idle");
    seq.fps = 30.0f;
    seq.numframes = num_frames;
    seq.animindex = animindex;
    seq.seqgroup = 0;
    mb.hdr.numseq = 1;
    mb.hdr.seqindex = mb.append(std::vector<MDLSequenceDesc>{seq});

    return mb.finish();
}

// ============================================================
// TEST SUITE: decode_palette_pixels
// ============================================================

TEST_SUITE("decode_palette_pixels") {
    TEST_CASE("maps palette indices to RGBA") {
        uint8_t palette[256 * 3] = {};
        palette[0] = 255; palette[1] = 0; palette[2] = 0;   // index 0 → red
        palette[3] = 0;   palette[4] = 255; palette[5] = 0; // index 1 → green

        uint8_t pixels[] = {0, 1};
        std::vector<uint8_t> out;
        decode_palette_pixels(pixels, palette, 2, false, out);

        REQUIRE(out.size() == 8);
        CHECK(out[0] == 255); CHECK(out[1] == 0); CHECK(out[2] == 0); CHECK(out[3] == 255);
        CHECK(out[4] == 0); CHECK(out[5] == 255); CHECK(out[6] == 0); CHECK(out[7] == 255);
    }

    TEST_CASE("has_transparency=false: index 255 is fully opaque") {
        uint8_t palette[256 * 3] = {};
        palette[255 * 3] = 128; palette[255 * 3 + 1] = 64; palette[255 * 3 + 2] = 32;

        uint8_t pixels[] = {255};
        std::vector<uint8_t> out;
        decode_palette_pixels(pixels, palette, 1, false, out);

        REQUIRE(out.size() == 4);
        CHECK(out[0] == 128); CHECK(out[1] == 64); CHECK(out[2] == 32);
        CHECK(out[3] == 255);
    }

    TEST_CASE("has_transparency=true: index 255 becomes (0,0,0,0)") {
        uint8_t palette[256 * 3] = {};
        palette[255 * 3] = 100; palette[255 * 3 + 1] = 100; palette[255 * 3 + 2] = 100;

        uint8_t pixels[] = {255};
        std::vector<uint8_t> out;
        decode_palette_pixels(pixels, palette, 1, true, out);

        REQUIRE(out.size() == 4);
        CHECK(out[0] == 0); CHECK(out[1] == 0); CHECK(out[2] == 0); CHECK(out[3] == 0);
    }

    TEST_CASE("has_transparency=true: non-255 indices stay opaque") {
        uint8_t palette[256 * 3] = {};
        palette[5 * 3] = 10; palette[5 * 3 + 1] = 20; palette[5 * 3 + 2] = 30;

        uint8_t pixels[] = {5};
        std::vector<uint8_t> out;
        decode_palette_pixels(pixels, palette, 1, true, out);

        REQUIRE(out.size() == 4);
        CHECK(out[0] == 10); CHECK(out[1] == 20); CHECK(out[2] == 30); CHECK(out[3] == 255);
    }

    TEST_CASE("output size is always pixel_count * 4") {
        uint8_t palette[256 * 3] = {};
        uint8_t pixels[7] = {};
        std::vector<uint8_t> out;
        decode_palette_pixels(pixels, palette, 7, false, out);
        CHECK(out.size() == 28);
    }

    TEST_CASE("zero pixels produces empty output") {
        uint8_t palette[256 * 3] = {};
        std::vector<uint8_t> out;
        decode_palette_pixels(nullptr, palette, 0, false, out);
        CHECK(out.empty());
    }
}

// ============================================================
// TEST SUITE: is_tool_texture
// ============================================================

TEST_SUITE("is_tool_texture") {
    TEST_CASE("recognizes all standard tool texture names") {
        CHECK(is_tool_texture("clip"));
        CHECK(is_tool_texture("null"));
        CHECK(is_tool_texture("origin"));
        CHECK(is_tool_texture("bevel"));
        CHECK(is_tool_texture("hint"));
        CHECK(is_tool_texture("skip"));
    }

    TEST_CASE("aaa prefix (any length) is a tool texture") {
        CHECK(is_tool_texture("aaa"));
        CHECK(is_tool_texture("aaatrigger")); // common trigger hull texture
        CHECK(is_tool_texture("aaabbb"));
    }

    TEST_CASE("empty name counts as a tool texture") {
        CHECK(is_tool_texture(""));
    }

    TEST_CASE("matching is case-insensitive") {
        CHECK(is_tool_texture("CLIP"));
        CHECK(is_tool_texture("Null"));
        CHECK(is_tool_texture("ORIGIN"));
        CHECK(is_tool_texture("AAA"));
        CHECK(is_tool_texture("AaaTrigger"));
    }

    TEST_CASE("regular textures are not tool textures") {
        CHECK_FALSE(is_tool_texture("wall"));
        CHECK_FALSE(is_tool_texture("!water"));
        CHECK_FALSE(is_tool_texture("*water"));
        CHECK_FALSE(is_tool_texture("concrete01"));
    }

    TEST_CASE("partial matches on non-aaa names are not tool textures") {
        CHECK_FALSE(is_tool_texture("clip_detail")); // exact match only
        CHECK_FALSE(is_tool_texture("null_wall"));
        CHECK_FALSE(is_tool_texture("originwall"));
        CHECK_FALSE(is_tool_texture("aa")); // one char short of aaa prefix
    }
}

// ============================================================
// TEST SUITE: BSPData::decompress_pvs
// ============================================================

namespace {
BSPLeaf make_leaf(int32_t visofs) {
    BSPLeaf leaf{};
    leaf.visofs = visofs;
    return leaf;
}
} // namespace

TEST_SUITE("BSPData::decompress_pvs") {
    TEST_CASE("out-of-range leaf indices return all-false") {
        BSPData bsp;
        bsp.leafs.resize(5);

        auto pvs = bsp.decompress_pvs(-1);
        REQUIRE(pvs.size() == 5);
        CHECK(std::none_of(pvs.begin(), pvs.end(), [](bool b) { return b; }));

        pvs = bsp.decompress_pvs(5);
        CHECK(std::none_of(pvs.begin(), pvs.end(), [](bool b) { return b; }));
    }

    TEST_CASE("leaf 0 is the void leaf and always returns all-false") {
        BSPData bsp;
        bsp.leafs.resize(4);
        bsp.visibility = {0xFF}; // would light up leaves if decoded

        auto pvs = bsp.decompress_pvs(0);
        REQUIRE(pvs.size() == 4);
        CHECK(std::none_of(pvs.begin(), pvs.end(), [](bool b) { return b; }));
    }

    TEST_CASE("visofs -1 means all leaves are potentially visible") {
        BSPData bsp;
        bsp.leafs.resize(5);
        bsp.leafs[2] = make_leaf(-1);

        auto pvs = bsp.decompress_pvs(2);
        REQUIRE(pvs.size() == 5);
        CHECK(std::all_of(pvs.begin(), pvs.end(), [](bool b) { return b; }));
    }

    TEST_CASE("literal PVS byte marks the correct leaves visible") {
        // 5 leaves → num_vis_leafs=4, num_bytes=1
        // 0x05 = 0b00000101: bits 0,2 → leaves 1,3 visible from leaf 1
        BSPData bsp;
        bsp.leafs.resize(5);
        bsp.leafs[1] = make_leaf(0);
        bsp.visibility = {0x05};

        auto pvs = bsp.decompress_pvs(1);
        REQUIRE(pvs.size() == 5);
        CHECK(pvs[1] == true);  // self-visibility
        CHECK(pvs[2] == false);
        CHECK(pvs[3] == true); // bit 2 set
        CHECK(pvs[4] == false);
    }

    TEST_CASE("a leaf always sees itself regardless of PVS data") {
        // 5 leaves; leaf 3 has a PVS byte that does not set bit 2 (which would mark leaf 3)
        BSPData bsp;
        bsp.leafs.resize(5);
        bsp.leafs[3] = make_leaf(0);
        bsp.visibility = {0x04}; // bit 2 = leaf 3 from the bitfield; self-vis also forces it

        auto pvs = bsp.decompress_pvs(3);
        CHECK(pvs[3] == true);

        // Verify self-vis is set even when PVS byte has no bits for this leaf
        bsp.leafs[4] = make_leaf(1);
        bsp.visibility.push_back(0x00); // byte at offset 1: no bits set
        pvs = bsp.decompress_pvs(4);
        CHECK(pvs[4] == true);
    }

    TEST_CASE("RLE zero run skips zero-bytes in the bitfield") {
        // 17 leaves → num_vis_leafs=16, num_bytes=2
        // {0x00, 0x01, 0xFF}:
        //   0x00 followed by 0x01 → skip 1 byte (byte_idx: 0→1)
        //   0xFF → literal for byte_idx=1 → pvs[9..16] all true
        BSPData bsp;
        bsp.leafs.resize(17);
        bsp.leafs[1] = make_leaf(0);
        bsp.visibility = {0x00, 0x01, 0xFF};

        auto pvs = bsp.decompress_pvs(1);
        // Byte 0 was skipped, so leaves 2-8 are false
        for (int i = 2; i <= 8; i++) CHECK(pvs[i] == false);
        // Byte 1 = 0xFF, so leaves 9-16 are true
        for (int i = 9; i <= 16; i++) CHECK(pvs[i] == true);
        CHECK(pvs[1] == true); // self-visibility
    }
}

// ============================================================
// TEST SUITE: BSP entity parsing
// ============================================================

TEST_SUITE("BSP entity parsing") {
    TEST_CASE("key-value pairs are extracted correctly") {
        const std::string ent_str =
            "{\"classname\" \"worldspawn\"\n"
            "\"sky\" \"desert\"\n"
            "}\n"
            "{\"classname\" \"info_player_start\"\n"
            "\"origin\" \"64 128 32\"\n"
            "}\n";

        auto blob = make_minimal_bsp(ent_str);
        BSPParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));

        const auto &ents = parser.get_data().entities;
        REQUIRE(ents.size() == 2);
        CHECK(ents[0].properties.at("classname") == "worldspawn");
        CHECK(ents[0].properties.at("sky") == "desert");
        CHECK(ents[1].properties.at("classname") == "info_player_start");
        CHECK(ents[1].properties.at("origin") == "64 128 32");
    }

    TEST_CASE("empty entity string produces no entities") {
        auto blob = make_minimal_bsp("");
        BSPParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        CHECK(parser.get_data().entities.empty());
    }

    TEST_CASE("wrong BSP version returns false") {
        BinaryBuilder b;
        b.le32(29); // wrong version
        for (int i = 0; i < 15; i++) {
            b.le32(0);
            b.le32(0);
        }
        BSPParser parser;
        CHECK_FALSE(parser.parse(b.data.data(), b.data.size()));
    }

    TEST_CASE("buffer too small to hold header returns false") {
        uint8_t tiny[4] = {30, 0, 0, 0};
        BSPParser parser;
        CHECK_FALSE(parser.parse(tiny, sizeof(tiny)));
    }
}

// ============================================================
// TEST SUITE: WADParser
// ============================================================

TEST_SUITE("WADParser") {
    TEST_CASE("parses texture RGBA data from a WAD3 blob") {
        uint8_t palette[256 * 3] = {};
        palette[0] = 255; // index 0 → red
        uint8_t pixels[4] = {0, 0, 0, 0};

        auto blob = make_minimal_wad3("WALL", 2, 2, pixels, palette);
        WADParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));

        CHECK(parser.get_texture_count() == 1);
        REQUIRE(parser.has_texture("WALL"));

        const auto *tex = parser.get_texture("WALL");
        REQUIRE(tex != nullptr);
        CHECK(tex->width == 2);
        CHECK(tex->height == 2);
        REQUIRE(tex->data.size() == 16); // 2*2*4
        CHECK(tex->data[0] == 255); // R
        CHECK(tex->data[1] == 0);   // G
        CHECK(tex->data[2] == 0);   // B
        CHECK(tex->data[3] == 255); // A (opaque)
    }

    TEST_CASE("texture lookup is case-insensitive") {
        uint8_t palette[256 * 3] = {};
        uint8_t pixels[4] = {};
        auto blob = make_minimal_wad3("MyTex", 2, 2, pixels, palette);

        WADParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        CHECK(parser.has_texture("MyTex"));
        CHECK(parser.has_texture("mytex"));
        CHECK(parser.has_texture("MYTEX"));
        CHECK_FALSE(parser.has_texture("other"));
    }

    TEST_CASE("texture name starting with '{' sets has_transparency") {
        uint8_t palette[256 * 3] = {};
        uint8_t pixels[4] = {};
        auto blob = make_minimal_wad3("{fence", 2, 2, pixels, palette);

        WADParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        REQUIRE(parser.has_texture("{fence"));
        const auto *tex = parser.get_texture("{fence");
        REQUIRE(tex != nullptr);
        CHECK(tex->has_transparency == true);
    }

    TEST_CASE("get_texture returns nullptr for an absent texture") {
        uint8_t palette[256 * 3] = {};
        uint8_t pixels[4] = {};
        auto blob = make_minimal_wad3("WALL", 2, 2, pixels, palette);

        WADParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        CHECK(parser.get_texture("missing") == nullptr);
    }

    TEST_CASE("invalid WAD identification returns false") {
        uint8_t bad[12] = {'X', 'A', 'D', '3', 1, 0, 0, 0, 12, 0, 0, 0};
        WADParser parser;
        CHECK_FALSE(parser.parse(bad, sizeof(bad)));
    }
}

// ============================================================
// TEST SUITE: SPRParser
// ============================================================

TEST_SUITE("SPRParser") {
    TEST_CASE("parses a single-frame sprite with correct dimensions and pixels") {
        uint8_t palette[256 * 3] = {};
        palette[0] = 0; palette[1] = 255; palette[2] = 0; // index 0 → green
        uint8_t pixels[4] = {0, 0, 0, 0};

        auto blob = make_minimal_spr(SPR_VP_PARALLEL, SPR_NORMAL, 2, 2, pixels, palette);
        SPRParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));

        const auto &d = parser.get_data();
        CHECK(d.type == SPR_VP_PARALLEL);
        CHECK(d.texture_format == SPR_NORMAL);
        REQUIRE(d.frames.size() == 1);
        CHECK(d.frames[0].width == 2);
        CHECK(d.frames[0].height == 2);
        CHECK(d.frames[0].data[0] == 0);   // R
        CHECK(d.frames[0].data[1] == 255); // G
        CHECK(d.frames[0].data[2] == 0);   // B
        CHECK(d.frames[0].data[3] == 255); // A (fully opaque for SPR_NORMAL)
    }

    TEST_CASE("ALPHATEST: index 255 becomes transparent, others stay opaque") {
        uint8_t palette[256 * 3] = {};
        palette[255 * 3] = 50;
        uint8_t pixels[4] = {0, 255, 0, 255};

        auto blob = make_minimal_spr(SPR_VP_PARALLEL, SPR_ALPHATEST, 2, 2, pixels, palette);
        SPRParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));

        const auto &d = parser.get_data();
        CHECK(d.frames[0].data[3] == 255); // pixel 0 (index 0) → opaque
        CHECK(d.frames[0].data[7] == 0);   // pixel 1 (index 255) → transparent
    }

    TEST_CASE("INDEXALPHA: palette index maps directly to alpha channel") {
        uint8_t palette[256 * 3] = {};
        uint8_t pixels[4] = {0, 128, 0, 255};

        auto blob = make_minimal_spr(SPR_VP_PARALLEL, SPR_INDEXALPHA, 2, 2, pixels, palette);
        SPRParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));

        const auto &d = parser.get_data();
        CHECK(d.frames[0].data[3] == 0);   // index 0 → alpha 0
        CHECK(d.frames[0].data[7] == 128); // index 128 → alpha 128
        CHECK(d.frames[0].data[15] == 255); // index 255 → alpha 255
    }

    TEST_CASE("wrong magic returns false") {
        BinaryBuilder b;
        b.le32(0x12345678); // wrong magic
        b.le32(SPR_VERSION);
        b.zeros(200);
        SPRParser parser;
        CHECK_FALSE(parser.parse(b.data.data(), b.data.size()));
    }

    TEST_CASE("wrong version returns false") {
        BinaryBuilder b;
        b.le32(SPR_MAGIC);
        b.le32(1); // wrong version (should be 2)
        b.zeros(200);
        SPRParser parser;
        CHECK_FALSE(parser.parse(b.data.data(), b.data.size()));
    }

    TEST_CASE("buffer too small returns false") {
        uint8_t tiny[10] = {};
        SPRParser parser;
        CHECK_FALSE(parser.parse(tiny, sizeof(tiny)));
    }
}

// ============================================================
// TEST SUITE: MDLParser — header validation
// ============================================================

TEST_SUITE("MDLParser") {
    TEST_CASE("buffer too small to hold header returns false") {
        uint8_t tiny[4] = {};
        MDLParser parser;
        CHECK_FALSE(parser.parse(tiny, sizeof(tiny)));
    }

    TEST_CASE("wrong magic returns false") {
        BinaryBuilder b;
        b.zeros(sizeof(MDLHeader));
        // 'IDSX' instead of 'IDST'
        b.data[0] = 'I'; b.data[1] = 'D'; b.data[2] = 'S'; b.data[3] = 'X';
        b.data[4] = 10; // version = 10
        MDLParser parser;
        CHECK_FALSE(parser.parse(b.data.data(), b.data.size()));
    }

    TEST_CASE("wrong version returns false") {
        BinaryBuilder b;
        b.zeros(sizeof(MDLHeader));
        // Correct magic 'IDST'
        b.data[0] = 'I'; b.data[1] = 'D'; b.data[2] = 'S'; b.data[3] = 'T';
        b.data[4] = 9; // wrong version (should be 10)
        MDLParser parser;
        CHECK_FALSE(parser.parse(b.data.data(), b.data.size()));
    }

    // ---------- bones ----------

    TEST_CASE("parses bone names, parents and rest pose") {
        MDLBuilder mb;
        std::vector<MDLBone> bones(2);
        set_fixed(bones[0].name, 32, "root");
        bones[0].parent = -1;
        for (int i = 0; i < 6; i++) bones[0].value[i] = (float)(i + 1);
        set_fixed(bones[1].name, 32, "spine");
        bones[1].parent = 0;
        for (int i = 0; i < 6; i++) bones[1].value[i] = (float)(10 * (i + 1));

        mb.hdr.numbones = 2;
        mb.hdr.boneindex = mb.append(bones);
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        const MDLData &d = parser.get_data();

        REQUIRE(d.bones.size() == 2);
        CHECK(d.bones[0].name == "root");
        CHECK(d.bones[0].parent == -1);
        // value[0..2] is position, value[3..5] is euler rotation.
        CHECK(d.bones[0].pos[0] == doctest::Approx(1.0f));
        CHECK(d.bones[0].pos[2] == doctest::Approx(3.0f));
        CHECK(d.bones[0].rot[0] == doctest::Approx(4.0f));
        CHECK(d.bones[0].rot[2] == doctest::Approx(6.0f));

        CHECK(d.bones[1].name == "spine");
        CHECK(d.bones[1].parent == 0);
        CHECK(d.bones[1].pos[1] == doctest::Approx(20.0f));
        CHECK(d.bones[1].rot[1] == doctest::Approx(50.0f));
    }

    TEST_CASE("bone name filling the whole 32-byte field is not over-read") {
        MDLBuilder mb;
        std::vector<MDLBone> bones(1);
        const std::string full(32, 'a'); // no room for a NUL terminator
        set_fixed(bones[0].name, 32, full);

        mb.hdr.numbones = 1;
        mb.hdr.boneindex = mb.append(bones);
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        REQUIRE(parser.get_data().bones.size() == 1);
        CHECK(parser.get_data().bones[0].name == full);
    }

    TEST_CASE("bone array reaching past the buffer is ignored") {
        MDLBuilder mb;
        mb.hdr.numbones = 4;
        mb.hdr.boneindex = (int32_t)mb.blob.size(); // no bone data actually appended
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        CHECK(parser.get_data().bones.empty());
    }

    // ---------- hitboxes ----------

    TEST_CASE("parses hitbox bone, group and bounds") {
        MDLBuilder mb;
        std::vector<MDLHitbox> boxes(2);
        boxes[0].bone = 3;
        boxes[0].group = 1; // head
        boxes[0].bbmin[0] = -1.0f; boxes[0].bbmin[1] = -2.0f; boxes[0].bbmin[2] = -3.0f;
        boxes[0].bbmax[0] = 1.0f;  boxes[0].bbmax[1] = 2.0f;  boxes[0].bbmax[2] = 3.0f;
        boxes[1].bone = 0;
        boxes[1].group = 0;

        mb.hdr.numhitboxes = 2;
        mb.hdr.hitboxindex = mb.append(boxes);
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        const MDLData &d = parser.get_data();

        REQUIRE(d.hitboxes.size() == 2);
        CHECK(d.hitboxes[0].bone == 3);
        CHECK(d.hitboxes[0].group == 1);
        CHECK(d.hitboxes[0].bbmin[2] == doctest::Approx(-3.0f));
        CHECK(d.hitboxes[0].bbmax[1] == doctest::Approx(2.0f));
        CHECK(d.hitboxes[1].group == 0);
    }

    TEST_CASE("hitbox array reaching past the buffer is ignored") {
        MDLBuilder mb;
        mb.hdr.numhitboxes = 8;
        mb.hdr.hitboxindex = (int32_t)mb.blob.size();
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        CHECK(parser.get_data().hitboxes.empty());
    }

    // ---------- textures ----------

    TEST_CASE("decodes indexed texture pixels through the palette") {
        MDLBuilder mb;

        const uint8_t indices[4] = {0, 1, 2, 3};
        uint8_t palette[256 * 3] = {};
        for (int i = 0; i < 4; i++) {
            palette[i * 3 + 0] = (uint8_t)(i * 10);
            palette[i * 3 + 1] = (uint8_t)(i * 20);
            palette[i * 3 + 2] = (uint8_t)(i * 30);
        }

        // Pixels and palette must sit contiguously: the parser reads the palette from
        // pixel_data + width*height.
        int32_t pixels_at = mb.append_bytes(indices, sizeof(indices));
        mb.append_bytes(palette, sizeof(palette));

        std::vector<MDLTexture> texs(1);
        set_fixed(texs[0].name, 64, "skin.bmp");
        texs[0].width = 2;
        texs[0].height = 2;
        texs[0].flags = 0;
        texs[0].index = pixels_at;

        mb.hdr.numtextures = 1;
        mb.hdr.textureindex = mb.append(texs);
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        const MDLData &d = parser.get_data();

        REQUIRE(d.textures.size() == 1);
        CHECK(d.textures[0].name == "skin.bmp");
        CHECK(d.textures[0].width == 2);
        CHECK(d.textures[0].height == 2);
        REQUIRE(d.textures[0].data.size() == 4 * 4); // RGBA

        for (int i = 0; i < 4; i++) {
            CHECK(d.textures[0].data[i * 4 + 0] == (uint8_t)(i * 10));
            CHECK(d.textures[0].data[i * 4 + 1] == (uint8_t)(i * 20));
            CHECK(d.textures[0].data[i * 4 + 2] == (uint8_t)(i * 30));
            CHECK(d.textures[0].data[i * 4 + 3] == 255); // always opaque
        }
    }

    TEST_CASE("implausible texture dimensions keep metadata but decode no pixels") {
        MDLBuilder mb;
        std::vector<MDLTexture> texs(1);
        set_fixed(texs[0].name, 64, "huge.bmp");
        texs[0].width = 8192; // over the 4096 cap
        texs[0].height = 8192;
        texs[0].index = 0;

        mb.hdr.numtextures = 1;
        mb.hdr.textureindex = mb.append(texs);
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        const MDLData &d = parser.get_data();

        REQUIRE(d.textures.size() == 1);
        CHECK(d.textures[0].name == "huge.bmp");
        CHECK(d.textures[0].data.empty());
    }

    TEST_CASE("texture whose pixel data runs past the buffer decodes no pixels") {
        MDLBuilder mb;
        std::vector<MDLTexture> texs(1);
        set_fixed(texs[0].name, 64, "trunc.bmp");
        texs[0].width = 64;
        texs[0].height = 64;
        texs[0].index = 16; // nowhere near enough room for 4096 px + a 768-byte palette

        mb.hdr.numtextures = 1;
        mb.hdr.textureindex = mb.append(texs);
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        REQUIRE(parser.get_data().textures.size() == 1);
        CHECK(parser.get_data().textures[0].data.empty());
    }

    // ---------- skins ----------

    TEST_CASE("parses the skin lookup table") {
        MDLBuilder mb;
        std::vector<int16_t> table = {0, 1, 2, 3, 4, 5}; // 3 refs x 2 families

        mb.hdr.numskinref = 3;
        mb.hdr.numskinfamilies = 2;
        mb.hdr.skinindex = mb.append(table);
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        const MDLData &d = parser.get_data();

        CHECK(d.num_skin_ref == 3);
        CHECK(d.num_skin_families == 2);
        REQUIRE(d.skin_table.size() == 6);
        CHECK(d.skin_table[0] == 0);
        CHECK(d.skin_table[5] == 5);
    }

    TEST_CASE("skin counts are recorded even when the table is absent") {
        MDLBuilder mb;
        mb.hdr.numskinref = 0;
        mb.hdr.numskinfamilies = 0;
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        CHECK(parser.get_data().num_skin_ref == 0);
        CHECK(parser.get_data().skin_table.empty());
    }

    TEST_CASE("oversized skin table is rejected by the sanity cap") {
        MDLBuilder mb;
        // 300 * 300 = 90000, past the parser's 65536 element cap.
        mb.hdr.numskinref = 300;
        mb.hdr.numskinfamilies = 300;
        mb.hdr.skinindex = (int32_t)mb.blob.size();
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        CHECK(parser.get_data().skin_table.empty());
    }

    // ---------- bodyparts ----------

    TEST_CASE("parses submodel vertices, normals and bone assignments") {
        MDLBuilder mb;

        std::vector<float> verts = {1, 2, 3, 4, 5, 6};   // 2 vertices
        std::vector<float> norms = {0, 0, 1, 0, 1, 0};   // 2 normals
        std::vector<uint8_t> vbone = {0, 1};
        std::vector<uint8_t> nbone = {1, 0};

        MDLModel model{};
        set_fixed(model.name, 64, "body");
        model.numverts = 2;
        model.vertindex = mb.append(verts);
        model.numnorms = 2;
        model.normindex = mb.append(norms);
        model.vertinfoindex = mb.append(vbone);
        model.norminfoindex = mb.append(nbone);
        model.nummesh = 0;

        MDLBodyPart bp{};
        set_fixed(bp.name, 64, "torso");
        bp.nummodels = 1;
        bp.modelindex = mb.append(std::vector<MDLModel>{model});

        mb.hdr.numbodyparts = 1;
        mb.hdr.bodypartindex = mb.append(std::vector<MDLBodyPart>{bp});
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        const MDLData &d = parser.get_data();

        REQUIRE(d.bodyparts.size() == 1);
        CHECK(d.bodyparts[0].name == "torso");
        REQUIRE(d.bodyparts[0].models.size() == 1);

        const auto &sm = d.bodyparts[0].models[0];
        CHECK(sm.name == "body");
        REQUIRE(sm.vertices.size() == 6);
        CHECK(sm.vertices[3] == doctest::Approx(4.0f));
        REQUIRE(sm.normals.size() == 6);
        CHECK(sm.normals[2] == doctest::Approx(1.0f));
        REQUIRE(sm.vert_bone.size() == 2);
        CHECK(sm.vert_bone[1] == 1);
        REQUIRE(sm.norm_bone.size() == 2);
        CHECK(sm.norm_bone[0] == 1);
    }

    // Builds a one-mesh model around a caller-supplied triangle command stream, so the
    // strip/fan cases below differ only in the command they encode.
    auto build_tri_model = [](const std::vector<uint8_t> &cmds, int16_t skinref,
                              int tex_w, int tex_h) {
        MDLBuilder mb;

        if (tex_w > 0) {
            // A texture is needed for the UV divide; give it real pixels + palette so
            // parse_textures accepts it.
            std::vector<uint8_t> px((size_t)tex_w * tex_h, 0);
            uint8_t palette[256 * 3] = {};
            int32_t pixels_at = mb.append_bytes(px.data(), px.size());
            mb.append_bytes(palette, sizeof(palette));

            std::vector<MDLTexture> texs(1);
            set_fixed(texs[0].name, 64, "skin.bmp");
            texs[0].width = tex_w;
            texs[0].height = tex_h;
            texs[0].index = pixels_at;
            mb.hdr.numtextures = 1;
            mb.hdr.textureindex = mb.append(texs);
        }

        MDLMesh mesh{};
        mesh.skinref = skinref;
        mesh.triindex = mb.append_bytes(cmds.data(), cmds.size());

        MDLModel model{};
        set_fixed(model.name, 64, "body");
        model.nummesh = 1;
        model.meshindex = mb.append(std::vector<MDLMesh>{mesh});

        MDLBodyPart bp{};
        set_fixed(bp.name, 64, "torso");
        bp.nummodels = 1;
        bp.modelindex = mb.append(std::vector<MDLModel>{model});

        mb.hdr.numbodyparts = 1;
        mb.hdr.bodypartindex = mb.append(std::vector<MDLBodyPart>{bp});
        return mb.finish();
    };

    TEST_CASE("expands a triangle strip with alternating winding") {
        // 4 strip vertices -> 2 triangles. GoldSrc flips the first two indices on odd
        // triangles to keep the winding consistent.
        std::vector<TriVertRaw> verts = {
            {10, 100, 0, 0}, {11, 101, 0, 0}, {12, 102, 0, 0}, {13, 103, 0, 0}};
        auto blob = build_tri_model(encode_tri_cmds(4, verts), 0, 0, 0);

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        const auto &meshes = parser.get_data().bodyparts.at(0).models.at(0).meshes;
        REQUIRE(meshes.size() == 1);

        const auto &tv = meshes[0].triangle_verts;
        REQUIRE(tv.size() == 6); // 2 triangles

        // v=2 (even): v0, v1, v2
        CHECK(tv[0].vertex_index == 10);
        CHECK(tv[1].vertex_index == 11);
        CHECK(tv[2].vertex_index == 12);
        // v=3 (odd): v2, v1, v3 — first two swapped
        CHECK(tv[3].vertex_index == 12);
        CHECK(tv[4].vertex_index == 11);
        CHECK(tv[5].vertex_index == 13);

        CHECK(tv[0].normal_index == 100);
        CHECK(tv[5].normal_index == 103);
    }

    TEST_CASE("expands a triangle fan") {
        // Negative command count = fan: every triangle shares vertex 0.
        std::vector<TriVertRaw> verts = {
            {20, 200, 0, 0}, {21, 201, 0, 0}, {22, 202, 0, 0}, {23, 203, 0, 0}};
        auto blob = build_tri_model(encode_tri_cmds(-4, verts), 0, 0, 0);

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        const auto &tv = parser.get_data().bodyparts.at(0).models.at(0).meshes.at(0).triangle_verts;
        REQUIRE(tv.size() == 6);

        CHECK(tv[0].vertex_index == 20);
        CHECK(tv[1].vertex_index == 21);
        CHECK(tv[2].vertex_index == 22);
        // Second triangle fans off the same hub vertex.
        CHECK(tv[3].vertex_index == 20);
        CHECK(tv[4].vertex_index == 22);
        CHECK(tv[5].vertex_index == 23);
    }

    TEST_CASE("texture coords are divided by the skin texture dimensions") {
        // Texel coords are stored as integers and normalised against the mesh's skin.
        // This is why parse_textures has to run before parse_bodyparts.
        std::vector<TriVertRaw> verts = {
            {0, 0, 32, 16}, {1, 1, 64, 32}, {2, 2, 0, 0}};
        auto blob = build_tri_model(encode_tri_cmds(3, verts), 0, 64, 32);

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        const auto &tv = parser.get_data().bodyparts.at(0).models.at(0).meshes.at(0).triangle_verts;
        REQUIRE(tv.size() == 3);

        CHECK(tv[0].s == doctest::Approx(32.0f / 64.0f));
        CHECK(tv[0].t == doctest::Approx(16.0f / 32.0f));
        CHECK(tv[1].s == doctest::Approx(1.0f));
        CHECK(tv[2].s == doctest::Approx(0.0f));
    }

    TEST_CASE("an empty triangle command stream yields no triangles") {
        auto blob = build_tri_model(std::vector<uint8_t>{}, 0, 0, 0);

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        CHECK(parser.get_data().bodyparts.at(0).models.at(0).meshes.at(0).triangle_verts.empty());
    }

    // ---------- sequences ----------

    TEST_CASE("parses sequence metadata") {
        MDLBuilder mb;
        std::vector<MDLSequenceDesc> seqs(1);
        set_fixed(seqs[0].label, 32, "walk");
        seqs[0].fps = 24.0f;
        seqs[0].numframes = 5;
        seqs[0].flags = 1; // looping
        seqs[0].linearmovement[0] = 7.5f;
        seqs[0].seqgroup = 0;
        seqs[0].animindex = 0; // no bones, so decode_animation bails immediately

        mb.hdr.numseq = 1;
        mb.hdr.seqindex = mb.append(seqs);
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        const MDLData &d = parser.get_data();

        REQUIRE(d.sequences.size() == 1);
        CHECK(d.sequences[0].name == "walk");
        CHECK(d.sequences[0].fps == doctest::Approx(24.0f));
        CHECK(d.sequences[0].num_frames == 5);
        CHECK(d.sequences[0].flags == 1);
        CHECK(d.sequences[0].linear_movement[0] == doctest::Approx(7.5f));
    }

    TEST_CASE("sequences in a non-zero seqgroup carry no decoded frames") {
        // seqgroup != 0 means the animation lives in an external .mdl sequence file that
        // this parser never opens, so metadata is kept but frames stay empty.
        float value[6] = {0, 0, 0, 0, 0, 0};
        float scale[6] = {1, 1, 1, 1, 1, 1};
        std::vector<uint8_t> channels[6];
        channels[0] = encode_anim_channel({{1, 2, {5}}});

        MDLBuilder mb;
        MDLBone bone{};
        set_fixed(bone.name, 32, "root");
        bone.parent = -1;
        for (int i = 0; i < 6; i++) { bone.value[i] = value[i]; bone.scale[i] = scale[i]; }
        mb.hdr.numbones = 1;
        mb.hdr.boneindex = mb.append(std::vector<MDLBone>{bone});

        int32_t animindex = mb.append_anim_1bone(channels);

        std::vector<MDLSequenceDesc> seqs(1);
        set_fixed(seqs[0].label, 32, "external");
        seqs[0].numframes = 2;
        seqs[0].animindex = animindex;
        seqs[0].seqgroup = 1; // external
        mb.hdr.numseq = 1;
        mb.hdr.seqindex = mb.append(seqs);
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        REQUIRE(parser.get_data().sequences.size() == 1);
        CHECK(parser.get_data().sequences[0].name == "external");
        CHECK(parser.get_data().sequences[0].frames.empty());
    }

    // ---------- decode_animation ----------

    TEST_CASE("channels with no animation data hold the bone's rest pose") {
        float value[6] = {10, 20, 30, 0.1f, 0.2f, 0.3f};
        float scale[6] = {1, 1, 1, 1, 1, 1};
        std::vector<uint8_t> channels[6]; // all absent -> offset 0 for every channel

        auto blob = make_anim_mdl(value, scale, 2, channels);
        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));

        const auto &frames = parser.get_data().sequences.at(0).frames;
        REQUIRE(frames.size() == 2);
        for (const auto &f : frames) {
            REQUIRE(f.bone_pos.size() == 3);
            CHECK(f.bone_pos[0] == doctest::Approx(10.0f));
            CHECK(f.bone_pos[1] == doctest::Approx(20.0f));
            CHECK(f.bone_pos[2] == doctest::Approx(30.0f));
            CHECK(f.bone_rot[0] == doctest::Approx(0.1f));
            CHECK(f.bone_rot[2] == doctest::Approx(0.3f));
        }
    }

    TEST_CASE("applies the bone's per-channel scale to animation deltas") {
        float value[6] = {10, 20, 30, 0.1f, 0.2f, 0.3f};
        float scale[6] = {0.5f, 1, 1, 0.01f, 1, 1};

        std::vector<uint8_t> channels[6];
        channels[0] = encode_anim_channel({{3, 3, {2, 4, 6}}});       // pos x
        channels[3] = encode_anim_channel({{3, 3, {100, 200, 300}}}); // rot x

        auto blob = make_anim_mdl(value, scale, 3, channels);
        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));

        const auto &frames = parser.get_data().sequences.at(0).frames;
        REQUIRE(frames.size() == 3);

        // value = base + delta * scale
        CHECK(frames[0].bone_pos[0] == doctest::Approx(11.0f));
        CHECK(frames[1].bone_pos[0] == doctest::Approx(12.0f));
        CHECK(frames[2].bone_pos[0] == doctest::Approx(13.0f));

        CHECK(frames[0].bone_rot[0] == doctest::Approx(1.1f));
        CHECK(frames[1].bone_rot[0] == doctest::Approx(2.1f));
        CHECK(frames[2].bone_rot[0] == doctest::Approx(3.1f));

        // Untouched channels still sit at the rest pose.
        CHECK(frames[1].bone_pos[1] == doctest::Approx(20.0f));
        CHECK(frames[1].bone_rot[2] == doctest::Approx(0.3f));
    }

    TEST_CASE("frames past the valid count repeat the last value") {
        float value[6] = {10, 0, 0, 0, 0, 0};
        float scale[6] = {0.5f, 1, 1, 1, 1, 1};

        // 4 frames but only 2 stored values: frames 2 and 3 clamp to the second one.
        std::vector<uint8_t> channels[6];
        channels[0] = encode_anim_channel({{2, 4, {2, 4}}});

        auto blob = make_anim_mdl(value, scale, 4, channels);
        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));

        const auto &frames = parser.get_data().sequences.at(0).frames;
        REQUIRE(frames.size() == 4);
        CHECK(frames[0].bone_pos[0] == doctest::Approx(11.0f));
        CHECK(frames[1].bone_pos[0] == doctest::Approx(12.0f));
        CHECK(frames[2].bone_pos[0] == doctest::Approx(12.0f));
        CHECK(frames[3].bone_pos[0] == doctest::Approx(12.0f));
    }

    TEST_CASE("a span with zero valid entries yields the rest pose") {
        float value[6] = {10, 0, 0, 0, 0, 0};
        float scale[6] = {0.5f, 1, 1, 1, 1, 1};

        std::vector<uint8_t> channels[6];
        channels[0] = encode_anim_channel({{0, 3, {}}}); // no stored values at all

        auto blob = make_anim_mdl(value, scale, 3, channels);
        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));

        const auto &frames = parser.get_data().sequences.at(0).frames;
        REQUIRE(frames.size() == 3);
        for (const auto &f : frames)
            CHECK(f.bone_pos[0] == doctest::Approx(10.0f));
    }

    TEST_CASE("consecutive spans cover successive frames") {
        float value[6] = {10, 0, 0, 0, 0, 0};
        float scale[6] = {0.5f, 1, 1, 1, 1, 1};

        // Span 1: 2 frames from a single value (clamped). Span 2: 2 distinct frames.
        std::vector<uint8_t> channels[6];
        channels[0] = encode_anim_channel({{1, 2, {2}}, {2, 2, {6, 8}}});

        auto blob = make_anim_mdl(value, scale, 4, channels);
        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));

        const auto &frames = parser.get_data().sequences.at(0).frames;
        REQUIRE(frames.size() == 4);
        CHECK(frames[0].bone_pos[0] == doctest::Approx(11.0f));
        CHECK(frames[1].bone_pos[0] == doctest::Approx(11.0f));
        CHECK(frames[2].bone_pos[0] == doctest::Approx(13.0f));
        CHECK(frames[3].bone_pos[0] == doctest::Approx(14.0f));
    }

    TEST_CASE("animation table reaching past the buffer leaves frames empty") {
        MDLBuilder mb;
        MDLBone bone{};
        set_fixed(bone.name, 32, "root");
        mb.hdr.numbones = 1;
        mb.hdr.boneindex = mb.append(std::vector<MDLBone>{bone});

        std::vector<MDLSequenceDesc> seqs(1);
        set_fixed(seqs[0].label, 32, "bad");
        seqs[0].numframes = 4;
        seqs[0].animindex = 0x7000000; // way past the end
        mb.hdr.numseq = 1;
        mb.hdr.seqindex = mb.append(seqs);
        auto blob = mb.finish();

        MDLParser parser;
        REQUIRE(parser.parse(blob.data(), blob.size()));
        REQUIRE(parser.get_data().sequences.size() == 1);
        CHECK(parser.get_data().sequences[0].frames.empty());
    }
}
