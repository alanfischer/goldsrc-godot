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
}
