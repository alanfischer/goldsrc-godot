#!/usr/bin/env python3
"""
Check for missing textures in a GoldSrc BSP file.

Parses the BSP texture lump and WAD files to find textures that are
neither embedded in the BSP nor available in any WAD file.
"""

import struct
import os
import sys
from collections import defaultdict

# ─── BSP Format Constants ───────────────────────────────────────────────

HLBSP_VERSION = 30

LUMP_ENTITIES   = 0
LUMP_PLANES     = 1
LUMP_TEXTURES   = 2
LUMP_VERTEXES   = 3
LUMP_VISIBILITY = 4
LUMP_NODES      = 5
LUMP_TEXINFO    = 6
LUMP_FACES      = 7
LUMP_LIGHTING   = 8
LUMP_CLIPNODES  = 9
LUMP_LEAFS      = 10
LUMP_MARKSURFACES = 11
LUMP_EDGES      = 12
LUMP_SURFEDGES  = 13
LUMP_MODELS     = 14
MAX_BSP_LUMPS   = 15

# Struct sizes
SIZEOF_LUMP     = 8   # int32 fileofs, int32 filelen
SIZEOF_FACE     = 20  # see BSPFace
SIZEOF_TEXINFO  = 40  # float vecs[2][4], int32 miptex, int32 flags
SIZEOF_MIPTEX   = 40  # char name[16], uint32 w, uint32 h, uint32 offsets[4]


def read_bsp(filepath):
    """Parse a GoldSrc BSP and return texture info and face-to-texture mapping."""
    with open(filepath, 'rb') as f:
        data = f.read()

    # Header: version + 15 lumps
    version = struct.unpack_from('<i', data, 0)[0]
    if version != HLBSP_VERSION:
        print(f"WARNING: BSP version {version}, expected {HLBSP_VERSION}")

    lumps = []
    for i in range(MAX_BSP_LUMPS):
        ofs = 4 + i * SIZEOF_LUMP
        fileofs, filelen = struct.unpack_from('<ii', data, ofs)
        lumps.append((fileofs, filelen))

    # ─── Parse texture lump ─────────────────────────────────────────
    tex_ofs, tex_len = lumps[LUMP_TEXTURES]
    nummiptex = struct.unpack_from('<i', data, tex_ofs)[0]

    # Read offset table
    miptex_offsets = []
    for i in range(nummiptex):
        off = struct.unpack_from('<i', data, tex_ofs + 4 + i * 4)[0]
        miptex_offsets.append(off)

    textures = []  # list of (name, width, height, has_embedded_data)
    for i, off in enumerate(miptex_offsets):
        if off < 0:
            textures.append((f"<invalid_offset_{i}>", 0, 0, False))
            continue

        abs_off = tex_ofs + off
        miptex_data = data[abs_off:abs_off + SIZEOF_MIPTEX]
        if len(miptex_data) < SIZEOF_MIPTEX:
            textures.append((f"<truncated_{i}>", 0, 0, False))
            continue

        name_bytes = miptex_data[0:16]
        name = name_bytes.split(b'\x00')[0].decode('ascii', errors='replace')
        width, height = struct.unpack_from('<II', miptex_data, 16)
        offsets = struct.unpack_from('<IIII', miptex_data, 24)

        # A texture has embedded data if offsets[0] > 0
        has_data = offsets[0] > 0

        textures.append((name, width, height, has_data))

    # ─── Parse texinfo lump ─────────────────────────────────────────
    ti_ofs, ti_len = lumps[LUMP_TEXINFO]
    num_texinfo = ti_len // SIZEOF_TEXINFO
    texinfo_miptex = []  # texinfo index -> miptex index
    for i in range(num_texinfo):
        base = ti_ofs + i * SIZEOF_TEXINFO
        # vecs is 32 bytes (2*4 floats), then int32 miptex, int32 flags
        miptex_idx = struct.unpack_from('<i', data, base + 32)[0]
        texinfo_miptex.append(miptex_idx)

    # ─── Parse face lump ────────────────────────────────────────────
    face_ofs, face_len = lumps[LUMP_FACES]
    num_faces = face_len // SIZEOF_FACE
    face_textures = []  # face index -> miptex index
    for i in range(num_faces):
        base = face_ofs + i * SIZEOF_FACE
        # BSPFace: int16 planenum, int16 side, int32 firstedge, int16 numedges, int16 texinfo, ...
        texinfo_idx = struct.unpack_from('<h', data, base + 10)[0]
        if 0 <= texinfo_idx < len(texinfo_miptex):
            miptex_idx = texinfo_miptex[texinfo_idx]
            face_textures.append(miptex_idx)
        else:
            face_textures.append(-1)

    return textures, face_textures


def read_wad(filepath):
    """Parse a GoldSrc WAD3 file and return a set of texture names (lowercased)."""
    with open(filepath, 'rb') as f:
        data = f.read()

    if len(data) < 12:
        print(f"  WARNING: WAD file too small: {filepath}")
        return set()

    ident = data[0:4]
    if ident not in (b'WAD2', b'WAD3'):
        print(f"  WARNING: Not a valid WAD file ({ident}): {filepath}")
        return set()

    numlumps, infotableofs = struct.unpack_from('<ii', data, 4)

    names = set()
    SIZEOF_WADLUMP = 32  # int32*3 + char + char + char*2 + char[16]

    for i in range(numlumps):
        entry_ofs = infotableofs + i * SIZEOF_WADLUMP
        if entry_ofs + SIZEOF_WADLUMP > len(data):
            break

        # Parse WADLumpInfo
        # int32 filepos, int32 disksize, int32 size, char type, char compression, char pad1, char pad2, char name[16]
        name_bytes = data[entry_ofs + 16 : entry_ofs + 32]
        name = name_bytes.split(b'\x00')[0].decode('ascii', errors='replace')
        lump_type = data[entry_ofs + 12]

        # Type 0x43 (67) = miptex in WAD3
        # Type 0x44 (68) = font
        # We care about miptex (type 67 = 0x43)
        if lump_type == 0x43:
            names.add(name.lower())

    return names


def parse_entity_lump(filepath):
    """Parse the entity lump to find wad key in worldspawn."""
    with open(filepath, 'rb') as f:
        data = f.read()

    version = struct.unpack_from('<i', data, 0)[0]
    lumps = []
    for i in range(MAX_BSP_LUMPS):
        ofs = 4 + i * SIZEOF_LUMP
        fileofs, filelen = struct.unpack_from('<ii', data, ofs)
        lumps.append((fileofs, filelen))

    ent_ofs, ent_len = lumps[LUMP_ENTITIES]
    ent_text = data[ent_ofs:ent_ofs + ent_len].decode('ascii', errors='replace')

    # Find "wad" key in entity string
    wad_value = None
    for line in ent_text.split('\n'):
        line = line.strip()
        if line.startswith('"wad"'):
            # Format: "wad" "path1;path2;..."
            parts = line.split('"')
            if len(parts) >= 4:
                wad_value = parts[3]
            break

    return wad_value


def main():
    # ─── Find BSP file ──────────────────────────────────────────────
    bsp_candidates = [
        "/Users/afischer/personal/goldsrc-godot/demo/assets/maps/ww_chasm.bsp",
        "/Users/afischer/personal/wizardwars/res/maps/ww_chasm.bsp",
    ]

    bsp_path = None
    for path in bsp_candidates:
        if os.path.exists(path):
            bsp_path = path
            break

    if not bsp_path:
        print("ERROR: Could not find ww_chasm.bsp")
        sys.exit(1)

    print(f"BSP file: {bsp_path}")
    print(f"BSP size: {os.path.getsize(bsp_path)} bytes")
    print()

    # ─── Parse entity lump for WAD references ───────────────────────
    wad_value = parse_entity_lump(bsp_path)
    print(f"Entity lump 'wad' key: {wad_value}")
    print()

    # ─── Find WAD files ─────────────────────────────────────────────
    wad_search_dirs = [
        "/Users/afischer/personal/goldsrc-godot/demo/assets",
        "/Users/afischer/personal/goldsrc-godot/demo/assets/wads",
        "/Users/afischer/personal/wizardwars/res",
    ]

    wad_names = ["wwwad.wad", "halflife.wad", "tfc.wad"]
    wad_files = {}

    for wad_name in wad_names:
        for search_dir in wad_search_dirs:
            candidate = os.path.join(search_dir, wad_name)
            if os.path.exists(candidate):
                wad_files[wad_name] = candidate
                break

    print("WAD files found:")
    for name, path in wad_files.items():
        print(f"  {name}: {path} ({os.path.getsize(path)} bytes)")
    for name in wad_names:
        if name not in wad_files:
            print(f"  {name}: NOT FOUND")
    print()

    # ─── Parse BSP textures ─────────────────────────────────────────
    textures, face_textures = read_bsp(bsp_path)

    print(f"Total miptex entries in BSP: {len(textures)}")
    print(f"Total faces in BSP: {len(face_textures)}")
    print()

    print("=" * 80)
    print("ALL TEXTURES IN BSP:")
    print("=" * 80)
    for i, (name, w, h, has_data) in enumerate(textures):
        status = "EMBEDDED" if has_data else "EXTERNAL (WAD)"
        print(f"  [{i:3d}] {name:24s}  {w:4d}x{h:<4d}  {status}")
    print()

    # ─── Parse WAD files ────────────────────────────────────────────
    wad_texture_names = {}  # wad_name -> set of texture names
    all_wad_names = set()

    for wad_name, wad_path in wad_files.items():
        names = read_wad(wad_path)
        wad_texture_names[wad_name] = names
        all_wad_names.update(names)
        print(f"WAD '{wad_name}': {len(names)} textures")

    print(f"Total unique WAD textures: {len(all_wad_names)}")
    print()

    # ─── Build face index map ───────────────────────────────────────
    # miptex_index -> list of face indices
    miptex_to_faces = defaultdict(list)
    for face_idx, miptex_idx in enumerate(face_textures):
        miptex_to_faces[miptex_idx].append(face_idx)

    # ─── Find missing textures ──────────────────────────────────────
    missing = []
    external_found = []
    embedded = []

    for i, (name, w, h, has_data) in enumerate(textures):
        if has_data:
            embedded.append((i, name))
            continue

        name_lower = name.lower()
        found_in = []
        for wad_name, names in wad_texture_names.items():
            if name_lower in names:
                found_in.append(wad_name)

        if found_in:
            external_found.append((i, name, found_in))
        else:
            face_indices = miptex_to_faces.get(i, [])
            missing.append((i, name, w, h, face_indices))

    # ─── Report ─────────────────────────────────────────────────────
    print("=" * 80)
    print("TEXTURE STATUS SUMMARY")
    print("=" * 80)
    print(f"  Embedded in BSP:          {len(embedded)}")
    print(f"  Found in WAD:             {len(external_found)}")
    print(f"  MISSING (not found):      {len(missing)}")
    print()

    if external_found:
        print("-" * 80)
        print("EXTERNAL TEXTURES (found in WAD):")
        print("-" * 80)
        for i, name, wads in external_found:
            faces = miptex_to_faces.get(i, [])
            print(f"  [{i:3d}] {name:24s}  -> {', '.join(wads)}  ({len(faces)} faces)")
        print()

    if missing:
        print("=" * 80)
        print("MISSING TEXTURES (NOT embedded, NOT in any WAD):")
        print("=" * 80)
        for i, name, w, h, face_indices in missing:
            print(f"\n  [{i:3d}] \"{name}\"  ({w}x{h})")
            print(f"        Used by {len(face_indices)} face(s): {face_indices[:50]}")
            if len(face_indices) > 50:
                print(f"        ... and {len(face_indices) - 50} more")

            # Check which WADs might have a similar name
            name_lower = name.lower()
            for wad_name, names in wad_texture_names.items():
                # Check for partial matches
                partial = [n for n in names if name_lower[:6] in n or n[:6] in name_lower]
                if partial:
                    print(f"        Similar in {wad_name}: {partial[:5]}")
        print()
    else:
        print("No missing textures found! All textures are either embedded or in WAD files.")
        print()

    # ─── Also check: are any faces referencing invalid miptex? ──────
    invalid_refs = [(fi, mi) for fi, mi in enumerate(face_textures)
                    if mi < 0 or mi >= len(textures)]
    if invalid_refs:
        print("=" * 80)
        print(f"FACES WITH INVALID MIPTEX REFERENCES: {len(invalid_refs)}")
        print("=" * 80)
        for face_idx, miptex_idx in invalid_refs[:20]:
            print(f"  Face {face_idx} -> miptex {miptex_idx} (out of range 0..{len(textures)-1})")
        print()

    # ─── Summary of face counts per missing texture ─────────────────
    if missing:
        total_missing_faces = sum(len(fl) for _, _, _, _, fl in missing)
        print("=" * 80)
        print(f"TOTAL FACES AFFECTED BY MISSING TEXTURES: {total_missing_faces} / {len(face_textures)}")
        print("=" * 80)


if __name__ == "__main__":
    main()
