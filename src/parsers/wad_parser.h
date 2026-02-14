#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

namespace goldsrc {

struct WADHeader {
	char identification[4]; // WAD2 or WAD3
	int32_t numlumps;
	int32_t infotableofs;
};

struct WADLumpInfo {
	int32_t filepos;
	int32_t disksize;
	int32_t size;
	char type;
	char compression;
	char pad1, pad2;
	char name[16];
};

inline constexpr int MIPLEVELS = 4;

struct WADMipTex {
	char name[16];
	uint32_t width, height;
	uint32_t offsets[MIPLEVELS];
};

struct WADTexture {
	std::string name;
	uint32_t width;
	uint32_t height;
	std::vector<uint8_t> data; // RGBA pixels
	bool has_transparency;
};

class WADParser {
public:
	bool parse(const uint8_t *data, size_t size);

	int get_texture_count() const { return (int)textures.size(); }
	const std::vector<std::string> &get_texture_names() const { return texture_names; }
	bool has_texture(const std::string &name) const;
	const WADTexture *get_texture(const std::string &name) const;

private:
	static std::string to_lower(const std::string &str) {
		std::string temp = str;
		std::transform(temp.begin(), temp.end(), temp.begin(),
			[](unsigned char c) { return std::tolower(c); });
		return temp;
	}

	void decode_texture(const WADMipTex *miptex, const uint8_t *base, size_t lump_size);

	std::vector<std::string> texture_names;
	std::map<std::string, WADTexture> textures;
};

} // namespace goldsrc
