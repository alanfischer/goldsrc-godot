#pragma once
#include <cstdint>
#include <cstring>
#include <vector>

namespace goldsrc {

// Read a single value of type T from a potentially-unaligned byte pointer.
template <typename T>
static inline T read_from(const uint8_t *p) {
    T v;
    memcpy(&v, p, sizeof(v));
    return v;
}

// Copy count values of type T starting at data+offset into a new vector.
// Caller must validate that [offset, offset + count*sizeof(T)) is in-bounds.
template <typename T>
static inline std::vector<T> read_arr(const uint8_t *data, size_t offset, size_t count) {
    std::vector<T> v(count);
    if (count > 0)
        memcpy(v.data(), data + offset, count * sizeof(T));
    return v;
}

} // namespace goldsrc
