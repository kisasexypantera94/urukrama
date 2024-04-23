#pragma once

#include <vector>

namespace urukrama {

std::tuple<std::vector<float>, size_t, size_t> FVecsRead(const char* fname);

template <typename... Ts>
struct SizeOf {
    static constexpr size_t value = (sizeof(Ts) + ...);
};


}  // namespace urukrama