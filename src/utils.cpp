#include "utils.hpp"

#include "lib/logger/logger.hpp"

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <ios>

namespace urukrama {

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
std::tuple<std::vector<float>, size_t, size_t> FVecsRead(const char* fname)
{
    std::ifstream file(fname, std::ios::binary);
    if (!file.is_open()) {
        ksp::log::Error("Could not open {}", fname);
        throw std::runtime_error(std::format("could not open {}", fname));
    }

    auto d = [&] {
        int d{};
        file.read(reinterpret_cast<char*>(&d), sizeof(int));
        assert((d > 0 && d < 1000000) || !"unreasonable dimension");
        return size_t(d);
    }();

    struct stat st {};
    stat(fname, &st);
    size_t sz = st.st_size;
    assert(sz % ((d + 1) * 4) == 0 || !"weird file size");
    size_t n = sz / ((d + 1) * 4);

    std::vector<float> x(n * (d + 1));
    file.read(reinterpret_cast<char*>(x.data()), std::streamsize(n * (d + 1) * sizeof(float)));

    const std::span x_span = x;

    // shift array to remove row headers
    for (size_t i = 0; i < n; i++) {
        std::memmove(x_span.subspan(i * d).data(), x_span.subspan(1 + i * (d + 1)).data(), d * sizeof(float));
    }

    x.resize(n * d);

    return std::make_tuple(std::move(x), d, n);
}
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

}  // namespace urukrama