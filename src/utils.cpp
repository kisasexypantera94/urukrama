#include "utils.hpp"

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace urukrama {

// NOLINTBEGIN
// https://github.com/facebookresearch/faiss/blob/main/demos/demo_sift1M.cpp#L35
std::tuple<std::vector<float>, size_t, size_t> FVecsRead(const char* fname)
{
    FILE* f = fopen(fname, "r");
    if (!f) {
        fprintf(stderr, "could not open %s\n", fname);
        perror("");
        abort();
    }
    int d;
    fread(&d, 1, sizeof(int), f);
    assert((d > 0 && d < 1000000) || !"unreasonable dimension");
    fseek(f, 0, SEEK_SET);
    struct stat st;
    fstat(fileno(f), &st);
    size_t sz = st.st_size;
    assert(sz % ((d + 1) * 4) == 0 || !"weird file size");
    size_t n = sz / ((d + 1) * 4);

    std::vector<float> x(n * (d + 1));
    size_t nr = fread(x.data(), sizeof(float), n * (d + 1), f);
    assert(nr == n * (d + 1) || !"could not read whole file");

    // shift array to remove row headers
    for (size_t i = 0; i < n; i++) {
        memmove(x.data() + i * d, x.data() + 1 + i * (d + 1), d * sizeof(float));
    }

    fclose(f);

    x.resize(n * d);

    return std::make_tuple(std::move(x), d, n);
}
// NOLINTEND

}  // namespace urukrama