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

std::vector<size_t> FisherYatesShuffle(std::size_t size, std::size_t max_size, std::mt19937_64& gen)
{
    assert(size <= max_size);

    std::vector<size_t> res(size);

    for (std::size_t i = 0; i != max_size; ++i) {
        std::uniform_int_distribution<> dis(0, i);
        std::size_t j = dis(gen);
        if (j < res.size()) {
            if (i < res.size()) {
                res[i] = res[j];
            }
            res[j] = i;
        }
    }

    return res;
}

std::vector<Point<float>> FVecsRead(const char* fname)
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

    float* x = new float[n * (d + 1)];
    size_t nr = fread(x, sizeof(float), n * (d + 1), f);
    assert(nr == n * (d + 1) || !"could not read whole file");

    std::vector<Point<float>> points;
    points.reserve(n);

    // shift array to remove row headers
    for (size_t i = 0; i < n; i++) {
        points.push_back(Eigen::Map<Point<float>>(x + 1 + i * (d + 1), d));
    }

    return points;
}

}  // namespace urukrama