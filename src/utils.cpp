#include "utils.hpp"

#include <cassert>

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

}  // namespace urukrama