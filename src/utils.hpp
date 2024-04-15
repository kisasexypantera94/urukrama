#pragma once

#include "types.hpp"

#include <boost/functional/hash.hpp>

#include <random>
#include <vector>

namespace urukrama {

std::vector<size_t> FisherYatesShuffle(std::size_t size, std::size_t max_size, std::mt19937_64& gen);

void EraseIf(auto& set, auto pred)
{
    for (auto first = set.begin(), last = set.end(); first != last;) {
        if (pred(*first)) {
            first = set.erase(first);
        } else {
            ++first;
        }
    }
}

std::tuple<std::vector<float>, size_t, size_t> FVecsRead(const char* fname);

}  // namespace urukrama