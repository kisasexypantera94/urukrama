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

template <typename K, typename V>
using HashPair = boost::hash<std::pair<K, V>>;

std::vector<Point<float>> FVecsRead(const char* fname);

}  // namespace urukrama