#pragma once

#include <boost/functional/hash.hpp>

#include <random>
#include <vector>

namespace urukrama {

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