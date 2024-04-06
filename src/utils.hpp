#pragma once

#include <random>
#include <vector>

namespace urukrama {

std::vector<size_t> FisherYatesShuffle(std::size_t size, std::size_t max_size, std::mt19937_64& gen);

}  // namespace urukrama