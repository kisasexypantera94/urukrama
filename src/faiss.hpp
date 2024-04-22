#pragma once

#include <cstdint>
#include <vector>

std::tuple<std::vector<float>, std::vector<int64_t>, std::vector<float>> ComputeClusters(
    const std::vector<float>& vecs,
    size_t dim,
    size_t number_of_clusters,
    size_t k = 1,
    size_t number_of_iterations = 20,
    bool verbose = true);

std::vector<uint8_t> ComputeProductQuantization(const std::vector<float>& vecs,
                                                size_t dim,
                                                size_t M = 32,
                                                size_t nbits = 8);