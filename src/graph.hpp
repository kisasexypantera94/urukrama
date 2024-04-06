#pragma once

#include "distance.hpp"
#include "types.hpp"

#include <unordered_map>

namespace urukrama {

template <CDistance Distance>
class GraphConstructor {
public:
    using T = Distance::Type;

public:
    GraphConstructor(std::span<Point<T>> points, const size_t R);

private:
    void Init();

private:
    const size_t m_R;
    std::span<Point<T>> m_points;
    std::unordered_map<size_t, std::vector<size_t>> m_n_out;  // TODO: robin_map
};

}  // namespace urukrama

#include "graph.tpp"