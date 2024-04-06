#include "graph.hpp"
#include "utils.hpp"

#include <iostream>
#include <random>
#include <ranges>

namespace urukrama {

template <CDistance Distance>
GraphConstructor<Distance>::GraphConstructor(std::span<Point<T>> points, const size_t R): m_R(R), m_points(points)
{
    Init();
}


template <CDistance Distance>
void GraphConstructor<Distance>::Init()
{
    std::mt19937_64 random_engine{std::random_device{}()};

    for (size_t idx = 0; idx < m_points.size(); ++idx) {
        auto indices = FisherYatesShuffle(m_R, m_points.size(), random_engine);

        for (const auto other_idx: indices) {
            m_n_out[idx].push_back(other_idx);
        }
    }
}

}  // namespace urukrama