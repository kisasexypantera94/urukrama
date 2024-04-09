#pragma once

#include "distance.hpp"
#include "types.hpp"

#include <set>
#include <unordered_map>
#include <unordered_set>

namespace urukrama {

template <typename T>
class GraphConstructor {
public:
    GraphConstructor(std::span<const Point<T>> points, const size_t R);

private:
    using GreedySearchResult = std::tuple<std::set<std::pair<T, size_t>>, std::unordered_set<size_t>>;

private:
    void Init();
    size_t FindMedoid();
    GreedySearchResult GreedySearch(size_t s_idx, const Point<T>& xq, size_t k, size_t L) const;
    void RobustPrune(size_t p_idx, std::unordered_set<size_t>&& visited, float alpha);

    static T Distance(const Point<T>& a, const Point<T>& y);

private:
    const size_t m_R;
    const size_t m_dimension;
    const std::span<const Point<T>> m_points;
    std::unordered_map<size_t, std::unordered_set<size_t>> m_n_out;  // TODO: robin_map
};

}  // namespace urukrama

#include "graph.tpp"