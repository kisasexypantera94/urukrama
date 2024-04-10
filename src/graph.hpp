#pragma once

#include "distance.hpp"
#include "types.hpp"
#include "utils.hpp"

#include <tsl/robin_map.h>
#include <tsl/robin_set.h>
#include <boost/container/flat_map.hpp>

#include <set>

namespace urukrama {

template <typename T>
class GraphConstructor {
public:
    GraphConstructor(std::span<const Point<T>> points, const size_t R);

private:
    template <typename K, typename V>
    using FlatMap = boost::container::flat_map<K, V>;

    template <typename K>
    using HashSet = tsl::robin_set<K>;

    template <typename K, typename V>
    using HashMap = tsl::robin_map<K, V>;

    using GreedySearchResult = std::tuple<FlatMap<T, size_t>, HashSet<size_t>>;

private:
    void Init();
    size_t FindMedoid();
    GreedySearchResult GreedySearch(size_t s_idx, const Point<T>& query, size_t k, size_t L);
    void RobustPrune(size_t p_idx, FlatMap<T, size_t>&& candidates, HashSet<size_t>&& visited, float alpha);

    static T Distance(const Point<T>& a, const Point<T>& y);

private:
    const size_t m_R;
    const size_t m_dimension;
    const std::span<const Point<T>> m_points;
    HashMap<size_t, HashSet<size_t>> m_n_out;  // TODO: robin_map
};

}  // namespace urukrama

#include "graph.tpp"