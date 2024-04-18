#pragma once

#include "types.hpp"

#include <tsl/robin_map.h>
#include <tsl/robin_set.h>

#include <span>


namespace urukrama {

template <typename T>
class Graph {
public:
    Graph(std::span<const Point<T>> points, const size_t R, const size_t L);

    Graph(const Graph&) = delete;
    Graph(Graph&&) = delete;
    Graph& operator=(const Graph&) = delete;
    Graph& operator=(Graph&&) = delete;

    ~Graph() = default;

private:
    template <typename K>
    using HashSet = tsl::robin_set<K>;

    template <typename K, typename V>
    using HashMap = tsl::robin_map<K, V>;

    using GreedySearchResult = std::tuple<std::vector<std::pair<T, size_t>>, std::vector<std::pair<T, size_t>>>;

private:
    void Init();
    size_t ProcessPoints(size_t s_idx, float alpha);
    size_t FindMedoid();
    GreedySearchResult GreedySearch(size_t s_idx, const Point<T>& query, size_t k);
    void RobustPrune(size_t p_idx, const std::vector<std::pair<T, size_t>>& candidates, float alpha);

    static T Distance(const Point<T>& a, const Point<T>& y);

private:
    const size_t m_R;
    const size_t m_L;
    const size_t m_dimension;
    const std::span<const Point<T>> m_points;
    HashMap<size_t, HashSet<size_t>> m_n_out;
};

}  // namespace urukrama
