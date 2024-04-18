#pragma once

#include "types.hpp"

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
    using GreedySearchResult = std::tuple<std::vector<std::pair<T, size_t>>, std::vector<std::pair<T, size_t>>>;

private:
    [[nodiscard]] std::vector<HashSet<size_t>> InitNeighbors() const;

    void BuildIndexInBatches(size_t s_idx) const;

    [[nodiscard]] size_t FindMedoid() const;

    [[nodiscard]] GreedySearchResult GreedySearch(size_t s_idx,
                                                  const Point<T>& query,
                                                  const std::vector<HashSet<size_t>>& n_out,
                                                  size_t k) const;

    [[nodiscard]] HashSet<size_t> RobustPrune(size_t p_idx,
                                              const std::vector<std::pair<T, size_t>>& candidates,
                                              float alpha) const;

    static T Distance(const Point<T>& a, const Point<T>& y);

private:
    const size_t m_R;
    const size_t m_L;
    const size_t m_dimension;
    const std::span<const Point<T>> m_points;
};

}  // namespace urukrama
