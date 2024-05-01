#pragma once

#include "types.hpp"


namespace urukrama {

template <typename T>
class InMemoryGraph {
    template <typename, bool>
    friend class OnDiskGraph;

public:
    InMemoryGraph(std::vector<Point<T>>&& points, size_t R, size_t L);

private:
    using GreedySearchResult = std::tuple<std::vector<std::pair<T, size_t>>, std::vector<std::pair<T, size_t>>>;

private:
    [[nodiscard]] size_t FindMedoid() const;

    [[nodiscard]] std::vector<HashSet<size_t>> BuildIndexInBatches(size_t s_idx) const;

    [[nodiscard]] std::vector<HashSet<size_t>> InitNeighbors() const;

    [[nodiscard]] GreedySearchResult GreedySearch(size_t s_idx,
                                                  const Point<T>& query,
                                                  const std::vector<HashSet<size_t>>& n_out,
                                                  size_t k) const;

    [[nodiscard]] HashSet<size_t> RobustPrune(size_t p_idx,
                                              const std::vector<std::pair<T, size_t>>& candidates,
                                              float alpha) const;

    [[nodiscard]] static T Distance(const Point<T>& a, const Point<T>& y);

private:
    size_t m_R;
    size_t m_L;
    std::vector<Point<T>> m_points;

    size_t m_medoid_idx;
    std::vector<HashSet<size_t>> m_n_out;
};

}  // namespace urukrama
