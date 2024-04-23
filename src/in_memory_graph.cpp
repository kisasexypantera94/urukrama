#include "in_memory_graph.hpp"
#include "bounded_sorted_vector.hpp"
#include "types.hpp"

#include "lib/logger/logger.hpp"

#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>

#include <algorithm>
#include <random>
#include <ranges>


namespace urukrama {

template <typename T>
InMemoryGraph<T>::InMemoryGraph(std::vector<Point<T>>&& points, size_t R, size_t L)
    : m_R(R)
    , m_L(L)
    , m_points(std::move(points))
    , m_medoid_idx(FindMedoid())
    , m_n_out(BuildIndexInBatches(m_medoid_idx))
{
}

template <typename T>
std::vector<HashSet<size_t>> InMemoryGraph<T>::InitNeighbors() const
{
    std::vector<HashSet<size_t>> n_out(m_points.size());

    std::mt19937_64 random_engine{std::random_device{}()};
    std::uniform_int_distribution<size_t> dis(0, m_points.size() - 1);

    for (size_t idx = 0; idx < m_points.size(); ++idx) {
        while (n_out[idx].size() < m_R) {
            if (auto other_idx = dis(random_engine); other_idx != idx) {
                n_out[idx].insert(other_idx);
            }
        }
    }

    return n_out;
}

template <typename T>
std::vector<HashSet<size_t>> InMemoryGraph<T>::BuildIndexInBatches(size_t s_idx) const
{
    auto n_out = InitNeighbors();
    auto prev_n_out = n_out;

    for (const float alpha: {1.0f, 1.2f}) {
        for (size_t start = 0, end = start; start < m_points.size(); start = end + 1,
                    end = std::min({start * 2, start + size_t(float(m_points.size()) * 0.02), m_points.size() - 1})) {
            std::atomic<size_t> cnt_found = 0;

            tbb::parallel_for(tbb::blocked_range<size_t>(start, end + 1), [&](tbb::blocked_range<size_t> r) {
                for (size_t p_idx = r.begin(); p_idx < r.end(); ++p_idx) {
                    const auto [top, visited] = GreedySearch(s_idx, m_points[p_idx], prev_n_out, 1);
                    cnt_found += (top.begin()->second == p_idx);

                    n_out[p_idx] = RobustPrune(p_idx, visited, alpha);
                }
            });

            const auto [affected_points, affected_points_indices] = [&] {
                HashMap<size_t, HashSet<size_t>> affected_points;
                std::vector<size_t> affected_points_indices;

                for (size_t p_idx = start; p_idx <= end; ++p_idx) {
                    for (const size_t n_idx: n_out[p_idx]) {
                        if (not affected_points.contains(n_idx)) {
                            affected_points_indices.emplace_back(n_idx);
                        }

                        affected_points[n_idx].insert(p_idx);
                    }
                }

                return std::make_pair(std::move(affected_points), std::move(affected_points_indices));
            }();

            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, affected_points_indices.size()),
                [&, &affected_points_indices = affected_points_indices, &affected_points = affected_points](
                    tbb::blocked_range<size_t> r) {
                    for (size_t i = r.begin(); i < r.end(); ++i) {
                        const auto& b_idx = affected_points_indices[i];

                        for (const size_t p_idx: affected_points.at(b_idx)) {
                            n_out[b_idx].insert(p_idx);
                        }

                        if (n_out[b_idx].size() > m_R) {
                            std::vector<std::pair<T, size_t>> candidates;
                            candidates.reserve(n_out[b_idx].size());

                            for (const size_t n_idx: n_out[b_idx]) {
                                candidates.emplace_back(Distance(m_points[b_idx], m_points[n_idx]), n_idx);
                            }

                            std::sort(candidates.begin(), candidates.end());

                            n_out[b_idx] = RobustPrune(b_idx, candidates, alpha);
                        }

                        prev_n_out[b_idx] = n_out[b_idx];
                    }
                });

            tbb::parallel_for(tbb::blocked_range<size_t>(start, end + 1), [&](tbb::blocked_range<size_t> r) {
                for (size_t p_idx = r.begin(); p_idx < r.end(); ++p_idx) {
                    prev_n_out[p_idx] = n_out[p_idx];
                }
            });

            ksp::log::Info("Processed batch: range=[{}..{}], precision=[{}]",
                           start,
                           end,
                           float(cnt_found) / float(end - start + 1));
        }
    }

    return n_out;
}

template <typename T>
T InMemoryGraph<T>::Distance(const Point<T>& a, const Point<T>& b)
{
    return (a - b).squaredNorm();
}

template <typename T>
size_t InMemoryGraph<T>::FindMedoid() const
{
    Point<T> centroid(m_points.front().size());

    for (const auto& [p_idx, p]: m_points | std::views::enumerate) {
        centroid += (p - centroid) / (p_idx + 1);
        centroid = centroid.unaryExpr([](const T x) { return std::isfinite(x) ? x : T(0); });
    }

    const auto medoid_it = std::ranges::min_element(m_points, {}, [&](const auto& p) { return Distance(centroid, p); });

    ksp::log::Info("Found medoid: idx=[{}], dist=[{}]",
                   std::distance(m_points.begin(), medoid_it),
                   Distance(centroid, *medoid_it));

    return std::distance(m_points.begin(), medoid_it);
}

template <typename T>
InMemoryGraph<T>::GreedySearchResult InMemoryGraph<T>::GreedySearch(size_t s_idx,
                                                                    const Point<T>& query,
                                                                    const std::vector<HashSet<size_t>>& n_out,
                                                                    size_t k) const
{
    HashSet<size_t> fast_visited;
    fast_visited.reserve(m_L * 2);

    std::vector<std::pair<T, size_t>> visited;
    visited.reserve(m_L * 2);

    BoundedSortedVector<T, size_t> candidates(m_L);
    candidates.reserve(m_L + 1);
    candidates.emplace(Distance(m_points[s_idx], query), s_idx);

    while (true) {
        auto it = std::find_if(candidates.begin(), candidates.end(), [&](const auto& c) {
            return not fast_visited.contains(c.second);
        });

        if (it == candidates.end()) {
            break;
        }

        const auto [min_distance, p_star_idx] = *it;

        fast_visited.insert(p_star_idx);
        visited.emplace_back(min_distance, p_star_idx);

        for (const size_t n_idx: n_out[p_star_idx]) {
            if (not fast_visited.contains(n_idx)) {
                candidates.emplace(Distance(m_points[n_idx], query), n_idx);
            }
        }
    }

    std::sort(visited.begin(), visited.end());

    return std::make_pair(std::move(candidates), std::move(visited));
}

template <typename T>
HashSet<size_t> InMemoryGraph<T>::RobustPrune(size_t p_idx,
                                              const std::vector<std::pair<T, size_t>>& candidates,
                                              float alpha) const
{
    HashSet<size_t> skip;

    HashSet<size_t> new_n_out;
    new_n_out.reserve(m_R);

    for (const auto [i, p_star_idx]: candidates | std::views::values | std::views::enumerate) {
        if (p_star_idx == p_idx or skip.contains(p_star_idx)) {
            continue;
        }

        new_n_out.insert(p_star_idx);

        if (new_n_out.size() == m_R) {
            break;
        }

        for (const auto [p_to_p_hat_dist, p_hat_idx]: candidates | std::views::drop(i + 1)) {
            if (not skip.contains(p_hat_idx)) {
                const T p_star_to_p_hat_distance = Distance(m_points[p_star_idx], m_points[p_hat_idx]);

                if (alpha * float(p_star_to_p_hat_distance) <= float(p_to_p_hat_dist)) {
                    skip.insert(p_hat_idx);
                }
            }
        }
    }

    return new_n_out;
}

template class InMemoryGraph<float>;

}  // namespace urukrama