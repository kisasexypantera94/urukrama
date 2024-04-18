#include "graph.hpp"

#include "bounded_sorted_vector.hpp"
#include "types.hpp"

#include "lib/logger/logger.hpp"

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>

#include <algorithm>
#include <iostream>
#include <latch>
#include <random>
#include <ranges>


namespace urukrama {

template <typename T>
Graph<T>::Graph(std::span<const Point<T>> points, const size_t R, const size_t L)
    : m_R(R), m_L(L), m_dimension(points.front().size()), m_points(points)
{
    const size_t s_idx = FindMedoid();

    const auto good = ProcessPoints(s_idx);

    std::cout << good / double(m_points.size()) << std::endl;

    // for (auto [p_idx, edges]: m_n_out) {
    //     for (auto e: edges) {
    //         std::cout << "(" << p_idx << ", " << e << "), " << std::endl;
    //     }
    // }
}


template <typename T>
std::vector<HashSet<size_t>> Graph<T>::InitNeighbors() const
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
size_t Graph<T>::ProcessPoints(size_t s_idx) const
{
    std::atomic<size_t> good = 0;

    boost::asio::thread_pool tp;
    auto n_out = InitNeighbors();

    for (const float alpha: {1.0f, 1.2f}) {
        size_t start = 0;
        size_t end = -1;
        while (end + 1 < m_points.size()) {
            start = end + 1;
            end = std::min(start * 2, start + 20000);
            ksp::log::Info("Batch: [{}..{}]", start, end);

            const auto batch =
                m_points | std::views::enumerate | std::views::drop(start) | std::views::take(end - start + 1);

            good = 0;

            ksp::log::Info("Step 1");
            {
                std::latch latch(batch.size());

                for (const auto& [p_idx, p]: batch) {
                    boost::asio::post(tp, [&, p_idx = p_idx, &p = p] {
                        auto [top, visited] = GreedySearch(s_idx, p, n_out, 1);
                        good += (top.begin()->second == p_idx);

                        // if (p_idx % 1000 == 0) {
                        //     ksp::log::Info("{} {}", p_idx, good.load());
                        // }

                        n_out[p_idx] = RobustPrune(p_idx, std::move(visited), alpha);
                        latch.count_down();
                    });
                }

                latch.wait();
            }

            ksp::log::Info("Step 2");
            HashMap<size_t, HashSet<size_t>> affected_points;
            for (const auto& [p_idx, p]: batch) {
                for (const auto& n_idx: n_out[p_idx]) {
                    affected_points[n_idx].insert(p_idx);
                }
            }

            ksp::log::Info("Step 3");
            {
                std::latch latch(long(affected_points.size()));

                for (const auto& [b_idx, n]: affected_points) {
                    boost::asio::post(tp, [&, &n = n, b_idx = b_idx] {
                        for (const auto& p_idx: n) {
                            n_out[b_idx].insert(p_idx);
                        }

                        if (n_out[b_idx].size() > m_R) {
                            std::vector<std::pair<T, size_t>> candidates;
                            candidates.reserve(n_out[b_idx].size());

                            for (const auto n_idx: n_out[b_idx]) {
                                candidates.emplace_back(Distance(m_points[b_idx], m_points[n_idx]), n_idx);
                            }

                            std::sort(candidates.begin(), candidates.end());

                            RobustPrune(b_idx, std::move(candidates), alpha);
                        }

                        latch.count_down();
                    });
                }

                latch.wait();
            }

            ksp::log::Info("Precision: {}", good / double(batch.size()));
        }
    }

    tp.join();

    return good;
}

template <typename T>
T Graph<T>::Distance(const Point<T>& a, const Point<T>& b)
{
    return (a - b).squaredNorm();
}

template <typename T>
size_t Graph<T>::FindMedoid() const
{
    Point<T> centroid = std::reduce(m_points.begin(), m_points.end(), Point<T>(m_dimension)) / m_points.size();

    const auto medoid_it = std::ranges::min_element(m_points, {}, [&](const auto& p) { return Distance(centroid, p); });

    return std::distance(m_points.begin(), medoid_it);
}

template <typename T>
Graph<T>::GreedySearchResult Graph<T>::GreedySearch(size_t s_idx,
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

        for (const auto n_idx: n_out[p_star_idx]) {
            if (not fast_visited.contains(n_idx)) {
                candidates.emplace(Distance(m_points[n_idx], query), n_idx);
            }
        }
    }

    std::sort(visited.begin(), visited.end());

    return std::make_pair(std::move(candidates), std::move(visited));
}

template <typename T>
HashSet<size_t> Graph<T>::RobustPrune(size_t p_idx,
                                      const std::vector<std::pair<T, size_t>>& candidates,
                                      float alpha) const
{
    HashSet<size_t> skip;
    HashSet<size_t> new_n_out;

    for (const auto& [i, p_star_idx]: candidates | std::views::values | std::views::enumerate) {
        if (p_star_idx == p_idx or skip.contains(p_star_idx)) {
            continue;
        }

        new_n_out.insert(p_star_idx);

        if (new_n_out.size() == m_R) {
            break;
        }

        for (const auto& [p_to_p_hat_dist, p_hat_idx]: candidates | std::views::drop(i + 1)) {
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

template class Graph<float>;
template class Graph<uint8_t>;

}  // namespace urukrama