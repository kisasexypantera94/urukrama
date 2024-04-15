#include "graph.hpp"

#include "types.hpp"

#include <algorithm>
#include <iostream>
#include <random>
#include <ranges>


namespace urukrama {

template <typename T>
GraphConstructor<T>::GraphConstructor(std::span<const Point<T>> points, const size_t R, const size_t L)
    : m_R(R), m_L(L), m_dimension(points.front().size()), m_points(points)
{
    Init();
    size_t s_idx = FindMedoid();

    // std::cout << s_idx << std::endl;

    const auto process = [&](const float alpha) {
        size_t good = 0;

        for (const auto& [p_idx, p]: m_points | std::views::enumerate) {
            auto [top, visited] = GreedySearch(s_idx, p, 1);
            good += (top.begin()->second == p_idx);
            // if (p_idx % 1000 == 0) {
            //     std::cout << p_idx << " " << visited.size() << " " << top.size() << " "
            //               << (top.begin()->second == p_idx) << std::endl;
            // }
            RobustPrune(p_idx, std::move(visited), alpha);

            for (size_t np_idx: m_n_out[p_idx]) {
                m_n_out[np_idx].insert(p_idx);

                if (m_n_out[np_idx].size() > m_R) {
                    std::vector<std::pair<T, size_t>> kek;
                    kek.reserve(m_n_out[np_idx].size());
                    for (auto n_idx: m_n_out[np_idx]) {
                        kek.emplace_back(Distance(m_points[np_idx], m_points[n_idx]), n_idx);
                    }
                    std::sort(kek.begin(), kek.end());
                    RobustPrune(np_idx, std::move(kek), alpha);
                }
            }
        }

        return good;
    };

    auto good0 = process(1.0);
    auto good1 = process(1.2);

    std::cout << good0 / double(m_points.size()) << " " << good1 / double(m_points.size()) << std::endl;

    // for (auto [p_idx, edges]: m_n_out) {
    //     for (auto e: edges) {
    //         std::cout << "(" << p_idx << ", " << e << "), " << std::endl;
    //     }
    // }
}


template <typename T>
void GraphConstructor<T>::Init()
{
    std::mt19937_64 random_engine{std::random_device{}()};
    std::uniform_int_distribution<size_t> dis(0, m_points.size() - 1);

    for (size_t idx = 0; idx < m_points.size(); ++idx) {
        while (m_n_out[idx].size() < m_R) {
            if (auto other_idx = dis(random_engine); other_idx != idx) {
                m_n_out[idx].insert(other_idx);
            }
        }
    }
}

template <typename T>
T GraphConstructor<T>::Distance(const Point<T>& a, const Point<T>& b)
{
    return (a - b).squaredNorm();
}

template <typename T>
size_t GraphConstructor<T>::FindMedoid()
{
    Point<T> centroid = std::reduce(m_points.begin(), m_points.end(), Point<T>(m_dimension)) / m_points.size();

    const auto medoid_it = std::ranges::min_element(m_points, {}, [&](const auto& p) { return Distance(centroid, p); });

    return std::distance(m_points.begin(), medoid_it);
}

template <typename T>
GraphConstructor<T>::GreedySearchResult GraphConstructor<T>::GreedySearch(size_t s_idx, const Point<T>& query, size_t k)
{
    HashSet<size_t> fast_visited;
    fast_visited.reserve(m_L * 2);

    std::vector<std::pair<T, size_t>> visited;
    visited.reserve(m_L * 2);

    std::vector<std::pair<T, size_t>> candidates{{Distance(m_points[s_idx], query), s_idx}};
    candidates.reserve(m_L + 1);

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

        for (const auto n_idx: m_n_out[p_star_idx]) {
            if (not fast_visited.contains(n_idx)) {
                const T distance = Distance(m_points[n_idx], query);

                if (candidates.size() > m_L and candidates.back().first < distance) {
                    continue;
                }

                const auto value = std::make_pair(distance, n_idx);
                candidates.insert(std::lower_bound(candidates.begin(), candidates.end(), value), value);

                if (candidates.size() > m_L) {
                    candidates.pop_back();
                }
            }
        }
    }

    std::sort(visited.begin(), visited.end());

    return std::make_pair(std::move(candidates), std::move(visited));
}

template <typename T>
void GraphConstructor<T>::RobustPrune(size_t p_idx, std::vector<std::pair<T, size_t>>&& candidates, float alpha)
{
    auto& p_n_out = m_n_out[p_idx];

    p_n_out.clear();

    HashSet<size_t> skip;

    for (const auto& [i, p_star_idx]: candidates | std::views::values | std::views::enumerate) {
        if (p_star_idx == p_idx or skip.contains(p_star_idx)) {
            continue;
        }

        p_n_out.insert(p_star_idx);

        if (p_n_out.size() == m_R) {
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
}

template class GraphConstructor<float>;
template class GraphConstructor<uint8_t>;

}  // namespace urukrama