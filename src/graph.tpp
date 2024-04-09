#include "graph.hpp"
#include "types.hpp"
#include "utils.hpp"

#include <execution>
#include <expected>
#include <iostream>
#include <limits>
#include <random>
#include <ranges>
#include <set>
#include <stdexcept>
#include <unordered_set>

namespace urukrama {

template <typename T>
GraphConstructor<T>::GraphConstructor(std::span<const Point<T>> points, const size_t R)
    : m_R(R), m_points(points), m_dimension(points.front().size())
{
    Init();
    size_t s_idx = FindMedoid();

    std::cout << s_idx << std::endl;

    const auto process = [&](const float alpha) {
        for (size_t p_idx = 0; const auto& p: points) {
            auto [top, visited] = GreedySearch(s_idx, p, 1, 1024);
            std::cout << p_idx << " " << visited.size() << " " << (top.begin()->second == p_idx) << std::endl;
            RobustPrune(p_idx, std::move(visited), alpha);

            for (size_t np_idx: m_n_out[p_idx]) {
                m_n_out[np_idx].insert(p_idx);

                if (m_n_out[np_idx].size() > m_R) {
                    RobustPrune(np_idx, {}, alpha);
                }
            }

            ++p_idx;
        }
    };

    process(1.0);
    process(3.0);

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
    std::uniform_int_distribution<> dis(0, m_points.size() - 1);

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
    Point<T> centroid(m_dimension);

    for (const auto& p: m_points) {
        centroid += p;
    }

    centroid /= m_points.size();

    T min_distance = std::numeric_limits<T>::max();
    size_t medoid_idx = 0;

    for (size_t idx = 0; idx < m_points.size(); ++idx) {
        auto distance = Distance(centroid, m_points[idx]);

        if (distance < min_distance) {
            min_distance = distance;
            medoid_idx = idx;
        }
    }

    return medoid_idx;
}

template <typename T>
GraphConstructor<T>::GreedySearchResult GraphConstructor<T>::GreedySearch(size_t s_idx,
                                                                          const Point<T>& xq,
                                                                          size_t k,
                                                                          size_t L) const
{
    std::unordered_set<size_t> visited;
    std::unordered_set<size_t> to_visit{s_idx};
    std::set<std::pair<T, size_t>> top;

    while (true) {
        std::erase_if(to_visit, [&](size_t p_idx) { return visited.contains(p_idx); });

        if (to_visit.empty()) {
            break;
        }

        const auto [p_star_idx, min_distance] = std::transform_reduce(
            to_visit.cbegin(),
            to_visit.cend(),
            std::make_pair(*to_visit.cbegin(), std::numeric_limits<T>::max()),
            [](const auto& x, const auto& y) {
                const auto& [min_idx, min_distance] = x;
                const auto& [cur_idx, cur_distance] = y;

                return cur_distance < min_distance ? y : x;
            },
            [&](size_t p_idx) { return std::make_pair(p_idx, Distance(m_points[p_idx], xq)); });

        visited.insert(p_star_idx);
        to_visit.erase(p_star_idx);

        top.emplace(min_distance, p_star_idx);

        if (min_distance < 1e-18) {
            return {top, visited};
        }

        if (top.size() > L) {
            top.erase(std::prev(top.end()));
        }

        for (auto n: m_n_out.at(p_star_idx)) {
            if (not visited.contains(n)) {
                to_visit.insert(n);
            }
        }
    }

    return {top, visited};
}

template <typename T>
void GraphConstructor<T>::RobustPrune(size_t p_idx, std::unordered_set<size_t>&& visited, float alpha)
{
    visited.merge(std::move(m_n_out[p_idx]));
    visited.erase(p_idx);
    m_n_out[p_idx] = {};

    while (not visited.empty()) {
        const auto [p_star_idx, min_distance] = std::transform_reduce(
            visited.cbegin(),
            visited.cend(),
            std::make_pair(*visited.cbegin(), std::numeric_limits<T>::max()),
            [](const auto& x, const auto& y) {
                const auto& [min_idx, min_distance] = x;
                const auto& [cur_idx, cur_distance] = y;

                return cur_distance < min_distance ? y : x;
            },
            [&](size_t vp_idx) { return std::make_pair(vp_idx, Distance(m_points[p_idx], m_points[vp_idx])); });

        m_n_out[p_idx].insert(p_star_idx);

        if (m_n_out[p_idx].size() == m_R) {
            break;
        }

        std::erase_if(visited, [&](size_t vp_idx) {
            return alpha * Distance(m_points[p_star_idx], m_points[vp_idx]) <=
                   Distance(m_points[p_idx], m_points[vp_idx]);
        });
    }
}

}  // namespace urukrama