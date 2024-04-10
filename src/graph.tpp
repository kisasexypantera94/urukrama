#include "graph.hpp"
#include "types.hpp"
#include "utils.hpp"

#include <boost/range/algorithm_ext/erase.hpp>

#include <execution>
#include <expected>
#include <iostream>
#include <limits>
#include <random>
#include <ranges>
#include <set>
#include <stdexcept>

namespace urukrama {

template <typename T>
GraphConstructor<T>::GraphConstructor(std::span<const Point<T>> points, const size_t R)
    : m_R(R), m_points(points), m_dimension(points.front().size())
{
    Init();
    size_t s_idx = FindMedoid();

    std::cout << s_idx << std::endl;

    const auto process = [&](const float alpha) {
        size_t good = 0;

        for (size_t p_idx = 0; const auto& p: points) {
            auto [top, visited] = GreedySearch(s_idx, p, 1, 75);
            good += (top.begin()->second == p_idx);
            if (p_idx % 1000 == 0) {
                std::cout << p_idx << " " << visited.size() << " " << top.size() << " "
                          << (top.begin()->second == p_idx) << std::endl;
            }
            RobustPrune(p_idx, std::move(top), std::move(visited), alpha);

            for (size_t np_idx: m_n_out[p_idx]) {
                m_n_out[np_idx].insert(p_idx);

                if (m_n_out[np_idx].size() > m_R) {
                    FlatMap<T, size_t> kek;
                    kek.reserve(m_n_out[np_idx].size());
                    for (auto n_idx: m_n_out[np_idx]) {
                        kek.emplace(Distance(m_points[np_idx], m_points[n_idx]), n_idx);
                    }
                    RobustPrune(np_idx, std::move(kek), {}, alpha);
                }
            }

            ++p_idx;
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
                                                                          const Point<T>& query,
                                                                          size_t k,
                                                                          size_t L)
{
    HashSet<size_t> visited;
    FlatMap<T, size_t> candidates{{Distance(m_points[s_idx], query), s_idx}};

    while (true) {
        auto candidates_end = candidates.begin() + std::min(candidates.size(), L);
        auto it = std::find_if(candidates.begin(), candidates_end, [&](const auto& c) {
            return not visited.contains(c.second);
        });

        if (it == candidates_end) {
            break;
        }

        const auto [min_distance, p_star_idx] = *it;

        visited.insert(p_star_idx);

        for (const auto n_idx: m_n_out.at(p_star_idx)) {
            if (not visited.contains(n_idx)) {
                const auto distance = Distance(m_points[n_idx], query);

                if (auto it = candidates.lower_bound(distance); candidates.index_of(it) < L) {
                    candidates.emplace_hint(it, Distance(m_points[n_idx], query), n_idx);
                }
            }
        }

        boost::range::remove_erase_if(candidates, [&, cur_idx = 0](const auto& c) mutable {
            return cur_idx++ >= L and not visited.contains(c.second);
        });
    }

    // boost::range::remove_erase_if(candidates,
    //                               [&, cur_idx = 0](const auto& c) mutable { return not visited.contains(c.second);
    //                               });

    return std::make_pair(std::move(candidates), std::move(visited));
}

template <typename T>
void GraphConstructor<T>::RobustPrune(size_t p_idx,
                                      FlatMap<T, size_t>&& candidates,
                                      HashSet<size_t>&& visited,
                                      float alpha)
{
    if (visited.size() < m_n_out[p_idx].size()) {
        std::swap(visited, m_n_out[p_idx]);
    }

    for (const auto& x: m_n_out[p_idx]) {
        visited.emplace(x);
    }

    visited.erase(p_idx);
    m_n_out[p_idx].clear();

    auto last_it = candidates.begin();

    while (true) {
        const auto it =
            std::find_if(last_it, candidates.end(), [&](const auto& c) { return visited.contains(c.second); });

        if (it == candidates.end()) {
            break;
        }

        const auto p_star_idx = it->second;
        last_it = it + 1;

        m_n_out[p_idx].insert(p_star_idx);

        if (m_n_out[p_idx].size() == m_R) {
            break;
        }

        EraseIf(visited, [&](size_t vp_idx) {
            return alpha * Distance(m_points[p_star_idx], m_points[vp_idx]) <=
                   Distance(m_points[p_idx], m_points[vp_idx]);
        });
    }
}

}  // namespace urukrama