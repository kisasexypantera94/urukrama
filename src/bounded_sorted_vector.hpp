#pragma once

#include <cstddef>
#include <vector>

template <typename K, typename V>
class BoundedSortedVector {
public:
    BoundedSortedVector(size_t limit): m_limit(limit) {}

public:
    void emplace(const K& key, const V& val)
    {
        if (m_data.size() > m_limit and m_data.back().first < key) {
            return;
        }

        const auto element = std::make_pair(key, val);
        m_data.insert(std::lower_bound(m_data.begin(), m_data.end(), element), element);

        if (m_data.size() > m_limit) {
            m_data.pop_back();
        }
    }

    void reserve(size_t num) { m_data.reserve(num); }
    auto begin() { return m_data.begin(); }
    auto end() { return m_data.end(); }

    operator std::vector<std::pair<K, V>>() && { return std::move(m_data); }

private:
    size_t m_limit;
    std::vector<std::pair<K, V>> m_data;
};