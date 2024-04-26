#pragma once

#include <cstddef>
#include <vector>

template <typename V>
class BoundedSortedVector {
public:
    BoundedSortedVector(size_t limit): m_limit(limit) {}

public:
    void insert(const V& val)
    {
        if (m_data.size() > m_limit and m_data.back() < val) {
            return;
        }

        m_data.emplace(std::lower_bound(m_data.begin(), m_data.end(), val), val);

        if (m_data.size() > m_limit) {
            m_data.pop_back();
        }
    }

    auto begin() { return m_data.begin(); }
    auto end() { return m_data.end(); }

    void reserve(size_t num) { m_data.reserve(num); }

    void resize(size_t n)
    {
        m_limit = n;
        m_data.resize(m_limit);
    }

    operator std::vector<V>() && { return std::move(m_data); }

private:
    size_t m_limit;
    std::vector<V> m_data;
};