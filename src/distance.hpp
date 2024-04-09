#pragma once

#include <concepts>
#include <ranges>
#include <span>

namespace urukrama {

template <typename D>
concept CDistance = requires(std::span<const typename D::Type> a) {
                        typename D::Type;

                        {
                            D::Compute(a, a)
                        } -> std::same_as<typename D::Type>;
                    };

template <typename T>
struct NaiveL2 {
    using Type = T;

    static T Compute(std::span<const T> a, std::span<const T> b)
    {
        T distance = 0;

        for (const auto [x, y]: std::views::zip(a, b)) {
            T diff = x - y;
            distance += diff * diff;
        }

        return distance;
    }
};

}  // namespace urukrama
