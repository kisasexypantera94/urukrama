#pragma once

#include <concepts>
#include <ranges>
#include <span>

namespace urukrama {

template <typename D>
concept CDistance = requires(std::span<typename D::Type> a) {
                        typename D::Type;

                        {
                            D::Compute(a, a)
                        } -> std::same_as<typename D::Type>;
                    };

template <typename T>
struct NaiveL2 {
    using Type = T;

    static auto Compute(std::span<T> a, std::span<T> b)
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
