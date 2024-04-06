#include "distance.hpp"
#include "graph.hpp"
#include "types.hpp"

#include <iostream>
#include <vector>

int main()
{
    auto a = std::vector<float>{0, 1, 2, 3, 4};
    auto b = std::vector<float>{5, 6.0003, 7, 8, 9};

    std::cout << urukrama::NaiveL2<float>::Compute(a, b) << std::endl;

    std::vector<urukrama::Point<float>> points;

    for (size_t i = 0; i < 1024; ++i) {
        points.push_back(urukrama::Point<float>{a});
    }

    urukrama::GraphConstructor<urukrama::NaiveL2<float>> gc(std::span{points}, 5);
}