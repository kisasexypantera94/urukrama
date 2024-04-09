#include "distance.hpp"
#include "graph.hpp"
#include "types.hpp"

#include <iostream>
#include <vector>

int main()
{
    std::vector<urukrama::Point<float>> points;

    for (size_t i = 0; i < 10000; ++i) {
        points.push_back(urukrama::Point<float>::Random(8));
    }

    urukrama::GraphConstructor gc(std::span{std::as_const(points)}, 16);
}