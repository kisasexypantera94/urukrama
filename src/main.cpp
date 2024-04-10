#include "distance.hpp"
#include "graph.hpp"
#include "types.hpp"
#include "utils.hpp"

#include <iostream>
#include <vector>

int main()
{
    // std::vector<urukrama::Point<float>> points;

    // for (size_t i = 0; i < 1000000; ++i) {
    //     points.push_back(urukrama::Point<float>::Random(128));
    // }

    auto points = urukrama::FVecsRead("/Users/dvgr/dev/data/deep1b/deep1m_base.fvecs");

    boost::container::flat_map<int, int> m;

    std::mt19937_64 random_engine{std::random_device{}()};
    std::ranges::shuffle(points, random_engine);

    urukrama::GraphConstructor gc(std::span{std::as_const(points)}, 70);
}