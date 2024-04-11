#include "distance.hpp"
#include "graph.hpp"
#include "types.hpp"
#include "utils.hpp"

#include <iostream>
#include <vector>

int main()
{
    auto points = urukrama::FVecsRead("/Users/dvgr/dev/data/deep1b/deep1m_base.fvecs");

    boost::container::flat_map<int, int> m;

    std::mt19937_64 random_engine{std::random_device{}()};
    std::ranges::shuffle(points, random_engine);

    urukrama::GraphConstructor gc(std::span{std::as_const(points)}, 70);

    return 0;
}
