#include <catch2/catch_all.hpp>

#include "MyLib/utils/b.hpp"

TEST_CASE("b_fn") { REQUIRE(b::b_fn() == 4); }
