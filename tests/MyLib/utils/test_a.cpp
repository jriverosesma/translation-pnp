#include <catch2/catch_all.hpp>

#include "MyLib/utils/a.hpp"

TEST_CASE("a_fn") { REQUIRE(a::a_fn() == 3); }
