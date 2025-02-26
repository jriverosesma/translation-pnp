#include <catch2/catch_all.hpp>

#include "MyLib/mylib.hpp"

TEST_CASE("mylib_fn") { REQUIRE(mylib::mylib_fn() == 7); }
