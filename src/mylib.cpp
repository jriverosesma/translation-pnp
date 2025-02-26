#include "MyLib/mylib.hpp"

#include "MyLib/utils/a.hpp"
#include "MyLib/utils/b.hpp"

namespace mylib {
int mylib_fn() { return a::a_fn() + b::b_fn(); }
}  // namespace mylib
