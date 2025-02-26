#include <pybind11/pybind11.h>

#include "MyLib/mylib.hpp"

namespace py = pybind11;

PYBIND11_MODULE(mylib_cpp, m) {
  m.doc() = "Python bindings for mylib_fn() function from MyLib using pybind11";

  // Expose mylib_fn function
  m.def("mylib_fn", &mylib::mylib_fn,
        R"doc(
      Returns the sum of a_fn() and b_fn().
    )doc");
}
