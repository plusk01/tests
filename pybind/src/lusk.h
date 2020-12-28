#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace lusk {

  namespace eigen {
    void pybind_eigen(py::module &m);
  }

  namespace smartptrs {
    void pybind_smartptrs(py::module &m);
  }

} // ns lusk