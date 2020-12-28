#include <pybind11/pybind11.h>

#include "lusk.h"

namespace lusk {

PYBIND11_MODULE(pybind_example, m) {
    m.doc() = "pybind module for lusk tests";

    {
        py::module m_submodule = m.def_submodule("eigen");
        eigen::pybind_eigen(m_submodule);
    }

    {
        py::module m_submodule = m.def_submodule("smartptrs");
        smartptrs::pybind_smartptrs(m_submodule);
    }

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}

} // ns lusk