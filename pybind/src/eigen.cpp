#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "lusk.h"

namespace lusk {
namespace eigen {

int add(int i, int j)
{
    return i + j;
}

void pybind_eigen(py::module &m)
{
    m.doc() = "eigen tests submodule";

    m.def("add", &add, R"pbdoc(
        Add two numbers
        Some other explanation about the add function.
    )pbdoc");

    m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers
        Some other explanation about the subtract function.
    )pbdoc");

    m.def("print_numpy",
            [](Eigen::Vector2d v) {
                std::cout << "V: " << v.transpose() << std::endl;
            }
        );

    m.def("print_stl_numpy",
            [](std::vector<Eigen::Vector2d> vs) {
                for (auto&& v : vs) {
                    std::cout << v.transpose() << std::endl;
                }
            }
        );
}

} // ns eigen
} // ns lusk