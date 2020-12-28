#include <iostream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>

#include "lusk.h"

namespace lusk {
namespace smartptrs {

class Base
{
public:
    Base(int v) : v_(v) {}
    virtual ~Base() = default;

public:
    int v_;
};

class Derived : public Base
{
public:
    Derived() : Base(8) { x_ = 2*v_; }

    static std::function<void(const std::shared_ptr<Base>&)> GetUpdateFunction(int y)
    {
        return [y](const std::shared_ptr<Base>& base) -> void {
            if (auto derived = std::dynamic_pointer_cast<Derived>(base)) {
                derived->x_ += y;
            }
        };
    }
public:
    int x_;
};

class Example
{
public:
    Example() = default;

    void print(const std::shared_ptr<Base>& base)
    {
        if (auto derived = std::dynamic_pointer_cast<Derived>(base)) {
            std::cout << "Derived with v = " << derived->v_ << " and ";
            std::cout << "x = " << derived->x_ << std::endl;
        } else {
            std::cout << "Base with v = " << base->v_ << std::endl;
        }
    }

    void process(const std::shared_ptr<Base>& base, const std::function<void(const std::shared_ptr<Base>&)>& f = nullptr)
    {
        if (!f) {
            if (auto derived = std::dynamic_pointer_cast<Derived>(base)) {
                static constexpr int y = 2;
                Derived::GetUpdateFunction(y)(derived);
            }
        } else {
            f(base);
        }
    }
};

void pybind_smartptrs(py::module &m)
{
    using namespace pybind11::literals;
    m.doc() = "Example submodule for smart pointers";

    py::class_<Base, std::shared_ptr<Base>>(m, "Base")
        .def(py::init<int>())
        .def("__repr__",
            [](const Base& b) {
                return "<Base with v = " + std::to_string(b.v_) + ">";
            })
        .def_readwrite("v", &Base::v_, "");

    py::class_<Derived, std::shared_ptr<Derived>, Base>(m, "Derived")
        .def(py::init())
        .def("__repr__",
            [](const Derived& d) {
                return "<Derived with v = " + std::to_string(d.v_)
                    + " and x = " + std::to_string(d.x_) +">";
            })
        .def_readwrite("x", &Derived::x_, "");

    py::class_<Example>(m, "Example")
        .def(py::init())
        .def("__repr__",
            [](const Example& d) {
                return "<Example>";
            })
        .def("print", &Example::print, "base"_a, "Print a base or derived object")
        .def("process", &Example::process, "base"_a, "f"_a = nullptr, "Process");

}

} // ns smartptrs
} // ns lusk