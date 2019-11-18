#ifndef PYRMLVECTOR_H
#define PYRMLVECTOR_H

#include "makelist.h"

#include <RMLVector.h>

#include <pybind11/pybind11.h>

#include <sstream>

// Expose a template instantiation of RMLVector
template<typename T>
void expose_RMLVector(pybind11::handle scope, std::string name)
{
    namespace py = pybind11;

    using size_type = decltype(RMLVector<T>::VectorDimension);
    using difference_type = typename std::make_signed<size_type>::type;

    py::class_<RMLVector<T>>(scope, name.c_str())
        .def(py::init([](py::sequence sequence) {
            auto n = static_cast<size_type>(py::len(sequence));

            if (n == 0)
                throw py::value_error("RMLVector does not support being empty");

            RMLVector<T> v(n);

            for (size_type i = 0; i < n; ++i)
                v.VecData[i] = sequence[i].template cast<T>();

            return v;
        }))
        .def("__len__", &RMLVector<T>::GetVecDim)
        .def("__getitem__",
            [](const RMLVector<T>& self, difference_type i) {
                if (i < 0)
                    i += self.VectorDimension;

                if (i < 0 || static_cast<size_type>(i) >= self.VectorDimension)
                    throw py::index_error("RMLVector index out of range");

                return self.VecData[static_cast<size_type>(i)];
            })
        .def("__setitem__",
            [](RMLVector<T>& self, difference_type i, const T& val) {
                if (i < 0)
                    i += self.VectorDimension;

                if (i < 0 || static_cast<size_type>(i) >= self.VectorDimension)
                    throw py::index_error("RMLVector index out of range");

                self.VecData[static_cast<size_type>(i)] = val;
             })
        .def("__iter__", [](const RMLVector<T>& self) {
            return py::make_iterator(self.VecData, std::next(self.VecData, self.VectorDimension));
        }, py::keep_alive<0, 1>())
        .def("__eq__", &RMLVector<T>::operator==)
        .def("__ne__", &RMLVector<T>::operator!=)
        .def("Set", &RMLVector<T>::Set)
        .def("tolist", [](const RMLVector<T>& self) {
            return make_list(self.VecData, std::next(self.VecData, self.VectorDimension));
        })
        .def("__repr__", [name](const RMLVector<T>& v) {
            std::ostringstream ss;
            ss << name << "([";

            for (size_type i = 0; i < v.VectorDimension - 1; ++i)
                ss << v.VecData[i] << ", ";

            ss << v.VecData[v.VectorDimension - 1] << "])";
            return ss.str();
        })
    ;
}

#endif // PYRMLVECTOR_H
