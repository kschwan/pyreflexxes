#ifndef PYRMLVECTOR_H
#define PYRMLVECTOR_H

#include <RMLVector.h>

#include <boost/python.hpp>

template<typename T>
struct RMLVectorName {};

template<>
struct RMLVectorName<RMLVector<double>>
{
    static const char* value() { return "RMLDoubleVector"; }
};

template<>
struct RMLVectorName<RMLVector<int>>
{
    static const char* value() { return "RMLIntVector"; }
};

template<>
struct RMLVectorName<RMLVector<bool>>
{
    static const char* value() { return "RMLBoolVector"; }
};

// Print RMLVector to output steam (used by __repr__ / __str__)
template<typename T>
std::ostream& operator<<(std::ostream& os, const RMLVector<T>& v)
{
    os << RMLVectorName<RMLVector<T>>::value() << "([";

    for (unsigned i = 0; i < v.VectorDimension - 1; ++i) {
        os << v.VecData[i] << ", ";
    }

    os << v.VecData[v.VectorDimension - 1] << "])";
    return os;
}

// Expose a template instantiation of RMLVector
template<typename T>
void expose_RMLVector()
{
    namespace py = boost::python;

    using size_type = decltype(RMLVector<T>::VectorDimension);
    using difference_type = typename std::make_signed<size_type>::type;

    py::class_<RMLVector<T>>(RMLVectorName<RMLVector<T>>::value(), py::no_init)
        .def("__init__", py::make_constructor(+[](py::object iterable) {
            auto n = static_cast<size_type>(py::len(iterable));

            if (n == 0)
                throw std::invalid_argument("RMLVector does not support being empty");

            auto v = std::make_shared<RMLVector<T>>(n);

            for (size_type i = 0; i < n; ++i)
                v->VecData[i] = py::extract<T>(iterable[i]);

            return v;
        }))
        .def("__len__", &RMLVector<T>::GetVecDim)
        .def("__getitem__",
            +[](const RMLVector<T>& self, difference_type i) {
                if (i < 0)
                    i += self.VectorDimension;
                if (i < 0 || (size_type)i >= self.VectorDimension)
                    throw std::out_of_range("RMLVector index out of range");
                return self.VecData[(size_type)i];
            })
        .def("__setitem__",
            +[](RMLVector<T>& self, difference_type i, const T& val) {
                if (i < 0)
                    i += self.VectorDimension;
                if (i < 0 || (size_type)i >= self.VectorDimension)
                    throw std::out_of_range("RMLVector index out of range");
                self.VecData[(size_type)i] = val;
             })
        .def("__iter__", py::range(+[](const RMLVector<T>& self) { return self.VecData; }, +[](const RMLVector<T>& self) { return std::next(self.VecData, self.VectorDimension); }))
        .def("__eq__", &RMLVector<T>::operator==)
        .def("__ne__", &RMLVector<T>::operator!=)
        .def("Set", &RMLVector<T>::Set)
        .def("tolist", +[](const RMLVector<T>& self) { return make_list(self.VecData, std::next(self.VecData, self.VectorDimension)); })
        .def(repr(py::self))
    ;
}

#endif // PYRMLVECTOR_H
