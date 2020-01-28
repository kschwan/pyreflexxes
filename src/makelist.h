#pragma once

#include <pybind11/pybind11.h>

#include <RMLVector.h>

template<typename Iterator>
inline auto make_list(Iterator first, Iterator last)
{
    pybind11::list list;

    for (; first != last; ++first)
        list.append(*first);

    return list;
}

template<typename T>
inline auto make_list(const RMLVector<T>& v)
{
    return make_list(v.VecData, std::next(v.VecData, v.VectorDimension));
}
