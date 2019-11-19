#ifndef VECTOR_TYPE_CASTER_H
#define VECTOR_TYPE_CASTER_H

#include "makelist.h"

#include <RMLVector.h>

#include <pybind11/pybind11.h>

namespace pybind11 {
namespace detail {

template<typename T>
struct type_caster<RMLVector<T>>
{
    type_caster()
        : value(0) // RMLVector<T> of size 0, ie. new T[0] (lacking a default ctor)
    {}

    // Python to C++
    bool load(handle src, bool /*convert*/)
    {
        if (!isinstance<sequence>(src))
            return false;

        auto s = reinterpret_borrow<sequence>(src);

        // Possibly reallocate the data array
        if (value.VectorDimension != s.size()) {
            delete[] value.VecData;
            value.VectorDimension = s.size();
            value.VecData = new T[value.VectorDimension];
        }

        // Copy data
        auto d = value.VecData;

        for (auto it : s) {
            *d = it.cast<T>();
            ++d;
        }

        return true;
    }

    // C++ to Python
    static handle cast(const RMLVector<T>& src, return_value_policy /*policy*/, handle /*parent*/)
    {
        if (src.VectorDimension == 0)
            return list().release();

        list l = make_list(src.VecData, std::next(src.VecData, src.VectorDimension));
        return l.release();
    }

    PYBIND11_TYPE_CASTER(RMLVector<T>, _("RMLVector<T>"));
};

} // namespace detail
} // namespace pybind11

#endif // VECTOR_TYPE_CASTER_H
