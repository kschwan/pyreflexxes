#ifndef MAKELIST_H
#define MAKELIST_H

#include <pybind11/pybind11.h>

template<typename Iterator>
inline auto make_list(Iterator first, Iterator last)
{
    pybind11::list list;

    for (; first != last; ++first) {
        list.append(*first);
    }

    return list;
}

#endif // MAKELIST_H
