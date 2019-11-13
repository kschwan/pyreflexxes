#ifndef MAKELIST_H
#define MAKELIST_H

#include <boost/python/list.hpp>

template<typename Iterator>
inline auto make_list(Iterator begin, Iterator end)
{
    boost::python::list list;

    for (; begin != end; ++begin) {
        list.append(*begin);
    }

    return list;
}

#endif // MAKELIST_H
