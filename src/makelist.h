#ifndef MAKELIST_H
#define MAKELIST_H

#include <boost/python/list.hpp>

template<typename Iterator>
inline auto make_list(Iterator first, Iterator last)
{
    boost::python::list list;

    for (; first != last; ++first) {
        list.append(*first);
    }

    return list;
}

#endif // MAKELIST_H
