/*
 * Copyright 2019 Kim Lindberg Schwaner <kils@mmmi.sdu.dk>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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

template<typename T, std::size_t N>
inline auto make_list(const T(&array)[N])
{
    return make_list(std::begin(array), std::end(array));
}

template<typename T>
inline auto make_list(const RMLVector<T>& v)
{
    return make_list(v.VecData, std::next(v.VecData, v.VectorDimension));
}
