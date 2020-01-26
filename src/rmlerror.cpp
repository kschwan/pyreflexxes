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

#include "rmlerror.h"

#include <ReflexxesAPI.h>

namespace {
std::string return_value_string(int value)
{
    switch (value) {
    case ReflexxesAPI::RML_WORKING:
        return "RML_WORKING: The algorithm is working; the final state of motion has not been reached yet";
    case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        return "RML_FINAL_STATE_REACHED: The final state of motion has been reached";
    case ReflexxesAPI::RML_ERROR:
        return "RML_ERROR: An unknown error has occurred";
    case ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES:
        return "RML_ERROR_INVALID_INPUT_VALUES: The applied input values are invalid";
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION:
        return "RML_ERROR_EXECUTION_TIME_CALCULATION: An error occurred during the calculation of the synchronization time";
    case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
        return "RML_ERROR_SYNCHRONIZATION: An error occurred during the synchronization of the trajectory";
    case ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS:
        return "RML_ERROR_NUMBER_OF_DOFS: The number of degrees of freedom of the input parameters, the output parameters and the algorithm do not match";
    case ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION:
        return "RML_ERROR_NO_PHASE_SYNCHRONIZATION: The input flag RMLFlags::ONLY_PHASE_SYNCHRONIZATION is set, but a phase-synchronized trajectory cannot be executed";
    case ReflexxesAPI::RML_ERROR_NULL_POINTER:
        return "RML_ERROR_NULL_POINTER: One of the pointers to the input objects is NULL";
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG:
        return "RML_ERROR_EXECUTION_TIME_TOO_BIG: The execution time of the computed trajectory is to big";
    case ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE:
        return "RML_ERROR_USER_TIME_OUT_OF_RANGE: The sample time for the previously computed trajectory is out of range";
#if defined(RML_TYPE_IV)
    case ReflexxesAPI::RML_NO_ERROR:
        return "RML_NO_ERROR: No error";
    case ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS:
        return "RML_ERROR_POSITIONAL_LIMITS: The computed trajectory will exceed the positional limits";
    case ReflexxesAPI::RML_ERROR_OVERRIDE_OUT_OF_RANGE:
        return "RML_ERROR_OVERRIDE_OUT_OF_RANGE: The override value is out of range";
#endif
    default:
        return "Unknown error or return value";
    }
}
} // namespace

RMLError::RMLError(int value)
    : std::runtime_error(return_value_string(value))
    , value(value)
{}
