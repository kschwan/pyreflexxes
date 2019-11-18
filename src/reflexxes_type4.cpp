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

#include "makelist.h"
#include "pyrmlvector.h"

#include <ReflexxesAPI.h>

#include <pybind11/pybind11.h>

namespace {
std::string error_string(int val)
{
    switch (val) {
    case ReflexxesAPI::RML_WORKING:
        return "RML_WORKING: The algorithm is working; the final state of motion has not been reached yet";
    case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        return "RML_FINAL_STATE_REACHED: The final state of motion has been reached";
    case ReflexxesAPI::RML_NO_ERROR:
        return "RML_NO_ERROR: No error";
    case ReflexxesAPI::RML_ERROR:
        return "RML_ERROR: An unknown error has occurred";
    case ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES:
        return "RML_ERROR_INVALID_INPUT_VALUES: The applied input values are invalid";
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION:
        return "RML_ERROR_EXECUTION_TIME_CALCULATION: An error occurred during the calculation of the synchronization time";
    case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
        return "RML_ERROR_SYNCHRONIZATION: An error occurred during the synchronization of the trajectory";
    case ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS:
        return "RML_ERROR_NUMBER_OF_DOFS: The number of degree of freedom of the input parameters, the output parameters and the algorithm do not match";
    case ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION:
        return "RML_ERROR_NO_PHASE_SYNCHRONIZATION: The input flag RMLFlags::ONLY_PHASE_SYNCHRONIZATION is set, but a phase-synchronized trajectory cannot be executed";
    case ReflexxesAPI::RML_ERROR_NULL_POINTER:
        return "RML_ERROR_NULL_POINTER: One of the pointers to the input objects is NULL";
    case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG:
        return "RML_ERROR_EXECUTION_TIME_TOO_BIG: The execution time of the computed trajectory is to big";
    case ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE:
        return "RML_ERROR_USER_TIME_OUT_OF_RANGE: The sample time for the previously computed trajectory is out of range";
    case ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS:
        return "RML_ERROR_POSITIONAL_LIMITS: The computed trajectory will exceed the positional limits";
    case ReflexxesAPI::RML_ERROR_OVERRIDE_OUT_OF_RANGE:
        return "RML_ERROR_OVERRIDE_OUT_OF_RANGE: The override value is out of range";
    default:
        return "(Not enumerated): Unknown error or return value";
    }
}
}

PYBIND11_MODULE(reflexxes_type4, m)
{
    namespace py = pybind11;

    m.def("error_string", error_string);

    // RMLVector(s)
    expose_RMLVector<bool>(m, "RMLBoolVector");
    expose_RMLVector<int>(m, "RMLIntVector");
    expose_RMLVector<double>(m, "RMLDoubleVector");

    // ReflexxesAPI
    py::class_<ReflexxesAPI> reflexxes_api(m, "ReflexxesAPI");
    reflexxes_api
        .def(py::init<unsigned, double>())
        .def(py::init<unsigned, double, unsigned>())
        .def(py::init<unsigned, double, unsigned, double>())
        .def("RMLPosition", &ReflexxesAPI::RMLPosition)
        .def("RMLPositionAtAGivenSampleTime", &ReflexxesAPI::RMLPositionAtAGivenSampleTime)
        .def("RMLVelocity", &ReflexxesAPI::RMLVelocity)
        .def("RMLVelocityAtAGivenSampleTime", &ReflexxesAPI::RMLVelocityAtAGivenSampleTime)
        .def("SetupOverrideFilter", &ReflexxesAPI::SetupOverrideFilter)
    ;
    py::enum_<ReflexxesAPI::RMLResultValue>(reflexxes_api, "RMLResultValue")
        .value("RML_WORKING", ReflexxesAPI::RML_WORKING)
        .value("RML_FINAL_STATE_REACHED", ReflexxesAPI::RML_FINAL_STATE_REACHED)
        .value("RML_ERROR", ReflexxesAPI::RML_ERROR)
        .value("RML_ERROR_INVALID_INPUT_VALUES", ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES)
        .value("RML_ERROR_EXECUTION_TIME_CALCULATION", ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION)
        .value("RML_ERROR_SYNCHRONIZATION", ReflexxesAPI::RML_ERROR_SYNCHRONIZATION)
        .value("RML_ERROR_NUMBER_OF_DOFS", ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS)
        .value("RML_ERROR_NO_PHASE_SYNCHRONIZATION", ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION)
        .value("RML_ERROR_NULL_POINTER", ReflexxesAPI::RML_ERROR_NULL_POINTER)
        .value("RML_ERROR_EXECUTION_TIME_TOO_BIG", ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG)
        .value("RML_ERROR_USER_TIME_OUT_OF_RANGE", ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE)
        .value("RML_ERROR_POSITIONAL_LIMITS", ReflexxesAPI::RML_ERROR_POSITIONAL_LIMITS)
        .value("RML_ERROR_OVERRIDE_OUT_OF_RANGE", ReflexxesAPI::RML_ERROR_OVERRIDE_OUT_OF_RANGE)
        .export_values()
    ;

    // RMLFlags
    py::class_<RMLFlags> rml_flags(m, "RMLFlags");
    rml_flags
        .def("__eq__", &RMLFlags::operator==)
        .def("__ne__", &RMLFlags::operator!=)
        .def_readwrite("SynchronizationBehavior", &RMLFlags::SynchronizationBehavior)
        .def_readwrite("PositionalLimitsBehavior", &RMLFlags::PositionalLimitsBehavior)
        .def_readwrite("EnableTheCalculationOfTheExtremumMotionStates", &RMLFlags::EnableTheCalculationOfTheExtremumMotionStates)
    ;
    py::enum_<RMLFlags::SyncBehaviorEnum>(rml_flags, "SyncBehaviorEnum")
        .value("PHASE_SYNCHRONIZATION_IF_POSSIBLE", RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE)
        .value("ONLY_TIME_SYNCHRONIZATION", RMLFlags::ONLY_TIME_SYNCHRONIZATION)
        .value("ONLY_PHASE_SYNCHRONIZATION", RMLFlags::ONLY_PHASE_SYNCHRONIZATION)
        .value("NO_SYNCHRONIZATION", RMLFlags::NO_SYNCHRONIZATION)
        .export_values()
    ;
    py::enum_<RMLFlags::PositionalLimitsEnum>(rml_flags, "PositionalLimitsEnum")
        .value("POSITIONAL_LIMITS_IGNORE", RMLFlags::POSITIONAL_LIMITS_IGNORE)
        .value("POSITIONAL_LIMITS_ERROR_MSG_ONLY", RMLFlags::POSITIONAL_LIMITS_ERROR_MSG_ONLY)
        .value("POSITIONAL_LIMITS_ACTIVELY_PREVENT", RMLFlags::POSITIONAL_LIMITS_ACTIVELY_PREVENT)
        .export_values()
    ;

    // RMLPositionFlags
    py::class_<RMLPositionFlags, RMLFlags> rml_position_flags(m, "RMLPositionFlags");
    rml_position_flags
        .def(py::init<>())
        .def("__eq__", &RMLPositionFlags::operator==)
        .def("__ne__", &RMLPositionFlags::operator!=)
        .def_readwrite("BehaviorAfterFinalStateOfMotionIsReached", &RMLPositionFlags::BehaviorAfterFinalStateOfMotionIsReached)
        .def_readwrite("KeepCurrentVelocityInCaseOfFallbackStrategy", &RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy)
    ;
    py::enum_<RMLPositionFlags::FinalMotionBehaviorEnum>(rml_position_flags, "FinalMotionBehaviorEnum")
        .value("KEEP_TARGET_VELOCITY", RMLPositionFlags::KEEP_TARGET_VELOCITY)
        .value("RECOMPUTE_TRAJECTORY", RMLPositionFlags::RECOMPUTE_TRAJECTORY)
        .export_values()
    ;
    py::enum_<RMLPositionFlags::BehaviorIfInitialStateBreachesConstraintsEnum>(rml_position_flags, "BehaviorIfInitialStateBreachesConstraintsEnum")
        .value("GET_INTO_BOUNDARIES_FAST", RMLPositionFlags::GET_INTO_BOUNDARIES_FAST)
        .value("GET_INTO_BOUNDARIES_AT_ZERO_ACCELERATION", RMLPositionFlags::GET_INTO_BOUNDARIES_AT_ZERO_ACCELERATION)
        .export_values()
    ;

    // RMLVelocityFlags
    py::class_<RMLVelocityFlags, RMLFlags>(m, "RMLVelocityFlags")
        .def(py::init<>())
        .def("__eq__", &RMLVelocityFlags::operator==)
        .def("__ne__", &RMLVelocityFlags::operator!=)
    ;

    // RMLInputParameters
    py::class_<RMLInputParameters> rml_input_parameters(m, "RMLInputParameters");
    rml_input_parameters
        .def_readonly("NumberOfDOFs", &RMLInputParameters::NumberOfDOFs)
        .def_readwrite("MinimumSynchronizationTime", &RMLInputParameters::MinimumSynchronizationTime)
        .def_readwrite("OverrideValue", &RMLInputParameters::OverrideValue)
        .def_property("SelectionVector",
            [](const RMLInputParameters& self) { return self.SelectionVector; },
            [](RMLInputParameters& self, const RMLBoolVector& v) { *self.SelectionVector = v; })
        .def_property("CurrentPositionVector",
            [](const RMLInputParameters& self) { return self.CurrentPositionVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.CurrentPositionVector = v; })
        .def_property("CurrentVelocityVector",
            [](const RMLInputParameters& self) { return self.CurrentVelocityVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.CurrentVelocityVector = v; })
        .def_property("CurrentAccelerationVector",
            [](const RMLInputParameters& self) { return self.CurrentAccelerationVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.CurrentAccelerationVector = v; })
        .def_property("MaxAccelerationVector",
            [](const RMLInputParameters& self) { return self.MaxAccelerationVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.MaxAccelerationVector = v; })
        .def_property("MaxJerkVector",
            [](const RMLInputParameters& self) { return self.MaxJerkVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.MaxJerkVector = v; })
        .def_property("TargetVelocityVector",
            [](const RMLInputParameters& self) { return self.TargetVelocityVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.TargetVelocityVector = v; })
        .def_property("MaxPositionVector",
            [](const RMLInputParameters& self) { return self.MaxPositionVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.MaxPositionVector = v; })
        .def_property("MinPositionVector",
            [](const RMLInputParameters& self) { return self.MinPositionVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.MinPositionVector = v; })
    ;
    py::enum_<RMLInputParameters::ErrorCodeForInvalidInputValues>(rml_input_parameters, "ErrorCodeForInvalidInputValues")
        .value("IP_NO_ERROR", RMLInputParameters::IP_NO_ERROR)
        .value("IP_MAX_VELOCITY", RMLInputParameters::IP_MAX_VELOCITY)
        .value("IP_MAX_ACCELERATION", RMLInputParameters::IP_MAX_ACCELERATION)
        .value("IP_MAX_JERK", RMLInputParameters::IP_MAX_JERK)
        .value("IP_TARGET_VELOCITY", RMLInputParameters::IP_TARGET_VELOCITY)
        .value("IP_ORDER_OF_MAGNITUDE", RMLInputParameters::IP_ORDER_OF_MAGNITUDE)
        .value("IP_MINIMUM_SYNC_TIME", RMLInputParameters::IP_MINIMUM_SYNC_TIME)
        .value("IP_OVERRIDE_VALUE", RMLInputParameters::IP_OVERRIDE_VALUE)
        .export_values()
    ;

    // RMLPositionInputParameters
    py::class_<RMLPositionInputParameters, RMLInputParameters>(m, "RMLPositionInputParameters")
        .def(py::init<unsigned>())
        .def("CheckForValidity", [](const RMLPositionInputParameters& self) { return self.CheckForValidity(); })
        .def_property("MaxVelocityVector",
            [](const RMLPositionInputParameters& self) { return self.MaxVelocityVector; },
            [](RMLPositionInputParameters& self, const RMLDoubleVector& v) { *self.MaxVelocityVector = v; })
        .def_property("TargetPositionVector",
            [](const RMLPositionInputParameters& self) { return self.TargetPositionVector; },
            [](RMLPositionInputParameters& self, const RMLDoubleVector& v) { *self.TargetPositionVector = v; })
        .def_property("AlternativeTargetVelocityVector",
            [](const RMLPositionInputParameters& self) { return self.AlternativeTargetVelocityVector; },
            [](RMLPositionInputParameters& self, const RMLDoubleVector& v) { *self.AlternativeTargetVelocityVector = v; })
    ;

    // RMLVelocityInputParameters
    py::class_<RMLVelocityInputParameters, RMLInputParameters>(m, "RMLVelocityInputParameters")
        .def(py::init<unsigned>())
        .def("CheckForValidity", [](const RMLVelocityInputParameters& self) { return self.CheckForValidity(); })
    ;

    // RMLOutputParameters
    py::class_<RMLOutputParameters> rml_output_parameters(m, "RMLOutputParameters");
    rml_output_parameters
        .def("GetErrorString", [](const RMLOutputParameters& self) { return std::string(self.GetErrorString()); })
        .def_readonly("ANewCalculationWasPerformed", &RMLOutputParameters::ANewCalculationWasPerformed, py::return_value_policy::copy)
        .def_readonly("TrajectoryIsPhaseSynchronized", &RMLOutputParameters::TrajectoryIsPhaseSynchronized, py::return_value_policy::copy)
        .def_readonly("OverrideFilterIsActive", &RMLOutputParameters::OverrideFilterIsActive, py::return_value_policy::copy)
        .def_readonly("NumberOfDOFs", &RMLOutputParameters::NumberOfDOFs, py::return_value_policy::copy)
        .def_readonly("DOFWithTheGreatestExecutionTime", &RMLOutputParameters::DOFWithTheGreatestExecutionTime, py::return_value_policy::copy)
        .def_readonly("ResultValue", &RMLOutputParameters::ResultValue, py::return_value_policy::copy)
        .def_readonly("SynchronizationTime", &RMLOutputParameters::SynchronizationTime, py::return_value_policy::copy)
        .def_readonly("CurrentOverrideValue", &RMLOutputParameters::CurrentOverrideValue, py::return_value_policy::copy)
        .def_readonly("NewPositionVector", &RMLOutputParameters::NewPositionVector, py::return_value_policy::copy)
        .def_readonly("NewVelocityVector", &RMLOutputParameters::NewVelocityVector, py::return_value_policy::copy)
        .def_readonly("NewAccelerationVector", &RMLOutputParameters::NewAccelerationVector, py::return_value_policy::copy)
        .def_readonly("MinExtremaTimesVector", &RMLOutputParameters::MinExtremaTimesVector, py::return_value_policy::copy)
        .def_readonly("MaxExtremaTimesVector", &RMLOutputParameters::MaxExtremaTimesVector, py::return_value_policy::copy)
        .def_readonly("MinPosExtremaPositionVectorOnly", &RMLOutputParameters::MinPosExtremaPositionVectorOnly, py::return_value_policy::copy)
        .def_readonly("MaxPosExtremaPositionVectorOnly", &RMLOutputParameters::MaxPosExtremaPositionVectorOnly, py::return_value_policy::copy)
        .def_readonly("ExecutionTimes", &RMLOutputParameters::ExecutionTimes, py::return_value_policy::copy)
        .def_property_readonly("MinPosExtremaPositionVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MinPosExtremaPositionVectorArray, std::next(self.MinPosExtremaPositionVectorArray, self.NumberOfDOFs)); },
            py::return_value_policy::copy)
        .def_property_readonly("MinPosExtremaVelocityVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MinPosExtremaVelocityVectorArray, std::next(self.MinPosExtremaVelocityVectorArray, self.NumberOfDOFs)); },
            py::return_value_policy::copy)
        .def_property_readonly("MinPosExtremaAccelerationVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MinPosExtremaAccelerationVectorArray, std::next(self.MinPosExtremaAccelerationVectorArray, self.NumberOfDOFs)); },
            py::return_value_policy::copy)
        .def_property_readonly("MaxPosExtremaPositionVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MaxPosExtremaPositionVectorArray, std::next(self.MaxPosExtremaPositionVectorArray, self.NumberOfDOFs)); },
            py::return_value_policy::copy)
        .def_property_readonly("MaxPosExtremaVelocityVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MaxPosExtremaVelocityVectorArray, std::next(self.MaxPosExtremaVelocityVectorArray, self.NumberOfDOFs)); },
            py::return_value_policy::copy)
        .def_property_readonly("MaxPosExtremaAccelerationVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MaxPosExtremaAccelerationVectorArray, std::next(self.MaxPosExtremaAccelerationVectorArray, self.NumberOfDOFs)); },
            py::return_value_policy::copy)
        .def_readonly("ExecutionTimes", &RMLOutputParameters::Polynomials, py::return_value_policy::copy)
    ;

    py::enum_<RMLOutputParameters::ReturnValue>(rml_output_parameters, "ReturnValue")
        .value("RETURN_SUCCESS", RMLOutputParameters::RETURN_SUCCESS)
        .value("RETURN_ERROR", RMLOutputParameters::RETURN_ERROR)
        .export_values()
    ;
    // RMLPositionOutputParameters
    py::class_<RMLPositionOutputParameters, RMLOutputParameters>(m, "RMLPositionOutputParameters")
        .def(py::init<unsigned>())
        .def_readonly("TrajectoryExceedsTargetPosition", &RMLPositionOutputParameters::TrajectoryExceedsTargetPosition, py::return_value_policy::copy)
    ;

    // RMLVelocityOutputParameters
    py::class_<RMLVelocityOutputParameters, RMLOutputParameters>(m, "RMLVelocityOutputParameters")
        .def(py::init<unsigned>())
        .def_readonly("PositionValuesAtTargetVelocity", &RMLVelocityOutputParameters::PositionValuesAtTargetVelocity, py::return_value_policy::copy)
    ;

    // RMLPolynomial
    py::class_<RMLPolynomial>(m, "RMLPolynomial")
        .def(py::init<>())
        .def_property_readonly("PositionPolynomialCoefficients", // TODO not readonly
            [](const RMLPolynomial& self) { return make_list(std::begin(self.PositionPolynomialCoefficients), std::end(self.PositionPolynomialCoefficients)); })
        .def_property_readonly("VelocityPolynomialCoefficients",
            [](const RMLPolynomial& self) { return make_list(std::begin(self.VelocityPolynomialCoefficients), std::end(self.VelocityPolynomialCoefficients)); })
        .def_property_readonly("AccelerationPolynomialCoefficients",
            [](const RMLPolynomial& self) { return make_list(std::begin(self.AccelerationPolynomialCoefficients), std::end(self.AccelerationPolynomialCoefficients)); })
        .def_readonly("Time_ValidUntil", &RMLPolynomial::Time_ValidUntil)
    ;

    // RMLOutputPolynomials
    py::class_<RMLOutputPolynomials>(m, "RMLOutputPolynomials")
        .def(py::init<unsigned>())
        .def_readonly("NumberOfDOFs", &RMLOutputPolynomials::NumberOfDOFs)
        .def_property_readonly("NumberOfCurrentlyValidSegments",
            [](const RMLOutputPolynomials& self) { return make_list(self.NumberOfCurrentlyValidSegments, std::next(self.NumberOfCurrentlyValidSegments, self.NumberOfDOFs)); })
        .def_property_readonly("Coefficients",
            [](const RMLOutputPolynomials& self) { return make_list(self.Coefficients, std::next(self.Coefficients, self.NumberOfDOFs)); })
    ;
}
