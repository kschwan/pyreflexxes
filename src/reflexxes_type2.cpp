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
#include "vector_type_caster.h"

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
    default:
        return "(Not enumerated): Unknown error or return value";
    }
}
} // namespace

PYBIND11_MODULE(reflexxes_type2, m)
{
    namespace py = pybind11;

    m.def("error_string", error_string);

    // These "feedback" functions are an optimization: They copy the "new"-vectors
    // of the output paramers to the "current"-vectors of the input parameters.
    // This saves lots of unnecessary conversions between RMLVectors and Python lists.
    m.def("feedback", [](RMLPositionInputParameters& ip, const RMLPositionOutputParameters& op) {
        *ip.CurrentPositionVector = *op.NewPositionVector;
        *ip.CurrentVelocityVector = *op.NewVelocityVector;
        *ip.CurrentAccelerationVector = *op.NewAccelerationVector;
    });
    m.def("feedback", [](RMLVelocityInputParameters& ip, const RMLVelocityOutputParameters& op) {
        *ip.CurrentPositionVector = *op.NewPositionVector;
        *ip.CurrentVelocityVector = *op.NewVelocityVector;
        *ip.CurrentAccelerationVector = *op.NewAccelerationVector;
    }, "Copy the \"new\"-vectors (position, velocity, acceleration) of the output params to the \"current\"-vectors of the input params");

    // ReflexxesAPI
    auto reflexxes_api = py::class_<ReflexxesAPI>(m, "ReflexxesAPI")
        .def(py::init<unsigned, double>())
        .def("RMLPosition", &ReflexxesAPI::RMLPosition)
        .def("RMLPositionAtAGivenSampleTime", &ReflexxesAPI::RMLPositionAtAGivenSampleTime)
        .def("RMLVelocity", &ReflexxesAPI::RMLVelocity)
        .def("RMLVelocityAtAGivenSampleTime", &ReflexxesAPI::RMLVelocityAtAGivenSampleTime)
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
        .export_values()
    ;

    // RMLFlags
    auto rml_flags = py::class_<RMLFlags>(m, "RMLFlags")
        .def("__eq__", &RMLFlags::operator==)
        .def("__ne__", &RMLFlags::operator!=)
        .def_readwrite("SynchronizationBehavior", &RMLFlags::SynchronizationBehavior)
        .def_readwrite("EnableTheCalculationOfTheExtremumMotionStates", &RMLFlags::EnableTheCalculationOfTheExtremumMotionStates)
    ;
    py::enum_<RMLFlags::SyncBehaviorEnum>(rml_flags, "SyncBehaviorEnum")
        .value("PHASE_SYNCHRONIZATION_IF_POSSIBLE", RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE)
        .value("ONLY_TIME_SYNCHRONIZATION", RMLFlags::ONLY_TIME_SYNCHRONIZATION)
        .value("ONLY_PHASE_SYNCHRONIZATION", RMLFlags::ONLY_PHASE_SYNCHRONIZATION)
        .value("NO_SYNCHRONIZATION", RMLFlags::NO_SYNCHRONIZATION)
        .export_values()
    ;

    // RMLPositionFlags
    auto rml_position_flags = py::class_<RMLPositionFlags, RMLFlags>(m, "RMLPositionFlags")
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

    // RMLVelocityFlags
    py::class_<RMLVelocityFlags, RMLFlags>(m, "RMLVelocityFlags")
        .def(py::init<>())
        .def("__eq__", &RMLVelocityFlags::operator==)
        .def("__ne__", &RMLVelocityFlags::operator!=)
    ;

    // RMLInputParameters
    auto rml_input_parameters = py::class_<RMLInputParameters>(m, "RMLInputParameters")
        .def_readonly("NumberOfDOFs", &RMLInputParameters::NumberOfDOFs)
        .def_readwrite("MinimumSynchronizationTime", &RMLInputParameters::MinimumSynchronizationTime)
        .def_property("SelectionVector",
            [](const RMLInputParameters& self) -> const auto& { return *self.SelectionVector; },
            [](RMLInputParameters& self, const RMLBoolVector& v) { *self.SelectionVector = v; })
        .def_property("CurrentPositionVector",
            [](const RMLInputParameters& self) -> const auto& { return *self.CurrentPositionVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.CurrentPositionVector = v; })
        .def_property("CurrentVelocityVector",
            [](const RMLInputParameters& self) -> const auto& { return *self.CurrentVelocityVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.CurrentVelocityVector = v; })
        .def_property("CurrentAccelerationVector",
            [](const RMLInputParameters& self) -> const auto& { return *self.CurrentAccelerationVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.CurrentAccelerationVector = v; })
        .def_property("MaxAccelerationVector",
            [](const RMLInputParameters& self) -> const auto& { return *self.MaxAccelerationVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.MaxAccelerationVector = v; })
        .def_property("MaxJerkVector",
            [](const RMLInputParameters& self) -> const auto& { return *self.MaxJerkVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.MaxJerkVector = v; })
        .def_property("TargetVelocityVector",
            [](const RMLInputParameters& self) -> const auto& { return *self.TargetVelocityVector; },
            [](RMLInputParameters& self, const RMLDoubleVector& v) { *self.TargetVelocityVector = v; })
    ;

    // RMLPositionInputParameters
    py::class_<RMLPositionInputParameters, RMLInputParameters>(m, "RMLPositionInputParameters")
        .def(py::init<unsigned>())
        .def("CheckForValidity", &RMLPositionInputParameters::CheckForValidity)
        .def_property("MaxVelocityVector",
            [](const RMLPositionInputParameters& self) -> const auto& { return *self.MaxVelocityVector; },
            [](RMLPositionInputParameters& self, const RMLDoubleVector& v) { *self.MaxVelocityVector = v; })
        .def_property("TargetPositionVector",
            [](const RMLPositionInputParameters& self) -> const auto& { return *self.TargetPositionVector; },
            [](RMLPositionInputParameters& self, const RMLDoubleVector& v) { *self.TargetPositionVector = v; })
        .def_property("AlternativeTargetVelocityVector",
            [](const RMLPositionInputParameters& self) -> const auto& { return *self.AlternativeTargetVelocityVector; },
            [](RMLPositionInputParameters& self, const RMLDoubleVector& v) { *self.AlternativeTargetVelocityVector = v; })
    ;

    // RMLVelocityInputParameters
    py::class_<RMLVelocityInputParameters, RMLInputParameters>(m, "RMLVelocityInputParameters")
        .def(py::init<unsigned>())
        .def("CheckForValidity", &RMLVelocityInputParameters::CheckForValidity)
    ;

    // RMLOutputParameters
    auto rml_output_parameters = py::class_<RMLOutputParameters>(m, "RMLOutputParameters")
        .def_readonly("ANewCalculationWasPerformed", &RMLOutputParameters::ANewCalculationWasPerformed)
        .def_readonly("TrajectoryIsPhaseSynchronized", &RMLOutputParameters::TrajectoryIsPhaseSynchronized)
        .def_readonly("NumberOfDOFs", &RMLOutputParameters::NumberOfDOFs)
        .def_readonly("DOFWithTheGreatestExecutionTime", &RMLOutputParameters::DOFWithTheGreatestExecutionTime)
        .def_readonly("SynchronizationTime", &RMLOutputParameters::SynchronizationTime)
        .def_property_readonly("NewPositionVector", [](const RMLOutputParameters& self) -> const auto& { return *self.NewPositionVector; })
        .def_property_readonly("NewVelocityVector", [](const RMLOutputParameters& self) -> const auto& { return *self.NewVelocityVector; })
        .def_property_readonly("NewAccelerationVector", [](const RMLOutputParameters& self) -> const auto& { return *self.NewAccelerationVector; })
        .def_property_readonly("MinExtremaTimesVector", [](const RMLOutputParameters& self) -> const auto& { return *self.MinExtremaTimesVector; })
        .def_property_readonly("MaxExtremaTimesVector", [](const RMLOutputParameters& self) -> const auto& { return *self.MaxExtremaTimesVector; })
        .def_property_readonly("MinPosExtremaPositionVectorOnly", [](const RMLOutputParameters& self) -> const auto& { return *self.MinPosExtremaPositionVectorOnly; })
        .def_property_readonly("MaxPosExtremaPositionVectorOnly", [](const RMLOutputParameters& self) -> const auto& { return *self.MaxPosExtremaPositionVectorOnly; })
        .def_property_readonly("ExecutionTimes", [](const RMLOutputParameters& self) -> const auto& { return *self.ExecutionTimes; })
        .def_property_readonly("MinPosExtremaPositionVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MinPosExtremaPositionVectorArray, std::next(self.MinPosExtremaPositionVectorArray, self.NumberOfDOFs)); })
        .def_property_readonly("MinPosExtremaVelocityVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MinPosExtremaVelocityVectorArray, std::next(self.MinPosExtremaVelocityVectorArray, self.NumberOfDOFs)); })
        .def_property_readonly("MinPosExtremaAccelerationVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MinPosExtremaAccelerationVectorArray, std::next(self.MinPosExtremaAccelerationVectorArray, self.NumberOfDOFs)); })
        .def_property_readonly("MaxPosExtremaPositionVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MaxPosExtremaPositionVectorArray, std::next(self.MaxPosExtremaPositionVectorArray, self.NumberOfDOFs)); })
        .def_property_readonly("MaxPosExtremaVelocityVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MaxPosExtremaVelocityVectorArray, std::next(self.MaxPosExtremaVelocityVectorArray, self.NumberOfDOFs)); })
        .def_property_readonly("MaxPosExtremaAccelerationVectorArray",
            [](const RMLOutputParameters& self) { return make_list(self.MaxPosExtremaAccelerationVectorArray, std::next(self.MaxPosExtremaAccelerationVectorArray, self.NumberOfDOFs)); })
    ;
    py::enum_<RMLOutputParameters::ReturnValue>(rml_output_parameters, "ReturnValue")
        .value("RETURN_SUCCESS", RMLOutputParameters::RETURN_SUCCESS)
        .value("RETURN_ERROR", RMLOutputParameters::RETURN_ERROR)
        .export_values()
    ;

    // RMLPositionOutputParameters
    py::class_<RMLPositionOutputParameters, RMLOutputParameters>(m, "RMLPositionOutputParameters")
        .def(py::init<unsigned>())
    ;

    // RMLVelocityOutputParameters
    py::class_<RMLVelocityOutputParameters, RMLOutputParameters>(m, "RMLVelocityOutputParameters")
        .def(py::init<unsigned>())
        .def_property_readonly("PositionValuesAtTargetVelocity", [](const RMLVelocityOutputParameters& self) -> const auto& { return *self.PositionValuesAtTargetVelocity; })
    ;
}
