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

#include <boost/noncopyable.hpp>
#include <boost/python.hpp>

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

BOOST_PYTHON_MODULE(reflexxes_type2)
{
    namespace py = boost::python;

    py::def("error_string", error_string);

    // RMLVector(s)
    expose_RMLVector<bool>();
    expose_RMLVector<int>();
    expose_RMLVector<double>();

    // ReflexxesAPI
    {
        py::scope scope =
        py::class_<ReflexxesAPI>("ReflexxesAPI", py::init<unsigned, double>())
            .def("RMLPosition", &ReflexxesAPI::RMLPosition)
            .def("RMLPositionAtAGivenSampleTime", &ReflexxesAPI::RMLPositionAtAGivenSampleTime)
            .def("RMLVelocity", &ReflexxesAPI::RMLVelocity)
            .def("RMLVelocityAtAGivenSampleTime", &ReflexxesAPI::RMLVelocityAtAGivenSampleTime)
        ;
        py::enum_<ReflexxesAPI::RMLResultValue>("RMLResultValue")
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
    }

    // RMLFlags
    {
        py::scope scope =
        py::class_<RMLFlags>("RMLFlags", py::no_init)
            .def("__eq__", &RMLFlags::operator==)
            .def("__ne__", &RMLFlags::operator!=)
            .def_readwrite("SynchronizationBehavior", &RMLFlags::SynchronizationBehavior)
            .def_readwrite("EnableTheCalculationOfTheExtremumMotionStates", &RMLFlags::EnableTheCalculationOfTheExtremumMotionStates)
        ;
        py::enum_<RMLFlags::SyncBehaviorEnum>("SyncBehaviorEnum")
            .value("PHASE_SYNCHRONIZATION_IF_POSSIBLE", RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE)
            .value("ONLY_TIME_SYNCHRONIZATION", RMLFlags::ONLY_TIME_SYNCHRONIZATION)
            .value("ONLY_PHASE_SYNCHRONIZATION", RMLFlags::ONLY_PHASE_SYNCHRONIZATION)
            .value("NO_SYNCHRONIZATION", RMLFlags::NO_SYNCHRONIZATION)
            .export_values()
        ;
    }

    // RMLPositionFlags
    {
        py::scope scope =
        py::class_<RMLPositionFlags, py::bases<RMLFlags>>("RMLPositionFlags")
            .def("__eq__", &RMLPositionFlags::operator==)
            .def("__ne__", &RMLPositionFlags::operator!=)
            .def_readwrite("BehaviorAfterFinalStateOfMotionIsReached", &RMLPositionFlags::BehaviorAfterFinalStateOfMotionIsReached)
            .def_readwrite("KeepCurrentVelocityInCaseOfFallbackStrategy", &RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy)
        ;
        py::enum_<RMLPositionFlags::FinalMotionBehaviorEnum>("FinalMotionBehaviorEnum")
            .value("KEEP_TARGET_VELOCITY", RMLPositionFlags::KEEP_TARGET_VELOCITY)
            .value("RECOMPUTE_TRAJECTORY", RMLPositionFlags::RECOMPUTE_TRAJECTORY)
            .export_values()
        ;
    }

    // RMLVelocityFlags
    py::class_<RMLVelocityFlags, py::bases<RMLFlags>>("RMLVelocityFlags")
        .def("__eq__", &RMLVelocityFlags::operator==)
        .def("__ne__", &RMLVelocityFlags::operator!=)
    ;

    // RMLInputParameters
    // RMLInputParameters objects have their members exposed through getter
    // functions that uses the 'return_internal_reference' call policy to allow
    // modifying the members. Properties also have setter functions.
    py::class_<RMLInputParameters>("RMLInputParameters", py::no_init) // Base class with protected ctor
        .def_readonly("NumberOfDOFs", &RMLInputParameters::NumberOfDOFs)
        .def_readwrite("MinimumSynchronizationTime", &RMLInputParameters::MinimumSynchronizationTime)
        .add_property("SelectionVector",
            py::make_function(+[](const RMLInputParameters& self) { return self.SelectionVector; }, py::return_internal_reference<>()),
            py::make_function(+[](RMLInputParameters& self, const RMLBoolVector& v) { *self.SelectionVector = v; }))
        .add_property("CurrentPositionVector",
            py::make_function(+[](const RMLInputParameters& self) { return self.CurrentPositionVector; }, py::return_internal_reference<>()),
            py::make_function(+[](RMLInputParameters& self, const RMLDoubleVector& v) { *self.CurrentPositionVector = v; }))
        .add_property("CurrentVelocityVector",
            py::make_function(+[](const RMLInputParameters& self) { return self.CurrentVelocityVector; }, py::return_internal_reference<>()),
            py::make_function(+[](RMLInputParameters& self, const RMLDoubleVector& v) { *self.CurrentVelocityVector = v; }))
        .add_property("CurrentAccelerationVector",
            py::make_function(+[](const RMLInputParameters& self) { return self.CurrentAccelerationVector; }, py::return_internal_reference<>()),
            py::make_function(+[](RMLInputParameters& self, const RMLDoubleVector& v) { *self.CurrentAccelerationVector = v; }))
        .add_property("MaxAccelerationVector",
            py::make_function(+[](const RMLInputParameters& self) { return self.MaxAccelerationVector; }, py::return_internal_reference<>()),
            py::make_function(+[](RMLInputParameters& self, const RMLDoubleVector& v) { *self.MaxAccelerationVector = v; }))
        .add_property("MaxJerkVector",
            py::make_function(+[](const RMLInputParameters& self) { return self.MaxJerkVector; }, py::return_internal_reference<>()),
            py::make_function(+[](RMLInputParameters& self, const RMLDoubleVector& v) { *self.MaxJerkVector = v; }))
        .add_property("TargetVelocityVector",
            py::make_function(+[](const RMLInputParameters& self) { return self.TargetVelocityVector; }, py::return_internal_reference<>()),
            py::make_function(+[](RMLInputParameters& self, const RMLDoubleVector& v) { *self.TargetVelocityVector = v; }))
    ;

    // RMLPositionInputParameters
    py::class_<RMLPositionInputParameters, py::bases<RMLInputParameters>>("RMLPositionInputParameters", py::init<unsigned>())
        .def("CheckForValidity", &RMLPositionInputParameters::CheckForValidity)
        .add_property("MaxVelocityVector",
            py::make_function(+[](const RMLPositionInputParameters& self) { return self.MaxVelocityVector; }, py::return_internal_reference<>()),
            py::make_function(+[](RMLPositionInputParameters& self, const RMLDoubleVector& v) { *self.MaxVelocityVector = v; }))
        .add_property("TargetPositionVector",
            py::make_function(+[](const RMLPositionInputParameters& self) { return self.TargetPositionVector; }, py::return_internal_reference<>()),
            py::make_function(+[](RMLPositionInputParameters& self, const RMLDoubleVector& v) { *self.TargetPositionVector = v; }))
        .add_property("AlternativeTargetVelocityVector",
            py::make_function(+[](const RMLPositionInputParameters& self) { return self.AlternativeTargetVelocityVector; }, py::return_internal_reference<>()),
            py::make_function(+[](RMLPositionInputParameters& self, const RMLDoubleVector& v) { *self.AlternativeTargetVelocityVector = v; }))
    ;

    // RMLVelocityInputParameters
    py::class_<RMLVelocityInputParameters, py::bases<RMLInputParameters>>("RMLVelocityInputParameters", py::init<unsigned>())
        .def("CheckForValidity", &RMLVelocityInputParameters::CheckForValidity)
    ;

    // RMLOutputParameters
    {
        // RMLOutputParameters objects have their members exposed through only
        // getter functions. Values are returned here are copies (so modifying
        // them will not change the member of the RMLOutputParameters object).
        // See https://www.boost.org/doc/libs/1_65_0/libs/python/doc/html/faq/is_return_internal_reference_eff.html
        py::scope scope =
        py::class_<RMLOutputParameters, boost::noncopyable>("RMLOutputParameters", py::no_init) // Base class with protected ctor (including copy ctor)
            .def_readonly("ANewCalculationWasPerformed", &RMLOutputParameters::ANewCalculationWasPerformed)
            .def_readonly("TrajectoryIsPhaseSynchronized", &RMLOutputParameters::TrajectoryIsPhaseSynchronized)
            .def_readonly("NumberOfDOFs", &RMLOutputParameters::NumberOfDOFs)
            .def_readonly("DOFWithTheGreatestExecutionTime", &RMLOutputParameters::DOFWithTheGreatestExecutionTime)
            .def_readonly("SynchronizationTime", &RMLOutputParameters::SynchronizationTime)
            .add_property("NewPositionVector", +[](const RMLOutputParameters& self) { return *self.NewPositionVector; })
            .add_property("NewVelocityVector", +[](const RMLOutputParameters& self) { return *self.NewVelocityVector; })
            .add_property("NewAccelerationVector", +[](const RMLOutputParameters& self) { return *self.NewAccelerationVector; })
            .add_property("MinExtremaTimesVector", +[](const RMLOutputParameters& self) { return *self.MinExtremaTimesVector; })
            .add_property("MaxExtremaTimesVector", +[](const RMLOutputParameters& self) { return *self.MaxExtremaTimesVector; })
            .add_property("MinPosExtremaPositionVectorOnly", +[](const RMLOutputParameters& self) { return *self.MinPosExtremaPositionVectorOnly; })
            .add_property("MaxPosExtremaPositionVectorOnly", +[](const RMLOutputParameters& self) { return *self.MaxPosExtremaPositionVectorOnly; })
            .add_property("ExecutionTimes", +[](const RMLOutputParameters& self) { return *self.ExecutionTimes; })
            // The returned lists contain copies of RMLVectors pointed to
            .add_property("MinPosExtremaPositionVectorArray",
                +[](const RMLOutputParameters& self) { return make_list(self.MinPosExtremaPositionVectorArray, std::next(self.MinPosExtremaPositionVectorArray, self.NumberOfDOFs)); })
            .add_property("MinPosExtremaVelocityVectorArray",
                +[](const RMLOutputParameters& self) { return make_list(self.MinPosExtremaVelocityVectorArray, std::next(self.MinPosExtremaVelocityVectorArray, self.NumberOfDOFs)); })
            .add_property("MinPosExtremaAccelerationVectorArray",
                +[](const RMLOutputParameters& self) { return make_list(self.MinPosExtremaAccelerationVectorArray, std::next(self.MinPosExtremaAccelerationVectorArray, self.NumberOfDOFs)); })
            .add_property("MaxPosExtremaPositionVectorArray",
                +[](const RMLOutputParameters& self) { return make_list(self.MaxPosExtremaPositionVectorArray, std::next(self.MaxPosExtremaPositionVectorArray, self.NumberOfDOFs)); })
            .add_property("MaxPosExtremaVelocityVectorArray",
                +[](const RMLOutputParameters& self) { return make_list(self.MaxPosExtremaVelocityVectorArray, std::next(self.MaxPosExtremaVelocityVectorArray, self.NumberOfDOFs)); })
            .add_property("MaxPosExtremaAccelerationVectorArray",
                +[](const RMLOutputParameters& self) { return make_list(self.MaxPosExtremaAccelerationVectorArray, std::next(self.MaxPosExtremaAccelerationVectorArray, self.NumberOfDOFs)); })
        ;

        py::enum_<RMLOutputParameters::ReturnValue>("ReturnValue")
            .value("RETURN_SUCCESS", RMLOutputParameters::RETURN_SUCCESS)
            .value("RETURN_ERROR", RMLOutputParameters::RETURN_ERROR)
            .export_values()
        ;
    }

    // RMLPositionOutputParameters
    py::class_<RMLPositionOutputParameters, py::bases<RMLOutputParameters>>("RMLPositionOutputParameters", py::init<unsigned>());

    // RMLVelocityOutputParameters
    py::class_<RMLVelocityOutputParameters, py::bases<RMLOutputParameters>>("RMLVelocityOutputParameters", py::init<unsigned>())
        .add_property("PositionValuesAtTargetVelocity", +[](const RMLVelocityOutputParameters& self) { return *self.PositionValuesAtTargetVelocity; })
    ;
}
