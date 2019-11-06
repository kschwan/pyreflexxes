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

#include <ReflexxesAPI.h>

#include <boost/noncopyable.hpp>
#include <boost/python.hpp>

namespace py = boost::python;

// Print RMLVector to output steam (used by __repr__ / __str__)
template<typename T>
struct RMLVectorName {};

template<>
struct RMLVectorName<RMLVector<double>>
{
    static const char* value() { return "RMLDoubleVector"; }
};

template<>
struct RMLVectorName<RMLVector<int>>
{
    static const char* value() { return "RMLIntVector"; }
};

template<>
struct RMLVectorName<RMLVector<bool>>
{
    static const char* value() { return "RMLBoolVector"; }
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const RMLVector<T>& v)
{
    os << RMLVectorName<RMLVector<T>>::value() << "([";

    for (unsigned i = 0; i < v.VectorDimension - 1; ++i) {
        os << v.VecData[i] << ", ";
    }

    os << v.VecData[v.VectorDimension - 1] << "])";
    return os;
}

// Copy RMLVector elements to Python list
template<typename T>
auto to_list(const RMLVector<T>& v)
{
    py::list list;

    for (unsigned i = 0; i < v.VectorDimension; ++i) {
        list.append(v.VecData[i]);
    }

    return list;
}

// Copy RMLDoubleVectors to Python list
auto to_list(RMLDoubleVector** vv, unsigned n)
{
    py::list list;

    for (unsigned i = 0; i < n; ++i) {
        list.append(*vv[i]);
    }

    return list;
}

// Create an RMLVector from a Python iterable
template<typename T>
std::shared_ptr<RMLVector<T>> make_vector(const py::object& iterable)
{
    auto n = static_cast<unsigned>(len(iterable));

    if (n == 0)
        throw std::invalid_argument("RMLVector does not support being empty");

    auto v = std::make_shared<RMLVector<T>>(n);

    for (unsigned i = 0; i < n; ++i) {
        v->VecData[i] = py::extract<T>(iterable[i]);
    }

    return v;
}

// Expose a template instantiation of RMLVector
template<typename T>
void expose_RMLVector(const char* name)
{
    py::class_<RMLVector<T>>(name, py::no_init)
        .def("__init__", py::make_constructor(make_vector<T>))
        .def("__len__", &RMLVector<T>::GetVecDim)
        .def("__getitem__",
             +[](const RMLVector<T>& self, unsigned idx) {
                 if (idx >= self.VectorDimension)
                     throw std::out_of_range("RMLVector index out of range");
                 return self.VecData[idx];
             })
        .def("__setitem__",
             +[](RMLVector<T>& self, unsigned idx, const T& val) {
                 if (idx >= self.VectorDimension)
                     throw std::out_of_range("RMLVector index out of range");
                 self.VecData[idx] = val;
             })
        .def("__iter__", py::range(+[](RMLVector<T>& self) { return self.VecData; }, +[](RMLVector<T>& self) { return self.VecData + self.VectorDimension; }))
        .def("__eq__", &RMLVector<T>::operator==)
        .def("__ne__", &RMLVector<T>::operator!=)
        .def("Set", &RMLVector<T>::Set)
        .def("tolist", +[](const RMLVector<T>& self) { return to_list(self); })
        .def(repr(py::self))
    ;
}

BOOST_PYTHON_MODULE(_reflexxes_type2)
{
    py::scope().attr("__version__") = "1.2.7";

    expose_RMLVector<double>("RMLDoubleVector");
    expose_RMLVector<int>("RMLIntVector");
    expose_RMLVector<bool>("RMLBoolVector");

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
        ;
    }

    {
        py::scope scope =
        py::class_<RMLFlags>("RMLFlags", py::no_init)
            .def("__eq__", &RMLFlags::operator==)
            .def("__ne__", &RMLFlags::operator!=)
            .def_readwrite("SynchronizationBehavior", &RMLFlags::SynchronizationBehavior)
            .def_readwrite("EnableTheCalculationOfTheExtremumMotionStates", &RMLFlags::EnableTheCalculationOfTheExtremumMotionStates)
        ;

        py::enum_<RMLFlags::SyncBehaviorEnum>("SyncBehavior")
            .value("PHASE_SYNCHRONIZATION_IF_POSSIBLE", RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE)
            .value("ONLY_TIME_SYNCHRONIZATION", RMLFlags::ONLY_TIME_SYNCHRONIZATION)
            .value("ONLY_PHASE_SYNCHRONIZATION", RMLFlags::ONLY_PHASE_SYNCHRONIZATION)
            .value("NO_SYNCHRONIZATION", RMLFlags::NO_SYNCHRONIZATION)
        ;
    }

    {
        py::scope scope =
        py::class_<RMLPositionFlags, py::bases<RMLFlags>>("RMLPositionFlags")
            .def("__eq__", &RMLPositionFlags::operator==)
            .def("__ne__", &RMLPositionFlags::operator!=)
            .def_readwrite("BehaviorAfterFinalStateOfMotionIsReached", &RMLPositionFlags::BehaviorAfterFinalStateOfMotionIsReached)
            .def_readwrite("KeepCurrentVelocityInCaseOfFallbackStrategy", &RMLPositionFlags::KeepCurrentVelocityInCaseOfFallbackStrategy)
        ;

        py::enum_<RMLPositionFlags::FinalMotionBehaviorEnum>("FinalMotionBehavior")
            .value("KEEP_TARGET_VELOCITY", RMLPositionFlags::KEEP_TARGET_VELOCITY)
            .value("RECOMPUTE_TRAJECTORY", RMLPositionFlags::RECOMPUTE_TRAJECTORY)
        ;
    }

    py::class_<RMLVelocityFlags, py::bases<RMLFlags>>("RMLVelocityFlags")
        .def("__eq__", &RMLVelocityFlags::operator==)
        .def("__ne__", &RMLVelocityFlags::operator!=)
    ;

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

    py::class_<RMLVelocityInputParameters, py::bases<RMLInputParameters>>("RMLVelocityInputParameters", py::init<unsigned>())
        .def("CheckForValidity", &RMLVelocityInputParameters::CheckForValidity)
    ;

    {
        // RMLOutputParameters objects have their members exposed through only
        // getter functions. Values are returned here are copies (so modifying
        // them will not change the member of the RMLOutputParameters object).
        // See https://www.boost.org/doc/libs/1_65_0/libs/python/doc/html/faq/is_return_internal_reference_eff.html
        py::scope scope =
        py::class_<RMLOutputParameters, boost::noncopyable>("RMLOutputParameters", py::no_init) // Base class with protected ctor (including copy ctor)
            .def_readonly("NumberOfDOFs", &RMLOutputParameters::NumberOfDOFs)
            .def_readonly("ANewCalculationWasPerformed", &RMLOutputParameters::ANewCalculationWasPerformed)
            .def_readonly("TrajectoryIsPhaseSynchronized", &RMLOutputParameters::TrajectoryIsPhaseSynchronized)
            .def_readonly("DOFWithTheGreatestExecutionTime", &RMLOutputParameters::DOFWithTheGreatestExecutionTime)
            .def_readonly("SynchronizationTime", &RMLOutputParameters::SynchronizationTime)
            .add_property("NewPositionVector",
                          py::make_function(+[](const RMLOutputParameters& self) -> const auto& { return *self.NewPositionVector; }, py::return_value_policy<py::copy_const_reference>()))
            .add_property("NewVelocityVector",
                          py::make_function(+[](const RMLOutputParameters& self) -> const auto& { return *self.NewVelocityVector; }, py::return_value_policy<py::copy_const_reference>()))
            .add_property("NewAccelerationVector",
                          py::make_function(+[](const RMLOutputParameters& self) -> const auto& { return *self.NewAccelerationVector; }, py::return_value_policy<py::copy_const_reference>()))
            .add_property("MinExtremaTimesVector",
                          py::make_function(+[](const RMLOutputParameters& self) -> const auto& { return *self.MinExtremaTimesVector; }, py::return_value_policy<py::copy_const_reference>()))
            .add_property("MaxExtremaTimesVector",
                          py::make_function(+[](const RMLOutputParameters& self) -> const auto& { return *self.MaxExtremaTimesVector; }, py::return_value_policy<py::copy_const_reference>()))
            .add_property("MinPosExtremaPositionVectorOnly",
                          py::make_function(+[](const RMLOutputParameters& self) -> const auto& { return *self.MinPosExtremaPositionVectorOnly; }, py::return_value_policy<py::copy_const_reference>()))
            .add_property("MaxPosExtremaPositionVectorOnly",
                          py::make_function(+[](const RMLOutputParameters& self) -> const auto& { return *self.MaxPosExtremaPositionVectorOnly; }, py::return_value_policy<py::copy_const_reference>()))
            .add_property("ExecutionTimes",
                          py::make_function(+[](const RMLOutputParameters& self) -> const auto& { return *self.ExecutionTimes; }, py::return_value_policy<py::copy_const_reference>()))
            // The returned lists contain *copies* of RMLVectors pointed to
            .add_property("MinPosExtremaPositionVectorArray",
                          +[](const RMLOutputParameters& self) { return to_list(self.MinPosExtremaPositionVectorArray, self.NumberOfDOFs); })
            .add_property("MinPosExtremaVelocityVectorArray",
                          +[](const RMLOutputParameters& self) { return to_list(self.MinPosExtremaVelocityVectorArray, self.NumberOfDOFs); })
            .add_property("MinPosExtremaAccelerationVectorArray",
                          +[](const RMLOutputParameters& self) { return to_list(self.MinPosExtremaAccelerationVectorArray, self.NumberOfDOFs); })
            .add_property("MaxPosExtremaPositionVectorArray",
                          +[](const RMLOutputParameters& self) { return to_list(self.MaxPosExtremaPositionVectorArray, self.NumberOfDOFs); })
            .add_property("MaxPosExtremaVelocityVectorArray",
                          +[](const RMLOutputParameters& self) { return to_list(self.MaxPosExtremaVelocityVectorArray, self.NumberOfDOFs); })
            .add_property("MaxPosExtremaAccelerationVectorArray",
                          +[](const RMLOutputParameters& self) { return to_list(self.MaxPosExtremaAccelerationVectorArray, self.NumberOfDOFs); })
        ;

        py::enum_<RMLOutputParameters::ReturnValue>("ReturnValue")
            .value("RETURN_SUCCESS", RMLOutputParameters::RETURN_SUCCESS)
            .value("RETURN_ERROR", RMLOutputParameters::RETURN_ERROR)
        ;
    }

    py::class_<RMLPositionOutputParameters, py::bases<RMLOutputParameters>>("RMLPositionOutputParameters", py::init<unsigned>());

    py::class_<RMLVelocityOutputParameters, py::bases<RMLOutputParameters>>("RMLVelocityOutputParameters", py::init<unsigned>())
        .add_property("PositionValuesAtTargetVelocity",
                      py::make_function(+[](const RMLVelocityOutputParameters& self) -> auto const& { return *self.PositionValuesAtTargetVelocity; }, py::return_value_policy<py::copy_const_reference>()))
    ;
}
