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
#include "rmlerror.h"

#include <ReflexxesAPI.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;

struct PositionTrajectoryIterator
{
    ReflexxesAPI& rml;
    RMLPositionInputParameters& ip;
    RMLPositionOutputParameters& op;
    RMLPositionFlags& flags;
    int ret;

    explicit PositionTrajectoryIterator(ReflexxesAPI& rml,
                                        RMLPositionInputParameters& ip,
                                        RMLPositionOutputParameters& op,
                                        RMLPositionFlags& flags)
        : rml(rml)
        , ip(ip)
        , op(op)
        , flags(flags)
        , ret(0)
    {}

    auto next()
    {
        if (ret == ReflexxesAPI::RML_FINAL_STATE_REACHED)
            throw pybind11::stop_iteration();

        ret = rml.RMLPosition(ip, &op, flags);

        if (ret < 0)
            throw RMLError(ret);

        // Loop feedback: Copy "new" position, velocity and acceleration vectors of
        // the input parameters to the "current" vectors of the output parameters
        *ip.CurrentPositionVector = *op.NewPositionVector;
        *ip.CurrentVelocityVector = *op.NewVelocityVector;
        *ip.CurrentAccelerationVector = *op.NewAccelerationVector;

        return py::make_tuple(make_list(*op.NewPositionVector),
                              make_list(*op.NewVelocityVector),
                              make_list(*op.NewAccelerationVector));
    }
};

struct PositionTrajectoryGenerator
{
    unsigned number_of_dofs;
    double cycle_time;
    ReflexxesAPI rml;
    RMLPositionInputParameters ip;
    RMLPositionOutputParameters op;
    RMLPositionFlags flags;

    explicit PositionTrajectoryGenerator(unsigned number_of_dofs,
                                         double cycle_time)
        : rml(number_of_dofs, cycle_time)
        , ip(number_of_dofs)
        , op(number_of_dofs)
        , flags()
    {
        ip.SelectionVector->Set(true);
    }

    explicit PositionTrajectoryGenerator(unsigned number_of_dofs,
                                         double cycle_time,
                                         const RMLDoubleVector& max_velocity,
                                         const RMLDoubleVector& max_acceleration)
        : rml(number_of_dofs, cycle_time)
        , ip(number_of_dofs)
        , op(number_of_dofs)
        , flags()
    {
        *ip.MaxVelocityVector = max_velocity;
        *ip.MaxAccelerationVector = max_acceleration;
        ip.SelectionVector->Set(true);
    }

    explicit PositionTrajectoryGenerator(unsigned number_of_dofs,
                                         double cycle_time,
                                         const RMLDoubleVector& max_velocity,
                                         const RMLDoubleVector& max_acceleration,
                                         const RMLDoubleVector& max_jerk)
        : number_of_dofs(number_of_dofs)
        , cycle_time(cycle_time)
        , rml(number_of_dofs, cycle_time)
        , ip(number_of_dofs)
        , op(number_of_dofs)
        , flags()
    {
        *ip.MaxVelocityVector = max_velocity;
        *ip.MaxAccelerationVector = max_acceleration;
        *ip.MaxJerkVector = max_jerk;
        ip.SelectionVector->Set(true);
    }

    PositionTrajectoryIterator trajectory(const RMLDoubleVector& target_position,
                                          double min_sync_time = 0.0)
    {
        *ip.TargetPositionVector = target_position;
        ip.TargetVelocityVector->Set(0);
        ip.MinimumSynchronizationTime = min_sync_time;
        return PositionTrajectoryIterator(rml, ip, op, flags);
    }

    PositionTrajectoryIterator trajectory(const RMLDoubleVector& target_position,
                                          const RMLDoubleVector& target_velocity,
                                          double min_sync_time = 0.0)
    {
        *ip.TargetPositionVector = target_position;
        *ip.TargetVelocityVector = target_velocity;
        ip.MinimumSynchronizationTime = min_sync_time;
        return PositionTrajectoryIterator(rml, ip, op, flags);
    }
};

void def_submodule_extra(py::module& module)
{
    using namespace pybind11::literals;

    auto m = module.def_submodule("extra");

    py::class_<PositionTrajectoryIterator>(m, "PositionTrajectoryIterator")
        .def("__next__", &PositionTrajectoryIterator::next, "Compute the next state of motion")
        .def("__iter__", [](PositionTrajectoryIterator& self) -> auto& { return self; })
    ;

    py::class_<PositionTrajectoryGenerator>(m, "PositionTrajectoryGenerator")
        .def(py::init<unsigned, double>(),
             "number_of_dofs"_a, "cycle_time"_a)
        .def(py::init<unsigned, double, const RMLDoubleVector&, const RMLDoubleVector&>(),
             "number_of_dofs"_a, "cycle_time"_a, "max_velocity"_a, "max_acceleration"_a)
        .def(py::init<unsigned, double, const RMLDoubleVector&, const RMLDoubleVector&, const RMLDoubleVector&>(),
             "number_of_dofs"_a, "cycle_time"_a, "max_velocity"_a, "max_acceleration"_a, "max_jerk"_a)
        .def_readonly("number_of_dofs", &PositionTrajectoryGenerator::number_of_dofs)
        .def_readonly("cycle_time", &PositionTrajectoryGenerator::cycle_time)
        .def_readonly("rml", &PositionTrajectoryGenerator::rml)
        .def_readonly("ip", &PositionTrajectoryGenerator::ip)
        .def_readonly("op", &PositionTrajectoryGenerator::op)
        .def_readonly("flags", &PositionTrajectoryGenerator::flags)
        .def("trajectory",
             py::overload_cast<const RMLDoubleVector&, double>(&PositionTrajectoryGenerator::trajectory),
             "target_position"_a, "min_sync_time"_a = 0.0,
             "Make an iterator returning the states of motion to target position (assuming target velocity to be zero)",
             py::keep_alive<0, 1>())
        .def("trajectory",
             py::overload_cast<const RMLDoubleVector&, const RMLDoubleVector&, double>(&PositionTrajectoryGenerator::trajectory),
             "target_position"_a, "target_velocity"_a, "min_sync_time"_a = 0.0,
             "Make an iterator returning the states of motion to target position",
             py::keep_alive<0, 1>())
        .def_property("selection",
            [](PositionTrajectoryGenerator& self) -> auto& { return *self.ip.SelectionVector; },
            [](PositionTrajectoryGenerator& self, const RMLBoolVector& v) { *self.ip.SelectionVector = v; })
        .def_property("current_position",
            [](PositionTrajectoryGenerator& self) -> auto& { return *self.ip.CurrentPositionVector; },
            [](PositionTrajectoryGenerator& self, const RMLDoubleVector& v) { *self.ip.CurrentPositionVector = v; })
        .def_property("current_velocity",
            [](PositionTrajectoryGenerator& self) -> auto& { return *self.ip.CurrentVelocityVector; },
            [](PositionTrajectoryGenerator& self, const RMLDoubleVector& v) { *self.ip.CurrentVelocityVector = v; })
        .def_property("current_acceleration",
            [](PositionTrajectoryGenerator& self) -> auto& { return *self.ip.CurrentAccelerationVector; },
            [](PositionTrajectoryGenerator& self, const RMLDoubleVector& v) { *self.ip.CurrentAccelerationVector = v; })
        .def_property("max_velocity",
            [](PositionTrajectoryGenerator& self) -> auto& { return *self.ip.MaxVelocityVector; },
            [](PositionTrajectoryGenerator& self, const RMLDoubleVector& v) { *self.ip.MaxVelocityVector = v; })
        .def_property("max_acceleration",
            [](PositionTrajectoryGenerator& self) -> auto& { return *self.ip.MaxAccelerationVector; },
            [](PositionTrajectoryGenerator& self, const RMLDoubleVector& v) { *self.ip.MaxAccelerationVector = v; })
        .def_property("target_position",
            [](PositionTrajectoryGenerator& self) -> auto& { return *self.ip.TargetPositionVector; },
            [](PositionTrajectoryGenerator& self, const RMLDoubleVector& v) { *self.ip.TargetPositionVector = v; })
        .def_property("target_velocity",
            [](PositionTrajectoryGenerator& self) -> auto& { return *self.ip.TargetVelocityVector; },
            [](PositionTrajectoryGenerator& self, const RMLDoubleVector& v) { *self.ip.TargetVelocityVector = v; })
    ;
}
