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
        .def("__next__", &PositionTrajectoryIterator::next)
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
        .def_readwrite("rml", &PositionTrajectoryGenerator::rml)
        .def_readwrite("ip", &PositionTrajectoryGenerator::ip)
        .def_readwrite("op", &PositionTrajectoryGenerator::op)
        .def_readwrite("flags", &PositionTrajectoryGenerator::flags)
        .def("trajectory", py::overload_cast<const RMLDoubleVector&, double>(&PositionTrajectoryGenerator::trajectory),
             "target_position"_a, "min_sync_time"_a = 0.0,
             py::keep_alive<0, 1>())
        .def("trajectory", py::overload_cast<const RMLDoubleVector&, const RMLDoubleVector&, double>(&PositionTrajectoryGenerator::trajectory),
             "target_position"_a, "target_velocity"_a, "min_sync_time"_a = 0.0,
             py::keep_alive<0, 1>())
        .def_property("current_position",
            [](const PositionTrajectoryGenerator& self) -> const auto& { return *self.ip.CurrentPositionVector; },
            [](PositionTrajectoryGenerator& self, const RMLDoubleVector& v) { *self.ip.CurrentPositionVector = v; })
        .def_property("current_velocity",
            [](const PositionTrajectoryGenerator& self) -> const auto& { return *self.ip.CurrentVelocityVector; },
            [](PositionTrajectoryGenerator& self, const RMLDoubleVector& v) { *self.ip.CurrentVelocityVector = v; })
        .def_property("current_acceleration",
            [](const PositionTrajectoryGenerator& self) -> const auto& { return *self.ip.CurrentAccelerationVector; },
            [](PositionTrajectoryGenerator& self, const RMLDoubleVector& v) { *self.ip.CurrentAccelerationVector = v; })
    ;
}
