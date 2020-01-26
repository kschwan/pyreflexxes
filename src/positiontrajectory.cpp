#include "positiontrajectory.h"
#include "rmlerror.h"

#include <pybind11/pybind11.h>

#include <iostream>

PositionTrajectoryIterator::PositionTrajectoryIterator(PositionTrajectoryGenerator& g)
    : rml(g.rml)
    , ip(g.ip)
    , op(g.op)
    , flags(g.flags)
    , ret(0)
{}

std::tuple<RMLDoubleVector, RMLDoubleVector, RMLDoubleVector> PositionTrajectoryIterator::next()
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

    return {*op.NewPositionVector, *op.NewVelocityVector, *op.NewAccelerationVector};
}


PositionTrajectoryGenerator::PositionTrajectoryGenerator(unsigned number_of_dofs,
                                                         double cycle_time)
    : rml(number_of_dofs, cycle_time)
    , ip(number_of_dofs)
    , op(number_of_dofs)
    , flags()
{
    ip.SelectionVector->Set(true);
}

PositionTrajectoryGenerator::PositionTrajectoryGenerator(unsigned number_of_dofs,
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

PositionTrajectoryGenerator::PositionTrajectoryGenerator(unsigned number_of_dofs,
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

PositionTrajectoryIterator
PositionTrajectoryGenerator::trajectory(const RMLDoubleVector& target_position,
                                        double min_sync_time)
{
    *ip.TargetPositionVector = target_position;
    ip.TargetVelocityVector->Set(0);
    ip.MinimumSynchronizationTime = min_sync_time;
    return PositionTrajectoryIterator(*this);
}

PositionTrajectoryIterator
PositionTrajectoryGenerator::trajectory(const RMLDoubleVector& target_position,
                                        const RMLDoubleVector& target_velocity,
                                        double min_sync_time)
{
    *ip.TargetPositionVector = target_position;
    *ip.TargetVelocityVector = target_velocity;
    ip.MinimumSynchronizationTime = min_sync_time;
    return PositionTrajectoryIterator(*this);
}
