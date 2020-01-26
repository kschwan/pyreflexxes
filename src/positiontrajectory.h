#pragma once

#include <ReflexxesAPI.h>

#include <tuple>

struct PositionTrajectoryGenerator;

struct PositionTrajectoryIterator
{
    ReflexxesAPI& rml;
    RMLPositionInputParameters& ip;
    RMLPositionOutputParameters& op;
    RMLPositionFlags& flags;
    int ret;

    explicit PositionTrajectoryIterator(PositionTrajectoryGenerator& g);

    std::tuple<RMLDoubleVector, RMLDoubleVector, RMLDoubleVector> next();
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
                                         double cycle_time);
    explicit PositionTrajectoryGenerator(unsigned number_of_dofs,
                                         double cycle_time,
                                         const RMLDoubleVector& max_velocity,
                                         const RMLDoubleVector& max_acceleration);
    explicit PositionTrajectoryGenerator(unsigned number_of_dofs,
                                         double cycle_time,
                                         const RMLDoubleVector& max_velocity,
                                         const RMLDoubleVector& max_acceleration,
                                         const RMLDoubleVector& max_jerk);

    PositionTrajectoryIterator trajectory(const RMLDoubleVector& target_position,
                                          double min_sync_time = 0.0);
    PositionTrajectoryIterator trajectory(const RMLDoubleVector& target_position,
                                  const RMLDoubleVector& target_velocity,
                                  double min_sync_time = 0.0);
};
