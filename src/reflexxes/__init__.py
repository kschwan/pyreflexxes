try:
    # Prefer Type IV
    from .reflexxes_type4 import (
        error_string,
        ReflexxesAPI,
        RMLBoolVector,
        RMLDoubleVector,
        RMLFlags,
        RMLInputParameters,
        RMLIntVector,
        RMLOutputParameters,
        RMLOutputPolynomials,
        RMLPolynomial,
        RMLPositionFlags,
        RMLPositionInputParameters,
        RMLPositionOutputParameters,
        RMLVelocityFlags,
        RMLVelocityInputParameters,
        RMLVelocityOutputParameters,
    )
    rml_type = 4
except ImportError:
    from .reflexxes_type2 import (
        error_string,
        ReflexxesAPI,
        RMLBoolVector,
        RMLDoubleVector,
        RMLFlags,
        RMLInputParameters,
        RMLIntVector,
        RMLOutputParameters,
        RMLPositionFlags,
        RMLPositionInputParameters,
        RMLPositionOutputParameters,
        RMLVelocityFlags,
        RMLVelocityInputParameters,
        RMLVelocityOutputParameters,
    )
    rml_type = 2


class RMLError(Exception):
    def __init__(self, error_code):
        super(RMLError, self).__init__(error_string(error_code))
        self.error_code = error_code


class PositionTrajectoryGenerator(object):
    def __init__(self, n_dof, cycle_time, max_velocity, max_acceleration, max_jerk=None):
        self.n_dof = n_dof
        self.cycle_time = cycle_time
        self.rml = ReflexxesAPI(n_dof, cycle_time)
        self.ip = RMLPositionInputParameters(n_dof)
        self.op = RMLPositionOutputParameters(n_dof)
        self.flags = RMLPositionFlags()

        self.ip.SelectionVector.Set(True)
        self.ip.MaxVelocityVector = RMLDoubleVector(max_velocity)
        self.ip.MaxAccelerationVector = RMLDoubleVector(max_acceleration)

        if max_jerk:  # needed for RML Type IV
            self.ip.MaxJerkVector = RMLDoubleVector(max_jerk)

    def trajectory(self, target_position, target_velocity=None, min_sync_time=None):
        self.ip.MinimumSynchronizationTime = 0.0 if min_sync_time is None else min_sync_time
        self.ip.TargetPositionVector = RMLDoubleVector(target_position)

        if target_velocity is None:
            self.ip.TargetVelocityVector.Set(0)
        else:
            self.ip.TargetVelocityVector = RMLDoubleVector(target_velocity)

        assert self.ip.CheckForValidity()
        return PositionTrajectory(self.rml, self.ip, self.op, self.flags)

    @property
    def max_velocity(self):
        return self.ip.MaxVelocityVector.tolist()

    @max_velocity.setter
    def max_velocity(self, max_velocity):
        self.ip.MaxVelocityVector = RMLDoubleVector(max_velocity)

    @property
    def max_acceleration(self):
        return self.ip.MaxAccelerationVector.tolist()

    @max_acceleration.setter
    def max_acceleration(self, max_acceleration):
        self.ip.MaxAccelerationVector = RMLDoubleVector(max_acceleration)

    @property
    def max_jerk(self):
        return self.ip.MaxJerkVector.tolist()

    @max_jerk.setter
    def max_jerk(self, max_jerk):
        self.ip.MaxJerkVector = RMLDoubleVector(max_jerk)

    @property
    def current_position(self):
        return self.ip.CurrentPositionVector.tolist()

    @current_position.setter
    def current_position(self, position):
        self.ip.CurrentPositionVector = RMLDoubleVector(position)

    @property
    def current_velocity(self):
        return self.ip.CurrentVelocityVector.tolist()

    @current_velocity.setter
    def current_velocity(self, velocity):
        self.ip.CurrentVelocityVector = RMLDoubleVector(velocity)

    @property
    def current_acceleration(self):
        return self.ip.CurrentAccelerationVector.tolist()

    @current_acceleration.setter
    def current_acceleration(self, acceleration):
        self.ip.CurrentAccelerationVector = RMLDoubleVector(acceleration)


class PositionTrajectory(object):
    def __init__(self, rml, ip, op, flags):
        self.rml = rml
        self.ip = ip
        self.op = op
        self.flags = flags

    def __iter__(self):
        return self

    def next(self):
        ret = self.rml.RMLPosition(self.ip, self.op, self.flags)

        if ret < 0:
            raise RMLError(ret)

        if ret == ReflexxesAPI.RML_FINAL_STATE_REACHED:
            raise StopIteration

        # Feedback
        self.ip.CurrentPositionVector = self.op.NewPositionVector
        self.ip.CurrentVelocityVector = self.op.NewVelocityVector
        self.ip.CurrentAccelerationVector = self.op.NewAccelerationVector

        return self.op.NewPositionVector, self.op.NewVelocityVector, self.op.NewAccelerationVector

