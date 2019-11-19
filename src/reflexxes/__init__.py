try:
    # Prefer Type IV
    from .reflexxes_type4 import (
        error_string,
        ReflexxesAPI,
        RMLFlags,
        RMLInputParameters,
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
        RMLFlags,
        RMLInputParameters,
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

        self.ip.SelectionVector = [True] * n_dof
        self.ip.MaxVelocityVector = max_velocity
        self.ip.MaxAccelerationVector = max_acceleration

        if max_jerk:  # needed for RML Type IV
            self.ip.MaxJerkVector = max_jerk

    def trajectory(self, target_position, target_velocity=None, min_sync_time=None):
        self.ip.MinimumSynchronizationTime = 0.0 if min_sync_time is None else min_sync_time
        self.ip.TargetPositionVector = target_position

        if target_velocity is None:
            self.ip.TargetVelocityVector = [0] * self.n_dof
        else:
            self.ip.TargetVelocityVector = target_velocity

        assert self.ip.CheckForValidity()

        return PositionTrajectory(self.rml, self.ip, self.op, self.flags)

    @property
    def max_velocity(self):
        return self.ip.MaxVelocityVector

    @max_velocity.setter
    def max_velocity(self, max_velocity):
        self.ip.MaxVelocityVector = max_velocity

    @property
    def max_acceleration(self):
        return self.ip.MaxAccelerationVector

    @max_acceleration.setter
    def max_acceleration(self, max_acceleration):
        self.ip.MaxAccelerationVector = max_acceleration

    @property
    def max_jerk(self):
        return self.ip.MaxJerkVector

    @max_jerk.setter
    def max_jerk(self, max_jerk):
        self.ip.MaxJerkVector = max_jerk

    @property
    def current_position(self):
        return self.ip.CurrentPositionVector

    @current_position.setter
    def current_position(self, position):
        self.ip.CurrentPositionVector = position

    @property
    def current_velocity(self):
        return self.ip.CurrentVelocityVector

    @current_velocity.setter
    def current_velocity(self, velocity):
        self.ip.CurrentVelocityVector = velocity

    @property
    def current_acceleration(self):
        return self.ip.CurrentAccelerationVector

    @current_acceleration.setter
    def current_acceleration(self, acceleration):
        self.ip.CurrentAccelerationVector = acceleration

    @property
    def target_position(self):
        return self.ip.TargetPositionVector

    @target_position.setter
    def target_position(self, position):
        self.ip.TargetPositionVector = position

    @property
    def target_velocity(self):
        return self.ip.TargetVelocityVector

    @target_velocity.setter
    def target_velocity(self, velocity):
        self.ip.TargetVelocityVector = velocity


class PositionTrajectory(object):
    def __init__(self, rml, ip, op, flags):
        self.rml = rml
        self.ip = ip
        self.op = op
        self.flags = flags

    def __iter__(self):
        return self

    def __next__(self):
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

    def next(self):
        # For Python 2.7 compatibility
        return self.__next__()
