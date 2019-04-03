from ._reflexxes_type2 import (
    __version__,
    ReflexxesAPI,
    RMLBoolVector,
    RMLDoubleVector,
    RMLIntVector,
    RMLPositionFlags,
    RMLPositionInputParameters,
    RMLPositionOutputParameters,
    RMLVelocityFlags,
    RMLVelocityInputParameters,
    RMLVelocityOutputParameters,

)


class RMLError(Exception):
    def __init__(self, code):
        super(RMLError, self).__init__()
        self.code = code


class PositionTrajectoryGenerator(object):
    def __init__(self, n_dof, cycle_time, max_velocity, max_acceleration):
        assert len(max_velocity) == n_dof
        assert len(max_acceleration) == n_dof

        self.n_dof = n_dof
        self.rml = ReflexxesAPI(n_dof, cycle_time)
        self.ip = RMLPositionInputParameters(n_dof)
        self.op = RMLPositionOutputParameters(n_dof)
        self.flags = RMLPositionFlags()

        self.ip.SelectionVector.Set(True)
        self.ip.MaxVelocityVector = RMLDoubleVector(*max_velocity)
        self.ip.MaxAccelerationVector = RMLDoubleVector(*max_acceleration)

    def trajectory(self, target_position, target_velocity, min_sync_time=0.0):
        assert len(target_position) == self.n_dof
        assert len(target_velocity) == self.n_dof
        self.ip.TargetPositionVector = RMLDoubleVector(*target_position)
        self.ip.TargetVelocityVector = RMLDoubleVector(*target_velocity)
        self.ip.MinimumSynchronizationTime = min_sync_time
        assert self.ip.CheckForValidity()
        return PositionTrajectory(self.rml, self.ip, self.op, self.flags)

    @property
    def max_velocity(self):
        return self.ip.MaxVelocityVector.tolist()

    @max_velocity.setter
    def max_velocity(self, max_velocity):
        assert len(max_velocity) == self.n_dof
        self.ip.MaxVelocityVector = RMLDoubleVector(*max_velocity)

    @property
    def max_acceleration(self):
        return self.ip.MaxAccelerationVector.tolist()

    @max_acceleration.setter
    def max_acceleration(self, max_acceleration):
        assert len(max_acceleration) == self.n_dof
        self.ip.MaxAccelerationVector = RMLDoubleVector(*max_acceleration)

    @property
    def current_position(self):
        return self.ip.CurrentPositionVector.tolist()

    @current_position.setter
    def current_position(self, position):
        assert len(position) == self.n_dof
        self.ip.CurrentPositionVector = RMLDoubleVector(*position)

    @property
    def current_velocity(self):
        return self.ip.CurrentVelocityVector.tolist()

    @current_velocity.setter
    def current_velocity(self, velocity):
        assert len(velocity) == self.n_dof
        self.ip.CurrentVelocityVector = RMLDoubleVector(*velocity)

    @property
    def current_acceleration(self):
        return self.ip.CurrentAccelerationVector.tolist()

    @current_acceleration.setter
    def current_acceleration(self, acceleration):
        assert len(acceleration) == self.n_dof
        self.ip.CurrentAccelerationVector = RMLDoubleVector(*acceleration)


class PositionTrajectory(object):
    def __init__(self, rml, ip, op, flags):
        self.rml = rml
        self.ip = ip
        self.op = op
        self.flags = flags
        self.ret = 0

    def __iter__(self):
        return self

    def next(self):
        # Check the previous return value here, so that we update/return the
        # vectors of the final step.
        if self.ret == ReflexxesAPI.RMLResultValue.RML_FINAL_STATE_REACHED:
            raise StopIteration

        self.ret = self.rml.RMLPosition(self.ip, self.op, self.flags)

        if self.ret < 0:
            raise RMLError(ret)

        # Feedback
        self.ip.CurrentPositionVector = self.op.NewPositionVector
        self.ip.CurrentVelocityVector = self.op.NewVelocityVector
        self.ip.CurrentAccelerationVector = self.op.NewAccelerationVector

        return self.op.NewPositionVector, self.op.NewVelocityVector, self.op.NewAccelerationVector

