import unittest
from reflexxes import *


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


class TestRML(unittest.TestCase):
    def test_flags(self):
        flags1 = RMLPositionFlags()
        flags2 = RMLPositionFlags()
        assert flags1 == flags2
        flags1.SynchronizationBehavior = RMLFlags.SyncBehavior.NO_SYNCHRONIZATION
        assert flags1 != flags2

    def test_vector(self):
        v1 = RMLDoubleVector([1, 2, 3, 4, 5, 6, 7])
        v2 = RMLDoubleVector([1, 2, 3, 4, 5, 6, 7])
        assert v1 == v2
        assert v1 != RMLDoubleVector([0, 2, 3, 4, 5, 6, 7])

        v3 = RMLDoubleVector([0]*123)
        v4 = RMLDoubleVector([0]*123)
        assert 123 == len(v3) == len(v4)

        for i in xrange(len(v3)):
            v3[i] = i
            v4[i] = i

        for i, val in enumerate(v3):
            assert val == i

        assert v3 == v4

        with self.assertRaises(IndexError):
            val = v3[len(v3)+1]

    def test_api(self):
        NUMBER_OF_DOFS = 3
        CYCLE_TIME_IN_SECONDS = 0.001

        flags = RMLPositionFlags()
        rml = ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS)
        ip = RMLPositionInputParameters(NUMBER_OF_DOFS)
        op = RMLPositionOutputParameters(NUMBER_OF_DOFS)

        ip.SelectionVector = RMLBoolVector([True, True, True])

        ip.CurrentPositionVector = RMLDoubleVector([100, 0, 50])
        ip.CurrentVelocityVector = RMLDoubleVector([100, -220, -50])
        ip.CurrentAccelerationVector = RMLDoubleVector([-150, 250, -50])

        ip.MaxVelocityVector = RMLDoubleVector([300, 100, 300])
        ip.MaxAccelerationVector = RMLDoubleVector([300, 200, 100])
        ip.MaxJerkVector = RMLDoubleVector([400, 300, 200])

        ip.TargetPositionVector = RMLDoubleVector([-600, -200, -350])
        ip.TargetVelocityVector = RMLDoubleVector([50, -50, -200])

        assert ip.CheckForValidity()

        ret = rml.RMLPosition(ip, op, flags)

        assert ret >= 0
        assert op.ANewCalculationWasPerformed
        assert all(isclose(a, b, rel_tol=0.001, abs_tol=0.3) for a, b in zip(op.NewPositionVector, ip.CurrentPositionVector))
        assert all(isclose(a, b, rel_tol=0.01) for a, b in zip(op.NewVelocityVector, ip.CurrentVelocityVector))
        assert all(isclose(a, b, rel_tol=0.5) for a, b in zip(op.NewAccelerationVector, ip.CurrentAccelerationVector))


if __name__ == '__main__':
    unittest.main()
