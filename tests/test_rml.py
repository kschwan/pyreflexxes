import reflexxes
import unittest


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


class TestRML(unittest.TestCase):
    def test_flags(self):
        flags1 = reflexxes.RMLPositionFlags()
        flags2 = reflexxes.RMLPositionFlags()
        assert flags1 == flags2
        flags1.SynchronizationBehavior = reflexxes.RMLFlags.NO_SYNCHRONIZATION
        assert flags1 != flags2

    def test_vector(self):
        v1 = reflexxes.RMLDoubleVector([1, 2, 3, 4, 5, 6, 7])
        v2 = [1, 2, 3, 4, 5, 6, 7]  # Implicit conversion
        assert v1 == v2
        assert v1 != reflexxes.RMLDoubleVector([0, 2, 3, 4, 5, 6, 7])
        v2[1] = 99
        v2[-1] = 999
        assert v2 == [1, 99, 3, 4, 5, 6, 999]  # Implicit conversion

        v3 = reflexxes.RMLDoubleVector([0]*123)
        v4 = reflexxes.RMLDoubleVector([0]*123)
        assert 123 == len(v3) == len(v4)

        for i in range(len(v3)):
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

        flags = reflexxes.RMLPositionFlags()
        rml = reflexxes.ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS)
        ip = reflexxes.RMLPositionInputParameters(NUMBER_OF_DOFS)
        op = reflexxes.RMLPositionOutputParameters(NUMBER_OF_DOFS)

        ip.SelectionVector = [True, True, True]

        ip.CurrentPositionVector = [100, 0, 50]
        ip.CurrentVelocityVector = [100, -220, -50]
        ip.CurrentAccelerationVector = [-150, 250, -50]

        ip.MaxVelocityVector = [300, 100, 300]
        ip.MaxAccelerationVector = [300, 200, 100]
        ip.MaxJerkVector = [400, 300, 200]

        ip.TargetPositionVector = [-600, -200, -350]
        ip.TargetVelocityVector = [50, -50, -200]

        assert ip.CheckForValidity()

        ret = rml.RMLPosition(ip, op, flags)

        assert ret >= 0
        assert op.ANewCalculationWasPerformed
        assert all(isclose(a, b, rel_tol=0.001, abs_tol=0.3) for a, b in zip(op.NewPositionVector, ip.CurrentPositionVector))
        assert all(isclose(a, b, rel_tol=0.01) for a, b in zip(op.NewVelocityVector, ip.CurrentVelocityVector))
        assert all(isclose(a, b, rel_tol=0.5) for a, b in zip(op.NewAccelerationVector, ip.CurrentAccelerationVector))


if __name__ == '__main__':
    unittest.main()
