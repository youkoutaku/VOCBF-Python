import unittest
import numpy as np
from vocbf.models import Model, KINDS
from vocbf.qp import quadratic
class Contracts(unittest.TestCase):
    def test_model_and_input_shapes(self):
        for kind in KINDS:
            m=Model(kind);x=m.initial()
            self.assertEqual(m.rhs(x,np.zeros(2)).shape,(m.nx,))
            sample=m.integrate(x,np.zeros(2),.05)
            self.assertTrue(np.isfinite(sample).all())
    def test_infeasibility_is_not_a_braking_command(self):
        u,_=quadratic(np.eye(2),np.zeros(2),[[1,0],[-1,0]],[-1,-1])
        self.assertIsNone(u)
    def test_second_order_input_contract(self):
        m=Model('dynamic_unicycle');x=np.array([0.,0.,.4,.7,-.3]);u=np.array([.2,.1])
        np.testing.assert_allclose(m.rhs(x,u),[.7*np.cos(.4),.7*np.sin(.4),-.3,.2,.1])
    def test_nominal_uses_requested_hold_for_velocity_bounds(self):
        for kind in ('double_integrator','dynamic_unicycle'):
            m=Model(kind);x=m.initial()
            if kind=='double_integrator':x[2:]=[1.49,0.]
            else:x[2:]=[0.,1.49,0.]
            dt=.2;u=m.nominal(x,np.array([20.,0.]),dt)
            lo,hi=m.bounds(x,dt)
            self.assertTrue(np.all(u>=lo-1e-12) and np.all(u<=hi+1e-12))
            self.assertLessEqual(m.integrate(x,u,dt)[-1][2 if kind=='double_integrator' else 3],m.vmax+1e-10)
if __name__=='__main__':unittest.main()
