import unittest
import numpy as np
import roboticstoolbox as rtb
from pmpClass import PMP

class TestPMP(unittest.TestCase):

    def setUp(self):
        # Create a test robot
        t0 = rtb.ET.tx(-0.5)
        r1 = rtb.ET.Rz()
        t1 = rtb.ET.tx(1)
        r2 = rtb.ET.Rz()
        t2 = rtb.ET.tx(1)
        r3 = rtb.ET.Rz()
        t3 = rtb.ET.tx(1)
        my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2, r3, t3]))

        # Create a PMP object for testing
        self.pmp = PMP(
        my_bot, K_ext= np.identity(3), A_int= np.identity(3), robotics_Library='rtb',
        rmse_threshold=0.005,time_limit=1000, step_size=1/5) 

    def test_calculate_fkine(self):
        q_current = np.array([0.1, 0.2, 0.3])
        fkine_result = self.pmp.calculate_fkine(q_current)
        self.assertIsNotNone(fkine_result)

    def test_calculate_error_vector(self):
        setpoint_X = np.array([1.0, 2.0, 3.0])
        current_X = np.array([0.9, 2.1, 2.8])
        error_vector = self.pmp.calculate_error_vector(setpoint_X, current_X)
        self.assertIsNotNone(error_vector)

    def test_calculate_setpoint_x(self):
        x_target = np.array([1.0, 2.0, 3.0])
        setpoint_X = self.pmp.calculate_setpoint_x(x_target)
        self.assertIsNotNone(setpoint_X)

    def test_calculate_current_x(self):
        q_current = np.array([0.1, 0.2, 0.3])
        current_X = self.pmp.calculate_current_x(q_current)
        self.assertIsNotNone(current_X)

    def test_calculate_jacobian(self):
        q_current = np.array([0.1, 0.2, 0.3])
        jacobian = self.pmp.calculate_jacobian(q_current)
        self.assertIsNotNone(jacobian)
    
    
    def test_calculate_Torque(self):
        Jacobian = np.identity(3)
        Force = np.array([1.0, 2.0, 3.0])
        torque = self.pmp.calculate_Torque(Jacobian, Force)
        self.assertIsNotNone(torque)

    def test_calculate_q_dot(self):
        A_int = np.identity(3)
        Torque = np.array([1.0, 2.0, 3.0])
        q_dot = self.pmp.calculate_q_dot(A_int, Torque)
        self.assertIsNotNone(q_dot)

    def test_calculate_Force(self):
        K_ext = np.identity(3)
        x_error = np.array([0.1, 0.2, 0.3])
        force = self.pmp.calculate_Force(K_ext, x_error)
        self.assertIsNotNone(force)

    def test_calculate_final_q_current(self):
        q_traj = np.array([[0.1, 0.2, 0.3], [0.2, 0.3, 0.4], [0.3, 0.4, 0.5]])
        q_current = self.pmp.calculate_final_q_current(q_traj)
        self.assertIsNotNone(q_current)
    
    def test_calculate_Rmse(self):
        # Test the calculate_Rmse method with known inputs
        x_error = [0.5, 1.5, 2.5]
        result = self.pmp.calculate_Rmse(x_error)
        self.assertIsNotNone(result)

    def test_run_step(self):
        # Test the run_step method with known inputs
        x_target = [1.0, 2.0, 3.0]
        q_current = [0.0, 0.0, 0.0]  # Example joint angles
        K_ext = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        A_int = [[1.0, 2.0], [3.0, 4.0]]
        result = self.pmp.run_step(x_target, q_current, K_ext, A_int)
        self.assertIsNotNone(result)

    def test_pmp_kinematics(self):
        # Test the pmp_kinematics method with known inputs
        target_position = [1.0, 2.0, 3.0]
        K_ext = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        A_int = [[1.0, 2.0], [3.0, 4.0]]
        result = self.pmp.pmp_kinematics(target_position, K_ext, A_int)
        self.assertIsNotNone(result)

    def test_calculate_final_q_current(self):
        # Test the calculate_final_q_current method with known inputs
        q_traj = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9]]
        result = self.pmp.calculate_final_q_current(q_traj)
        self.assertEqual(result, [0.7, 0.8, 0.9])

    def test_execute_pmp_kinematics(self):
        # Test the execute_pmp_kinematics method with known inputs
        target_position = [1.0, 2.0, 3.0]
        K_ext = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        A_int = [[1.0, 2.0], [3.0, 4.0]]
        result = self.pmp.execute_pmp_kinematics(target_position, K_ext, A_int)
        self.assertIsNotNone(result)


    def test_step(self):
        x = np.array([0.0, 0.0, 0.0])
        x_dot = np.array([0.0, 0.0, 0.0])
        force = np.array([0.0, 0.0, 0.0])
        q = np.array([0.1, 0.2, 0.3])
        q_dot = np.array([0.0, 0.0, 0.0])
        torque = np.array([0.0, 0.0, 0.0])
        setpoint_x = np.array([1.0, 2.0, 3.0])
        K_ext = np.identity(3)
        A_int = np.identity(3)
     
        x_result, x_dot_result, force_result, q_result, q_dot_result, torque_result = self.pmp.step(
            x=x,
            x_dot=x_dot,
            force=force,
            q=q,
            q_dot=q_dot,
            torque=torque,
            setpoint_x=setpoint_x,
            K_ext=K_ext,
            A_int=A_int,
      
        )
        self.assertIsNotNone(x_result)
        self.assertIsNotNone(force_result)
        self.assertIsNotNone(q_result)
        self.assertIsNotNone(q_dot_result)
        self.assertIsNotNone(torque_result)

if __name__ == '__main__':
    unittest.main()
