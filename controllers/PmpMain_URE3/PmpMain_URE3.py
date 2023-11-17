from controller import Supervisor
from pmpClass import PMP
from myBot import *
#from ikpy_chain import create_ikpy_chain
import numpy as np


#RTB
#my_bot = create_robot()
#my_bot = create_robot3dof()
#my_bot= create_robot4dof()
my_bot = create_URE3()
#my_bot = create_robot3dof_for3D()
#print(my_bot.q)
#WEBOTS
# Initialize the Supervisor
fyp_supervisor = Supervisor()

def define_target(supervisor,target=None):
    target = supervisor.getFromDef(target) # Get the target object from the Supervisor
    target_position = target.getPosition()
    # Get the position of the target object
    #print("Ball Pos:", target_position)
    return target_position


def set_motors_forURE3(robot,motor_positions):
    motors = []
    motor_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint","wrist_1_joint","wrist_2_joint", "wrist_3_joint"]
    #the other joints from the proto , "wrist_1_joint","wrist_2_joint", "wrist_3_joint"
    for motor_name in motor_names:
        motor = robot.getDevice(motor_name)
        motor.setVelocity(0.2)
        motors.append(motor)
        print(motors)
    while robot.step(16) != -1:
        for i in range(len(motors)):
            motors[i].setPosition(motor_positions[i])

#PMP
# Define stiffness and admittance matrices
stiffness_matrix = [[0.05,0,0],[0,0.02,0],[0,0,0.01]]
admittance_matrix = np.identity(6)*0.1



#stiffness_matrix = np.identity(3) #make 3d matrics
# Create the PMP controller instance using Robotic Toolbox
pmp_controller = PMP(
    my_bot, K_ext=stiffness_matrix, A_int=admittance_matrix, robotics_Library='rtb',
    rmse_threshold=0.005,time_limit=1000, step_size=1/5) 
target_position= define_target(fyp_supervisor,target='Ball')
#target_position[2]=0
print("target_position is:",target_position) 
q_traj, time, rmse, x_error = pmp_controller.execute_pmp_kinematics(target_position)
 #the k_ext and torque are not inistailize
#print("q_traj", q_traj)
#print("time:", time)
#print("rmse:", rmse)
#print("x_error:", x_error)
print(my_bot.n)

motor_positions = q_traj[-1] 

#print("motor_positions:",motor_positions)

set_motors_forURE3(fyp_supervisor, motor_positions)





