import numpy as np
from controller import Supervisor
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink, URDFLink
from spatialmath import SE3

iCub_Supervisor = Supervisor()
timeStep = int(iCub_Supervisor.getBasicTimeStep()) * 4

target = iCub_Supervisor.getFromDef('Ball')
arm = iCub_Supervisor.getSelf()
if target is None:
    raise ValueError("Could not find target node")


sensor_names = ["torso_1_PS", "torso_2_PS", "torso_3_PS", "left_arm_1_PS", "left_arm_2_PS", "left_arm_3_PS", "left_elbow_PS", "left_forearm_PS", "left_wrist_1_PS", "left_wrist_2_PS"]
for sensor_name in sensor_names:
    position_sensor = iCub_Supervisor.getDevice(sensor_name)
    position_sensor.enable(timeStep)

motor_names = ["torso_1", "torso_2", "torso_3", "left_arm_1", "left_arm_2", "left_arm_3", "left_elbow", "left_forearm", "left_wrist_1", "left_wrist_2"]
motors = []
for motor_name in motor_names:
    motor = iCub_Supervisor.getDevice(motor_name)
    motor.setVelocity(0.2)
    motors.append(motor)

targetPosition = target.getPosition()
armPosition = arm.getPosition()
x = targetPosition[0] - armPosition[0]
y = targetPosition[1] - armPosition[1]
z = targetPosition[2] - armPosition[2]
print("x:", x, "y:", y, "z:", z)

target_pose = [x, y, z]

#IKPY Joints
""""
dh_params = [
    DHLink(d=0.032, a=0, alpha=np.pi/2, offset=0),
    DHLink(d=0, a=-0.0055, alpha=np.pi/2, offset=-np.pi/2),
    DHLink(d=0.0233647, a=-0.1433, alpha=-np.pi/2, offset=105*np.pi/180),
    DHLink(d=0, a=0.10774, alpha=-np.pi/2, offset=np.pi/2),
    DHLink(d=0, a=0, alpha=np.pi/2, offset=-np.pi/2),
    DHLink(d=0.015, a=0.15228, alpha=-np.pi/2, offset=75*np.pi/180),
    DHLink(d=-0.015, a=0, alpha=np.pi/2, offset=0),
    DHLink(d=0, a=0.1373, alpha=np.pi/2, offset=-np.pi/2),
    DHLink(d=0, a=0, alpha=np.pi/2, offset=np.pi/2),
    DHLink(d=0.0625, a=-0.016, alpha=0, offset=0)
]
"""
""""
dh_params = [
    DHLink(name="link1", d=0.032, a=0, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link2", d=0, a=-0.0055, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link3", d=0.0233647, a=-0.1433, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link4", d=0, a=0.10774, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link5", d=0, a=0, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link6", d=0.015, a=0.15228, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link7", d=-0.015, a=0, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link8", d=0, a=0.1373, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link9", d=0, a=0, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link10", d=0.0625, a=-0.016, bounds=None, use_symbolic_matrix=True)
]
"""
"""
dh_params = [
    DHLink(name="link1", length=0.0, d=0.032, a=0, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link2", length=0.0, d=0, a=-0.0055, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link3", length=0.0, d=0.0233647, a=-0.1433, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link4", length=0.0, d=0, a=0.10774, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link5", length=0.0, d=0, a=0, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link6", length=0.0, d=0.015, a=0.15228, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link7", length=0.0, d=-0.015, a=0, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link8", length=0.0, d=0, a=0.1373, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link9", length=0.0, d=0, a=0, bounds=None, use_symbolic_matrix=True),
    DHLink(name="link10", length=0.0, d=0.0625, a=-0.016, bounds=None, use_symbolic_matrix=True)
]

links = []
links.append(OriginLink())

for i in range(len(dh_params)):
    links.append(dh_params[i])
"""

#From iCub_joints_ik
link_joint_names = ['base_link','torso_1','torso','torso_2','solid','torso_3','solid_8','left_arm_1','left_arm', 

                'left_arm_2','solid_9','left_arm_3','solid_10','left_elbow','solid_11','left_forearm','solid_12','left_wrist_1', 'solid_13','left_wrist_2', 'solid_14'] 

active_joints_mask1 = [False,False, False,False,True,True,True,True,True,True,True] 

#look up the rtb version of this, then fkin
iCub_Robot = Chain.from_urdf_file(urdf_file="Robot2.urdf",base_elements=link_joint_names,active_links_mask=active_joints_mask1) 
print(iCub_Robot)


#IKPY Calculations
ikResults = iCub_Robot.inverse_kinematics(target_pose)
position_value = iCub_Robot.forward_kinematics(ikResults)
position = np.array(position_value)  
print("position:", position)
print("ikresults of Ikpy:", ikResults)

for i in range(len(motors)):
    motors[i].setPosition(ikResults[i])
