from controller import Robot
from controller import PositionSensor
TIME_STEP=64
robot = Robot()
robot_name = robot.getName()
print('Name of the robot: ' + robot_name + '\n')

#Get Joints
left_arm_joints=[robot.getDevice('left_arm_1'),robot.getDevice('left_arm_2'),robot.getDevice('left_arm_3'),robot.getDevice('left_elbow'),robot.getDevice('left_forearm'),robot.getDevice('left_wrist_1'),robot.getDevice('left_wrist_2')]
right_arm_joints=[robot.getDevice('right_arm_1'),robot.getDevice('right_arm_2'),robot.getDevice('right_arm_3'),robot.getDevice('right_elbow'),robot.getDevice('right_forearm'),robot.getDevice('right_wrist_1'),robot.getDevice('right_wrist_2')]
torso_joints=[robot.getDevice('torso_1'),robot.getDevice('torso_2'),robot.getDevice('torso_3')]
neck_joints=[robot.getDevice('NECK_TILT'),robot.getDevice('NECK_SWING'),robot.getDevice('NECK_PAN')]
left_leg_joints=[robot.getDevice('left_leg_1'),robot.getDevice('left_leg_2'),robot.getDevice('left_leg_3'),robot.getDevice('left_knee'),robot.getDevice('left_ankle')]
right_leg_joints=[robot.getDevice('right_leg_1'),robot.getDevice('right_leg_2'),robot.getDevice('right_leg_3'),robot.getDevice('right_knee'),robot.getDevice('right_ankle')]

#default =[(left_arm_joints[0], {'position': 1.0,'velocity': 1.0}),(left_arm_joints[1], {'position': 0.5,'velocity': 1.0}),(left_arm_joints[2], {'position': -0.5,'velocity': 1.0}),(left_arm_joints[3], {'position': 0.8,'velocity': 1.0}),(left_arm_joints[4], {'position': -0.2,'velocity': 1.0}),(left_arm_joints[5], {'position': 0.5,'velocity': 1.0}),(left_arm_joints[6], {'position': -0.5,'velocity': 1.0})]
#default = [(right_arm_joints[0], {'position': -1.0,'velocity': 1.0}), (right_arm_joints[1], {'position': -0.5,'velocity': 1.0}), (right_arm_joints[2], {'position': 0.5,'velocity': 1.0})]            

#Defines the MoveLimb function
def move_Limb(left_arm=None, right_arm=None, torso=None, neck=None, left_leg=None, right_leg=None):
    if left_arm:
        for joint, values in left_arm:
            #if joint:
            joint.setPosition(values['position'])
            joint.setVelocity(values['velocity'])
    if right_arm:
        for joint, values in right_arm:
            joint.setPosition(values['position'])
            joint.setVelocity(values['velocity']) 
    if torso:
        for joint, values in right_arm:
            joint.setPosition(values['position'])
            joint.setVelocity(values['velocity'])
    if neck:
        for joint, values in right_arm:
            joint.setPosition(values['position'])
            joint.setVelocity(values['velocity'])
    if left_leg:
        for joint, values in right_arm:
            joint.setPosition(values['position'])
            joint.setVelocity(values['velocity'])
    if right_leg:
        for joint, values in right_arm:
            joint.setPosition(values['position'])
            joint.setVelocity(values['velocity'])





#movesets
leftArmSet1=[1,1,1,1,0,1,1]
leftArmVelocity=[1,1,1,1,1,1,1]
left_arm_movement1 = [(left_arm_joints[i], {'position': leftArmSet1[i],'velocity': leftArmVelocity[i]}) for i in range(len(left_arm_joints))]

leftArmSet2=[1.5708,0.7854,0.7854,2.4435,-1.0472 ,1,1]
left_arm_movement2 = [(left_arm_joints[i], {'position': leftArmSet2[i],'velocity': leftArmVelocity[i]}) for i in range(len(left_arm_joints))]

leftArmSet3=[1.5,1.5,-1,1.5,0,0,0]
left_arm_movement3 = [(left_arm_joints[i], {'position': leftArmSet3[i],'velocity': leftArmVelocity[i]}) for i in range(len(left_arm_joints))]

rightArmSet1=[1,1,-1,2,0,1,1]
right_arm_movement1 = [(right_arm_joints[i], {'position': rightArmSet1[i],'velocity': 1.0}) for i in range(len(right_arm_joints))]
#torso_movement = [(torso_joints[i], {'position': Set1[i],'velocity': 1.0}) for i in range(len(torso_joints))]
#neck_movement = [(neck_joints[i], {'position': Set1[i],'velocity': 1.0}) for i in range(len(neck_joints))]
#left_leg_movement = [(left_leg_joints[i], {'position': Set1[i],'velocity': 1.0}) for i in range(len(left_leg_joints))]
#right_leg_movement = [(right_leg_joints[i], {'position': Set1[i],'velocity': 1.0}) for i in range(len(right_leg_joints))]

#calls function that Moves the robot's limbs
#move_Limb(left_arm=left_arm_movement3,right_arm=right_arm_movement1)



