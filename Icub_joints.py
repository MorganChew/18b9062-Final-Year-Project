"""Icub_joints controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Keyboard
# create the Robot instance.
robot = Robot()
robot_name = robot.getName()
print('Name of the robot: ' + robot_name + '\n')
# get the time step of the current world.

#Create funtion for set and get
#right arm (Use better naming)
r1 = robot.getDevice('right_arm_1')
r2 = robot.getDevice('right_arm_2')
r3 = robot.getDevice('right_arm_3')
r4 = robot.getDevice('right_elbow')
r5 = robot.getDevice('right_forearm')
r6 = robot.getDevice('right_wrist_1')
r7 = robot.getDevice('right_wrist_2')

#left arm 
l1 = robot.getDevice('left_arm_1')
l2 = robot.getDevice('left_arm_2')
l3 = robot.getDevice('left_arm_3')
l4 = robot.getDevice('left_elbow')
l5 = robot.getDevice('left_forearm')
l6 = robot.getDevice('left_wrist_1')
l7 = robot.getDevice('left_wrist_2')

#get sensor
#to checks if correct or incorrect movement has occured
#obstacle detection
#function to get sensors

#Neck (3)

#Torso (3)

#right hand and left hand

#init 
#obj 1 call and send four args to initialize
#Default sets in place (init everything)
#it would decide which limb to initialize
#function(arg1,arg2,arg3,arg4)

#the init values will depend on the situation
#TBD

r1.setPosition(0)
r2.setPosition(0)
r3.setPosition(0)
r4.setPosition(0)
r5.setPosition(0)
r6.setPosition(0)
r7.setPosition(0)

l1.setPosition(0)
l2.setPosition(0)
l3.setPosition(0)
l4.setPosition(0)
l5.setPosition(0)
l6.setPosition(0)
l7.setPosition(0)

#
def move(bp1,bp2):
#if bp1 contains right arm
#use dict

    if(limb==1):
        x=1
        r1.setPosition(x)
        r1.setVelocity(1)
    elif(limb==2):
        x=2
        r2.setPosition(x)
        r2.setVelocity(1)     
    elif(limb==3):
        x=1
        r3.setPosition(x)
        r3.setVelocity(1)
    elif(limb==4):
        x=1
        r4.setPosition(x)
        r4.setVelocity(1)      
    elif(limb==5):
        x=0.5
        r5.setPosition(x)
        r5.setVelocity(1)
    elif(limb==6):
        x=1
        r6.setPosition(x)
        r6.setVelocity(1)   
    elif(limb==7):
        x=1
        r7.setPosition(x)
        r7.setVelocity(1)

  
while True:
#function1(getting access to the joins/senors)
#function2(read sensors to get current value)
#function3(init)

#part 2 The Format on (how to present the output) (eg, list,dict)
#function4/move(args such as the limbs (5 args))
#joint angles 

#body_part(as an arg) = 7 joints contain in the arms (others may vary)
#move the arm (forward postion and get joint angles)
#list them down to create the dict

#freeze the legs (TBD)
#sit down(?)
#stick legs to the ground(?)
#create a chair for icub to sit on(best op)

#do the conversion, add joints yourself(accoount for fingers), replace with a gripper
#getcamera
#add sensor to each joint (softillusion webots)
#tutorial 13 for hand  16 for sensor



    print("------------COMMANDS--------------")
    timestep = int(robot.getBasicTimeStep())
    keyboard = Keyboard()
    keyboard.enable(timestep)
    while robot.step(timestep) != -1:

        key = keyboard.getKey()

        if key == Keyboard.SHIFT + ord('A'):
            move(1)
          
        elif key == Keyboard.SHIFT + ord('B'):
            move(2)
                
        elif key == Keyboard.SHIFT + ord('C'):
             move(3)
            
        elif key == Keyboard.SHIFT + ord('D'):
             move(4)
            
        elif key == Keyboard.SHIFT + ord('E'):
             move(5)
         
        elif key == Keyboard.SHIFT + ord('F'):
             move(6)
          
        elif key == Keyboard.SHIFT + ord('G'):
             move(7)