# robotics_toolbox.py
import roboticstoolbox as rtb
import numpy as np


def create_robot2dof():
    t0 = rtb.ET.tx(-0.5)
    r1 = rtb.ET.Rz()
    t1 = rtb.ET.tx(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(1)
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2]))
    return my_bot


def create_robot3dof():
    t0 = rtb.ET.tx(-0.5)
    r1 = rtb.ET.Rz()
    t1 = rtb.ET.tx(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(1)
    r3 = rtb.ET.Rz()
    t3 = rtb.ET.tx(1)
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2,r3,t3]))
    
    return my_bot

def create_robot4dof():
    t0 = rtb.ET.tx(-0.5)
    r1 = rtb.ET.Rz()
    t1 = rtb.ET.tx(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(1)
    r3 = rtb.ET.Rz()
    t3 = rtb.ET.tx(1)
    r4 = rtb.ET.Rz()
    t4 = rtb.ET.tx(1)
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2,r3,t3,r4,t4]))
    
    return my_bot

#make it move in 3d
#how to it come to a solution in a nice way
#software guideline in python
#create sample test in code
#peps.python style guideline.python 
#design some unittest (from python docs)
def create_robot2dof_for3D():
    # Define the transformation sequences for your robot
    t0 = rtb.ET.tz(0.2)  # Modify the translation along the z-axis
    r1 = rtb.ET.Ry()
    t1 = rtb.ET.tx(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(1)

    # Define the robot using the ETS sequence
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2]))

    return my_bot



def create_robot3dof_for3D_Old():
    # Define the transformation and rotaionation sequences for your robot
    t0 = rtb.ET.tz(0.5)  # Modify the translation along the z-axis
    r1 = rtb.ET.Ry()
    t1 = rtb.ET.tz(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(0.5)
    r3 = rtb.ET.Rz()
    t3 = rtb.ET.tx(1)

    # Define the robot using the ETS sequence
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2,r3,t3]))

    return my_bot


def create_URE3_New():
    # Define the transformation and rotaionation sequences for your robot
    t0 = rtb.ET.tz(0.0244)
    t1 = rtb.ET.tz(0.0244)
    r1 = rtb.ET.Rx()
    t2 = rtb.ET.tz(0.0244)
    r2 = rtb.ET.Rz()
    t3 = rtb.ET.tx(-0.0392)
    t4 = rtb.ET.tz(0.02)
    r3 = rtb.ET.Rz()
    t5 = rtb.ET.tx(-0.0392)
    t6 = rtb.ET.tz(0.02)
    r4 = rtb.ET.Rx()
    t7 = rtb.ET.tx(-0.0392)
    t8 = rtb.ET.tz(0.02)
    r5 = rtb.ET.Rx()
    t9 = rtb.ET.tz(0.152)
    t9 = rtb.ET.tx(0.12)
    r6 = rtb.ET.Ry()
    t10 = rtb.ET.ty(0.12)
    t11 = rtb.ET.tz(0.12)


    

    
    

    # Define the robot using the ETS sequence
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2,r3,t3,t4,t5,r4,t6,t7,r5,t8,t9,r6,t10,t11]))
    
    return my_bot
    #rtb visualations
    #import urdf to rtb 
    #modifty pmpmain into a controller for separate robots projects
    #Three joints, 2Joints planar , 3dof spacial, 6 dof

def create_URE3():
    # Define the transformation and rotaionation sequences for your robot
    t0 = rtb.ET.tz(0.5)  # Modify the translation along the z-axis
    t1 = rtb.ET.tz(1)
    r1 = rtb.ET.Ry()
    t2 = rtb.ET.tz(1)
    r2 = rtb.ET.Rz()

    t3 = rtb.ET.tx(0.5)
    t4 = rtb.ET.tz(-1)
    r3 = rtb.ET.Rz()
    t4 = rtb.ET.tx(1)
    t5 = rtb.ET.tz(1)

    r4 = rtb.ET.Rx()
    t6 = rtb.ET.tx(1)
    t7 = rtb.ET.tz(1)

    r5 = rtb.ET.Rz()
    t8 = rtb.ET.tx(1)
    t9 = rtb.ET.tz(1)

    r6 = rtb.ET.Rz()
    t10 = rtb.ET.tx(1)
    t11 = rtb.ET.tz(1)
    
    

    # Define the robot using the ETS sequence
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2,r3,t3,t4,t5,r4,t6,t7,r5,t8,t9,r6,t10,t11]))
    
    return my_bot