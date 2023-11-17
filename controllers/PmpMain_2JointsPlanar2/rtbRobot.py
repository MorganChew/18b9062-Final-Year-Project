
import roboticstoolbox as rtb



def create_2JointsPlanar():
    #Defines the transformation sequences for your robot
    t0 = rtb.ET.tx(-0.5)
    r1 = rtb.ET.Rz()
    t1 = rtb.ET.tx(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(1)
    #Defines the robot using the ETS sequence
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2]))
    return my_bot


def create_2JointsPlanar2():
    #Defines the transformation sequences for your robot
    t0 = rtb.ET.tx(-0.25)
    r1 = rtb.ET.Rz()
    t1 = rtb.ET.tx(0.5)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(1.5)
    #Defines the robot using the ETS sequence
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2]))
    return my_bot


def create_3JointsPlanar():
    t0 = rtb.ET.tx(-0.5)
    r1 = rtb.ET.Rz()
    t1 = rtb.ET.tx(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(1)
    r3 = rtb.ET.Rz()
    t3 = rtb.ET.tx(1)
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2,r3,t3]))
    
    return my_bot


def create_3JointsPlanar2():
    t0 = rtb.ET.tx(-0.5)
    r1 = rtb.ET.Rz()
    t1 = rtb.ET.tx(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(1)
    r3 = rtb.ET.Rz()
    t3 = rtb.ET.tx(0.5)
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2,r3,t3]))
    
    return my_bot

def create_3JointsPlanar3():
    t0 = rtb.ET.tx(-0.25)
    r1 = rtb.ET.Rz()
    t1 = rtb.ET.tx(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(0.5)
    r3 = rtb.ET.Rz()
    t3 = rtb.ET.tx(0.5)
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2,r3,t3]))
    
    return my_bot


def create_robot2dof_for3D():
    t0 = rtb.ET.tz(0.2) 
    r1 = rtb.ET.Ry()
    t1 = rtb.ET.tx(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(1)

   
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2]))

    return my_bot

def create_robot3dof_for3D_Old():
    t0 = rtb.ET.tz(0.5)  
    r1 = rtb.ET.Ry()
    t1 = rtb.ET.tz(1)
    r2 = rtb.ET.Rz()
    t2 = rtb.ET.tx(0.5)
    r3 = rtb.ET.Rz()
    t3 = rtb.ET.tx(1)

    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2,r3,t3]))

    return my_bot

def create_robot3dof_for3D():
   
    t0 = rtb.ET.tz(0.5)  
    t1 = rtb.ET.tz(1)
    r1 = rtb.ET.Ry()
    t2 = rtb.ET.tz(1)
    r2 = rtb.ET.Rz()

    t3 = rtb.ET.tx(0.5)
    t4 = rtb.ET.tz(-1)
    r3 = rtb.ET.Rz()
    t4 = rtb.ET.tx(1)
    t5 = rtb.ET.tz(1)

    r4 = rtb.ET.Rz()
    t6 = rtb.ET.tx(1)
    t7 = rtb.ET.tz(1)
    my_bot = rtb.Robot(rtb.ETS([t0, r1, t1, r2, t2,r3,t3,t4,t5,r4,t6,t7]))
    
    return my_bot

