from controller import Supervisor
from rtbRobot import create_2JointsPlanar
#from ikpy_chain import create_ikpy_chain
from pmpClass import PMP
#RTB

my_bot[0] = create_2JointsPlanar()
my_bot[1] = create_2JointsPlanar()
#print("my bot:"my_bot.q)

#WEBOTS
# Initialize the Supervisor
fyp_supervisor = Supervisor()

def define_target(supervisor,target=None):
    if supervisor is None:
        raise ValueError("Error: supervisor is None")
    #print("is supervisor passed to define_target",supervisor)
    if target is None:
        raise ValueError("Error: target is None")
    #print("is target passed to define_target",target)
    target = supervisor.getFromDef(target) # Get the target object from the Supervisor
    target_position = target.getPosition()
    return target_position

def set_motors_for2joints(robot,motor_positions):
    print("are motor positions passed to set motors",motor_positions)
    motors = []
    motor_names = ["joint1", "joint2"]
    for motor_name in motor_names:
        motor = robot.getDevice(motor_name)
        if robot is None:
            raise ValueError("Error: robot is None")
        motor.setVelocity(0.2)
        motors.append(motor)
        #print(motors)
        
    while robot.step(16) != -1:  
        for i in range(len(motors)):
            motors[i].setPosition(motor_positions[i])
            print("are motor positions being used to set positions",motor_positions)

def get_position_sensor_data(robot):
    sensor_names = ["joint1_ps", "joint2_ps"]
    sensors = {}

    for sensor_name in sensor_names:
        sensor = robot.getPositionSensor(sensor_name)
        sensors[sensor_name] = sensor

    sensor_data = {}

    while robot.step(16) != -1:  # Adjust the time step as needed
        for sensor_name, sensor in sensors.items():
            sensor_data[sensor_name] = sensor.getValue()

        print("Position Sensor Data:", sensor_data)


# Define stiffness and admittance matrices
stiffness_matrix = [[1,0,0],[0,1,0],[0,0,1]]
admittance_matrix = [[1,0,],[0,1]]


# Create the PMP controller instance using Robotic Toolbox
pmp_controller = PMP(
    my_bot, K_ext=stiffness_matrix, A_int=admittance_matrix, robotics_Library='rtb',
    rmse_threshold=0.005,time_limit=1000, step_size=1/5) 
target_position= define_target(fyp_supervisor,target='Ball')
#no change


#target_position[2]=0
#print("target_position is:",target_position) 
q_traj, time, rmse, x_error = pmp_controller.execute_pmp_kinematics(target_position)
 #the k_ext and torque are not inistailize
#print("q_traj", q_traj)
#print("time:", time)
#print("rmse:", rmse)
#print("x_error:", x_error)

#motor_positions = [-0.8201585, -0.6536719]
#motor_positions = [3, 5]

motor_positions = q_traj[-1] 
#motor_positions=[1.24565, -0.8866]
#print("is pmp providing motor_positions:",motor_positions)
set_motors_for2joints(fyp_supervisor, motor_positions)
print("Script ran completely")


