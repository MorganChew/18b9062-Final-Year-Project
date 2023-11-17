from controller import Robot, Supervisor

# Create a Supervisor instance
supervisor = Supervisor()

# Import the PROTO file
robot_node = supervisor.getFromProtoFile("my_robot.proto")

# Get the robot's joints
joint1 = supervisor.getMotor("joint1")
joint2 = supervisor.getMotor("joint2")

# Set initial joint angles
joint1.setPosition(0.0)
joint2.setPosition(0.0)

# Main control loop
while supervisor.step(64) != -1:
    # Perform robot control here
    
    # Example: Set joint angles to move the robot
    joint1.setPosition(1.57)  # Set joint1 angle to 90 degrees (in radians)
    joint2.setPosition(0.0)   # Set joint2 angle to 0 degrees (in radians)
