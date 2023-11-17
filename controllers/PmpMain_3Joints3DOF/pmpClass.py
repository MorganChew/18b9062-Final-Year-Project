import roboticstoolbox as rtb
#figure out without importing ikp_chain rtb
import numpy as np
#from ikpy_chain import create_ikpy_chain
#from robotics_toolbox import create_robot3dof


class PMP:
    def __init__(self, my_bot,x_target=None, q_current=None ,robotics_Library=None,K_ext=None, A_int=None,TBG=None,rmse_threshold=None,time_limit=None,step_size=None):
        self.my_bot = my_bot
        #Send q current externally or use my_bot
        self.x_target = x_target
        #self.q_current = my_bot.q
        #self.q_current = q_current if q_current is not None else np.zeros(my_bot.n)
        self.q_current = q_current if q_current is not None else [0.0] * my_bot.n  # Use the robot's joint count
        self.robotics_Library = robotics_Library
        self.K_ext = K_ext
        self.A_int = A_int
        self.TBG=TBG
        self.x=0
        self.q=my_bot.q
        self.jacobian = None  
        self.force = None
        self.torque = None
        self.q_dot = None
        self.x_dot= None
        self.rmse_threshold=rmse_threshold
        self.time_limit=time_limit
        self.step_size=step_size

        
        
        #set_bot function to figoure out the fkine

        #x refer as x target,
        #x_dot not found ,
        #q as q_current  ,
        # q_dot not found,
        #F= calucate force, 
        #tau+calucalto torque 

    def calculate_fkine(self, q_current=None, robotics_Library=None):
        transformation_matrix = self.my_bot.fkine(q_current).t
        return transformation_matrix
       

    def calculate_error_vector(self, setpoint_X, current_X):
        if current_X is None:
            raise ValueError("Error: current_X is None")
        if setpoint_X is None:
            raise ValueError("Error: setpoint_X is None")
        x_error = np.subtract(setpoint_X, current_X)
        return x_error

    def calculate_setpoint_x(self,x_target):
        setpoint_X=x_target
        return setpoint_X
    
    def calculate_current_x(self,q_current):
        current_X=self.calculate_fkine(q_current)
        return current_X
    
    def calculate_jacobian(self, q_current):
        return self.my_bot.jacob0_analytical(q_current)[:3, :]
        #need to investigate the use of the number (figure out if it is refering to position/orentation or the joints)
    
    def calculate_Torque(self, Jacobian=None,Force=None):
        if Force is None:
            raise ValueError("Error: Force is None")
        if Jacobian is None:
            raise ValueError("Error: Jacobian is None")
        torque =np.dot(Jacobian.transpose(), Force)
        #print(torque)
        if  torque is None:
            raise ValueError("Error: Jacobian is None")
        return torque
    
    def calculate_q_dot(self,A_int,Torque):
        q_dot = np.dot(A_int, Torque)
        return q_dot
    
    def calculate_Force(self,K_ext,x_error):
        if K_ext is None:
            raise ValueError("Error: K_ext is None")
        if x_error is None:
            raise ValueError("Error: x_error is None")
        force = np.dot(K_ext, x_error)
        return force
    
    
    #tbg is a union of time, so s
    def calculate_tbg(self,x_error):
        # x_error is used as it represents the difference between the desired target position (setpoint) and the current end effector position of the robot. I
        distance_to_target = np.linalg.norm(x_error)
        tbg = 1.0 + distance_to_target #The TBG is calculated using the following formula:
        #distance_to_target represents the Euclidean distance between the current end effector position and the target position
        #The calculated tbg value adds 1 to the distance, effectively creating a scaling factor that increases with the distance to the target.
        # The idea is that the farther the end effector is from the target, the more the TBG will increase, influencing the joint angle update rate.
        return tbg
    #the error keeps decreaing , force is decresing,  tbg says the closer to the target , the tbg would increase overtime as errir increases
    #check the paper again


    def run_step(self, x_target=None, q_current=None, K_ext=None, A_int=None, TBG=None):
        #q_traj = np.empty([1, self.my_bot.n])
        #q_traj = [[1, self.my_bot.n]]
        q_traj = [[]]
        self.x = self.calculate_fkine(q_current) #x
        if self.x is None:
            raise ValueError("Error: calculate_fkine returned None for current_X")
        setpoint_X = x_target
        if setpoint_X is None:
                raise ValueError("Error: setpoint_X is None")
        rmse = 100
    
        time = 0
    
        while rmse > self.rmse_threshold and time <self.time_limit: #make changeable #come as arguments
            self.x,self.x_dot,self.force,self.q,self.q_dot,self.torque = self.step(x=self.x,x_dot=self.x_dot,force=self.force,q=self.q,q_dot=self.q_dot,torque=self.torque,setpoint_x=setpoint_X, K_ext=K_ext, A_int=A_int,tbg=TBG)
            #x,x_dot,force,q,q_dot,torque needs to be local/ global variables
            x_error = self.calculate_error_vector(setpoint_X, self.x)
            if x_error is None:
                raise ValueError("Error: calculate_error_vector returned None for x_error")
            rmse = np.sqrt(np.mean(np.square(x_error)))
            if time % 1 == 0: #make this number come from outside
                #q_traj = np.append(q_traj, [self.q], axis=0)
                q_traj.append(self.q)  
                  # Append 3-element q_current
            time += 1
            #print("force:",self.force)
            
        return q_traj, time, rmse, x_error #come as arguments # q_current
    
    #updating those 6 variables
    def step(self,x,x_dot,force,q,q_dot,torque,setpoint_x, K_ext, A_int,tbg):
            x_error = self.calculate_error_vector(setpoint_x, x)
            self.force = self.calculate_Force(self.K_ext, x_error)
            self.jacobian = self.calculate_jacobian(q)
            self.torque = self.calculate_Torque(self.jacobian, self.force)
            #torque outside as it reduces the strain and optimizes the path towards the target
            #tbg = self.calculate_tbg(x_error)
            self.q_dot = self.calculate_q_dot(self.A_int, self.torque) 
            #allows setting from the outside, this is done as it would be used to calculate the desired velocity
            self.q = self.q + (self.q_dot * self.step_size) #change 1/5
            self.x = self.calculate_fkine(q)
            
            #rmse = np.sqrt(np.mean(np.square(x_error)))

            
            return self.x,self.x_dot,self.force,self.q,self.q_dot,self.torque
    
    
    #function is responsible for computing the trajectory of joint angles for the robot using the Position-Based Motion Planning (PMP) algorithm
    def pmp_kinematics(self, target_position=None,K_ext=None, A_int=None):
        # Create a 3D target position by adding a zero in the z-direction
        x_target = np.array([target_position[0], target_position[1], target_position[2]]) #axis will depend on the input #cannot be specificed as zero or anything
        #q_current = np.array([0.0, 0.0])  # Initialize q_current
        q_traj = self.run_step(x_target=x_target, q_current=self.q_current, K_ext=K_ext, A_int=A_int, TBG=None)
        #step would tkae three inputs and return three outputs

        #This method computes the joint trajectory needed to move the robot's end effector from the current position to the target position while considering external stiffness and internal admittance.
        return q_traj

    def calculate_final_q_current(self,q_traj):
        q_current = q_traj[-1]
        #self.set_motors(self.supervisor, q_current)
        return q_current
    
    def execute_pmp_kinematics(self, target_position=None, K_ext=None, A_int=None,robotics_Library=None):
        self.q_traj, time, rmse, x_error= self.pmp_kinematics(target_position=target_position,K_ext= K_ext, A_int=A_int)
        self.q_current = self.calculate_final_q_current(self.q_traj)
        #self.set_motors(self.supervisor, self.q_current)
        return self.q_traj, time, rmse, x_error
    
    def set_stiffness(self, K_ext):
        self.K_ext = K_ext
    
    def set_admittance(self, A_int):
        self.A_int = A_int
    
  
#move out functions that rely on the simulation, functions should be indedenpoent of librarys