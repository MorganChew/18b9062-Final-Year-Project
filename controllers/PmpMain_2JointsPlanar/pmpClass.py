
class PMP:
    def __init__(self, my_bot,x_target=None, q_current=None ,robotics_Library=None,K_ext=None, A_int=None,rmse_threshold=None,time_limit=None,step_size=None):
        self.my_bot = my_bot
        self.x_target = x_target
        self.q_current = q_current if q_current is not None else [0.0] * my_bot.n  # Uses the robot's joint count
        self.robotics_Library = robotics_Library
        self.K_ext = K_ext
        self.A_int = A_int
        self.x=0
        self.q=my_bot.q
        self.jacobian = [None]*my_bot.n
        self.force = None
        self.torque = None
        self.q_dot = None
        self.x_dot= None
        self.rmse_threshold=rmse_threshold
        self.time_limit=time_limit
        self.step_size=step_size

    def calculate_fkine(self, q_current=None, robotics_Library=None):
        if self.robotics_Library == 'rtb' :
            transformation_matrix = self.my_bot.fkine(q_current).t
        else:
            raise Exception("no robotics library is defined")
        #print("Calculate fkine:",transformation_matrix)
        return transformation_matrix
       

    def calculate_error_vector(self, setpoint_X, current_X):
        x_error = []
        for setpoint, current in zip(setpoint_X, current_X):
            x_error.append(setpoint - current)
    
        return x_error

    def calculate_setpoint_x(self,x_target):
        setpoint_X=x_target
        #print("setpoint_X:",setpoint_X)
        return setpoint_X
    
    def calculate_current_x(self,q_current):
        #print("q_current for calculating current X:",q_current)
        current_X=self.calculate_fkine(q_current)
        #print("current_x",current_X)
        return current_X
    
    def calculate_jacobian(self, q_current):
        if self.robotics_Library == 'rtb' :
        #print("q_current for calculating jacobian:",q_current)
            return self.my_bot.jacob0_analytical(q_current)[:3, :]
        else:
            raise Exception("no robotics library is defined")

        
    
    def calculate_Torque(self, Jacobian=None,Force=None):
        Jacobian_transpose = [[Jacobian[j][i] for j in range(len(Jacobian))] for i in range(len(Jacobian[0]))]

    # Initialize torque as a list of zeros
        torque = [0] * len(Jacobian_transpose)

    # Perform the dot product
        for i in range(len(Jacobian_transpose)):
            for j in range(len(Force)):
                torque[i] += Jacobian_transpose[i][j] * Force[j]

        return torque
    
    def calculate_q_dot(self,A_int,Torque):
        q_dot = [0] * len(A_int)
        for i in range(len(A_int)):
            for j in range(len(Torque)):
                q_dot[i] += A_int[i][j] * Torque[j]
        return q_dot
        
    def calculate_Force(self,K_ext,x_error):
        # Initialize force as a list of zeros
        force = [0] * len(K_ext)
        for i in range(len(K_ext)):
            for j in range(len(x_error)):
                force[i] += K_ext[i][j] * x_error[j]
        #print("force:",force)
        return force
    
    def calculate_Rmse(self,x_error):
    # Calculate the sum of squared errors
        sum_of_squares = sum([error ** 2 for error in x_error])

    # Calculate the mean of squared errors
        mean_of_squares = sum_of_squares / len(x_error)

    # Calculate the square root of the mean
        rmse = mean_of_squares ** 0.5
        return rmse


    #PMP LOOP
    def run_step(self, x_target=None, q_current=None, K_ext=None, A_int=None):
        q_traj = [[]]
        self.x = self.calculate_fkine(q_current) 
        if self.x is None:
            raise ValueError("Error: calculate_fkine returned None for current_X")
        setpoint_X = x_target
        if setpoint_X is None:
            raise ValueError("Error: setpoint_X is None")
        rmse = 100
    
        time = 0
    
        while rmse > self.rmse_threshold and time <self.time_limit: 
            self.x,self.x_dot,self.force,self.q,self.q_dot,self.torque = self.step(x=self.x,x_dot=self.x_dot,force=self.force,q=self.q,q_dot=self.q_dot,torque=self.torque,setpoint_x=setpoint_X, K_ext=K_ext, A_int=A_int) 
            #self.q can also be updated through proprioception/ joint sensors
            #self.x can also be updated using visual feedback
            self.x = self.calculate_fkine(self.q) #this comes from rtb library
            x_error = self.calculate_error_vector(setpoint_X, self.x)
            if x_error is None:
                raise ValueError("Error: calculate_error_vector returned None for x_error")
            #rmse = np.sqrt(np.mean(np.square(x_error))
            rmse = self.calculate_Rmse(x_error)
            if time % 1 == 0: #make this number come from outside
                q_traj.append(self.q)  
            time += 1
            #print("force:",self.force)
            
        return q_traj, time, rmse, x_error #come as arguments # q_current
    
    #one step in PMP
    def step(self,x,x_dot,force,q,q_dot,torque,setpoint_x, K_ext, A_int):
            #use local variables
            x_error = self.calculate_error_vector(setpoint_x, x)
            force = self.calculate_Force(K_ext, x_error)
            jacobian = self.calculate_jacobian(q)
            torque = self.calculate_Torque(jacobian, force)
            q_dot = self.calculate_q_dot(A_int, torque) 
            q_dot = [float(val) for val in q_dot]
            q = [q + (q_dot_val * self.step_size) for q, q_dot_val in zip(q, q_dot)]
            return x,x_dot,force,q,q_dot,torque
    
    #This method computes the joint trajectory needed to move the robot's end effector from the current position to the target position while considering external stiffness and internal admittance.
        #return q_traj
    def execute_pmp_kinematics(self, target_position=None,K_ext=None, A_int=None):
        if K_ext is not None:
            self.set_K_ext(K_ext)
        if A_int is not None:
            self.set_A_int(A_int)
        #self.q_traj, time, rmse, x_error= self.pmp_kinematics(target_position=target_position,K_ext= self.K_ext, A_int=self.A_int)
        #self.q_current = self.calculate_final_q_current(self.q_traj)
       # Create a 3D target position by adding a zero in the z-direction
        x_target = [target_position[0], target_position[1], target_position[2]] #axis will depend on the input and cannot be specificed as zero or anything
        self.q_traj, time, rmse, x_error = self.run_step(x_target=x_target, q_current=self.q_current, K_ext=self.K_ext, A_int=self.A_int)
        return self.q_traj, time, rmse, x_error

    def set_K_ext(self,K_ext):
        self.K_ext = K_ext
        return
    
    def set_A_int(self,A_int):
        self.A_int= A_int
        return

    def calculate_final_q_current(self,q_traj):
        q_current = q_traj[-1]
        return q_current
    
#Recent Changes
#renamed to pmp
#combine kinemtaics to one
#added non updates for aint kext\
#added if statements for jacobina and fkine