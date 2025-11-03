import numpy as np
from scipy.linalg import expm

"""
    This EKF code for IMU-GPS sensor fusion
    This code takes IMU inout at 1000 hz and predict
    ALso takes GPS input at 100Hz and covar mat R and update the estimate.
    This code does not hold logic for GPS or IMU data processing 
    state = [x, y, theta, Vx, Vy, b_acc, b_gyro]
    """
class EKF:   
    def __init__(self, sigma, white_accel_dev , white_gyro_dev, accel_bias_random_dev, gyro_bias_random_dev, dt):
        #Input arguments
        self.P = sigma        
        self.accel_bias_random_dev = accel_bias_random_dev   #std deviation
        self.gyro_bias_random_dev = gyro_bias_random_dev    #bias std deviation
        self.dt = dt
        self.Q = self.crete_Q_mat(white_accel_dev, white_gyro_dev, accel_bias_random_dev, gyro_bias_random_dev)  # white noise covar for accelerometer
        self.rho = 1.225
        self.C_d = 0.4 
        self.Af = 1.6
        self.V_wind = 10
        self.mass = 2000 
        self.lr = 2.00
        self.lf = 2.00
        

    def crete_Q_mat(self, white_accel_dev, white_gyro_dev, accel_bias_random_dev, gyro_bias_random_dev):        
      
        #white nose
        # covar_acc_x = white_accel_dev[0]**2
        # covar_acc_y = white_accel_dev[1]**2
        # covar_gyro = white_gyro_dev**2 
        # # accel_bias_x_covar = accel_bias_random_dev[0]**2 
        # # accel_bias_y_covar = accel_bias_random_dev[1]**2 
        # gyro_bias_covar = gyro_bias_random_dev**2 
        # covar_array = [ covar_acc_x, covar_acc_y , covar_gyro, gyro_bias_covar]        
        # Q_mat = np.diag(covar_array) 
        # #print("Q_mat", Q_mat)
        
        
        covar_angle = (self.dt * white_gyro_dev)**2
        coavr_vel_x = 2 * (self.dt * white_accel_dev[0])**2 
        coavr_vel_y = 2 * (self.dt * white_accel_dev[1])**2 
        covar_dist_x = (0.5 * self.dt**2 * (white_accel_dev[0]) + coavr_vel_x * self.dt)**2
        covar_dist_y = (0.5 * self.dt**2 * (white_accel_dev[1]) + coavr_vel_y * self.dt)**2

        covar_bias_ax = accel_bias_random_dev[0]**2
        covar_bias_ay = accel_bias_random_dev[1]**2
        covar_bis_g = gyro_bias_random_dev**2
        covar_array = [covar_dist_x, covar_dist_y, covar_angle, coavr_vel_x, coavr_vel_y, covar_bis_g]
        Q_mat = np.diag(covar_array)

        return Q_mat 

    def process_model(self, X, U):
        ##extract states 
        x_pos = X[0]
        y_pos = X[1] 
        theta = X[2]
        Vx = X[3]
        Vy = X[4]
        # bias_acc = X[5:7]
        bias_gyro = X[5]

        ## extratc inputs
        accel_x = U[0]  
        accel_y = U[1]  
        ang_vel = U[2] - bias_gyro               
        accel_vec = np.array([accel_x, accel_y]).reshape(2,1)
        
        ## apply model 
        x_pos_dt = Vx
        y_pos_dt = Vy
        theta_dt = ang_vel 

        R_mat = np.array([[np.cos(theta), -np.sin(theta)],
                 [np.sin(theta), np.cos(theta)]]).reshape(2,2)          
        #print("R", R_mat)          
        vel_world_dt = R_mat @ accel_vec 
        print("U", U)  
        # print("bias_acc", bias_acc)
        # print("accel_x", accel_x, "accel_y", accel_y)
        
        # print("bias y", bias_acc[1])
        # print("theta", theta)
        # print("sin theta", np.sin(theta))
        # print("cos theta", np.cos(theta))
        # print("vel_ref_dt", vel_ref_dt)      
        Vx_dt =  vel_world_dt[0,0] 
        Vy_dt =  vel_world_dt[1,0]
        bias_acc_x_dt = np.random.normal(0, self.accel_bias_random_dev[0])
        bias_acc_y_dt = np.random.normal(0, self.accel_bias_random_dev[1])
        bias_gyro_dt = np.random.normal(0, self.gyro_bias_random_dev)            
        X_dt = np.array([x_pos_dt, y_pos_dt, theta_dt, Vx_dt, Vy_dt, bias_gyro_dt])        
        #X_dt = np.array([x_pos_dt, y_pos_dt, theta_dt, Vx_dt, Vy_dt, bias_gyro_dt])
        return X_dt 
    

    def apply_RK45(self, X, U):
        ##applt RK 45 here
        k1 = self.process_model(X, U)          
        k2 = self.process_model(X + self.dt/2 * k1, U)
        k3 = self.process_model(X + self.dt/2 * k2, U)
        k4 = self.process_model(X + self.dt * k3, U)
        dx = (k1 + 2* k2 + 2 * k3 + k4) / 6     

        print("dx", dx) 

        X = X + dx * self.dt      
        return X, dx
         

    def predict(self, X, U, S):
        # print("  ")
        # print("inside predict")
        # print("X before", X)        

        X, dx = self.apply_RK45(X, U)    
        X[2] = (X[2] + np.pi) % (2*np.pi) - (np.pi)     # -pi to pi conversion 
        F = self.get_FMat(X, U)
        V = self.get_VMat(X)  
         
        k_factor = 1000/20
        #print("K_fact", k_factor)   
        #print(" (V @ self.Q @ V.T)", V @ self.Q @ V.T)   
        #print("Q * k_factor", self.Q * k_factor) 
        S = F @ S @ F.T +   self.Q  * k_factor  # (V @ self.Q @ V.T) #* k_factor    #self.Q    
        #print("trace", np.trace(S))
        return X, S, dx 


    def get_FMat(self, X, U):
        theta = X[2]
        # bax, bay, bg = X[5], X[6], X[7]
        ax, ay, w = U

        f32 = -np.sin(theta) * (ax) - np.cos(theta) * (ay)
        f42 =  np.cos(theta) * (ax) - np.sin(theta) * (ay)

        
        st = np.sin(theta)
        ct = np.cos(theta) 

        # F = np.zeros((8, 8))
        # F[0, 3] = 1
        # F[1, 4] = 1
        # F[2, 7] = -1
        # F[3, 2] = f32
        # F[3, 5] = -np.cos(theta)
        # F[3, 6] =  np.sin(theta)
        # F[4, 2] = f42        
        # F[4, 5] = -np.sin(theta)
        # F[4, 6] = -np.cos(theta)
        # FMat = np.eye(8) + self.dt * F

              

        # AMat = np.array([ [0, 0, 0, 1, 0, 0,0, 0], 
        #                   [0, 0, 0, 0, 1, 0,0, 0 ],
        #                   [0, 0, 0, 0, 0, 0,0, -1],
        #                   [0, 0, f32, 0, 0, -ct, st, 0],                          
        #                   [0, 0, f42, 0, 0, -st, -ct, 0],                   
        #                   [0, 0, 0, 0, 0,  0, 0,  0],
        #                   [0, 0, 0, 0, 0,  0, 0,  0],
        #                   [0, 0, 0, 0, 0,  0, 0,  0] ]) 

        AMat = np.array([ [0, 0, 0, 1, 0,  0], 
                          [0, 0, 0, 0, 1, 0 ],
                          [0, 0, 0, 0, 0,  -1],
                          [0, 0, f32, 0, 0,  0],                          
                          [0, 0, f42, 0, 0, 0],               
                          [0, 0, 0, 0, 0,  0] ]) 
        
        
        FMat = np.eye(6) + self.dt * AMat
       
        return FMat
        

    def get_VMat(self, X):
        #noise vector = [wax, way, wg, wbax, wbay, wbg]
        # VMat = del fun() / del w_vec
        

        
        # V = np.zeros((8, 6))
        # V[2, 2] = 1
        # V[3, 0] = np.cos(theta)
        # V[3, 1] = -np.sin(theta)
        # V[4, 0] = np.sin(theta)
        # V[4, 1] =  np.cos(theta)
        # V[5, 3] = 1
        # V[6, 4] = 1
        # V[7, 5] = 1
          
        theta = X[2]
        ct = np.cos(theta)
        st = np.sin(theta)       

        # Mat = np.array([ [0,0,0,0,0,0],
        #                  [0,0,0,0,0,0],
        #                  [0,0,1,0,0,0],
        #                  [ct,-st,0,0,0,0],
        #                  [st,ct,0, 0,0,0],           
        #                  [0,0,0,1,0,0],
        #                  [0,0,0,0,1,0],
        #                  [0,0,0,0,0,1]], dtype=float)
        
        Mat = np.array([ [0,0,0,0],
                         [0,0,0,0],
                         [0,0,1,0],
                         [ct,-st,0,0],
                         [st,ct,0, 0],    
                         [0,0,0,1]], dtype=float)
        
        VMat = self.dt * Mat       
           
        return VMat
        
    
    def get_GMat(self):
        GMat = np.array([[1,0,0,0,0,0],
                         [0,1,0,0,0,0],
                         [0,0,0,1,0,0],
                         [0,0,0,0,1,0]
                         ])
        
        return GMat

    def get_WMat(self):
        WMat = np.eye(2)
        return WMat
        
    def update(self, X, S, z, R):
        #print("inside update function")
        #z = np array of [x, y] GPS pose
        #X = 8 * 1 state vector
        #R = 2 by 2 noise matrix for GPS reading
        #S = covariance of estimated state        
        
        G = self.get_GMat()        
        W = self.get_WMat() 
       
        # print(" ")
        # print("Inside update")
        # print("z", z)

        # print("R", R)
         
        # print("S", S )
        K = (S @ G.T) @ np.linalg.pinv(G @ S @ G.T + R )  
        # print("K", K)
        # print("dx ", K @ (z - G @ X)) 

        X = X  + K @ (z - G @ X) 
        X[2] = (X[2] + np.pi) % (2*np.pi) - (np.pi)                
        S = S - K @ G @ S    

        #print("trace", np.trace(S)) 
        return X, S  
    


    
   

    




    


        
        



