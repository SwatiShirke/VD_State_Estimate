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
        # coavr_vel = self.dt * covar_acc
        # covar_dist = 0.5 * self.dt**2 * covar_acc
        # covar_angle = self.dt * covar_gyro

        #white nose
        covar_acc_x = white_accel_dev[0]
        covar_acc_y = white_accel_dev[1]**2
        covar_gyro = white_gyro_dev**2 
        accel_bias_x_covar = accel_bias_random_dev[0]**2 
        accel_bias_y_covar = accel_bias_random_dev[1]**2 
        gyro_bias_covar = gyro_bias_random_dev**2 
        covar_array = [ covar_acc_x, covar_acc_y , covar_gyro, accel_bias_x_covar, accel_bias_y_covar, gyro_bias_covar] 
        Q_mat = np.diag(covar_array)

        
        # covar_dist_x = (0.5 * self.dt**2 * (white_accel_dev[0]))**2
        # covar_dist_y = (0.5 * self.dt**2 * (white_accel_dev[1]))**2
        # covar_angle = (self.dt * white_gyro_dev)**2
        # coavr_vel_x = (self.dt * white_accel_dev[0])**2 
        # coavr_vel_y = (self.dt * white_accel_dev[1])**2 
        # covar_bias_ax = accel_bias_random_dev[0]**2
        # covar_bias_ay = accel_bias_random_dev[1]**2
        # covar_bis_g = gyro_bias_random_dev**2
        # covar_array = [covar_dist_x, covar_dist_y, covar_angle, coavr_vel_x, coavr_vel_y, covar_bias_ax, covar_bias_ay,covar_bis_g]
        # Q_mat = np.diag(covar_array)

        return Q_mat 

    def process_model(self, X, U):
        ##extract states 
        x_pos = X[0]
        y_pos = X[1] 
        theta = X[2]
        Vx = X[3]
        Vy = X[4]
        bias_acc = X[5:7]
        bias_gyro = X[7]
        accel_x = U[0]   - bias_acc[0] 
        accel_y = U[1] - bias_acc[1] 
        ang_vel = U[2] - bias_gyro  
        
       
        x_pos_dt = Vx * np.cos(theta) -  Vy * np.sin(theta )
        y_pos_dt = Vx * np.sin(theta) + Vy * np.cos(theta )
        theta_dt = ang_vel

        # ## air drag calculations
        # f_drag = 0.5 * self.rho * self.C_d * self.Af * (Vx + self.V_wind)**2
        # air_drag = f_drag / self.mass 
    

        Vx_dt =  accel_x  #- air_drag    # Vy * ang_vel    
        Vy_dt =  accel_y # - Vx * ang_vel 
        bias_acc_x_dt = np.random.normal(0, self.accel_bias_random_dev[0])
        bias_acc_y_dt = np.random.normal(0, self.accel_bias_random_dev[1])
        bias_gyro_dt = np.random.normal(0, self.gyro_bias_random_dev)
        
        X_dt = np.array([x_pos_dt, y_pos_dt, theta_dt, Vx_dt, Vy_dt, bias_acc_x_dt, bias_acc_y_dt, bias_gyro_dt])
  
        return X_dt 
    

    def apply_RK45(self, X, U):
        ##applt RK 45 here
        k1 = self.process_model(X, U)          
        k2 = self.process_model(X + self.dt/2 * k1, U)
        k3 = self.process_model(X + self.dt/2 * k2, U)
        k4 = self.process_model(X + self.dt * k3, U)
        dx = (k1 + 2* k2 + 2 * k3 + k4) / 6 
        # print(" ")
        print("X", X)
        # print("dx", dx[3])

        print("dx", dx) 

        X = X + dx * self.dt  
        
        #print("velocity x ", X[3]) 
        return X, dx
         

    def predict(self, X, U, S):
        print("  ")
        print("inside predict")
        print("X before", X)
        # print("S before ", S)

        X, dx = self.apply_RK45(X, U)    
        X[2] = (X[2] + np.pi) % (2*np.pi) - (np.pi)      
        F = self.get_FMat(X, U)
        V = self.get_VMat(X)          
        S = F @ S @ F.T +  V @ self.Q @ V.T   #self.Q 
   
        return X, S, dx 

    def get_FMat(self, X, U):
        x_pos = X[0]
        y_pos = X[1] 
        theta = X[2]
        Vx = X[3]
        Vy = X[4]
        bax = X[5]
        bay = X[6] 
        bg = X[7]
        ax = U[0]
        ay = U[1]
        w = U[2]

        St = np.sin(theta)
        Ct = np.cos(theta)

        AMat = np.array([ [0, 0, -Vx * St - Vy * Ct, Ct, - St, 0,0,0], 
                                  [0, 0, Vx * Ct - Vy * St, St, Ct, 0,0,0 ],
                                  [0, 0, 0, 0, 0, 0, 0, -1],
                                  [0, 0, 0, 0, 0, -1, 0, 0],
                                  [0, 0, 0, 0, 0, 0, -1, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0] ])
        
        FMat = np.eye(8) + self.dt * AMat
        #FMat = expm(self.dt * AMat)
 
        return FMat
        

    def get_VMat(self, X):
        #noise vector = [wax, way, wg, wbax, wbay, wbg]
        # VMat = del fun() / del w_vec
        # it is derivative of system dynamics equations with respect to noise 
        # first 3 components are white noise, directly affects sensor reading, last 3 are bias random walk
        # X = X.reshape(X.shape[0])
        x_pos = X[0]
        y_pos = X[1] 
        theta = X[2]
        Vx = X[3]
        Vy = X[4]
        bax = X[5]
        bay = X[6]
        bg = X[7]
        # ax = U[0]
        # ay = U[1]
        # w = U[2]

        Mat = np.array([ [0,0,0,0,0,0],
                         [0,0,0,0,0,0],
                         [0,0,1,0,0,0],
                         [1,0,0,0,0,0],
                         [0,1,0, 0,0,0],
                         [0,0,0,1,0,0],
                         [0,0,0,0,1,0],
                         [0,0,0,0,0,1]], dtype=float)
        
        VMat = self.dt * Mat        
        return VMat
        
    
    def get_GMat(self):
        GMat = np.array([[1,0,0,0,0,0,0,0],
                         [0,1,0,0,0,0,0,0],
                         [0,0,1,0,0,0,0,0],
                         [0,0,0,1,0,0,0,0]
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
        eps = 1e-9   
        #+ 
        print(" ")
        print("Inside update")
       
        K = (S @ G.T) @ np.linalg.pinv(G @ S @ G.T + R )  
        print("dx ", K @ (z - G @ X)) 

        X = X  + K @ (z - G @ X) 
        X[2] = (X[2] + np.pi) % (2*np.pi) - (np.pi)                
        S = S - K @ G @ S 

        

        # Joseph form (numerically stable covariance update)
        # I = np.eye(S.shape[0])        
        # S_post = (I - K @ G) @ S @ (I - K @ G).T + K @ R @ K.T
        # # # Enforce symmetry (remove tiny numerical asymmetry)
        # S = 0.5 * (S_post + S_post.T)
        # print("X after update", X)
        # print("S", S) 

        return X, S  
    


    
   

    




    


        
        



