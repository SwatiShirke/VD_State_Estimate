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
        # x_pos = X[0]
        # y_pos = X[1] 
        # theta = X[2]
        # Vx = X[3]
        # Vy = X[4]
        # bias_acc = X[5:7]
        # bias_gyro = X[7]
        # accel_x = U[0]   - bias_acc[0] 
        # accel_y = U[1] - bias_acc[1] 
        # ang_vel = U[2] - bias_gyro          
        # accel_vec = np.array([accel_x, accel_y]).reshape(2,1)

        # x_pos_dt = Vx
        # y_pos_dt = Vy
        # theta_dt = ang_vel
        # R_mat = np.array([[np.cos(theta), -np.sin(theta)],
        #          [np.sin(theta), np.cos(theta)]]).reshape(2,2)        
        # vel_ref_dt = R_mat @ accel_vec       
        # Vx_dt =  vel_ref_dt[0,0] 
        # Vy_dt =  vel_ref_dt[1,0]
        # bias_acc_x_dt = np.random.normal(0, self.accel_bias_random_dev[0])
        # bias_acc_y_dt = np.random.normal(0, self.accel_bias_random_dev[1])
        # bias_gyro_dt = np.random.normal(0, self.gyro_bias_random_dev)

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
        
        beta = np.arctan2(self.lr * np.tan(theta), (self.lf + self.lr))
        x_pos_dt = Vx * np.cos(theta + beta) -  Vy * np.sin(theta + beta)
        y_pos_dt = Vx * np.sin(theta + beta) + Vy * np.cos(theta + beta)
        theta_dt = ang_vel

       
        Vx_dt =    Vy * ang_vel  + accel_x  #- air_drag    #   
        Vy_dt =  - Vx * ang_vel  + accel_y  # 
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
        F = self.get_FMat_New(X, U)
        V = self.get_VMat(X)          
        S = F @ S @ F.T +  V @ self.Q @ V.T   #self.Q 
   
        return X, S, dx 


    def get_FMat_New(self, X, U):
        x_pos, y_pos, theta, Vx, Vy, bax, bay, bg = X
        ax, ay, w = U

        # Precompute sideslip beta and its derivative
        beta = np.arctan2(self.lr * np.tan(theta), (self.lf + self.lr))
        dbeta_dtheta = (self.lr / (self.lf + self.lr)) / (np.cos(theta)**2 + (self.lr / (self.lf + self.lr))**2 * np.tan(theta)**2)

        St = np.sin(theta)
        Ct = np.cos(theta)
        s = np.sin(theta + beta)
        c = np.cos(theta + beta)
        one_plus_dtheta = 1 + dbeta_dtheta

        F = np.zeros((8,8))

        # x_pos_dot derivatives
        F[0,2] = -Vx * s * one_plus_dtheta - Vy * c * one_plus_dtheta
        F[0,3] = c
        F[0,4] = -s

        # y_pos_dot derivatives
        F[1,2] = Vx * c * one_plus_dtheta - Vy * s * one_plus_dtheta
        F[1,3] = s
        F[1,4] = c

        # theta_dot derivatives
        F[2,7] = -1  # d(theta_dot)/d(bg)

        # Vx_dot derivatives
        F[3,5] = -1  # d(Vx_dot)/d(bax)

        # Vy_dot derivatives
        F[4,6] = -1  # d(Vy_dot)/d(bay)

        ##for cross-coupling terms 
        F[3,4] = (w-bg)
        F[3,7] = -Vy
        F[4,3] = -(w-bg) 
        F[4,7] = Vx

        F_mat = np.eye(8) + self.dt * F
        return F_mat

    def get_FMat(self, X, U):
        # theta = X[2]
        # bax, bay, bg = X[5], X[6], X[7]
        # ax, ay, w = U

        # f43 = -np.sin(theta) * (ax - bax) - np.cos(theta) * (ay - bay)
        # f53 =  np.cos(theta) * (ax - bax) - np.sin(theta) * (ay - bay)

        # F = np.zeros((8, 8))
        # F[0, 3] = 1
        # F[1, 4] = 1
        # F[2, 7] = -1
        # F[3, 2] = f43
        # F[4, 2] = f53
        # F[3, 5] = -np.cos(theta)
        # F[3, 6] =  np.sin(theta)
        # F[4, 5] = -np.sin(theta)
        # F[4, 6] = -np.cos(theta)
        # FMat = np.eye(8) + self.dt * F

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
                                  [0, 0, 0, 0, (w-bg), -1, 0, -Vy],
                                  #[0, 0, 0, 0, 0, -1, 0, 0],
                                  [0, 0, 0, -(w-bg), 0, 0, -1, Vx],
                                  #[0, 0, 0, 0, 0, 0, -1, 0],
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
        # x_pos = X[0]
        # y_pos = X[1] 
        # theta = X[2]
        # Vx = X[3]
        # Vy = X[4]
        # bax = X[5]
        # bay = X[6]
        # bg = X[7]
        # # ax = U[0]
        # # ay = U[1]
        # # w = U[2]

        # V = np.zeros((8, 6))
        # V[2, 2] = 1
        # V[3, 0] = np.cos(theta)
        # V[3, 1] = -np.sin(theta)
        # V[4, 0] = np.sin(theta)
        # V[4, 1] =  np.cos(theta)
        # V[5, 3] = 1
        # V[6, 4] = 1
        # V[7, 5] = 1
        #VMat = self.dt * V     

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
                         [1,0,Vy,0,0,0],
                         [0,1,-Vx, 0,0,0],                       
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
    


    
   

    




    


        
        



