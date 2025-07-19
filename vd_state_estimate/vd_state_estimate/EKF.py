import numpy as np



class EKF:
    """
    This EKF code for IMU-GPS sensor fusion
    This code takes IMU inout at 1000 hz and predict
    ALso takes GPS input at 100Hz and covar mat R and update the estimate.
    This code does not hold logic for GPS or IMU data processing 
    state = [x, y, theta, Vx, Vy, b_acc, b_gyro]
    """
    
    def __init__(self, sigma, covar_acc, covar_gyro, accel_bias_random, gyro_bias_random, dt):
        #Input arguments
        self.P = sigma  
        self.Q = self.crete_Q_mat(covar_acc, covar_gyro) 
        self.accel_bias_random = accel_bias_random
        self.gyro_bias_random = gyro_bias_random    
        self.dt = dt

    def crete_Q_mat(self, covar_acc, covar_gyro):
        coavr_vel = self.dt * covar_acc
        covar_dist = 0.5 * self.dt**2 * covar_acc
        covar_angle = self.dt * covar_gyro
        covar_array = [covar_dist, covar_dist, covar_angle, coavr_vel, coavr_vel, 0,0,0]
        Q_mat = np.diag(covar_array)
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
        accel = U[0:2]
        ang_vel = U[2]
        
        ##model
        x_pos_dt = Vx * np.cos(theta) - Vy * np.sin(theta)
        y_pos_dt = Vx * np.sin(theta) + Vy * np.cos(theta)
        theta_dt = ang_vel 
        Vx_dt = Vy * ang_vel + accel[0]
        Vy_dt = - Vx * ang_vel + accel[1]
        bias_acc_x_dt = np.random.normal(0, self.accel_bias_random[0])
        bias_acc_y_dt = np.random.normal(0, self.accel_bias_random[1])
        bias_gyro_dt = np.random.normal(0, self.gyro_bias_random[1])
        X_dt = [x_pos_dt, y_pos_dt, theta_dt, Vx_dt, Vy_dt, bias_acc_x_dt, bias_acc_y_dt, bias_gyro_dt]
        return X_dt 
    

    def apply_RK45(self, X, U):
        ##applt RK 45 here
        k1 = self.process_model(X, U)
        k2 = self.process_model(X + self.dt/2 * k1, U)
        k3 = self.process_model(X + self.dt/2 * k2, U)
        k4 = self.process_model(X + self.dt * k3, U)
        dx = (k1 + 2* k2 + 2 * k3 + k4) / 6 
        X = X + dx * self.dt  
        return X
         

    def predict(self, X, U, S):
        X = self.apply_RK45(X, U)
        F = self.get_FMat()
        V = self.get_VMat()
        S = F @ S @ F.T + V @ self.Q @ V.T
        return X, S 

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
                                  [0, 0, 0, 0, (w - bg), -1, 0, -Vy],
                                  [0, 0, 0, -(w - bg), 0, 0, -1, Vx],
                                  [0, 0, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0] ])
        
        FMat = np.eye(8) + self.dt * AMat
        return FMat
        

    def get_VMat(self, X, U):
        #noise vector = [wax, way, wg, wbax, wbay, wbg]
        # VMat = del fun() / del w_vec
        # it is derivative of system dynamics equations with respect to noise 
        # first 3 components are white noise, directly affects sensor reading, last 3 are bias random walk
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

        Mat = np.array([[0,0,0,0,0,0],
                         [0,0,0,0,0,0],
                         [0,0,1,0,0,0],
                         [1,0,Vy,0,0,0],
                         [0,1,-Vx, 0,0,0],
                         [0,0,0,1,0,0],
                         [0,0,0,0,1,0],
                         [0,0,0,0,0,1]])
        
        VMat = self.dt * Mat        
        return VMat
        
    
    def get_GMat(self):
        GMat = np.array([[1,0,0,0,0,0,0,0],
                         [0,1,0,0,0,0,0,0]])
        
        return GMat

    def get_WMat(self):
        WMat = np.eye(2)
        return WMat
        
    def update(self, X, S, z, R):
        #z = np array of [x, y] GPS pose
        #X = 8 * 1 state vector
        #R = 2 by 2 noise matrix for GPS reading
        #S = covariance of estimated state

        F = self.get_FMat()
        V = self.get_VMat()
        G = self.get_GMat()
        W = self.get_WMat()
        
        K = S @ G.T / (G @ S @ G.T + W @ R @ W.T)
        X = X + K @ (z - G @ X)
        S = S - K @ G @ S
        return X, S 
    


    
   

    




    


        
        



