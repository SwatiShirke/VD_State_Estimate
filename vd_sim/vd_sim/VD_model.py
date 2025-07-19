import numpy as np
from vd_sim.model_utils import *
import scipy.integrate

class VD_dynamics:
    def __init__(self, params, z, t_span):
        self.params = params
        self.z_state = z
        self.current_input = np.zeros(2)
        self.Kp = 600
        self.Ki = 0.0005
        self.Kd = 0.01
        self.t_solve = t_span
        self.cumm_error = 0
        self.last_error = 0
       

    def VD_model_closed_loop(self,t, z):        
        torque_in = self.current_input[0]
        delta = self.current_input[1]
        
        #states Vx, Vy, r, x, y, psi, w1, w1, w3, w4
        Vx = z[0]
        Vy = z[1]
        r  = z[2]
        x  = z[3]
        y  = z[4]
        psi= z[5]

        #torque_in = self.apply_PID(ref_vel, Vx)
        
        ##calculate intial accel values for load transfer calculations 
        ax = Vy * r
        ay = -Vx * r

        #input torue to each wheel 
        tau_in = cal_wheel_torque_in(self.params, torque_in)        
        tau_in_fl = tau_in[0]
        tau_in_fr = tau_in[1]
        tau_in_rl = tau_in[2]
        tau_in_rr = tau_in[3]

        #calculate forces
        t_forces = tire_forces(self.params, z, delta, ax, ay, torque_in)
        Fx_sum, Fy_f, Fy_r, Fy_del = vehicle_forces(self.params, z, t_forces, delta)

        Vx_dt = Vy * r + (Fx_sum / self.params.m)
        Vy_dt = -Vx * r + ((Fy_f+ Fy_r) / self.params.m)

        t_forces = tire_forces(self.params, z, delta, Vx_dt, Vy_dt,torque_in)
        Fx_sum, Fy_f, Fy_r, Fy_del = vehicle_forces(self.params, z, t_forces, delta)

        #state dt equations
        Vx_dt = Vy * r + (Fx_sum / self.params.m)
        Vy_dt = -Vx * r + ((Fy_f+ Fy_r) / self.params.m)
        r_dt =  (self.params.a * Fy_f - self.params.b *Fy_r) / self.params.Izz_v

        x_dt = Vx * np.cos(psi) - Vy * np.sin(psi)
        y_dt = Vx * np.sin(psi) + Vy * np.cos(psi)
        psi_dt  = r

        #%set tire forces 
        Fx_fl = t_forces[0]
        Fx_fr = t_forces[1]
        Fx_rl = t_forces[2]
        Fx_rr = t_forces[3] 

        #%wheel velocities 
        w_fl_dt = (tau_in_fl - Fx_fl * self.params.R_f) / self.params.Iyy_w_f
        w_fr_dt = (tau_in_fr - Fx_fr * self.params.R_f) / self.params.Iyy_w_f
        w_rl_dt = (tau_in_rl - Fx_rl * self.params.R_r) / self.params.Iyy_w_r
        w_rr_dt = (tau_in_rr - Fx_rr * self.params.R_r) / self.params.Iyy_w_r
        s_dt = Vx
        dz = [Vx_dt, Vy_dt, r_dt, x_dt, y_dt, psi_dt, w_fl_dt, w_fr_dt, w_rl_dt, w_rr_dt, s_dt]
        return dz 

    def apply_PID(self, ref_vel, Vx):
        error = (ref_vel - Vx )                
        torque = error * self.Kp + self.cumm_error * self.Ki + (self.last_error - error)* self.Kd
        self.cumm_error += error 
        self.last_error = error
        return torque

    def solve_VD(self, u):
        self.current_input = u
        # dz = self.VD_model_closed_loop( self.z_state, u)
        # print(dz)
        # self.cumm_error = 0
        # self.last_error = 0
        sol = scipy.integrate.solve_ivp(self.VD_model_closed_loop,self.t_solve, self.z_state, method= 'LSODA')
        #print(sol.y[-5])
        self.z_state  = sol.y[:, -1]
        return self.z_state
        

    def get_state(self):
        return self.z_state
    

if __name__ == "__main__":
    filename = "./params.yaml"
    params_object = load_params_from_yaml(filename)
    VD_model = VD_dynamics(params_object)
    u = [30,0]
    VD_model.solve_VD(u)
    z= VD_model.get_state()
    print(z)
    