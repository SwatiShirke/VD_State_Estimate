import yaml 
import numpy as np
from dataclasses import dataclass
import ipdb

@dataclass
class VehicleParams:
    m: float
    rho: float
    C_d: float
    Af: float
    V_wind: float
    a: float
    b: float
    l: float
    tf: float
    tr: float
    w: float
    Izz_v: float
    max_torque: float
    max_power: float
    C_x: float
    C_y: float
    R_f: float
    R_r: float
    tire_m_f: float
    tire_m_r: float
    Iyy_w_f: float
    Iyy_w_r: float
    slip_min: float
    no_drive_wheels: int
    g: float
    h: float
    min_vel: float
    max_vel: float
    Kp: float
    Ki: float
    Kd: float


def load_params_from_yaml(file_path: str) -> VehicleParams:
    with open(file_path, 'r') as file:
        params = yaml.safe_load(file)['params']
        return VehicleParams(
            m=params['m'],
            rho=params['rho'],
            C_d=params['C_d'],
            Af=params['Af'],
            V_wind=params['V_wind'],
            a=params['a'],
            b=params['b'],
            l=params['l'],
            tf=params['tf'],
            tr=params['tr'],
            w=params['w'],
            Izz_v=params['Izz_v'],
            max_torque=params['max_torque'],
            max_power=params['max_power'],
            C_x=params['C_x'],
            C_y=params['C_y'],
            R_f=params['R_f'],
            R_r=params['R_r'],
            tire_m_f=params['tire_m_f'],
            tire_m_r=params['tire_m_r'],
            Iyy_w_f=params['Iyy_w_f'],
            Iyy_w_r=params['Iyy_w_r'],
            slip_min=params['slip_min'],
            no_drive_wheels=params['no_drive_wheels'],
            g=params['g'],
            h=params['h'],
            min_vel= params['min_vel'],
            max_vel= params['max_vel'],
            Kp= params["Kp"],
            Ki= params["Ki"],
            Kd= params["Kd"]
        )


def cal_wheel_torque_in(params, torque_in):
    torque_in = max(min(params.max_torque,torque_in ), -params.max_torque)
    split_ratio = np.array([0.5, 0.5, 0.0, 0.0])  
    torque_in = split_ratio * torque_in
    return torque_in

def tire_forces(params, z, delta, ax, ay, torque_in):
    
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #% Weight distribution (assuming 50-50 split)
    weight_front = 0.5  #% Percentage of weight on front axle
    weight_rear = 1 - weight_front  #% Percentage on rear axle

    #%Static normal forces without acceleration
    F_drag =air_drag_force(params, z)
    load_transfer_longitudinal = (params.m * ax * params.h)
    Fz_front = ((weight_front * params.m * params.g * params.b) - load_transfer_longitudinal - 0 * params.h -F_drag * params.h ) / params.l   #% Force on each front tire
    Fz_rear = ((weight_rear * params.m * params.g * params.a) + load_transfer_longitudinal + 0 * params.h) / params.l     #% Force on each rear tire



    # % Load transfer due to lateral acceleration (cornering)
    # %load_transfer_lateral = (params.m * ay * params.h) / params.tf;

    #% Normal forces on each tire
    
    Fz_fl = Fz_front/2-(Fz_front/params.g)*ay* params.h / params.tf   #% Front-left tire
    Fz_fr = Fz_front/2+(Fz_front/params.g)*ay* params.h / params.tf  #% Front-right tire
    Fz_rl = Fz_rear/2-(Fz_rear/params.g)*ay* params.h / params.tr   #% Rear-left tire
    Fz_rr = Fz_rear/2+(Fz_rear/params.g)*ay* params.h / params.tr   #% Rear-right tire
    
    F_z = np.array([Fz_fl, Fz_fr, Fz_rl, Fz_rr])
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    Slips = caculate_slip(params, z,torque_in)
    F_x=fricx(Slips ,F_z) #%% Fx = [Fflx Ffrx Frlx Frrx]T - longitudinal forces are calulated using slip ratio -> 
                                                        #% friction is calculated from that
    alpha = calculate_slip_angles(params,z, delta )                                                   # % fx = f * fz
    F_y=fricy(alpha,F_z) #%%  Fy = [Ffly Ffry Frly Frry]T - lateral forces calculation using similar logic
    f_vector = np.array([F_x , F_y]).flatten()
    return f_vector

def air_drag_force(params, z):
    Vx = z[0]
    f_drag = 0 if Vx == 0 else 0.5 * params.rho * params.C_d * params.Af * (Vx + params.V_wind)**2
    return f_drag

def caculate_slip(params, z ,torque_in):
    #%wheel longitudinal velocity and angular rate
    Vx = z[0]
    r = z[2]
    #%%calculate linear velocities of each wheel 
    V_fl = Vx - (r * params.tf /2)
    V_fr = Vx + (r * params.tf /2)
    V_rl = Vx - (r * params.tr /2)
    V_rr = Vx + (r * params.tr /2)
    w_fl = z[6]
    w_fr = z[7]
    w_rl = z[8]
    w_rr = z[9] 
      
    if torque_in <= 0:
      D1 = V_fl 
      D2 = V_fr
      D3 = V_rl
      D4 = V_rr
    else:
      D1 = params.R_f * w_fl
      D2 = params.R_f * w_fr
      D3 = params.R_r * w_rl
      D4 = params.R_r * w_rr
    
    
    S_fl = min( max(-1, 0 if (D1 == 0) else (params.R_f * w_fl - V_fl )/ D1 ), 1)

    S_fr = min(max(-1, 0 if (D2 == 0) else (params.R_f * w_fr - V_fr) / D2), 1)
    S_rl = min(max(-1, 0 if (D3 == 0) else (params.R_r * w_rl - V_rl) / D3), 1) 
    S_rr = min(max(-1, 0 if (D4 == 0) else (params.R_r * w_rr - V_rr )/ D4), 1)

    slips = np.array([S_fl, S_fr, S_rl, S_rr])
    #ipdb.set_trace()
    #print("slips",slips)
    return slips

def calculate_slip_angles(params,z, delta):
    Vx = z[0]
    Vy = z[1]
    r =  z[2]
    
    if Vx == 0:
        D1 = 0 
        D2 = 0
        D3 = 0
        D4 = 0
    else:
        D1 =  (Vy + params.a * r )/  Vx 
        D2 =  (Vy + params.a * r )/  Vx 
        D3 =  (Vy - params.b * r )/  Vx 
        D4 =  (Vy - params.b * r )/  Vx


    term_a = np.arctan( D1 )
    term_b = np.arctan( D2 )
    term_c = np.arctan( D3 )
    term_d = np.arctan( D4 )

    alpha_fl =  0 if term_a == 0 else delta - term_a
    alpha_fr =  0 if term_b == 0 else delta - term_b 
    alpha_rl = 0 if term_c == 0 else -term_c
    alpha_rr = 0 if term_d == 0 else -term_d
    
    slip_angles = -1 * np.array([alpha_fl, alpha_fr, alpha_rl, alpha_rr ])
    return slip_angles

def fricx(Slips ,F_z):    
    B=11.275 
    C=1.56 
    D=1 
    E=-0.97   
    f = D  * np.sin(C * np.arctan(B * Slips - E * (B * Slips - np.arctan(B * Slips))))
    fx=f*F_z
    return fx

def fricy(alpha,F_z):    
    B=11.275
    C=1.56
    D=1
    E=-0.97
    f =  D * np.sin(C * np.arctan(B * alpha - E * (B * alpha - np.arctan(B * alpha))))
    fy= -1.4 *f*F_z
    return fy   

def vehicle_forces(params, z, t_forces, delta):     
    Fx_sum = longitudinal_forces(params, z, t_forces, delta)
    Fy_f, Fy_r,Fy_del = lateral_forces(params, z, t_forces,delta)
    return Fx_sum, Fy_f, Fy_r ,Fy_del

def longitudinal_forces(params, z, tire_forces, delta):
    F_traction = long_traction_forces( z, tire_forces, delta)
    F_drag =  air_drag_force(params, z)
    F_rolling = rolling_resistance(params, z)
    F_gradient = 0
    f_sum = F_traction - F_drag - F_rolling - F_gradient
    return f_sum


def long_traction_forces( z, tire_forces, delta):    
    f_vector = tire_forces
    Fx_fl = f_vector[0]
    Fx_fr = f_vector[1]
    Fx_rl = f_vector[2]
    Fx_rr = f_vector[3]

    Fy_fl = f_vector[4]
    Fy_fr = f_vector[5]
    Fy_rl = f_vector[6]
    Fy_rr = f_vector[7]
    #%temp_delta_value = (Fy_fr + Fy_fl) * sin(delta)     
    f_traction = ((Fx_fr + Fx_fl) * np.cos(delta) ) - ((Fy_fr + Fy_fl) * np.sin(delta) ) + (Fx_rl + Fx_rr)
    return f_traction 

def rolling_resistance(params, z):
    F_rolling = 0
    return F_rolling

def lateral_forces(params, z, tire_forces,delta):
    f_vector = tire_forces
    Fx_fl = f_vector[0]
    Fx_fr = f_vector[1]
    Fx_rl = f_vector[2]
    Fx_rr = f_vector[3]

    Fy_fl = f_vector[4]
    Fy_fr = f_vector[5]
    Fy_rl = f_vector[6]
    Fy_rr = f_vector[7]
          
    fy_f =  ((Fx_fr + Fx_fl) * np.sin(delta) +  (Fy_fr + Fy_fl) * np.cos(delta))          
    fy_r =  (Fy_rl + Fy_rr)
    fy_del = params.tf/2 * (Fx_fr - Fx_fl) * np.cos(delta)  + params.tr * (Fx_rr - Fx_rl) + params.tf/2 * (Fy_fr - Fy_fl)* np.sin(delta)  
    
    return fy_f, fy_r, fy_del

def apply_PID(Kp, Ki, Kd, ref_vel, Vx):
    torque = (ref_vel - Vx ) * Kp
    return torque 