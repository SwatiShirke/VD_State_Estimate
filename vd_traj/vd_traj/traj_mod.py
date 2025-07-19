import numpy as np
import os
from pathlib import Path
from scipy.interpolate import interp1d
from ament_index_python.packages import get_package_share_directory
import os
package_share_directory = get_package_share_directory('vd_traj')
tracks_directory = os.path.join(package_share_directory, 'tracks')


class traj:
    def __init__(self):
        self.current_traj = None
        self.s_interpld = None
        self.x_interpld = None
        self.y_interpld = None
        self.yaw_interpld = None
        self.vel_interpld = None 

    def set_traj(self, traj_type):
        self.current_traj = traj_type
        if traj_type == "sine_wave":
            file_name = "sine_wave.txt"
            self.getTrack(file_name)


    def getTrack(self, filename):
        track_file = os.path.join(tracks_directory, filename)
        array=np.loadtxt(track_file)
        Sref=array[:,0]
        Xref=array[:,1]
        Yref=array[:,2]
        Yawref=array[:,3]
        Velref=array[:,4]
        
        self.x_interpld = interp1d(Sref,Xref, kind='cubic', fill_value="extrapolate")
        self.y_interpld = interp1d(Sref, Yref, kind='cubic', fill_value="extrapolate")
        self.yaw_interpld = interp1d(Sref, Yawref, kind='cubic', fill_value="extrapolate")
        self.vel_interpld = interp1d(Sref, Velref, kind='cubic', fill_value="extrapolate")
        self.last_pose = [ Sref[-1], Xref[-1], Yref[-1], Yawref[-1], Velref[-1] ]        



