import math
import utm
import numpy as np
import h5py as h5

from controller.positions import AbsolutePosition

class Utils():

    @staticmethod
    def toPolar(x, y):
        angle = math.atan2(y, x)
        norm = math.sqrt(x * x + y * y)
        return (angle),norm

    @staticmethod
    def displace(current_position,dx,dy):
        """ Returns lat lon after dx and dy displacement """

        current_utm_coords = utm.from_latlon(current_position.lat, current_position.lon)
        x = current_utm_coords[0] + dx
        y = current_utm_coords[1] + dy

        displaced = utm.to_latlon(x,y,current_utm_coords[2],current_utm_coords[3])
        return displaced[0],displaced[1]

    @staticmethod
    def displacement(current_position,virtual_position):
        """ Return the displacement in m between current positon and virtual position"""

        current_position_utm_coords = utm.from_latlon(current_position.lat, current_position.lon)
        virtual_position_utm_coords = utm.from_latlon(virtual_position.lat, virtual_position.lon)

        dx = virtual_position_utm_coords[0] - current_position_utm_coords[0]
        dy = virtual_position_utm_coords[1] - current_position_utm_coords[1]

        return dx,dy
    
    @staticmethod
    def nonabs_1D_dist(x, X):
        res = np.zeros((x.shape[0], X.shape[0]))

        for i in range(res.shape[0]):
            for j in range(res.shape[1]):
                res[i, j] = x[i] - X[j]

        return res

    @staticmethod
    def save_raw_mission_data(out_path,measurements,grads,delta_ref,traj,measurement_pos):
        """
        Write raw mission data to out_path\raw.m5 when algal bloom tracking node is closed
        """

        with h5.File(out_path+"/raw.h5", 'w') as f:
            f.create_dataset("traj", data=traj)
            f.create_dataset("measurement_vals", data=measurements)
            f.create_dataset("measurement_pos", data=measurement_pos)
            f.create_dataset("grad_vals", data=grads)
            f.attrs.create("delta_ref", data=delta_ref)
            

    @staticmethod
    def save_mission(out_path,grid,meas_per,sample_params,track_params):
        """
        Save mission measurements/traj/etc

        Works by reading the last saved output/raw.m5 file and combining this with the simulated grid of the current mission
        """

        with h5.File(out_path+"/raw.h5", 'r') as f:
            traj = f["traj"][()]
            measurement_vals = f["measurement_vals"][()]
            measurement_pos = f["measurement_pos"][()]

            grad_vals = f["grad_vals"][()]
            delta_ref = f.attrs["delta_ref"]

        with h5.File(out_path+"/mission.m5", 'w') as f:
            f.create_dataset("traj", data=traj)
            f.create_dataset("chl", data=grid.data)
            f.create_dataset("lon", data=grid.lon)
            f.create_dataset("lat", data=grid.lat)
            f.create_dataset("time", data=grid.time)
            f.create_dataset("measurement_vals", data=measurement_vals)
            f.create_dataset("measurement_pos", data=measurement_pos)
            f.create_dataset("grad_vals", data=grad_vals)
            f.attrs.create("t_idx", data=grid.t_idx)
            f.attrs.create("delta_ref", data=delta_ref)
            f.attrs.create("meas_period", data=meas_per) 

            for key in sample_params:
                f.attrs.create(key, data=sample_params[key]) 

            for key in track_params:
                f.attrs.create(key, data=track_params[key])
            