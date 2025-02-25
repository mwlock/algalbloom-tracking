import math
import utm
import numpy as np
import h5py as h5

out_path="."


with h5.File("new_map.h5", 'r') as f:

    chl =  f["chl"][()] 
    lon =  f["lon"][()] 
    lat =  f["lat"][()] 

with h5.File(out_path+"/old.h5", 'r') as f:

    traj = f["traj"][()]
    time = f["time"][()]
    measurement_vals = f["measurement_vals"][()]
    grad_vals = f["grad_vals"][()]

    t_idx = f.attrs["t_idx"]
    delta_ref = f.attrs["delta_ref"]
    meas_period = f.attrs["meas_period"]

with h5.File(out_path+"/mission.m5", 'w') as f:
    f.create_dataset("traj", data=traj)
    f.create_dataset("chl", data=chl)
    f.create_dataset("lon", data=lon)
    f.create_dataset("lat", data=lat)
    f.create_dataset("time", data=time)
    f.create_dataset("measurement_vals", data=measurement_vals)
    f.create_dataset("grad_vals", data=grad_vals)
    f.attrs.create("t_idx", data=t_idx)
    f.attrs.create("delta_ref", data=delta_ref)
    f.attrs.create("meas_period", data=meas_period) 