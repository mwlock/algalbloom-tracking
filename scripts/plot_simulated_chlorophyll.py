import matplotlib.pyplot as plt
import scipy.io
import numpy as np
from scipy.interpolate import RegularGridInterpolator


print('import')
fig,ax = plt.subplots()

class GeoGrid:
    def __init__(self, data, lon, lat, time, t_idx, include_time=False):
        self.data = data
        self.lon = lon
        self.lat = lat
        self.time = time
        self.t_idx = t_idx

        if include_time is False:
            self.field = RegularGridInterpolator((self.lon, self.lat), self.data[:,:,self.t_idx])
        else:
            self.field = RegularGridInterpolator((self.lon, self.lat, self.time), self.data)

    def is_within_limits(self, x, include_time=False):
        if include_time is False:
            if (self.lon[0] <= x[0] <= self.lon[-1]) and (self.lat[0] <= x[1] <= self.lat[-1]):
                return True
        else:
            if (self.lon[0] <= x[0] <= self.lon[-1]) and (self.lat[0] <= x[1] <= self.lat[-1]) and (self.time[0] <= x[2] <= self.time[-1]):
                return True


def read_mat_data(timestamp,include_time=False,scale_factor=1,lat_shift=0,lon_shift = 0):

    # Get datapath
    base_path = '../data'

    # Read mat files
    chl = scipy.io.loadmat(base_path+'/chl.mat')['chl']
    lat = scipy.io.loadmat(base_path+'/lat.mat')['lat']
    lon = scipy.io.loadmat(base_path+'/lon.mat')['lon']
    time = scipy.io.loadmat(base_path+'/time.mat')['time']

    # Reshape
    lat = np.reshape(lat,[-1,])
    lon = np.reshape(lon,[-1,])
    chl = np.swapaxes(chl,0,2)
    time = np.reshape(time,[-1,])    

    # Scale data        
    lat = ((lat - lat[0])*scale_factor)+lat[0]
    lon = ((lon - lon[0])*scale_factor)+lon[0]

    # Shift the data
    lat = lat + lat_shift
    lon = lon + lon_shift

    # Logging
    print('Scale factor : {}'.format(scale_factor))
    print("Dimensions of lat {} - {}".format(lat[0],lat[-1]))
    print("Dimensions of lon {} - {}".format(lon[0],lon[-1]))

    t_idx = np.argmin(np.abs(timestamp - time))

    return GeoGrid(chl, lon, lat, time, t_idx, include_time=include_time)

scale_factor =  float(1)/100
delta_ref = 7.25

# WGS84 grid (lookup-table for sampling)
include_time = False
timestamp = 1618610399

grid = read_mat_data(timestamp, include_time=include_time,scale_factor=scale_factor)

ax.set_aspect('equal')
xx, yy = np.meshgrid(grid.lon, grid.lat, indexing='ij')
p = plt.pcolormesh(xx, yy, grid.data[:,:,grid.t_idx], cmap='viridis', shading='auto', vmin=0, vmax=10)
cax = fig.add_axes([ax.get_position().x1+0.01,ax.get_position().y0,0.02,ax.get_position().height])
cp = fig.colorbar(p, cax=cax)
cp.set_label("Chl a density [mm/mm3]")
ax.contour(xx, yy, grid.data[:,:,grid.t_idx], levels=[delta_ref])
plt.show()