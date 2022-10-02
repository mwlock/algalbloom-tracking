#!/usr/bin/env python3

# Original author : Alexandre Rocha
# https://github.com/avrocha/front-tracking-algorithm

import sys

import numpy as np
import h5py as h5
import matplotlib.pyplot as plt
from matplotlib import animation
# from scipy.interpolate.interpolate import RegularGridInterpolator

from scipy.interpolate import RegularGridInterpolator
from scipy import spatial

from argparse import ArgumentParser

import geopy.distance

# Setup plotting style

# https://github.com/garrettj403/SciencePlots/issues/15

plt.style.reload_library()
plt.style.use(['science', 'no-latex'])
plt.rcParams.update({'xtick.labelsize': 20,
                    'ytick.labelsize': 20,
                    'axes.titlesize': 20,
                    'axes.labelsize': 20,
                    'legend.fontsize': 20,
                    'legend.frameon' : True
                    })


# Added runtime arguments
def parse_args():
    parser = ArgumentParser()
    parser.add_argument("path", type=str, help="Path to the HDF5 file containing the processed data.")
    parser.add_argument("--ref", action="store_true", help="Plot comparison between measurements and \
                                                            reference value instead single plot."),
    parser.add_argument("--ref_error", action="store_true", help="Plot distance between path and reference level.")                                                            
    parser.add_argument("--grad_error", action='store_true', help="Plot cosine of gradient deviation.")
    parser.add_argument("--pdf", action='store_true', help="Save plots in pdf format")
    parser.add_argument('-z','--zoom', nargs='+', help='Zoom on a particlar region of the map [x0,y0,width,height]', \
        required=False,type=lambda s: [float(item) for item in s.split(',')])

    return parser.parse_args()

# Parse in arguments
args = parse_args()

# Set plot format
extension = 'png'
if args.pdf:
    extension = 'pdf'

# Read h5 file
with h5.File(args.path, 'r') as f:
    lon = f["lon"][()]
    lat = f["lat"][()]
    chl = f["chl"][()]
    time = f["time"][()]
    traj = f["traj"][()]
    delta_vals = f["measurement_vals"][()]
    grad_vals = f["grad_vals"][()]
    delta_ref = f.attrs["delta_ref"]
    time_step = f.attrs["meas_period"]
    meas_per = f.attrs["meas_period"]
    t_idx = f.attrs["t_idx"]

    attributes = f.attrs.items()

# Create data grid
chl_interp = RegularGridInterpolator((lon, lat, time), chl)

def trim_zeros(arr):
     """Returns a trimmed view of an n-D array excluding any outer
     regions which contain only zeros.
     """
     slices = tuple(slice(idx.min(), idx.max() + 1) for idx in np.nonzero(arr))
     return arr[slices]

# Change n, offset values bellow to plot different parts of the trajectory
traj = trim_zeros(traj)

n = 0
offset = traj.shape[0]-1

if traj.shape[1] == 3:
    t_idx = np.argmin(np.abs(traj[n+offset,-1] - time))

# Print attributes
if sys.version_info.major == 2:
    for att in attributes:
        print("{} : {}".format(att[0],att[1]))


def plot_trajectory(axis, show_contour_legend = False):
    """ Plot SAM trajectory """
    xx, yy = np.meshgrid(lon, lat, indexing='ij')
    p = axis.pcolormesh(xx, yy, chl[:,:,t_idx], cmap='viridis', shading='auto', vmin=0, vmax=10)
    cs = axis.contour(xx, yy, chl[:,:,t_idx], levels=[delta_ref])
    axis.clabel(cs, inline=1, fontsize=10)
    axis.plot(traj[n:n+offset,0], traj[n:n+offset,1], 'r', linewidth=3)

    path = None
    if show_contour_legend:
        # https://matplotlib.org/stable/gallery/images_contours_and_fields/contour_demo.html
        # https://www.tutorialspoint.com/how-to-get-coordinates-from-the-contour-in-matplotlib

        # Determine which path is the longest (treat this as the gradient path)
        longest_path = 0
        for i in cs.collections[0].get_paths():
            path_length = i.vertices.shape[0]
            if path_length>longest_path:
                longest_path = path_length
                path  = i

        # plt.figure()
        # x = path.vertices[:, 0]
        # y = path.vertices[:, 1]
        # plt.plot(x,y)
        # plt.plot(traj[n:n+offset,0], traj[n:n+offset,1], 'r', linewidth=3)
        # plt.title('Reference path')
        # plt.legend(['Reference path','True path'])
        
        path = path.vertices

    return p,path

def plot_inset(axis,inset,zoom):
    """Add inset (zoom) to plot
    

    Parameters
    ----------
    inset : list, required
        Position of where to placed 'zoomed' axis. 
        [x0, y0, width, height] where [0,5,0,5,0.3,0.3] represents a region in the top right hand corner
        30% the width and height of the original plot.
    zoom : list, required
        coordinates of the original plot that should be zoomed into
        [x_lim_0,x_lim_1,y_lim_0,y_lim_1]
    """
    
    # Create an inset axis at coordinates [inset]
    axin = axis.inset_axes(inset) 

    # Plot the data on the inset axis\stable\gallery\lines_bars_and_markers\joinstyle.html
    plot_trajectory(axis=axin)

    # Zoom in on the noisy data in the inset axis
    axin.set_xlim(zoom[0], zoom[1])
    axin.set_ylim(zoom[2], zoom[3])

    # Hide inset axis ticks
    axin.set_xticks([])
    axin.set_yticks([])

    # Add the lines to indicate where the inset axis is coming from
    # axis.indicate_inset(axin,edgecolor="black",linestyle="-.")
    axis.indicate_inset_zoom(axin,edgecolor="black",linestyle="-.")    


# Plot trajectory
fig, ax = plt.subplots(figsize=(15, 7))
p,ref_path = plot_trajectory(axis=ax,show_contour_legend=True)
ax.set_aspect('equal')
cax = fig.add_axes([ax.get_position().x1+0.01,ax.get_position().y0,0.02,ax.get_position().height])
cp = fig.colorbar(p, cax=cax)
cp.set_label("Chl a density [mm/mm3]")
ax.set_xlabel("Longitude (degrees E)")
ax.set_ylabel("Latitude (degrees N)")
plt.grid(True)

if args.zoom:

    # Show zoomed in portion of the trajectory
    # https://matplotlib.org/stable/gallery/subplots_axes_and_figures/zoom_inset_axes.html
    # https://stackoverflow.com/questions/13583153/how-to-zoomed-a-portion-of-image-and-insert-in-the-same-plot-in-matplotlib
    # https://towardsdatascience.com/mastering-inset-axes-in-matplotlib-458d2fdfd0c0

    # https://proplot.readthedocs.io/en/latest/insets_panels.html

    # Portion of figure to zoom
    a = 0.1
    b = 0.2

    # Get lenght of lat and lon
    lon_length = (lon[-1] - lon[0])
    lat_length = (lat[-1] - lat[0])

    print(args.zoom[0][0])
    # print(args.zoom[1])

    # Get centre coordinates of where to zoom
    x_centre = lon_length*args.zoom[0][0] + lon[0]
    y_centre = lat_length*args.zoom[0][1] + lat[0]

    # Determine x_0 and y_0 of zoomed region
    x0 = x_centre - lon_length*a/2
    y0 = y_centre - lat_length*b/2

    # Determine x_1 and y_1 of zoomed region
    x1 = x_centre + lon_length*a/2
    y1 = y_centre + lat_length*b/2

    # Consider replacing with https://matplotlib.org/stable/gallery/axes_grid1/inset_locator_demo.html

    plot_inset(axis=ax,inset=[0.6, 0.3, a*3, b*3],zoom=[x0,x1,y0,y1])
    # plt.tight_layout()

# plt.show()
# Plot gradient arrows
# for index in range(delta_vals.shape[0]):
#     if index % 10 == 0 :
#         ax.arrow(x=traj[index,0], y=traj[index,1], dx=0.00005*grad_vals[index][0], dy=0.00005*grad_vals[index][1], width=.00002)
plt.savefig("../plots/traj.{}".format(extension),bbox_inches='tight')

# Front detection idx and x-axis construction - only for full trajectories
if args.grad_error or args.ref_error or args.ref:
    idx_trig = 0

    # Determine index of traj where AUV reaches the front
    for i in range(len(delta_vals)):
        if delta_ref - 5e-3 <= delta_vals[i]:
            idx_trig = i
            break

    if traj.shape[1] == 3:
        delta_t = (traj[-1,-1] - traj[0,-1])/3600
        it = np.linspace(0, delta_t, len(delta_vals))

    # Create mission time axis
    elif traj.shape[1] == 2:
        it = np.linspace(0, time_step*(len(traj[:, 0])-1)/3600, len(delta_vals))
        idx_trig_time = it[idx_trig]

if args.grad_error or args.ref:
    
    # gt => ground truth
    gt_grad_vals = np.zeros([delta_vals.shape[0], 2])
    dot_prod_cos = np.zeros(delta_vals.shape[0])
    grad_vals_cos = np.zeros(delta_vals.shape[0])
    gt_grad_vals_cos = np.zeros(delta_vals.shape[0])

    if traj.shape[1] == 2:

        # Ground truth gradient
        gt_grad = np.gradient(chl[:,:,t_idx])
        gt_grad_norm = np.sqrt(gt_grad[0]**2 + gt_grad[1]**2)
        gt_gradient = (RegularGridInterpolator((lon, lat), gt_grad[0]/gt_grad_norm),
                    RegularGridInterpolator((lon, lat), gt_grad[1]/gt_grad_norm))
        
        # Compute ground truth gradients
        for i in range(1,delta_vals.shape[0]-1):
            if i % 2000 == 0:
                print("Computing gradient... Current iteration:", i)

            x = int(i*meas_per)
            gt_grad_vals[i, 0] = gt_gradient[0]((traj[x,0], traj[x,1]))
            gt_grad_vals[i, 1] = gt_gradient[1]((traj[x,0], traj[x,1]))
            dot_prod_cos[i] = np.dot(grad_vals[i], gt_grad_vals[i]) / (np.linalg.norm(grad_vals[i]) * np.linalg.norm(gt_grad_vals[i]))

            grad_vals_cos[i] = grad_vals[i, 0] / np.linalg.norm(grad_vals[i])
            gt_grad_vals_cos[i] = gt_grad_vals[i, 0] / np.linalg.norm(gt_grad_vals[i])

        # Determine gradient angle
        gt_grad_angles = np.arctan2(gt_grad_vals[:, 1],gt_grad_vals[:, 0])
        grad_angles = np.arctan2(grad_vals[:, 1],grad_vals[:, 0])

    else:
        gt_grad = np.gradient(chl)
        gt_grad_norm = np.sqrt(gt_grad[0]**2 + gt_grad[1]**2)
        gt_gradient = (RegularGridInterpolator((lon, lat, time), gt_grad[0]/gt_grad_norm),
                    RegularGridInterpolator((lon, lat, time), gt_grad[1]/gt_grad_norm))

        # Compute ground truth gradients
        for i in range(delta_vals.shape[0]-1):
            if i % 10000 == 0:
                print("Computing gradient... Current iteration:", i)

            gt_grad_vals[i, 0] = gt_gradient[0]((traj[i*meas_per,0], traj[i*meas_per,1], traj[i*meas_per,2]))
            gt_grad_vals[i, 1] = gt_gradient[1]((traj[i*meas_per,0], traj[i*meas_per,1], traj[i*meas_per,2]))
            dot_prod_cos[i] = np.dot(grad_vals[i*meas_per], gt_grad_vals[i]) / (np.linalg.norm(grad_vals[i*meas_per]) * np.linalg.norm(gt_grad_vals[i]))

            grad_vals_cos[i] = grad_vals[i*meas_per, 0] / np.linalg.norm(grad_vals[i*meas_per])
            gt_grad_vals_cos[i] = gt_grad_vals[i, 0] / np.linalg.norm(gt_grad_vals[i])

    grad_ref = np.ones(dot_prod_cos.shape)
    error = np.mean(np.abs(dot_prod_cos[idx_trig:] - grad_ref[idx_trig:])/grad_ref[idx_trig:]) * 100
    print("Cosine average relative error = %.4f %%" % (error))
    # Dot-product cosine plot
    plt.subplots(figsize=(15, 7))
    plt.plot(it, dot_prod_cos, 'k-', linewidth=0.8, label='Gradient deviation cosine')
    plt.plot(it, grad_ref, 'r-', linewidth=1.5, label='Reference')
    plt.plot(np.tile(it[idx_trig], 10), np.linspace(np.min(grad_vals), 1.2, 10), 'r--')
    plt.xlabel('Mission time [h]')
    plt.ylabel('Cosine')
    # plt.title("Gradient deviation cosine\nAverage relative error = %.4f %%" % (error))
    plt.axis([0, np.max(it), -1.2, 1.2])
    plt.legend(loc=4, shadow=True)
    plt.grid(True)
    plt.savefig("../plots/cos_deviation.{}".format(extension),bbox_inches='tight')

    # Cosine comparison
    plt.subplots(figsize=(15, 7))
    plt.plot(it, grad_vals_cos, 'k-', linewidth=0.8, label='Estimated from GP Model')
    plt.plot(it, gt_grad_vals_cos, 'r-', linewidth=1.5, label='Ground truth')
    plt.plot(np.tile(it[idx_trig], 10), np.linspace(np.min(grad_vals), 1.2, 10), 'r--')
    plt.xlabel('Mission time [h]')
    plt.ylabel('Cosine')
    # plt.title("Cosine of GT and Estimated gradient")
    plt.axis([0, np.max(it), -1.2, 1.2])
    plt.legend(loc=4, shadow=True)
    plt.grid(True)
    plt.savefig("../plots/cos.{}".format(extension),bbox_inches='tight')

    # Plot gradient angle
    plt.subplots(figsize=(15, 7))
    plt.plot(it, gt_grad_angles, 'r-', linewidth=0.8, label='Ground truth')
    plt.plot(it, grad_angles, 'k-', linewidth=1.5, label='Estimated from GP Model')
    plt.xlabel('Mission time [h]')
    plt.ylabel('Gradient [rad]')
    # plt.axis([0, np.max(it), -1.2, 1.2])
    plt.legend(loc=4, shadow=True)
    plt.grid(True)
    plt.savefig("../plots/grad.{}".format(extension),bbox_inches='tight')

    # Plot gradients
    # step = int(grad_vals.shape[0] / 20)
    # for i in range(0, grad_vals.shape[0], step):
    #     q = plt.quiver(traj[i,0], traj[i,1], grad_vals[i,0], grad_vals[i,1], color='black', width=0.004)
    #     q_1 = plt.quiver(traj[i,0], traj[i,1], gt_grad_vals[i,0], gt_grad_vals[i,1], color='red', width=0.004)


# Reference tracking error
if args.ref:
    error = np.mean(np.abs(delta_vals[idx_trig:] - delta_ref)/delta_ref)*100
    print("Reference average relative error = %.4f %%" % (error))
    plt.subplots(figsize=(15, 7))
    plt.plot(it, delta_vals, 'k-', linewidth=1, label="Measured Chl density")
    plt.plot(it, np.tile(delta_ref, len(it)), 'r-', label="Chl density reference")
    plt.plot(np.tile(it[idx_trig], 10), np.linspace(np.min(delta_vals), delta_ref*1.4, 10), 'r--')
    plt.xlabel('Mission time [h]')
    plt.ylabel('Chl a density [mm/mm3]')
    # plt.axis([0, it[-1], np.min(delta_vals), 0.5+np.max(delta_vals)])
    plt.axis([0, it[-1], 0, 12])
    # plt.title("Measurements \n Average relative error = %.4f %%" % (error))
    plt.legend(loc=4, shadow=True)
    plt.grid(True)
    plt.savefig("../plots/ref.{}".format(extension),bbox_inches='tight')

# Euclidean distance between position and ref position
if args.ref_error:

    # https://stackoverflow.com/questions/10818546/finding-index-of-nearest-point-in-numpy-arrays-of-x-and-y-coordinates
    # https://stackoverflow.com/questions/19412462/getting-distance-between-two-points-based-on-latitude-longitude

    # Array to store distance
    dist = np.zeros(len(traj[n:n+offset,0]))
    it = np.linspace(0, time_step*(len(traj[:, 0])-1)/3600, len(traj[n:n+offset,0]))

    # Data matrices
    true_path = np.array([traj[n:n+offset,0], traj[n:n+offset,1]])
    true_path = true_path.transpose()

    # Tree struct
    tree = spatial.KDTree(ref_path)

    print("Calculating distance error")
    percent = round(len(true_path)/100)*10
    
    for ind, point in enumerate(true_path):
        # Closest point between true path and ref path
        __ ,index = tree.query(point)

        # Compute euclidean distance
        distance = geopy.distance.geodesic(point, ref_path[index]).m

        dist[ind] = distance
        if ind % percent == 0:
            print('Complete {:.2f} %'.format(ind/len(true_path)*100))
        
    plt.subplots(figsize=(15, 7))
    plt.plot(it,dist,'k')
    plt.xlabel('Mission time [h]')
    plt.ylabel('Distance to front [m]')
    # plt.yscale("log")
    # plt.plot(np.tile(it[idx_trig], 10), np.linspace(np.min(delta_vals), delta_ref*1.4, 10), 'r--')
    # plt.plot(np.tile(it[idx_trig], 10), np.linspace(np.min(delta_vals), delta_ref*1.4, 10), 'r--')
    plt.plot(np.tile(idx_trig_time, 10), np.linspace(np.max(dist), 0, 10), 'r--')
    plt.grid(True)
    plt.savefig("../plots/dist_to_front.{}".format(extension),bbox_inches='tight')


plt.show()