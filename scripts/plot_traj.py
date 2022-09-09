#!/usr/bin/env python3

# Original author : Alexandre Rocha
# https://github.com/avrocha/front-tracking-algorithm

import numpy as np
import h5py as h5
import matplotlib.pyplot as plt
from matplotlib import animation
# from scipy.interpolate.interpolate import RegularGridInterpolator
from scipy.interpolate import RegularGridInterpolator

from argparse import ArgumentParser


def parse_args():
    parser = ArgumentParser()
    parser.add_argument("path", type=str, help="Path to the HDF5 file containing the processed data.")
    parser.add_argument("--anim", action="store_true", help="Plot animation instead single plot.")
    parser.add_argument("--save_anim", type=str, help="Save the animation in the given file.")
    parser.add_argument("--ref", action="store_true", help="Plot comparison between measurements and \
                                                            reference value instead single plot.")
    parser.add_argument("--grad_error", action='store_true', help="Plot cosine of gradient deviation.")

    return parser.parse_args()


def update_anim(i, traj, line, pcm):
    line.set_data(traj[:i, 0], traj[:i, 1])

    t_idx = np.argmin(np.abs(traj[i,-1] - time))
    pcm.set_array(chl[:,:,t_idx])

    return line, pcm


def plot_anim(traj):
    fig = plt.figure()
    pcm = plt.pcolormesh(xx, yy, chl[:,:,0], cmap='jet', shading='auto')
    plt.colorbar()

    line, = plt.plot(traj[:, 0], traj[:, 1], 'r-', linewidth=2)

    plt.xlabel('WGS84 Longitude')
    plt.ylabel('WGS84 Latitude')

    ani = animation.FuncAnimation(fig, update_anim, fargs=[traj, line, pcm])

    writervideo = animation.FFMpegWriter(fps=60)

    if args.save_anim:
        ani.save(args.save_anim, writer=writervideo)
    else:
        plt.show()


args = parse_args()

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

chl_interp = RegularGridInterpolator((lon, lat, time), chl)

if args.anim:
    traj_short = traj[0:-1:int(traj.shape[0]/300)]
    plot_anim(traj_short)

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

# Plot trajectory
fig, ax = plt.subplots()
ax.set_aspect('equal')
xx, yy = np.meshgrid(lon, lat, indexing='ij')
p = plt.pcolormesh(xx, yy, chl[:,:,t_idx], cmap='viridis', shading='auto', vmin=0, vmax=10)
cax = fig.add_axes([ax.get_position().x1+0.01,ax.get_position().y0,0.02,ax.get_position().height])
cp = fig.colorbar(p, cax=cax)
cp.set_label("Chl a density [mm/mm3]")
ax.contour(xx, yy, chl[:,:,t_idx], levels=[delta_ref])
ax.plot(traj[n:n+offset,0], traj[n:n+offset,1], 'r', linewidth=3)
ax.set_xlabel("Longitude (degrees E)")
ax.set_ylabel("Latitude (degrees N)")
plt.savefig("traj.png",bbox_inches='tight')

# Plot gradient arrows
for index in range(delta_vals.shape[0]):
    if index % 100 == 0 :
        ax.arrow(x=traj[index,0], y=traj[index,1], dx=0.00005*grad_vals[index][0], dy=0.00005*grad_vals[index][1], width=.00002)
plt.show()

# Front detection idx and x-axis construction - only for full trajectories
if args.grad_error or args.ref:
    idx_trig = 0
    for i in range(len(delta_vals)):
        if delta_ref - 5e-3 <= delta_vals[i]:
            idx_trig = i
            break

    if traj.shape[1] == 3:
        delta_t = (traj[-1,-1] - traj[0,-1])/3600
        it = np.linspace(0, delta_t, len(delta_vals))
    elif traj.shape[1] == 2:
        it = np.linspace(0, time_step*(len(traj[:, 0])-1)/3600, len(delta_vals))


if args.grad_error:

    it = it
    
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
    plt.figure()
    plt.plot(it, dot_prod_cos, 'k-', linewidth=0.8, label='Gradient deviation cosine')
    plt.plot(it, grad_ref, 'r-', linewidth=1.5, label='Reference')
    plt.plot(np.tile(it[idx_trig], 10), np.linspace(np.min(grad_vals), 1.2, 10), 'r--')
    plt.xlabel('Mission time [h]')
    plt.ylabel('Cosine')
    # plt.title("Gradient deviation cosine\nAverage relative error = %.4f %%" % (error))
    plt.axis([0, np.max(it), -1.2, 1.2])
    plt.legend(loc=4, shadow=True)

    # Cosine comparison
    plt.figure()
    plt.plot(it, grad_vals_cos, 'k-', linewidth=0.8, label='Estimated from GP Model')
    plt.plot(it, gt_grad_vals_cos, 'r-', linewidth=1.5, label='Ground truth')
    plt.plot(np.tile(it[idx_trig], 10), np.linspace(np.min(grad_vals), 1.2, 10), 'r--')
    plt.xlabel('Mission time [h]')
    plt.ylabel('Cosine')
    # plt.title("Cosine of GT and Estimated gradient")
    plt.axis([0, np.max(it), -1.2, 1.2])
    plt.legend(loc=4, shadow=True)
    plt.grid(True)
    plt.savefig("cos.png",bbox_inches='tight')

    # Plot gradient angle
    plt.figure()
    plt.plot(it, gt_grad_angles, 'k-', linewidth=0.8, label='Ground truth')
    plt.plot(it, grad_angles, 'r-', linewidth=1.5, label='Estimated from GP Model')    
    plt.xlabel('Mission time [h]')
    plt.ylabel('Gradient [rad]')
    # plt.axis([0, np.max(it), -1.2, 1.2])
    plt.legend(loc=4, shadow=True)
    plt.grid(True)
    plt.savefig("grad.png",bbox_inches='tight')

    # Plot gradients
    # step = int(grad_vals.shape[0] / 20)
    # for i in range(0, grad_vals.shape[0], step):
    #     q = plt.quiver(traj[i,0], traj[i,1], grad_vals[i,0], grad_vals[i,1], color='black', width=0.004)
    #     q_1 = plt.quiver(traj[i,0], traj[i,1], gt_grad_vals[i,0], gt_grad_vals[i,1], color='red', width=0.004)


# Reference tracking error
if args.ref:
    it = it/5
    error = np.mean(np.abs(delta_vals[idx_trig:] - delta_ref)/delta_ref)*100
    print("Reference average relative error = %.4f %%" % (error))
    plt.figure()
    plt.plot(it, delta_vals, 'k-', linewidth=1, label="Measured Chl density")
    plt.plot(it, np.tile(delta_ref, len(it)), 'r-', label="Chl density reference")
    plt.plot(np.tile(it[idx_trig], 10), np.linspace(np.min(delta_vals), delta_ref*1.4, 10), 'r--')
    plt.xlabel('Mission time [h]')
    plt.ylabel('Chl a density [mm/mm3]')
    # plt.axis([0, it[-1], np.min(delta_vals), 0.5+np.max(delta_vals)])
    plt.axis([0, it[-1], delta_ref-1.5, delta_ref+1.5])
    # plt.title("Measurements \n Average relative error = %.4f %%" % (error))
    plt.legend(loc=4, shadow=True)
    plt.grid(True)
    plt.savefig("chl.png",bbox_inches='tight')

plt.show()