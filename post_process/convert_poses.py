import os
import numpy as np
import cv2
import csv
from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy

"""
convert pose to transformation matrix
"""
def tf_from_vect(quaternions, translation):
    tf = np.zeros((4,4))
    tf[:3,:3] = R.from_quat(quaternions).as_matrix()
    tf[:3,3] = translation
    tf[3,3] = 1
    return tf

"""
x axis transform
"""
def rot_phi(phi):
    return np.array([[1,0,0,0],
                    [0,np.cos(phi),-np.sin(phi),0],
                    [0,np.sin(phi), np.cos(phi),0],
                    [0,0,0,1]])
"""
y axis transform
"""
def rot_theta(th):
    return np.array([[np.cos(th),0,np.sin(th),0],
                    [0,1,0,0],
                    [-np.sin(th),0, np.cos(th),0],
                    [0,0,0,1]])
"""
z axis transform
"""
def rot_psi(psi):
    return np.array([[np.cos(psi),-np.sin(psi),0, 0],
                    [np.sin(psi), np.cos(psi),0, 0],
                    [0, 0, 1, 0],
                    [0,0,0,1]])

"""
generate a .npy file of poses for nerf training
"""
def generate_npy(fname, tf2save_array, segmented = True, idx_range = None):
    if segmented == True:
        np.save(fname, tf2save_array)
    else:
        tf2save_array_short = tf2save_array[idx_range[0]:idx_range[1],:]
        np.save(fname, tf2save_array_short)

"""
function to plot the transforms before frame conversion and after
"""
def plot_transforms(quad_pose_world_frame, cam_pose_world_frame):
    fig = plt.figure()
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')

    for i in range(0, len(quad_pose_world_frame),50):
        # QUAD ORIENTATION
        quad_orientation = quad_pose_world_frame[i][0:3,0:3]
        quad_position = quad_pose_world_frame[i][0:3,3]
        x_array_to_plot = np.hstack((quad_position.reshape((3,1)), quad_position.reshape((3,1))+quad_orientation[:,0].reshape((3,1))))
        y_array_to_plot = np.hstack((quad_position.reshape((3,1)), quad_position.reshape((3,1))+quad_orientation[:,1].reshape((3,1))))
        z_array_to_plot = np.hstack((quad_position.reshape((3,1)), quad_position.reshape((3,1))+quad_orientation[:,2].reshape((3,1))))
        ax1.plot(x_array_to_plot[0,:], x_array_to_plot[1,:], x_array_to_plot[2,:], '-r')
        ax1.plot(y_array_to_plot[0,:], y_array_to_plot[1,:], y_array_to_plot[2,:], '-g')
        ax1.plot(z_array_to_plot[0,:], z_array_to_plot[1,:], z_array_to_plot[2,:], '-b')
        ax1.set_xlabel('x')
        ax1.set_ylabel('y')
        ax1.set_zlabel('z')
        ax1.set_xlim(-1.5, 1.5)
        ax1.set_ylim(-1.5, 1.5)
        ax1.set_zlim(0.0, 3.0)
        ax1.set_title('Original FLU Frame')

        # CAMERA ORIENTATION
        cam_orientation = cam_pose_world_frame[i][0:3,0:3]
        cam_position = cam_pose_world_frame[i][0:3,3]
        x_array_to_plot = np.hstack((cam_position.reshape((3,1)), cam_position.reshape((3,1))+cam_orientation[:,0].reshape((3,1))))
        y_array_to_plot = np.hstack((cam_position.reshape((3,1)), cam_position.reshape((3,1))+cam_orientation[:,1].reshape((3,1))))
        z_array_to_plot = np.hstack((cam_position.reshape((3,1)), cam_position.reshape((3,1))+cam_orientation[:,2].reshape((3,1))))
        ax2.plot(x_array_to_plot[0,:], x_array_to_plot[1,:], x_array_to_plot[2,:], '-r')
        ax2.plot(y_array_to_plot[0,:], y_array_to_plot[1,:], y_array_to_plot[2,:], '-g')
        ax2.plot(z_array_to_plot[0,:], z_array_to_plot[1,:], z_array_to_plot[2,:], '-b')
        ax2.set_xlabel('x')
        ax2.set_ylabel('y')
        ax2.set_zlabel('z')
        ax2.set_xlim(-1.5, 1.5)
        ax2.set_ylim(-1.5, 1.5)
        ax2.set_zlim(0.0, 3.0)
        ax2.set_title('New Camera Frame')

    plt.show()

"""
function to convert from our data collection conventions to the nerf codebase conventions
inputs:
    relative = True or False. Determines if we load target poses to compute a relative pose.
    gen_npy = True or False. Determines if we save a poses_bounds.npy file to be used as input to the nerf codebase
    npy_fname = String. If gen_npy is True, then we will save the .npy file with this filename
    gen_plot = True or False. Determines if we plot the transforms
    drone_poses_fname = String. filename of csv file with drone / camera poses from data collection stage.
    target_poses_fname = String. filename of csv file with target poses from data collection stage. Only used if we are computing relative pose.
    idx_range = [min_idx, max_idx]. Range of indices of the csv file for which we want to generate a .npy file.
"""
def convert_drone_poses(relative = False, gen_npy = False, npy_fname = 'poses_bounds.npy', gen_plot = False, drone_poses_fname = 'drone_poses.csv', target_poses_fname = None, idx_range = None):
    # load csv file of drone poses
    drone_poses = np.genfromtxt(drone_poses_fname, delimiter=',')
    # load csv file of target poses if we are computing relative poses
    if relative == True:
        target_poses = np.genfromtxt(target_poses_fname, delimiter=',')

    # initialize lists for plotting
    drone_poses_list = []
    drone_poses_converted_list = []

    # camera intrinsics
    height = 480.
    width = 640.
    focal_length = 261.7647204842273
    intrinsics = np.array([height, width, focal_length])

    for ii in range(drone_poses.shape[0]):

        # drone pose to transformation matrix
        drone_row = drone_poses[ii,:]
        drone_translation, drone_quaternion = drone_row[0:3], drone_row[3:7]
        drone_pose = tf_from_vect(drone_quaternion, drone_translation)

        # target pose to transformation matrix
        if relative == True:
            target_row = target_poses[ii,:]
            target_translation, target_quaternion = target_row[0:3], target_row[3:7]
            target_pose = tf_from_vect(target_quaternion, target_translation)
            drone_pose = target_pose.T @ drone_pose

        # store relative or world pose in original frames
        drone_poses_list.append(drone_pose)

        # Down right back convention for nerf code base
        tf = rot_phi(np.pi/2) @ rot_psi(np.pi/2) @ drone_pose @ rot_phi(-np.pi/2) @ rot_theta(-np.pi/2) @ rot_psi(np.pi/2)

        # store converted pose in nerf code base frames
        drone_poses_converted_list.append(tf)

        # append the camera intrinsics
        tf2save = np.hstack((tf[0:3, :], intrinsics.reshape((3,1))))
        tf2save = tf2save.flatten()

        # append the depth bounds
        close_depth = 3.5
        far_depth = 8.5
        depth_bounds = np.array([close_depth, far_depth])
        tf2save = np.hstack((tf2save, depth_bounds))

        if ii == 0:
            tf2save_array = copy.deepcopy(tf2save)
        else:
            tf2save_array = np.vstack((tf2save_array, tf2save))

    if gen_npy == True:
        generate_npy(npy_fname, tf2save_array, segmented = False, idx_range = idx_range)

    if gen_plot == True:
        plot_transforms(drone_poses_list, drone_poses_converted_list)

if __name__=='__main__':
    convert_drone_poses(relative = True, gen_npy = True, npy_fname = 'poses_bounds.npy', gen_plot = True, drone_poses_fname = 'drone_poses.csv', target_poses_fname = 'target_poses.csv', idx_range = [0,5])
