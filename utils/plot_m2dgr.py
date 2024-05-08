#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from const import R2D
from coorTran import BLH2ENU, BLH2NED, XYZ2BLH
from resCompare import calPoseErr, calmetrics
from scipy.spatial.transform import Rotation
import datetime

def plot_lioekf(odo: np.ndarray, seq: str):
    fig, ax = plt.subplots()
    ax.plot(odo[:, 2], odo[:, 1])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.axis('equal')
    ax.legend(seq)

    odo_t = odo[:, 0] - odo[0, 0]
    fig, axs = plt.subplots(3, 1,figsize=(12, 5))
    axs[0].plot(odo_t, odo[:, 7])
    axs[0].set_ylabel('Roll (deg)')
    axs[0].legend(seq)
    axs[1].plot(odo_t, odo[:, 8])
    axs[1].set_ylabel('Pitch (deg)')
    axs[2].plot(odo_t, odo[:, 9])
    axs[2].set_ylabel('Heading (deg)')
    plt.show()

# Load data from text file
# dataset
dataset = "m2dgr"
seqs = ["street_01", "street_02", "street_03", "street_04", "street_05", "street_06", "street_08"] 
#"street_01", "street_02", "street_03", "street_04", "street_05", "street_06", "street_07", "street_08", "street_09", "street_10"

lioekf_path = "/home/yibin/code/kiss-lio-ros/src/lio-ekf/output/"

gt_dir = "/mnt/data/" + dataset + "/"


for seq in seqs:
    path = lioekf_path + dataset + "/" + seq + "/"
    gt_path = gt_dir
    odo = np.loadtxt(path + "odo.txt")
    
    gt = np.loadtxt(gt_path + seq + ".txt")
    gt_timestamp = gt[:,0]
    XYZ = gt[:,1:4]
    BLH = XYZ2BLH(XYZ)
    gt_euler = np.zeros_like(BLH)

    for i in range(XYZ.shape[0]):
        tmp_q = gt[i,4:8]
        gt_euler[i, :] = Rotation.from_quat(tmp_q).as_euler('xyz', degrees=True)



    # plot_lioekf(odo, seq)

    gt_local = np.concatenate((gt_timestamp[:, np.newaxis], BLH2ENU(BLH[0, 0:3], BLH[:, 0:3])), axis=1)

    odo_pose = np.hstack((odo[:, 0:4], odo[:, 7:10]))
    gt_pose = np.hstack((gt_local, gt_euler))

    
    rot_alioimu2refimu = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])


    # metric: [avg_tra, avg_rot, ate_rot, ate_trans]

    alio_pose_err, alio_pose_metric,alio_traj, alio_gt = calPoseErr(odo_pose, gt_pose, rot_alioimu2refimu, "LIO-EKF")
    
    
    if(seq == "street_05"):
        alio_gt[:, 1] = -alio_gt[:, 1]
        alio_gt[:, 2] = alio_gt[:, 2]

    fig, ax = plt.subplots()
    ax.plot(alio_traj[:, 2], alio_traj[:, 1])
    ax.plot(alio_gt[:, 2], alio_gt[:, 1])
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.axis('equal')
    ax.legend(['LIO-EKF', 'Ground Truth'])
    # plt.savefig(path + 'trajectories.png', dpi=300)
    

    # horizontal position error
    alio_hor_err = np.sqrt(np.multiply(alio_pose_err[:,1],alio_pose_err[:,1]) + np.multiply(alio_pose_err[:,2],alio_pose_err[:,2]))


    # plot position error
    fig, axs = plt.subplots(2, 1 ,figsize=(12, 5))

    axs[0].plot(alio_pose_err[:,0]-alio_pose_err[0,0], alio_hor_err)
    axs[0].set_xlabel('Time(s)')
    axs[0].set_ylabel('Horizon Pos. Error(m)')
    axs[0].legend(["LIO-EKF"])

    axs[1].plot(alio_pose_err[:,0]-alio_pose_err[0,0], alio_pose_err[:,3])
    axs[1].set_xlabel('Time(s)')
    axs[1].set_ylabel('Vertical Pos. Error(m)')
    #plt.savefig(path + 'pos_err.png', dpi=300)
    
    # plot attitude error
    fig, axs = plt.subplots(3, 1,figsize=(12, 5))

    axs[0].plot(alio_pose_err[:,0]-alio_pose_err[0,0], alio_pose_err[:,4])
    axs[0].set_xlabel('Time(s)')
    axs[0].set_ylabel('Roll Error(deg)')
    axs[0].legend(["LIO-EKF"])

    axs[1].plot(alio_pose_err[:,0]-alio_pose_err[0,0], alio_pose_err[:,5])
    axs[1].set_xlabel('Time(s)')
    axs[1].set_ylabel('Pitch Error(deg)')

    axs[2].plot(alio_pose_err[:,0]-alio_pose_err[0,0], alio_pose_err[:,6])
    axs[2].set_xlabel('Time(s)')
    axs[2].set_ylabel('Heading Error(deg)')
    #plt.savefig(path + 'att_err.png', dpi=300)
    
    plt.show()
    
    now = datetime.datetime.now()
    time_str = now.strftime("%Y-%m-%d %H:%M:%S")
    # Print evaluation metrics
    with open(path + 'evaluation.txt', 'a') as f:
        f.write("\n"+time_str+"\n")
        f.write("*********Evaluation metrics**********\n") 
        f.write("Dataset: "+dataset+"   Seq: " +seq +"\n")
        f.write("          [avg_rot  avg_trans  ate_rot  ate_trans]\n")
        f.write('LIO-EKF:      {:5.2f}     {:5.2f}    {:5.2f}      {:5.2f}\n'.format(alio_pose_metric[1],alio_pose_metric[0],alio_pose_metric[2]*R2D,alio_pose_metric[3]))
    with open(path + 'evaluation.txt', 'r') as f:
        print(f.read())