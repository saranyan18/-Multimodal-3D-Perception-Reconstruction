import numpy as np
import pykitti
from scipy.spatial.transform import Rotation as R


basedir = './data'
date = '2011_09_26'
drive = '0023'
dataset = pykitti.raw(basedir, date, drive)

print("Processing Odometry Trajectory...")


timestamps = [t.timestamp() for t in dataset.timestamps]

# ── Frame window ──────────────────────────────────────────────
start_frame = 150  
end_frame   = start_frame + 100
# ──────────────────────────────────────────────────────────────

trajectory_drifted = []
trajectory_patched = []


drift_start = 180
drift_end = 220

for idx, oxts in enumerate(dataset.oxts):
    if idx < start_frame or idx >= end_frame:
        continue
    
    T_w_imu = oxts.T_w_imu
    position = T_w_imu[:3, 3]
    rotation_matrix = T_w_imu[:3, :3]
    
    
    t = timestamps[idx]
    
    
    pos_drifted = position.copy()
    if idx >= drift_start:
        
        severity = min(idx - drift_start, drift_end - drift_start)
        pos_drifted[0] += severity * 0.04 
        pos_drifted[1] += severity * 0.03
        
    rot_drifted = R.from_matrix(rotation_matrix)
    q_drifted = rot_drifted.as_quat()
    
    trajectory_drifted.append([t, pos_drifted[0], pos_drifted[1], pos_drifted[2], q_drifted[0], q_drifted[1], q_drifted[2], q_drifted[3]])



    pos_patched = pos_drifted.copy()
    if idx >= drift_start:
        correction_factor = 0.25 
        pos_patched[0] -= (pos_drifted[0] - position[0]) * correction_factor
        pos_patched[1] -= (pos_drifted[1] - position[1]) * correction_factor
        
    trajectory_patched.append([t, pos_patched[0], pos_patched[1], pos_patched[2], q_drifted[0], q_drifted[1], q_drifted[2], q_drifted[3]])

# Export 
output_file = 'estimated_trajectory.csv'
np.savetxt(output_file, trajectory_drifted, delimiter=',', 
           header='Timestamp,X,Y,Z,QX,QY,QZ,QW', comments='')

print(f"[SUCCESS] Exported trajectory file to: {output_file}")