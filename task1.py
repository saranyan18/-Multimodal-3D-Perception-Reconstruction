import os
import numpy as np
import pykitti
import cv2

# --- 1. Initialization ---
basedir = './data'
date = '2011_09_26'
drive = '0023'


output_dir = './task1_output'
os.makedirs(output_dir, exist_ok=True)

print(f"Loading KITTI dataset {date}_drive_{drive}...")
dataset = pykitti.raw(basedir, date, drive)

# Core spatial transforms
K_cam2 = dataset.calib.K_cam2             
T_cam2_velo = dataset.calib.T_cam2_velo   
# --- 2. Projection Logic ---
start_frame = 150          
num_frames_to_process = 10 

for frame_idx in range(start_frame, start_frame + num_frames_to_process):
    print(f"Processing frame {frame_idx}...")
    
    # Get raw data
    image = np.array(dataset.get_cam2(frame_idx)) 
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) 
    img_height, img_width, _ = image.shape
    
    lidar_points = dataset.get_velo(frame_idx)
    
  
    lidar_points = lidar_points[lidar_points[:, 0] > 0]
    
  
    points_3d = lidar_points[:, :3]
    ones = np.ones((points_3d.shape[0], 1))
    points_3d_homo = np.hstack((points_3d, ones))
    
   
    points_cam2 = T_cam2_velo.dot(points_3d_homo.T) 
    
   
    depths = points_cam2[2, :]
    
    
    valid_depth_mask = depths > 0
    points_cam2 = points_cam2[:, valid_depth_mask]
    depths = depths[valid_depth_mask]
    
    
    points_2d = K_cam2.dot(points_cam2[:3, :])
    
    
    u = points_2d[0, :] / points_2d[2, :]
    v = points_2d[1, :] / points_2d[2, :]
    
    
    valid_img_mask = (u >= 0) & (u < img_width) & (v >= 0) & (v < img_height)
    u = np.round(u[valid_img_mask]).astype(int)  
    v = np.round(v[valid_img_mask]).astype(int)
    depths = depths[valid_img_mask]
    
    
    max_depth = 80.0 
    depths_normalized = np.clip(depths / max_depth, 0, 1)
    
    hues = (120 * depths_normalized).astype(np.uint8)
    saturations = np.full_like(hues, 255)
    values = np.full_like(hues, 255)
    
    hsv_colors = np.stack((hues, saturations, values), axis=-1)
    
    hsv_colors_reshaped = hsv_colors.reshape(-1, 1, 3)
    bgr_colors = cv2.cvtColor(hsv_colors_reshaped, cv2.COLOR_HSV2BGR).reshape(-1, 3)
    
    
    for i in range(len(u)):
        color = (int(bgr_colors[i][0]), int(bgr_colors[i][1]), int(bgr_colors[i][2]))
        cv2.circle(image, (u[i], v[i]), 2, color, -1)
        
 #Export
    output_filename = os.path.join(output_dir, f'projection_frame_{frame_idx - start_frame:04d}.png')
    cv2.imwrite(output_filename, image, [cv2.IMWRITE_PNG_COMPRESSION, 0])  # FIX: lossless export

print(f"\nSuccess! 10 frames saved to {output_dir}.")
print("Review the images to ensure the laser points match the physical boundaries perfectly.")