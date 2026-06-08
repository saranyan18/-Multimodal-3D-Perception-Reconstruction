import os
import numpy as np
import pykitti
import cv2

# --- 1. Initialization ---
basedir = './data'
date = '2011_09_26'
drive = '0023'

# Create output directory for the deliverables
output_dir = './task1_output'
os.makedirs(output_dir, exist_ok=True)

print(f"Loading KITTI dataset {date}_drive_{drive}...")
dataset = pykitti.raw(basedir, date, drive)

# Core spatial transforms
K_cam2 = dataset.calib.K_cam2             # 3x3 Intrinsic Matrix (Camera 2)
T_cam2_velo = dataset.calib.T_cam2_velo   # 4x4 Extrinsic Matrix (LiDAR to Camera 2)

# --- 2. Projection Logic ---
start_frame = 150          # ← change this to pick any window
num_frames_to_process = 10 # proof deliverable needs 10 frames

for frame_idx in range(start_frame, start_frame + num_frames_to_process):
    print(f"Processing frame {frame_idx}...")
    
    # Get raw data
    image = np.array(dataset.get_cam2(frame_idx)) # RGB image
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) # Convert to BGR for OpenCV saving
    img_height, img_width, _ = image.shape
    
    lidar_points = dataset.get_velo(frame_idx) # Nx4 Array [X, Y, Z, Reflectance]
    
    # Filter 1: Only keep LiDAR points in front of the sensor (X > 0 in velo frame)
    lidar_points = lidar_points[lidar_points[:, 0] > 0]
    
    # Convert to homogeneous coordinates (add a 1 to the end of each point: [X, Y, Z, 1])
    points_3d = lidar_points[:, :3]
    ones = np.ones((points_3d.shape[0], 1))
    points_3d_homo = np.hstack((points_3d, ones))
    
    # Transform from LiDAR frame to Camera 2 frame
    # P_cam2 = T_cam2_velo * P_velo
    points_cam2 = T_cam2_velo.dot(points_3d_homo.T) # Result is 4xN
    
    # In the camera frame, Z is the forward distance (depth)
    depths = points_cam2[2, :]
    
    # Filter 2: Only keep points that are physically in front of the camera (Z > 0)
    valid_depth_mask = depths > 0
    points_cam2 = points_cam2[:, valid_depth_mask]
    depths = depths[valid_depth_mask]
    
    # Project 3D camera coordinates to 2D image pixels
    # s * [u, v, 1]^T = K_cam2 * P_cam2(1:3)
    points_2d = K_cam2.dot(points_cam2[:3, :])
    
    # Normalize by the Z-coordinate (the scalar 's') to get final pixel coordinates
    u = points_2d[0, :] / points_2d[2, :]
    v = points_2d[1, :] / points_2d[2, :]
    
    # Filter 3: Drop points that fall outside the image boundaries
    valid_img_mask = (u >= 0) & (u < img_width) & (v >= 0) & (v < img_height)
    u = np.round(u[valid_img_mask]).astype(int)  # FIX: round before int-cast for sub-pixel accuracy
    v = np.round(v[valid_img_mask]).astype(int)
    depths = depths[valid_img_mask]
    
    # --- 3. Visualization & Color-Coding ---
    # Normalize depths to a 0-255 scale for coloring (assuming max useful depth is ~80 meters)
    max_depth = 80.0 
    depths_normalized = np.clip(depths / max_depth, 0, 1)
    
    # Create a color map (Red = close, Blue = far, to match their prompt request)
    # We use HSV color space where Hue ranges from 0 (Red) to 120 (Blue-ish in OpenCV's 0-180 scale)
    hues = (120 * depths_normalized).astype(np.uint8)
    saturations = np.full_like(hues, 255)
    values = np.full_like(hues, 255)
    
    hsv_colors = np.stack((hues, saturations, values), axis=-1)
    # Convert the HSV colors to BGR so we can plot them on the image
    hsv_colors_reshaped = hsv_colors.reshape(-1, 1, 3)
    bgr_colors = cv2.cvtColor(hsv_colors_reshaped, cv2.COLOR_HSV2BGR).reshape(-1, 3)
    
    # Draw the points on the image
    for i in range(len(u)):
        color = (int(bgr_colors[i][0]), int(bgr_colors[i][1]), int(bgr_colors[i][2]))
        # Draw small circles (radius 2) for the point cloud
        cv2.circle(image, (u[i], v[i]), 2, color, -1)
        
    # --- 4. Export ---
    output_filename = os.path.join(output_dir, f'projection_frame_{frame_idx - start_frame:04d}.png')
    cv2.imwrite(output_filename, image, [cv2.IMWRITE_PNG_COMPRESSION, 0])  # FIX: lossless export

print(f"\nSuccess! 10 frames saved to {output_dir}.")
print("Review the images to ensure the laser points match the physical boundaries perfectly.")