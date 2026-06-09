import os
import json
import numpy as np
import pykitti

def main():
    basedir = './data'
    date = '2011_09_26'
    drive = '0023'
    
    print("Loading KITTI dataset for Task 3 Multimodal Fusion...")
    if not os.path.exists(basedir):
        print(f"[ERROR] Data directory {basedir} does not exist.")
        return

    dataset = pykitti.raw(basedir, date, drive)

    output_dir = './splat_data'
    images_dir = os.path.join(output_dir, "images")
    os.makedirs(images_dir, exist_ok=True)
    

    K = dataset.calib.K_cam2
    T_cam2_velo = dataset.calib.T_cam2_velo
    T_velo_imu = dataset.calib.T_velo_imu


    T_cam2_imu = T_cam2_velo.dot(T_velo_imu)
    T_imu_cam2 = np.linalg.inv(T_cam2_imu)


    R_cv_to_gl = np.array([
        [1,  0,  0,  0],
        [0, -1,  0,  0],
        [0,  0, -1,  0],
        [0,  0,  0,  1]
    ])


    meta = {
        "fl_x": float(K[0, 0]),
        "fl_y": float(K[1, 1]),
        "cx": float(K[0, 2]),
        "cy": float(K[1, 2]),
        "w": 1242, 
        "h": 375,  
        "camera_model": "OPENCV",
        "ply_file_path": "sparse_pc.ply",
        "frames": []
    }

    # ── Frame window ──────────────────────────────────────────────
    start_frame = 150   
    num_frames  = 100
    end_frame   = min(start_frame + num_frames, len(dataset.oxts))
    # ──────────────────────────────────────────────────────────────
    print(f"Processing frames {start_frame}–{end_frame - 1} ({end_frame - start_frame} frames)...")

    world_points = []

    for idx in range(start_frame, end_frame):
        try:
           
            T_w_imu = dataset.oxts[idx].T_w_imu
            T_w_cam2_cv = T_w_imu.dot(T_imu_cam2)
            T_w_cam2_gl = T_w_cam2_cv.dot(R_cv_to_gl)
            
            relative_idx = idx - start_frame
            relative_img_path = f"images/frame_{relative_idx:04d}.png"
            meta["frames"].append({
                "file_path": relative_img_path,
                "transform_matrix": T_w_cam2_gl.tolist()
            })

           
            img = dataset.get_cam2(idx)
            if img is not None:
                img_path = os.path.join(images_dir, f"frame_{relative_idx:04d}.png")
                img.save(img_path, format="PNG")  
                if idx == 0:
                    meta["w"] = img.width
                    meta["h"] = img.height
                
          
            raw_velo = dataset.get_velo(idx)
            if raw_velo is not None:
            
                raw_velo = raw_velo[raw_velo[:, 0] > 2]
                raw_velo = raw_velo[raw_velo[:, 0] < 80]
                
             
                raw_velo = raw_velo[::5]
                
              
                pts_3d = np.hstack((raw_velo[:, :3], np.ones((raw_velo.shape[0], 1))))
                
             
                pts_cam2 = T_cam2_velo.dot(pts_3d.T)
                pts_w_cv = T_w_cam2_cv.dot(pts_cam2).T
                
                for p in pts_w_cv:
                    world_points.append(p[:3])
                    
        except Exception as e:
            print(f"[ERROR] Failed packing frame {idx}: {e}")

  
    ply_out_path = os.path.join(output_dir, "sparse_pc.ply")
    print(f"Writing {len(world_points)} real-world LiDAR points to {ply_out_path}...")
    
    with open(ply_out_path, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(world_points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for pt in world_points:
            f.write(f"{pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f}\n")

  
    json_out_path = os.path.join(output_dir, "transforms.json")
    with open(json_out_path, "w") as f:
        json.dump(meta, f, indent=4)
        
    print(f"[SUCCESS] Task 3 sandbox built with LiDAR point clouds initialized")

if __name__ == "__main__":
    main()
