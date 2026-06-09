import os
import subprocess
import sys

def main():
    print("=================================================================")
    print("TASK 2: OFFICIAL 'EVO' TRAJECTORY EVALUATION")
    print("=================================================================\n")
    
    gt_file = 'ground_truth.tum'
    est_file = 'estimated_trajectory.tum'
    report_file = 'task2_evo_report.txt'
    

    if not os.path.exists(gt_file) or not os.path.exists(est_file):
        print(f"[CRITICAL] Missing TUM files. Please run 'python task2.py' first.")
        sys.exit(1)
        
    print(f"Executing evo_ape (Absolute Pose Error) alignment protocol...")
    
   
    try:

        result = subprocess.run(
            ["evo_ape", "tum", gt_file, est_file, "--align"],
            capture_output=True,
            text=True,
            check=True
        )
        

        with open(report_file, "w") as f:
            f.write("=== IDLE ROBOTICS: TASK 2 METRICS EVALUATION ===\n")
            f.write("Tool: evo (Python Trajectory Evaluation Pack)\n")
            f.write("Metric: Absolute Pose Error (APE) with rigid alignment\n\n")
            f.write(result.stdout)
            
        print("\n" + result.stdout)
        print(f"[SUCCESS] Official metrics parsed and saved to: {report_file}")
        
    except FileNotFoundError:
        print("[ERROR] The 'evo' toolkit is not installed on your system.")
    except subprocess.CalledProcessError as e:
        print(" [ERROR] The evo evaluation failed.")
        print(e.stderr)

if __name__ == "__main__":
    main()
