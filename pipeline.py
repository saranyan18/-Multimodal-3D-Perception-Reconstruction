import os
import subprocess
import sys

def run_step(script_name):
    print("\n" + "="*50)
    print(f"LAUNCHING PIPELINE STEP: {script_name}")
    print("="*50 + "\n")
    
   
    result = subprocess.run([sys.executable, script_name])
    
    if result.returncode != 0:
        print(f"\n[ERROR] {script_name} failed with exit code {result.returncode}. Pipeline halted.")
        sys.exit(result.returncode)
    else:
        print(f"\n[SUCCESS] Completed {script_name}.")

def main():
    print("=================================================================")
    print("Starting Idle Robotics Multimodal Perception & Reconstruction Pipeline")
    print("=================================================================")
    

    if os.path.exists("task1.py"):
        run_step("task1.py")
    else:
        print("[ERROR] Missing task1.py file.")
        sys.exit(1)
        

    if os.path.exists("task2.py"):
        run_step("task2.py")
    else:
        print("[ERROR] Missing task2.py file.")
        sys.exit(1)


    if os.path.exists("eval_task2.py"):
        run_step("eval_task2.py")
    else:
        print("[WARNING] eval_task2.py missing. Skipping official evo metrics.")
        
 
    if os.path.exists("task3.py"):
        run_step("task3.py")
    else:
        print("[ERROR] Missing task3.py file.")
        sys.exit(1)
        
    print("\n" + "="*65)
    print("ALL CORE DATA PIPELINE PROCESSING STAGES EXECUTED SUCCESSFULLY!")
    print(" -> Task 1 Proofs:  './task1_output' folder")
    print(" -> Task 2 Metrics:  'task2_evo_report.txt' file")
    print(" -> Task 3 Dataset Sandbox: Ready at './splat_data'")
    print("\nRun 'ns-train splatfacto --data ./splat_data' to train.")
    print("="*65 + "\n")

if __name__ == "__main__":
    main()
