#!/usr/bin/env python3
"""
Submission Package Verification Script

This script verifies that the submission package contains all required files
according to the capstone project requirements.
"""

import os
from pathlib import Path


def verify_submission_package():
    """Verify the submission package is complete."""
    print("🔍 Verifying Submission Package")
    print("=" * 50)
    
    # Check main files
    required_main_files = [
        "README.txt",
        "code",
        "results"
    ]
    
    print("Main Files:")
    for file in required_main_files:
        exists = os.path.exists(file)
        status = "✅" if exists else "❌"
        print(f"  {status} {file}")
    
    print()
    
    # Check code directory
    required_code_files = [
        "code/__init__.py",
        "code/next_state.py", 
        "code/trajectory_generator.py",
        "code/feedback_control.py",
        "code/run_capstone.py",
        "code/scenarios.py",
        "code/run_milestone4.py",
        "code/usage_examples.py",
        "code/requirements.txt"
    ]
    
    print("Code Directory:")
    for file in required_code_files:
        exists = os.path.exists(file)
        status = "✅" if exists else "❌"
        print(f"  {status} {file}")
    
    print()
    
    # Check results directories
    results_dirs = ["results/best", "results/overshoot", "results/newTask"]
    required_result_files = [
        "README.txt",
        "youBot_output.csv", 
        "Xerr_log.csv",
        "Xerr_plot.pdf",
        "program_log.txt"
    ]
    
    print("Results Directories:")
    all_good = True
    
    for result_dir in results_dirs:
        print(f"\n  📁 {result_dir}:")
        dir_exists = os.path.exists(result_dir)
        if not dir_exists:
            print(f"    ❌ Directory missing!")
            all_good = False
            continue
            
        for file in required_result_files:
            file_path = os.path.join(result_dir, file)
            exists = os.path.exists(file_path)
            status = "✅" if exists else "❌"
            print(f"    {status} {file}")
            if not exists:
                all_good = False
    
    print("\n" + "=" * 50)
    
    if all_good:
        print("🎉 SUBMISSION PACKAGE VERIFICATION: PASSED")
        print("All required files and directories are present!")
        
        # Check file sizes
        print("\nFile Sizes:")
        for result_dir in results_dirs:
            csv_path = os.path.join(result_dir, "youBot_output.csv")
            if os.path.exists(csv_path):
                size_kb = os.path.getsize(csv_path) / 1024
                print(f"  {result_dir}/youBot_output.csv: {size_kb:.1f} KB")
        
        print("\nPackage ready for submission!")
        print("To create zip file: Select all files → Right-click → 'Send to Compressed folder'")
        
    else:
        print("❌ SUBMISSION PACKAGE VERIFICATION: FAILED")
        print("Some required files are missing. Please run the submission generator again.")
    
    return all_good


def main():
    """Main verification function."""
    return verify_submission_package()


if __name__ == "__main__":
    main()
