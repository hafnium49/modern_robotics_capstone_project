#!/usr/bin/env python3
"""
Milestone 4 Complete Demonstration Script

This script runs all the main control scenarios to demonstrate the complete
implementation according to the Final Step instructions.
"""

import subprocess
import sys
import os
from pathlib import Path

def run_scenario(scenario_name, description):
    """Run a single scenario and report results."""
    print(f"\n{'='*60}")
    print(f"Running {description}")
    print(f"{'='*60}")
    
    try:
        # Run the scenario
        result = subprocess.run([
            sys.executable, "run_milestone4.py", scenario_name, 
            "--output", f"results/{scenario_name}"
        ], capture_output=True, text=True, cwd=Path(__file__).parent)
        
        if result.returncode == 0:
            print(f"✅ {description} completed successfully")
            
            # Run analysis
            analysis_result = subprocess.run([
                sys.executable, "run_milestone4.py", "--analyze", 
                "--output", f"results/{scenario_name}"
            ], capture_output=True, text=True, cwd=Path(__file__).parent)
            
            if analysis_result.returncode == 0:
                print("\n📊 Performance Analysis:")
                print(analysis_result.stdout)
            else:
                print(f"⚠️ Analysis failed: {analysis_result.stderr}")
                
        else:
            print(f"❌ {description} failed: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Error running {description}: {e}")
        return False
        
    return True

def main():
    """Run complete Milestone 4 demonstration."""
    print("🤖 Modern Robotics Capstone Project - Milestone 4 Demonstration")
    print("================================================================")
    print()
    print("This demonstration runs all control scenarios according to the")
    print("Final Step instructions, showing:")
    print("1. Feedforward-only control (Kp = Ki = 0)")
    print("2. Proportional control (Ki = 0)")
    print("3. Feedforward + PI control")
    print()
    
    # Define scenarios to run
    scenarios = [
        ("feedforward", "Feedforward-Only Control (Kp = Ki = 0)"),
        ("proportional", "Proportional Control (Ki = 0)"),
        ("feedforward_pi", "Feedforward + PI Control"),
    ]
    
    results = {}
    
    # Run each scenario
    for scenario, description in scenarios:
        success = run_scenario(scenario, description)
        results[scenario] = success
        
    # Summary
    print(f"\n{'='*60}")
    print("DEMONSTRATION SUMMARY")
    print(f"{'='*60}")
    
    for scenario, description in scenarios:
        status = "✅ SUCCESS" if results[scenario] else "❌ FAILED"
        print(f"{description:<40} {status}")
        
    print(f"\nResults saved in:")
    for scenario, _ in scenarios:
        if results[scenario]:
            print(f"  - results/{scenario}/")
            
    print(f"\nTo view results:")
    print(f"  python run_milestone4.py --compare")
    
    # Check if all succeeded
    all_success = all(results.values())
    if all_success:
        print(f"\n🎉 All scenarios completed successfully!")
        print(f"   Milestone 4 implementation is ready for submission.")
    else:
        print(f"\n⚠️ Some scenarios failed. Check error messages above.")
        
    return 0 if all_success else 1

if __name__ == "__main__":
    sys.exit(main())
