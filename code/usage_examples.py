#!/usr/bin/env python3
"""
Usage examples for Modern Robotics Capstone Project

Run this script to see example usage of the mobile manipulation software.
"""

import os
import sys

def main():
    print("Modern Robotics Capstone Project - Usage Examples")
    print("=" * 50)
    print()
    print("Basic usage:")
    print("  python run_milestone4.py best")
    print("  python run_milestone4.py overshoot") 
    print("  python run_milestone4.py newTask")
    print()
    print("Generate complete submission:")
    print("  python generate_submission.py")
    print()
    print("Individual control modes:")
    print("  python run_milestone4.py feedforward")
    print("  python run_milestone4.py proportional")
    print("  python run_milestone4.py feedforward_pi")
    print()
    print("Analysis:")
    print("  python run_milestone4.py --analyze best")
    print("  python run_milestone4.py --compare")
    print()
    print("Custom output directory:")
    print("  python run_milestone4.py best --output my_results")

if __name__ == "__main__":
    main()
