#!/usr/bin/env python3
"""
Quick pytest wrapper that avoids 'code' module conflicts.

Usage examples:
  python test.py code/tests/test_milestone4.py::TestMilestone4Setup::test_default_cube_poses
  python test.py code/tests/test_milestone1.py
  python test.py  # Run all tests
"""

import sys
import subprocess

def main():
    """Run pytest with conflict-avoiding options."""
    
    # Base pytest command with conflict-avoiding options
    cmd = [
        sys.executable, '-m', 'pytest',
        '-v',
        '--tb=short', 
        '-p', 'no:pdb',
        '-p', 'no:debugging',
        '--disable-warnings'
    ]
    
    # Add user arguments or default to all tests
    if len(sys.argv) > 1:
        cmd.extend(sys.argv[1:])
    else:
        cmd.append('code/tests/')
    
    # Run pytest
    result = subprocess.run(cmd)
    return result.returncode

if __name__ == "__main__":
    sys.exit(main())
