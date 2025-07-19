#!/usr/bin/env python3
"""
Comprehensive Test Runner for Modern Robotics Capstone Project

This script provides multiple ways to run tests without pytest conflicts.

Usage examples:
  python code/test.py                                      # Run all tests with pytest
  python code/test.py --individual                         # Run tests individually without pytest
  python code/test.py tests/test_milestone1.py             # Run specific test file
  python code/test.py tests/test_milestone4.py::TestMilestone4Setup::test_default_cube_poses
"""

import sys
import os
import importlib
import subprocess
from pathlib import Path

def run_tests_with_pytest(test_args=None):
    """Run tests using pytest with proper module isolation."""
    print("Running tests with pytest...")
    
    # Run pytest from project root to avoid 'code' module conflicts
    try:
        # Get project root (parent directory of code)
        project_root = Path(__file__).parent.parent.absolute()
        
        # Set PYTHONPATH to ensure proper imports
        env = os.environ.copy()
        env['PYTHONPATH'] = str(project_root)
        
        # Base pytest command with conflict-avoiding options
        cmd = [
            sys.executable, '-m', 'pytest', 
            '-v',
            '--tb=short',  # Shorter traceback to avoid pdb issues
            '-p', 'no:pdb',  # Disable pdb plugin
            '-p', 'no:debugging',  # Disable debugging plugin
            '--disable-warnings'  # Disable warnings for cleaner output
        ]
        
        # Add specific test arguments or default to all tests
        if test_args:
            # Convert relative paths to work from project root
            converted_args = []
            for arg in test_args:
                if arg.startswith('tests/') or arg.startswith('tests\\'):
                    # Convert tests/ to code/tests/
                    converted_args.append(arg.replace('tests/', 'code/tests/').replace('tests\\', 'code\\tests\\'))
                else:
                    converted_args.append(arg)
            cmd.extend(converted_args)
        else:
            cmd.append('code/tests/')
        
        # Run from project root
        result = subprocess.run(cmd, env=env, capture_output=False, cwd=str(project_root))
        return result.returncode == 0
    except Exception as e:
        print(f"Error running pytest: {e}")
        return False

def run_individual_test_modules():
    """Run test modules individually without pytest."""
    print("Running individual test modules...")
    
    # Store original directory and path
    original_cwd = os.getcwd()
    original_path = sys.path[:]
    
    try:
        # We're already in code directory, just add to path
        sys.path.insert(0, '.')
        
        test_modules = [
            'tests.test_milestone1',
            'tests.test_milestone2', 
            'tests.test_milestone3',
            'tests.test_milestone4'
        ]
        
        results = {}
        
        for module_name in test_modules:
            print(f"\n--- Testing {module_name} ---")
            try:
                module = importlib.import_module(module_name)
                
                # Run test functions
                test_functions = [name for name in dir(module) if name.startswith('test_')]
                passed = 0
                failed = 0
                
                for test_func_name in test_functions:
                    try:
                        test_func = getattr(module, test_func_name)
                        test_func()
                        print(f"  ✓ {test_func_name}")
                        passed += 1
                    except Exception as e:
                        print(f"  ✗ {test_func_name}: {e}")
                        failed += 1
                
                results[module_name] = {'passed': passed, 'failed': failed}
                
            except ImportError as e:
                print(f"  Error importing {module_name}: {e}")
                results[module_name] = {'passed': 0, 'failed': 1}
        
        # Summary
        print("\n" + "="*50)
        print("TEST SUMMARY")
        print("="*50)
        
        total_passed = 0
        total_failed = 0
        
        for module, result in results.items():
            passed = result['passed'] 
            failed = result['failed']
            total_passed += passed
            total_failed += failed
            status = "✓" if failed == 0 else "✗"
            print(f"{status} {module}: {passed} passed, {failed} failed")
        
        print(f"\nOverall: {total_passed} passed, {total_failed} failed")
        return total_failed == 0
        
    finally:
        # Restore original directory and path
        os.chdir(original_cwd)
        sys.path[:] = original_path

def main():
    """Main test runner."""
    print("Modern Robotics Capstone Project - Test Runner")
    print("=" * 50)
    
    # Parse arguments
    if '--individual' in sys.argv:
        # Remove --individual from args and run individual tests
        success = run_individual_test_modules()
    elif len(sys.argv) > 1:
        # Run pytest with specific arguments
        test_args = [arg for arg in sys.argv[1:] if arg != '--individual']
        success = run_tests_with_pytest(test_args)
    else:
        # Default: run all tests with pytest
        success = run_tests_with_pytest()
    
    if success:
        print("\n🎉 All tests passed!")
        return 0
    else:
        print("\n❌ Some tests failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())
