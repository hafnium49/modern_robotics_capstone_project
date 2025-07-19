#!/usr/bin/env python3
"""
Test Runner for Modern Robotics Capstone Project

This script provides a simple way to run tests without pytest conflicts.
"""

import sys
import os
import importlib
import subprocess
from pathlib import Path

def run_tests_with_pytest():
    """Run tests using pytest from the code directory."""
    print("Running tests with pytest...")
    
    # Change to code directory
    os.chdir('code')
    
    # Run pytest
    try:
        result = subprocess.run([
            sys.executable, '-m', 'pytest', 
            'tests/', '-v'
        ], capture_output=False)
        return result.returncode == 0
    except Exception as e:
        print(f"Error running pytest: {e}")
        return False

def run_individual_test_modules():
    """Run test modules individually without pytest."""
    print("Running individual test modules...")
    
    # Change to code directory and add to path
    os.chdir('code')
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

def main():
    """Main test runner."""
    print("Modern Robotics Capstone Project - Test Runner")
    print("=" * 50)
    
    if len(sys.argv) > 1 and sys.argv[1] == '--pytest':
        success = run_tests_with_pytest()
    else:
        success = run_individual_test_modules()
    
    if success:
        print("\n🎉 All tests passed!")
        return 0
    else:
        print("\n❌ Some tests failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())
