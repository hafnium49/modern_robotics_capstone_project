#!/usr/bin/env python3
"""
Isolated Test Launcher for Modern Robotics Capstone Project

This script runs pytest in a subprocess with proper module isolation
to avoid conflicts with the 'code' directory name.
"""

import sys
import os
import subprocess
from pathlib import Path
import tempfile
import shutil

def run_pytest_isolated(*test_args):
    """Run pytest in an isolated subprocess to avoid module conflicts."""
    
    # Create a temporary directory for the isolated run
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        project_root = Path.cwd().absolute()
        
        # Create the launcher script that handles the import conflict
        launcher_script = f'''
import sys
import os
from pathlib import Path
import importlib.util

# Remove current directory from path to avoid conflicts
original_path = sys.path[:]
sys.path = [p for p in sys.path if not p.endswith("code")]

# Add project root to path
project_root = Path(r"{project_root}")
sys.path.insert(0, str(project_root))

# Temporarily remove the code directory from sys.modules
modules_to_restore = {{}}
for module_name in list(sys.modules.keys()):
    if module_name.startswith("code"):
        modules_to_restore[module_name] = sys.modules[module_name]
        del sys.modules[module_name]

try:
    # Import pytest after cleaning up the module conflicts
    import pytest
    
    # Restore the code modules
    sys.modules.update(modules_to_restore)
    
    # Run pytest with the provided arguments
    if __name__ == "__main__":
        sys.exit(pytest.main(sys.argv[1:]))
        
except ImportError as e:
    print(f"Error importing pytest: {{e}}")
    sys.exit(1)
'''
        
        # Write the launcher script
        launcher_path = temp_path / "isolated_pytest.py"
        with open(launcher_path, 'w') as f:
            f.write(launcher_script)
        
        # Run pytest through the launcher
        cmd = [sys.executable, str(launcher_path)] + list(test_args)
        
        # Set environment
        env = os.environ.copy()
        env['PYTHONPATH'] = str(project_root)
        # Disable pdb in pytest to avoid the code module conflict
        env['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] = '1'
        
        result = subprocess.run(cmd, env=env, cwd=str(project_root))
        return result.returncode

def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: python pytest_runner.py <pytest_args>")
        print("Example: python pytest_runner.py code/tests/test_milestone4.py::test_basic_capstone_integration -v")
        return 1
    
    # Pass all arguments to pytest
    return run_pytest_isolated(*sys.argv[1:])

if __name__ == "__main__":
    sys.exit(main())
