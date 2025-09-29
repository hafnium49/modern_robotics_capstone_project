#!/usr/bin/env python3
"""
Script to add hide-input tags to all code cells in a Jupyter notebook
for cleaner slideshow presentation.
"""

import json
import sys
import os

def add_hide_input_tags(notebook_path):
    """Add hide-input tags to all code cells in the notebook."""
    
    # Read the notebook
    with open(notebook_path, 'r', encoding='utf-8') as f:
        notebook = json.load(f)
    
    # Counter for modified cells
    modified_count = 0
    
    # Process each cell
    for cell in notebook['cells']:
        if cell['cell_type'] == 'code':
            # Initialize metadata if it doesn't exist
            if 'metadata' not in cell:
                cell['metadata'] = {}
            
            # Add tags if not present
            if 'tags' not in cell['metadata']:
                cell['metadata']['tags'] = []
            
            # Add hide-input tag if not already present
            if 'hide-input' not in cell['metadata']['tags']:
                cell['metadata']['tags'].append('hide-input')
                modified_count += 1
                print(f"Added hide-input tag to code cell")
    
    # Create backup
    backup_path = notebook_path + '.backup'
    if not os.path.exists(backup_path):
        with open(backup_path, 'w', encoding='utf-8') as f:
            json.dump(notebook, f, ensure_ascii=False, indent=2)
        print(f"Backup created: {backup_path}")
    
    # Write the modified notebook
    with open(notebook_path, 'w', encoding='utf-8') as f:
        json.dump(notebook, f, ensure_ascii=False, indent=2)
    
    print(f"Modified {modified_count} code cells with hide-input tags")
    return modified_count

if __name__ == "__main__":
    notebook_path = "Modern_Robotics_Ultimate_Presentation_JP.ipynb"
    
    if not os.path.exists(notebook_path):
        print(f"Error: Notebook file {notebook_path} not found!")
        sys.exit(1)
    
    try:
        count = add_hide_input_tags(notebook_path)
        print(f"Successfully processed notebook: {count} cells modified")
    except Exception as e:
        print(f"Error processing notebook: {e}")
        sys.exit(1)