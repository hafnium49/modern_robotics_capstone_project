#!/usr/bin/env python3
"""
Command-line driver for Modern Robotics Capstone Project

Usage:
    python run_milestone4.py [scenario] [options]

Scenarios:
    feedforward    - Feedforward-only control (Kp = Ki = 0)
    proportional   - P-control only (Ki = 0)
    pi_only        - PI-control only 
    feedforward_pi - Feedforward + PI control
    best           - Well-tuned gains for best performance (default)
    overshoot      - High gains that cause overshoot
    newTask        - Custom cube poses (new task)
    gain_study     - Run multiple gain combinations
    
Options:
    --output DIR    - Output directory (default: results/{scenario})
    --analyze       - Analyze existing results instead of running simulation
    --compare       - Compare all scenario results
"""

import sys
import argparse
import os
from pathlib import Path

# Add the project root to the path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from modern_robotics_sim.run_capstone import main as run_main
from modern_robotics_sim.scenarios import (
    run_feedforward_only_scenario, run_proportional_only_scenario, 
    run_pi_only_scenario, run_feedforward_plus_pi_scenario,
    run_best_scenario, run_overshoot_scenario, run_newTask_scenario,
    analyze_simulation_results, compare_scenarios, create_gain_study
)


def main():
    """Main command-line interface."""
    parser = argparse.ArgumentParser(
        description="Modern Robotics Capstone Project - Milestone 4",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python run_milestone4.py feedforward             # Run feedforward-only
    python run_milestone4.py proportional            # Run P-control
    python run_milestone4.py feedforward_pi          # Run feedforward + PI
    python run_milestone4.py best                    # Run best scenario
    python run_milestone4.py overshoot --output ./out # Run overshoot to ./out
    python run_milestone4.py --analyze best          # Analyze best results
    python run_milestone4.py --compare               # Compare all scenarios
    python run_milestone4.py gain_study              # Run gain tuning study
        """
    )
    
    parser.add_argument(
        'scenario', 
        nargs='?',
        default='best',
        choices=['feedforward', 'proportional', 'pi_only', 'feedforward_pi', 'best', 'overshoot', 'newTask', 'gain_study'],
        help='Scenario to run (default: best)'
    )
    
    parser.add_argument(
        '--output', '-o',
        help='Output directory (default: results/{scenario})'
    )
    
    parser.add_argument(
        '--analyze', '-a',
        action='store_true',
        help='Analyze existing results instead of running simulation'
    )
    
    parser.add_argument(
        '--compare', '-c',
        action='store_true', 
        help='Compare results from all scenarios'
    )
    
    args = parser.parse_args()
    
    # Handle comparison mode
    if args.compare:
        compare_scenarios()
        return
    
    # Handle analysis mode
    if args.analyze:
        if args.output:
            result_dir = args.output
        else:
            result_dir = f"results/{args.scenario}"
            
        if not os.path.exists(result_dir):
            print(f"Error: Result directory '{result_dir}' not found")
            return 1
            
        analyze_simulation_results(result_dir)
        return
    
    # Set output directory
    if args.output:
        output_dir = args.output
    else:
        output_dir = f"results/{args.scenario}"
    
    # Run the appropriate scenario
    print(f"Running scenario: {args.scenario}")
    print(f"Output directory: {output_dir}")
    print()
    
    try:
        if args.scenario == 'feedforward':
            success = run_feedforward_only_scenario(output_dir)
        elif args.scenario == 'proportional':
            success = run_proportional_only_scenario(2.0, output_dir)
        elif args.scenario == 'pi_only':
            success = run_pi_only_scenario(0.1, output_dir)
        elif args.scenario == 'feedforward_pi':
            success = run_feedforward_plus_pi_scenario(output_dir)
        elif args.scenario == 'best':
            success = run_best_scenario(output_dir)
        elif args.scenario == 'overshoot':
            success = run_overshoot_scenario(output_dir)
        elif args.scenario == 'newTask':
            success = run_newTask_scenario(output_dir)
        elif args.scenario == 'gain_study':
            results = create_gain_study(output_dir)
            success = len(results) > 0
        else:
            print(f"Unknown scenario: {args.scenario}")
            return 1
            
        if success:
            print(f"\n[SUCCESS] Scenario '{args.scenario}' completed successfully!")
            print(f"Check {output_dir}/ for results")
            
            # Offer analysis
            if args.scenario != 'gain_study':
                print(f"\nTo analyze results, run:")
                print(f"    python run_milestone4.py --analyze {args.scenario}")
        else:
            print(f"\n[FAILED] Scenario '{args.scenario}' failed!")
            return 1
            
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user")
        return 1
    except Exception as e:
        print(f"\n[ERROR] Error running scenario '{args.scenario}': {e}")
        return 1
        
    return 0


if __name__ == "__main__":
    sys.exit(main())
