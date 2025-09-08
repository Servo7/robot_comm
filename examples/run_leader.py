#!/usr/bin/env python3
"""
Example script for running the leader node.
This would typically run on the Jetson with lerobot.
"""

import sys
import time
import argparse
import numpy as np
from robot_teleop import LeaderNode


def simulate_robot_joints(t):
    """
    Simulate robot joint values for testing.
    In production, this would read from actual robot hardware.
    
    Args:
        t: Time value for simulation
        
    Returns:
        List of 6 joint values
    """
    return [
        0.5 * np.sin(t),           # Joint 0: oscillating
        0.3 * np.cos(t * 0.7),      # Joint 1: slower oscillation
        0.4 * np.sin(t * 1.3),      # Joint 2: faster oscillation
        0.2 * np.cos(t * 0.5),      # Joint 3: slow rotation
        0.35 * np.sin(t * 0.9),     # Joint 4: medium oscillation
        0.6 * np.cos(t * 1.1),      # Joint 5: medium-fast oscillation
    ]


def main():
    parser = argparse.ArgumentParser(description="Leader node for robot teleoperation")
    parser.add_argument("--port", type=int, default=5555,
                        help="Port to publish joint states (default: 5555)")
    parser.add_argument("--address", type=str, default="*",
                        help="Address to bind to (default: * for all interfaces)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Publishing rate in Hz (default: 100)")
    parser.add_argument("--simulate", action="store_true",
                        help="Use simulated joint values for testing")
    
    args = parser.parse_args()
    
    # Create leader node
    leader = LeaderNode(
        publish_port=args.port,
        publish_address=args.address,
        topic="leader_joints"
    )
    
    print(f"Leader node started on port {args.port}")
    print(f"Publishing at {args.rate} Hz")
    print("Press Ctrl+C to stop")
    
    if args.simulate:
        print("Using simulated joint values")
        # Simulation mode
        start_time = time.time()
        
        def get_simulated_joints():
            t = time.time() - start_time
            return simulate_robot_joints(t)
        
        leader.publish_loop(get_simulated_joints, rate=args.rate)
    else:
        # Production mode - integrate with actual robot
        print("Production mode: Implement get_joints_from_robot() for your hardware")
        
        def get_joints_from_robot():
            # TODO: Replace with actual robot joint reading
            # Example for lerobot integration:
            # from lerobot import Robot
            # robot = Robot()
            # return robot.get_joint_positions()
            
            # For now, return dummy values
            return [0.0] * 6
        
        leader.publish_loop(get_joints_from_robot, rate=args.rate)


if __name__ == "__main__":
    main()