#!/usr/bin/env python3
"""
Example script for running the follower node.
This would typically run on the laptop controlling the follower robot.
"""

import sys
import time
import argparse
from robot_teleop import FollowerNode


def process_joint_commands(joints, timestamp):
    """
    Process received joint commands.
    In production, this would send commands to the actual robot.
    
    Args:
        joints: List of joint values
        timestamp: Timestamp of the message
    """
    # Format joint values for display
    joint_str = ", ".join([f"{j:6.3f}" for j in joints])
    age = time.time() - timestamp
    
    print(f"Joints: [{joint_str}] | Age: {age:.3f}s")
    
    # TODO: In production, send these to actual robot
    # Example:
    # robot.set_joint_positions(joints)


def main():
    parser = argparse.ArgumentParser(description="Follower node for robot teleoperation")
    parser.add_argument("--address", type=str, default="localhost",
                        help="Address of the master node (default: localhost)")
    parser.add_argument("--port", type=int, default=5556,
                        help="Port to subscribe for commands (default: 5556)")
    parser.add_argument("--mode", choices=["callback", "polling"], default="callback",
                        help="Operation mode: callback or polling (default: callback)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Callback rate in Hz (default: 100)")
    
    args = parser.parse_args()
    
    # Create follower node
    follower = FollowerNode(
        subscribe_address=args.address,
        subscribe_port=args.port,
        topic="follower_commands"
    )
    
    print(f"Follower node connecting to {args.address}:{args.port}")
    print(f"Mode: {args.mode}")
    print("Press Ctrl+C to stop")
    
    if args.mode == "callback":
        # Start background receiving
        follower.start()
        
        print(f"Waiting for joint commands (callback at {args.rate} Hz)...")
        
        # Wait for initial data
        initial_joints = follower.wait_for_joints(timeout=5.0)
        if initial_joints:
            print(f"Received initial joints: {len(initial_joints)} values")
        else:
            print("Warning: No initial joints received within 5 seconds")
        
        # Run callback loop
        try:
            follower.subscribe_with_callback(process_joint_commands, rate=args.rate)
        except KeyboardInterrupt:
            print("\nStopping follower node...")
            
    else:  # polling mode
        # Start background receiving
        follower.start()
        
        print("Polling for joint commands...")
        
        try:
            while True:
                joints, timestamp = follower.get_latest_joints_with_timestamp()
                
                if joints is not None:
                    process_joint_commands(joints, timestamp)
                else:
                    print("Waiting for data...")
                
                time.sleep(1.0 / args.rate)
                
        except KeyboardInterrupt:
            print("\nStopping follower node...")
    
    # Clean up
    follower.close()


if __name__ == "__main__":
    main()