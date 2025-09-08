#!/usr/bin/env python3
"""
Example script for running the master node.
This can run anywhere and provides safety validation and transformation.
"""

import sys
import argparse
import os
from robot_teleop import MasterNode


def main():
    parser = argparse.ArgumentParser(description="Master node for robot teleoperation")
    
    # Network configuration
    parser.add_argument("--leader-address", type=str, default="localhost",
                        help="Address of the leader node (default: localhost)")
    parser.add_argument("--leader-port", type=int, default=5555,
                        help="Port to subscribe to leader (default: 5555)")
    parser.add_argument("--follower-port", type=int, default=5556,
                        help="Port to publish to follower (default: 5556)")
    parser.add_argument("--follower-address", type=str, default="*",
                        help="Address to bind publisher (default: * for all interfaces)")
    
    # Configuration
    parser.add_argument("--config", type=str, 
                        default=os.path.join(os.path.dirname(__file__), "..", "config", "joint_limits.yaml"),
                        help="Path to configuration file with joint limits")
    parser.add_argument("--no-limits", action="store_true",
                        help="Disable joint limit checking (not recommended)")
    
    args = parser.parse_args()
    
    # Resolve config path
    config_path = os.path.abspath(args.config) if not args.no_limits else None
    
    if config_path and not os.path.exists(config_path):
        print(f"Warning: Config file not found at {config_path}")
        print("Continuing without joint limits (unsafe!)")
        config_path = None
    
    # Create master node
    master = MasterNode(
        subscribe_address=args.leader_address,
        subscribe_port=args.leader_port,
        publish_port=args.follower_port,
        publish_address=args.follower_address,
        subscribe_topic="leader_joints",
        publish_topic="follower_commands",
        config_path=config_path
    )
    
    print("Master node configuration:")
    print(f"  Subscribing to leader at {args.leader_address}:{args.leader_port}")
    print(f"  Publishing for follower on port {args.follower_port}")
    
    if config_path:
        print(f"  Using joint limits from: {config_path}")
    else:
        print("  WARNING: No joint limits configured - all values will pass through!")
    
    print("\nMaster node running...")
    print("Press Ctrl+C to stop")
    
    # Start processing
    try:
        master.start()
    except KeyboardInterrupt:
        print("\nShutting down master node...")
        master.stop()


if __name__ == "__main__":
    main()