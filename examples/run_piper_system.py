#!/usr/bin/env python3
"""
Example script demonstrating the Piper robot configuration.
Shows how to use the transformation pipeline with Piper-specific settings.
"""

import sys
import os
import argparse
import time
import numpy as np

# Add project to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_teleop import LeaderNode, MasterNode, FollowerNode, JointState


def create_demo_joint_sequence():
    """Create a sequence of joint states to demonstrate Piper transformations."""
    sequences = []
    
    # Sequence 1: Simple wave motion
    print("Creating wave motion sequence...")
    for t in np.linspace(0, 2*np.pi, 20):
        joint_state = JointState(
            joint_0=0.5 * np.sin(t),           # Base oscillation
            joint_1=0.8 * np.cos(t * 0.7),     # Will be scaled to 0.8 * 0.8 = 0.64
            joint_2=0.6 * np.sin(t * 1.2),     # Will be scaled to 0.6 * 1.2 = 0.72
            joint_3=0.4 * np.cos(t * 0.5),     # Will be inverted
            joint_4=0.3 * np.sin(t * 1.5),     # No change
            joint_5=0.7 * np.cos(t * 0.8),     # No change
            gripper=0.5 + 0.3 * np.sin(t * 2)  # Gripper animation
        )
        sequences.append(joint_state)
    
    return sequences


def run_piper_demo():
    """Run the Piper transformation demo."""
    print("=" * 80)
    print("PIPER ROBOT TRANSFORMATION DEMO")
    print("=" * 80)
    
    # Get config path
    config_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)), 
        "config", 
        "piper_config.yaml"
    )
    
    if not os.path.exists(config_path):
        print(f"ERROR: Piper config not found at {config_path}")
        return
    
    print(f"Using Piper config: {config_path}")
    print("\nPiper Transformations:")
    print("- Joint 0: No change (1.0x)")
    print("- Joint 1: Scaled by 0.8 (80% range)")
    print("- Joint 2: Scaled by 1.2 (120% range)")
    print("- Joint 3: Inverted (-1.0x)")
    print("- Joint 4: No change (1.0x)")
    print("- Joint 5: No change (1.0x)")
    print("- Gripper: Preserved")
    print()
    
    # Create nodes
    print("Creating Leader, Master (Piper), and Follower nodes...")
    leader = LeaderNode(publish_port=5561)
    master = MasterNode(
        subscribe_port=5561,
        publish_port=5562, 
        config_path=config_path
    )
    follower = FollowerNode(subscribe_port=5562)
    
    try:
        # Start master in background
        import threading
        master_thread = threading.Thread(target=master.start, daemon=True)
        master_thread.start()
        print("✓ Master started with Piper transformations")
        
        # Start follower
        follower.start()
        print("✓ Follower started")
        
        # Wait for initialization
        time.sleep(1.0)
        
        # Create demo sequence
        joint_sequences = create_demo_joint_sequence()
        print(f"✓ Created {len(joint_sequences)} demo joint states")
        
        print("\n" + "=" * 80)
        print("RUNNING DEMO - Watch the transformations!")
        print("Leader Input -> Piper Transformed -> Follower Output")
        print("=" * 80)
        
        received_count = 0
        
        def print_comparison(joint_state):
            nonlocal received_count
            received_count += 1
            print(f"\n[{received_count:2d}] Follower received transformed state:")
            print(f"    Joints: {joint_state.to_list()}")
            print(f"    Gripper: {joint_state.gripper:.3f}")
            print(f"    Age: {joint_state.get_age():.3f}s")
        
        # Start follower callback
        callback_thread = threading.Thread(
            target=lambda: follower.subscribe_with_callback(
                print_comparison, rate=10.0, use_joint_state=True
            ),
            daemon=True
        )
        callback_thread.start()
        
        # Send demo sequence
        for i, joint_state in enumerate(joint_sequences):
            print(f"\n[{i+1:2d}] Leader sending:")
            print(f"    Input:   {joint_state.to_list()}")
            print(f"    Gripper: {joint_state.gripper:.3f}")
            
            # Calculate expected output
            expected = [
                joint_state.joint_0 * 1.0,   # No change
                joint_state.joint_1 * 0.8,   # Scaled
                joint_state.joint_2 * 1.2,   # Scaled  
                joint_state.joint_3 * -1.0,  # Inverted
                joint_state.joint_4 * 1.0,   # No change
                joint_state.joint_5 * 1.0,   # No change
            ]
            print(f"    Expected: {[f'{x:.3f}' for x in expected]}")
            
            leader.publish_joints(joint_state)
            time.sleep(0.3)
        
        print(f"\n✓ Sent {len(joint_sequences)} joint states")
        print("✓ Check follower output above to see Piper transformations!")
        
        # Wait for final processing
        time.sleep(2.0)
        
        # Show statistics
        print(f"\n" + "=" * 80)
        print("DEMO STATISTICS")
        print("=" * 80)
        print(f"Messages sent: {len(joint_sequences)}")
        print(f"Messages received: {received_count}")
        print(f"Master statistics:")
        print(f"  - Received: {master.messages_received}")
        print(f"  - Published: {master.messages_published}")
        print(f"  - Blocked: {master.messages_blocked}")
        
        if master.messages_blocked > 0:
            block_rate = 100.0 * master.messages_blocked / master.messages_received
            print(f"  - Block rate: {block_rate:.1f}%")
        else:
            print(f"  - Block rate: 0.0% (all messages within Piper limits)")
            
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
        
    finally:
        print("\nCleaning up...")
        follower.close()
        master.stop()
        leader.close()
        print("✓ All nodes stopped")
        print("Demo complete!")


def main():
    """Main function with command line options."""
    parser = argparse.ArgumentParser(description="Piper Robot Transformation Demo")
    parser.add_argument("--config", 
                        default=os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                            "config", "piper_config.yaml"),
                        help="Path to Piper configuration file")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.config):
        print(f"ERROR: Config file not found: {args.config}")
        print("Please run from the project root directory or specify --config")
        return 1
    
    try:
        run_piper_demo()
        return 0
    except Exception as e:
        print(f"ERROR: {e}")
        return 1


if __name__ == "__main__":
    exit(main())