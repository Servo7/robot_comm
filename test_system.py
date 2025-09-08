#!/usr/bin/env python3
"""
Test script to run all components together for debugging.
This script starts leader, master, and follower in separate threads/processes.
"""

import time
import sys
import os
import threading
import numpy as np
from multiprocessing import Process, Queue
import logging

# Add the project to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_teleop import LeaderNode, MasterNode, FollowerNode, JointState

# Configure logging to see all debug messages
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

def run_leader(status_queue):
    """Run the leader node with test data that will trigger some blocks."""
    print("\n[LEADER] Starting leader node...")
    leader = LeaderNode(publish_port=5555)
    
    start_time = time.time()
    iteration = 0
    
    try:
        while True:
            t = time.time() - start_time
            
            # Generate test joint state - some will violate limits
            joint_values = [
                0.5 * np.sin(t),           # joint_0: ±0.5 (within ±1.0 limit)
                0.7 * np.cos(t * 0.7),      # joint_1: ±0.7 (VIOLATES ±0.5 limit!)
                0.4 * np.sin(t * 1.3),      # joint_2: ±0.4 (within ±π/2 limit)
                0.2 * np.cos(t * 0.5),      # joint_3: ±0.2 (within ±π limit)
                0.35 * np.sin(t * 0.9),     # joint_4: ±0.35 (VIOLATES ±0.3 limit!)
                0.6 * np.cos(t * 1.1),      # joint_5: ±0.6 (within ±π limit)
            ]
            
            # Create JointState with varying gripper
            gripper_state = 0.5 + 0.5 * np.sin(t * 0.3)  # Gripper oscillates 0-1
            joint_state = JointState.from_list(joint_values, gripper=gripper_state)
            
            leader.publish_joints(joint_state)
            
            # Status update every second
            if iteration % 100 == 0:
                status_msg = f"[LEADER] Published iteration {iteration}: {joint_state}"
                print(status_msg)
                if status_queue:
                    status_queue.put(('leader', iteration, joint_state))
            
            iteration += 1
            time.sleep(0.01)  # 100Hz
            
    except KeyboardInterrupt:
        print("[LEADER] Shutting down...")
    finally:
        leader.close()


def run_master(status_queue):
    """Run the master node with test limits."""
    print("\n[MASTER] Starting master node with test limits...")
    
    config_path = os.path.join(os.path.dirname(__file__), "config", "test_limits.yaml")
    
    # Custom master with more verbose logging
    master = MasterNode(
        subscribe_port=5555,
        subscribe_address="localhost",
        publish_port=5556,
        config_path=config_path
    )
    
    print(f"[MASTER] Loaded config from: {config_path}")
    print(f"[MASTER] Joint limits: {master.joint_limits}")
    
    # Start the master
    try:
        # We'll modify the master to send status updates
        original_process = master.process_joint_state
        
        def process_with_status(joint_state):
            result = original_process(joint_state)
            if master.messages_received % 10 == 0:
                status = {
                    'received': master.messages_received,
                    'published': master.messages_published,
                    'blocked': master.messages_blocked,
                    'block_rate': 100.0 * master.messages_blocked / max(1, master.messages_received)
                }
                print(f"[MASTER] Stats: Received={status['received']}, Published={status['published']}, "
                      f"Blocked={status['blocked']} ({status['block_rate']:.1f}%)")
                if status_queue:
                    status_queue.put(('master', master.messages_received, status))
            return result
        
        master.process_joint_state = process_with_status
        master.start()
        
    except KeyboardInterrupt:
        print("[MASTER] Shutting down...")
    finally:
        master.stop()


def run_follower(status_queue):
    """Run the follower node and print received joints."""
    print("\n[FOLLOWER] Starting follower node...")
    
    follower = FollowerNode(
        subscribe_port=5556,
        subscribe_address="localhost"
    )
    
    follower.start()
    
    print("[FOLLOWER] Waiting for joint commands...")
    
    last_joint_state = None
    no_data_counter = 0
    iteration = 0
    
    try:
        while True:
            joint_state = follower.get_latest_joint_state()
            
            if joint_state is not None:
                # Only print if joint state changed
                if joint_state.timestamp != (last_joint_state.timestamp if last_joint_state else 0):
                    age = joint_state.get_age()
                    print(f"[FOLLOWER] Received: {joint_state} | Age: {age:.3f}s")
                    last_joint_state = joint_state
                    no_data_counter = 0
                    
                    if status_queue and iteration % 10 == 0:
                        status_queue.put(('follower', iteration, joint_state))
            else:
                no_data_counter += 1
                if no_data_counter % 100 == 0:
                    print("[FOLLOWER] No data received yet...")
            
            iteration += 1
            time.sleep(0.01)  # 100Hz check rate
            
    except KeyboardInterrupt:
        print("[FOLLOWER] Shutting down...")
    finally:
        follower.close()


def main():
    """Main test function."""
    print("=" * 80)
    print("ROBOT TELEOPERATION TEST SYSTEM")
    print("=" * 80)
    print("\nThis test will:")
    print("1. Start a LEADER that publishes joints (some violating limits)")
    print("2. Start a MASTER with restrictive limits that blocks unsafe values")
    print("3. Start a FOLLOWER that receives only safe joint values")
    print("\nExpected behavior:")
    print("- Joint 1 (±0.7) will be BLOCKED (limit is ±0.5)")
    print("- Joint 4 (±0.35) will be BLOCKED (limit is ±0.3)")
    print("- Other joints should pass through")
    print("- Gripper state will be preserved through the pipeline")
    print("\n" + "=" * 80)
    
    # Create status queue for inter-process communication
    status_queue = Queue()
    
    # Start processes
    processes = []
    
    # Start Master first (needs to bind publisher)
    print("\nStarting MASTER process...")
    master_process = Process(target=run_master, args=(status_queue,))
    master_process.start()
    processes.append(master_process)
    time.sleep(1)  # Give master time to initialize
    
    # Start Leader
    print("\nStarting LEADER process...")
    leader_process = Process(target=run_leader, args=(status_queue,))
    leader_process.start()
    processes.append(leader_process)
    time.sleep(1)  # Give leader time to initialize
    
    # Start Follower
    print("\nStarting FOLLOWER process...")
    follower_process = Process(target=run_follower, args=(status_queue,))
    follower_process.start()
    processes.append(follower_process)
    
    print("\n" + "=" * 80)
    print("SYSTEM RUNNING - Press Ctrl+C to stop")
    print("=" * 80 + "\n")
    
    try:
        # Monitor status updates
        while True:
            if not status_queue.empty():
                component, iteration, data = status_queue.get()
                # Status updates are already printed by components
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n" + "=" * 80)
        print("SHUTTING DOWN TEST SYSTEM")
        print("=" * 80)
        
        # Terminate all processes
        for p in processes:
            p.terminate()
        
        # Wait for clean shutdown
        for p in processes:
            p.join(timeout=2)
        
        print("\nTest complete!")


if __name__ == "__main__":
    main()