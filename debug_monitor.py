#!/usr/bin/env python3
"""
Debug monitor script to watch the teleoperation system.
This connects as a separate follower to monitor the master's output.
"""

import sys
import os
import time
import json
import zmq
from datetime import datetime

# Add the project to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_teleop.utils import deserialize_joint_message


class SystemMonitor:
    """Monitor the teleoperation system for debugging."""
    
    def __init__(self):
        self.context = zmq.Context()
        
        # Subscribe to master output (what follower receives)
        self.master_sub = self.context.socket(zmq.SUB)
        self.master_sub.connect("tcp://localhost:5556")
        self.master_sub.setsockopt_string(zmq.SUBSCRIBE, "follower_commands")
        
        # Statistics
        self.messages_received = 0
        self.last_timestamp = None
        self.start_time = time.time()
        self.joint_history = []
        self.max_history = 100
        
    def print_header(self):
        """Print monitoring header."""
        print("=" * 100)
        print("ROBOT TELEOPERATION DEBUG MONITOR")
        print("=" * 100)
        print(f"Monitoring master output at tcp://localhost:5556")
        print(f"Started at: {datetime.now().strftime('%H:%M:%S')}")
        print("-" * 100)
        print(f"{'Time':<12} {'Age(ms)':<8} {'Joint Values':<60} {'Rate(Hz)':<8}")
        print("-" * 100)
    
    def monitor(self):
        """Main monitoring loop."""
        self.print_header()
        
        last_rate_check = time.time()
        rate_counter = 0
        current_rate = 0.0
        
        try:
            while True:
                # Check for messages with timeout
                if self.master_sub.poll(timeout=100):
                    message = self.master_sub.recv_string()
                    
                    # Parse message
                    if message.startswith("follower_commands"):
                        json_data = message[len("follower_commands"):].strip()
                        data = json.loads(json_data)
                        joints, timestamp = deserialize_joint_message(data)
                        
                        self.messages_received += 1
                        rate_counter += 1
                        
                        # Calculate message age
                        current_time = time.time()
                        age_ms = (current_time - timestamp) * 1000 if timestamp else 0
                        
                        # Format joint values
                        joint_str = "[" + ", ".join([f"{j:6.3f}" for j in joints]) + "]"
                        
                        # Calculate rate every second
                        if current_time - last_rate_check >= 1.0:
                            current_rate = rate_counter / (current_time - last_rate_check)
                            rate_counter = 0
                            last_rate_check = current_time
                        
                        # Print status line
                        time_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                        print(f"{time_str} {age_ms:6.1f}ms  {joint_str:<60} {current_rate:6.1f}Hz")
                        
                        # Store in history
                        self.joint_history.append({
                            'timestamp': timestamp,
                            'joints': joints,
                            'age_ms': age_ms
                        })
                        
                        # Limit history size
                        if len(self.joint_history) > self.max_history:
                            self.joint_history.pop(0)
                        
                        self.last_timestamp = timestamp
                    
                else:
                    # No message received in timeout
                    if self.messages_received == 0:
                        print(f"{datetime.now().strftime('%H:%M:%S')} - Waiting for messages...")
                    else:
                        # Check if we've stopped receiving messages
                        if self.last_timestamp and (time.time() - self.last_timestamp) > 2.0:
                            print(f"{datetime.now().strftime('%H:%M:%S')} - WARNING: No messages for >2 seconds")
                
                # Print statistics every 10 seconds
                if self.messages_received > 0 and self.messages_received % 1000 == 0:
                    self.print_statistics()
                    
        except KeyboardInterrupt:
            print("\n" + "=" * 100)
            print("MONITOR SHUTDOWN")
            self.print_final_statistics()
        finally:
            self.cleanup()
    
    def print_statistics(self):
        """Print current statistics."""
        runtime = time.time() - self.start_time
        avg_rate = self.messages_received / runtime if runtime > 0 else 0
        
        print("-" * 100)
        print(f"STATS: Messages={self.messages_received}, Runtime={runtime:.1f}s, "
              f"Avg Rate={avg_rate:.1f}Hz")
        
        if self.joint_history:
            recent = self.joint_history[-10:]  # Last 10 messages
            avg_age = sum(msg['age_ms'] for msg in recent) / len(recent)
            print(f"Recent avg message age: {avg_age:.1f}ms")
        
        print("-" * 100)
    
    def print_final_statistics(self):
        """Print final statistics on shutdown."""
        print("FINAL STATISTICS:")
        print(f"Total messages received: {self.messages_received}")
        
        runtime = time.time() - self.start_time
        print(f"Total runtime: {runtime:.1f} seconds")
        
        if runtime > 0:
            avg_rate = self.messages_received / runtime
            print(f"Average message rate: {avg_rate:.1f} Hz")
        
        if self.joint_history:
            ages = [msg['age_ms'] for msg in self.joint_history]
            print(f"Message latency - Min: {min(ages):.1f}ms, "
                  f"Max: {max(ages):.1f}ms, "
                  f"Avg: {sum(ages)/len(ages):.1f}ms")
        
        print("=" * 100)
    
    def cleanup(self):
        """Clean up ZMQ resources."""
        self.master_sub.close()
        self.context.term()


def main():
    """Main function."""
    if len(sys.argv) > 1:
        if sys.argv[1] == "--help":
            print("Debug Monitor for Robot Teleoperation System")
            print("\nUsage:")
            print("  python debug_monitor.py          # Monitor master output")
            print("  python debug_monitor.py --help   # Show this help")
            print("\nThis script monitors the messages being sent from the master")
            print("to the follower. Use it to debug:")
            print("- Message rates and latency")
            print("- Joint values being transmitted") 
            print("- Whether joint limits are working")
            return
    
    monitor = SystemMonitor()
    monitor.monitor()


if __name__ == "__main__":
    main()