"""
Integration tests for the complete robot teleoperation system.
Tests leader -> master -> follower pipeline.
"""

import pytest
import time
import threading
from unittest.mock import patch
import tempfile
import os

from robot_teleop import LeaderNode, MasterNode, FollowerNode, JointState


class TestIntegration:
    """Integration tests for the complete system."""

    @pytest.fixture
    def temp_config(self):
        """Create temporary config file for testing."""
        config_content = """
joint_limits:
  joint_0:
    min: -1.0
    max: 1.0
    name: "Base rotation"
  joint_1:
    min: -0.5
    max: 0.5
    name: "Shoulder pitch - RESTRICTED"
  joint_2:
    min: -1.5708
    max: 1.5708
    name: "Elbow pitch"
  joint_3:
    min: -3.14159
    max: 3.14159
    name: "Wrist roll"
  joint_4:
    min: -0.3
    max: 0.3
    name: "Wrist pitch - RESTRICTED"
  joint_5:
    min: -3.14159
    max: 3.14159
    name: "Wrist yaw"
"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(config_content)
            temp_path = f.name
        
        yield temp_path
        
        # Cleanup
        os.unlink(temp_path)

    @pytest.fixture
    def leader_node(self):
        """Create a leader node for testing."""
        leader = LeaderNode(publish_port=5557)  # Use different port for tests
        yield leader
        leader.close()

    @pytest.fixture
    def master_node(self, temp_config):
        """Create a master node for testing."""
        master = MasterNode(
            subscribe_port=5557,
            publish_port=5558,
            config_path=temp_config
        )
        yield master
        master.stop()

    @pytest.fixture
    def follower_node(self):
        """Create a follower node for testing."""
        follower = FollowerNode(subscribe_port=5558)
        yield follower
        follower.close()

    @pytest.mark.integration
    def test_basic_pipeline(self, leader_node, master_node, follower_node):
        """Test basic leader -> master -> follower pipeline."""
        # Start master in background thread
        master_thread = threading.Thread(target=master_node.start, daemon=True)
        master_thread.start()
        
        # Start follower
        follower_node.start()
        
        # Give system time to initialize
        time.sleep(0.5)
        
        # Create test joint state
        test_joint_state = JointState(
            joint_0=0.5,    # Within limits
            joint_1=0.2,    # Within limits
            joint_2=0.8,    # Within limits
            joint_3=1.0,    # Within limits
            joint_4=0.1,    # Within limits
            joint_5=-1.0,   # Within limits
            gripper=0.7
        )
        
        # Publish from leader
        leader_node.publish_joints(test_joint_state)
        
        # Wait for message to propagate
        time.sleep(0.2)
        
        # Check if follower received the message
        received_state = follower_node.wait_for_joint_state(timeout=2.0)
        
        assert received_state is not None
        assert abs(received_state.joint_0 - 0.5) < 0.01
        assert abs(received_state.joint_1 - 0.2) < 0.01
        assert abs(received_state.gripper - 0.7) < 0.01

    @pytest.mark.integration
    def test_joint_limits_blocking(self, leader_node, master_node, follower_node):
        """Test that master blocks joints exceeding limits."""
        # Start master in background thread
        master_thread = threading.Thread(target=master_node.start, daemon=True)
        master_thread.start()
        
        # Start follower
        follower_node.start()
        
        # Give system time to initialize
        time.sleep(0.5)
        
        # Create joint state that violates limits
        violating_joint_state = JointState(
            joint_0=0.5,    # OK
            joint_1=0.8,    # VIOLATES limit (max 0.5)
            joint_2=0.3,    # OK
            joint_3=0.5,    # OK
            joint_4=0.5,    # VIOLATES limit (max 0.3)
            joint_5=1.0,    # OK
            gripper=0.5
        )
        
        # Publish violating state
        leader_node.publish_joints(violating_joint_state)
        
        # Wait a bit
        time.sleep(0.2)
        
        # Should NOT receive anything (blocked by master)
        received_state = follower_node.get_latest_joint_state()
        assert received_state is None
        
        # Now send valid state
        valid_joint_state = JointState(
            joint_0=0.5,    # OK
            joint_1=0.2,    # OK
            joint_2=0.3,    # OK
            joint_3=0.5,    # OK
            joint_4=0.1,    # OK
            joint_5=1.0,    # OK
            gripper=0.8
        )
        
        leader_node.publish_joints(valid_joint_state)
        time.sleep(0.2)
        
        # Should receive the valid state
        received_state = follower_node.wait_for_joint_state(timeout=2.0)
        assert received_state is not None
        assert abs(received_state.gripper - 0.8) < 0.01

    @pytest.mark.integration
    def test_multiple_messages(self, leader_node, master_node, follower_node):
        """Test sending multiple messages through the pipeline."""
        # Start master in background thread
        master_thread = threading.Thread(target=master_node.start, daemon=True)
        master_thread.start()
        
        # Start follower
        follower_node.start()
        
        # Give system time to initialize
        time.sleep(0.5)
        
        received_states = []
        
        def collect_states(joint_state):
            received_states.append(joint_state)
        
        # Start callback in background
        callback_thread = threading.Thread(
            target=lambda: follower_node.subscribe_with_callback(
                collect_states, rate=50.0, use_joint_state=True
            ),
            daemon=True
        )
        callback_thread.start()
        
        # Send multiple valid messages
        for i in range(5):
            joint_state = JointState(
                joint_0=0.1 * i,
                joint_1=0.05 * i,  # Within limits
                joint_2=0.2 * i,
                joint_3=0.1 * i,
                joint_4=0.05 * i,  # Within limits
                joint_5=0.1 * i,
                gripper=0.1 + 0.1 * i
            )
            leader_node.publish_joints(joint_state)
            time.sleep(0.1)
        
        # Wait for messages to be processed
        time.sleep(1.0)
        
        # Should have received all messages
        assert len(received_states) >= 3  # Allow for some timing variations
        
        # Check that gripper values are increasing
        gripper_values = [state.gripper for state in received_states]
        assert len(set(gripper_values)) > 1  # Should have different values

    @pytest.mark.integration
    def test_backward_compatibility(self, leader_node, master_node, follower_node):
        """Test backward compatibility with list-based joint values."""
        # Start master in background thread
        master_thread = threading.Thread(target=master_node.start, daemon=True)
        master_thread.start()
        
        # Start follower
        follower_node.start()
        
        # Give system time to initialize
        time.sleep(0.5)
        
        # Use old-style list publishing
        joint_list = [0.1, 0.2, 0.3, 0.4, 0.1, 0.5]  # All within limits
        leader_node.publish_joints(joint_list, gripper=0.6)
        
        # Wait for message
        time.sleep(0.2)
        
        # Should still work
        received_state = follower_node.wait_for_joint_state(timeout=2.0)
        assert received_state is not None
        assert abs(received_state.joint_0 - 0.1) < 0.01
        assert abs(received_state.gripper - 0.6) < 0.01
        
        # Test old-style getting
        joint_list = follower_node.get_latest_joints()
        assert joint_list is not None
        assert len(joint_list) == 6
        assert abs(joint_list[0] - 0.1) < 0.01

    @pytest.mark.integration 
    def test_message_statistics(self, leader_node, master_node, follower_node):
        """Test that master tracks statistics correctly."""
        # Start master in background thread
        master_thread = threading.Thread(target=master_node.start, daemon=True)
        master_thread.start()
        
        # Start follower
        follower_node.start()
        
        # Give system time to initialize
        time.sleep(0.5)
        
        # Send some valid messages
        for i in range(3):
            valid_state = JointState(joint_0=0.1, joint_1=0.1, gripper=0.5)
            leader_node.publish_joints(valid_state)
            time.sleep(0.1)
        
        # Send some invalid messages
        for i in range(2):
            invalid_state = JointState(joint_1=0.8, gripper=0.5)  # Exceeds limit
            leader_node.publish_joints(invalid_state)
            time.sleep(0.1)
        
        # Wait for processing
        time.sleep(0.5)
        
        # Check statistics
        assert master_node.messages_received >= 5
        assert master_node.messages_published >= 3
        assert master_node.messages_blocked >= 2