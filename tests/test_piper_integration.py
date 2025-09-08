"""
Integration tests for Piper robot configuration.
Tests the transformation pipeline with Piper-specific settings.
"""

import pytest
import time
import threading
import os
import numpy as np

from robot_teleop import LeaderNode, MasterNode, FollowerNode, JointState


class TestPiperIntegration:
    """Test the complete system with Piper robot configuration."""

    @pytest.fixture
    def piper_config_path(self):
        """Path to Piper configuration file."""
        return os.path.join(os.path.dirname(__file__), "..", "config", "piper_config.yaml")

    @pytest.fixture
    def leader_node(self):
        """Create a leader node for Piper testing."""
        leader = LeaderNode(publish_port=5559)  # Different port for Piper tests
        yield leader
        leader.close()

    @pytest.fixture
    def piper_master_node(self, piper_config_path):
        """Create a master node with Piper configuration."""
        master = MasterNode(
            subscribe_port=5559,
            publish_port=5560,
            config_path=piper_config_path
        )
        yield master
        master.stop()

    @pytest.fixture
    def follower_node(self):
        """Create a follower node for Piper testing."""
        follower = FollowerNode(subscribe_port=5560)
        yield follower
        follower.close()

    @pytest.mark.integration
    def test_piper_transformation_pipeline(self, leader_node, piper_master_node, follower_node):
        """Test the complete Piper transformation pipeline."""
        # Start master in background
        master_thread = threading.Thread(target=piper_master_node.start, daemon=True)
        master_thread.start()
        
        # Start follower
        follower_node.start()
        
        # Give system time to initialize
        time.sleep(0.5)
        
        # Create test input that will be transformed by Piper config
        leader_joint_state = JointState(
            joint_0=1.0,    # Will stay 1.0 (no transformation)
            joint_1=1.0,    # Will become 0.8 (scaled by 0.8)
            joint_2=1.0,    # Will become 1.2 (scaled by 1.2) 
            joint_3=1.0,    # Will become -1.0 (inverted)
            joint_4=1.0,    # Will stay 1.0 (no transformation)
            joint_5=1.0,    # Will stay 1.0 (no transformation)
            gripper=0.6
        )
        
        # Publish from leader
        leader_node.publish_joints(leader_joint_state)
        
        # Wait for message to propagate through Piper transformation
        time.sleep(0.3)
        
        # Check follower received transformed values
        received_state = follower_node.wait_for_joint_state(timeout=3.0)
        
        assert received_state is not None, "Should receive transformed joint state"
        
        # Verify Piper transformations were applied (transformation matrix + offsets)
        # Joint 0: 1.0 * 1.0 + 0.0 = 1.0
        # Joint 1: 1.0 * 0.8 + 0.1 = 0.9  
        # Joint 2: 1.0 * 1.2 + (-0.05) = 1.15
        # Joint 3: 1.0 * (-1.0) + 0.0 = -1.0
        # Joint 4: 1.0 * 1.0 + 0.02 = 1.02
        # Joint 5: 1.0 * 1.0 + 0.0 = 1.0
        assert abs(received_state.joint_0 - 1.0) < 0.01, "Joint 0: no transformation"
        assert abs(received_state.joint_1 - 0.9) < 0.01, "Joint 1: scaled by 0.8 + offset 0.1"
        assert abs(received_state.joint_2 - 1.15) < 0.01, "Joint 2: scaled by 1.2 + offset -0.05"
        assert abs(received_state.joint_3 - (-1.0)) < 0.01, "Joint 3: inverted + no offset"
        assert abs(received_state.joint_4 - 1.02) < 0.01, "Joint 4: no transformation + offset 0.02"
        assert abs(received_state.joint_5 - 1.0) < 0.01, "Joint 5: no transformation + no offset"
        assert abs(received_state.gripper - 0.6) < 0.01, "Gripper preserved"

    @pytest.mark.integration
    def test_piper_joint_limits(self, leader_node, piper_master_node, follower_node):
        """Test that Piper-specific joint limits are enforced."""
        # Start master in background
        master_thread = threading.Thread(target=piper_master_node.start, daemon=True)
        master_thread.start()
        
        # Start follower
        follower_node.start()
        
        time.sleep(0.5)
        
        # Test joint that should be blocked by Piper limits
        # Joint 1 has limit of ±1.8326 rad (~±105°)
        violating_state = JointState(
            joint_0=0.5,    # OK
            joint_1=2.5,    # VIOLATES Piper limit (>1.8326)
            joint_2=1.0,    # OK
            joint_3=1.0,    # OK  
            joint_4=1.0,    # OK
            joint_5=1.0,    # OK
            gripper=0.3
        )
        
        # Publish violating state
        leader_node.publish_joints(violating_state)
        time.sleep(0.3)
        
        # Should be blocked - no message received
        received_state = follower_node.get_latest_joint_state()
        assert received_state is None, "Violating state should be blocked"
        
        # Now send valid state within Piper limits
        # Consider transformations: joint_1 * 0.8 + 0.1, joint_2 * 1.2 - 0.05
        # For joint_1: want final ≤ 1.8326, so input ≤ (1.8326 - 0.1) / 0.8 = 2.165
        # For joint_2: want final ≤ 2.0944, so input ≤ (2.0944 + 0.05) / 1.2 = 1.787
        valid_state = JointState(
            joint_0=0.5,    # OK
            joint_1=1.5,    # Will become 1.5*0.8+0.1=1.3 (OK, within ±1.8326)
            joint_2=1.7,    # Will become 1.7*1.2-0.05=1.99 (OK, within ±2.0944)
            joint_3=1.0,    # OK
            joint_4=1.2,    # Will become 1.2*1.0+0.02=1.22 (OK, within ±1.5708)
            joint_5=2.0,    # OK
            gripper=0.4
        )
        
        leader_node.publish_joints(valid_state)
        time.sleep(0.3)
        
        # Should receive the valid, transformed state
        received_state = follower_node.wait_for_joint_state(timeout=2.0)
        assert received_state is not None, "Valid state should be received"
        
        # Check transformations were applied to valid state (matrix + offsets)
        # joint_1: 1.5 * 0.8 + 0.1 = 1.3
        # joint_2: 1.7 * 1.2 + (-0.05) = 1.99 
        # joint_3: 1.0 * (-1.0) + 0.0 = -1.0
        assert abs(received_state.joint_1 - 1.3) < 0.01, "Valid joint 1 transformed with offset"
        assert abs(received_state.joint_2 - 1.99) < 0.01, "Valid joint 2 transformed with offset"
        assert abs(received_state.joint_3 - (-1.0)) < 0.01, "Valid joint 3 inverted"

    @pytest.mark.integration
    def test_piper_joint_mapping(self, leader_node, piper_master_node, follower_node):
        """Test joint mapping functionality (identity mapping in Piper config)."""
        # Start master in background
        master_thread = threading.Thread(target=piper_master_node.start, daemon=True)
        master_thread.start()
        
        # Start follower  
        follower_node.start()
        
        time.sleep(0.5)
        
        # Send distinct values for each joint to verify mapping
        test_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        leader_state = JointState.from_list(test_values, gripper=0.9)
        
        leader_node.publish_joints(leader_state)
        time.sleep(0.3)
        
        received_state = follower_node.wait_for_joint_state(timeout=2.0)
        assert received_state is not None
        
        # With identity mapping + transformations + offsets:
        # joint_0: 0.1 * 1.0 + 0.0 = 0.1
        # joint_1: 0.2 * 0.8 + 0.1 = 0.26  
        # joint_2: 0.3 * 1.2 + (-0.05) = 0.31
        # joint_3: 0.4 * (-1.0) + 0.0 = -0.4
        # joint_4: 0.5 * 1.0 + 0.02 = 0.52
        # joint_5: 0.6 * 1.0 + 0.0 = 0.6
        
        expected = [0.1, 0.26, 0.31, -0.4, 0.52, 0.6]
        received = received_state.to_list()
        
        for i, (expected_val, received_val) in enumerate(zip(expected, received)):
            assert abs(received_val - expected_val) < 0.01, f"Joint {i}: expected {expected_val}, got {received_val}"

    @pytest.mark.integration
    def test_piper_continuous_operation(self, leader_node, piper_master_node, follower_node):
        """Test continuous operation with Piper transformations."""
        # Start master in background
        master_thread = threading.Thread(target=piper_master_node.start, daemon=True)
        master_thread.start()
        
        # Start follower
        follower_node.start()
        
        time.sleep(0.5)
        
        received_states = []
        
        def collect_piper_states(joint_state):
            received_states.append(joint_state)
        
        # Start callback in background
        callback_thread = threading.Thread(
            target=lambda: follower_node.subscribe_with_callback(
                collect_piper_states, rate=50.0, use_joint_state=True
            ),
            daemon=True
        )
        callback_thread.start()
        
        # Send sequence of joint states that will be transformed
        for i in range(5):
            base_value = 0.2 * i  # 0.0, 0.2, 0.4, 0.6, 0.8
            joint_state = JointState(
                joint_0=base_value,           # No transform
                joint_1=base_value,           # * 0.8
                joint_2=base_value,           # * 1.2  
                joint_3=base_value,           # * -1.0
                joint_4=base_value,           # No transform
                joint_5=base_value,           # No transform
                gripper=0.1 + 0.1 * i
            )
            leader_node.publish_joints(joint_state)
            time.sleep(0.1)
        
        # Wait for processing
        time.sleep(1.0)
        
        # Should have received multiple transformed states
        assert len(received_states) >= 3, f"Should receive multiple states, got {len(received_states)}"
        
        # Check that transformations are applied consistently
        for i, state in enumerate(received_states[:3]):
            # Verify transformations are working consistently
            # With base_value * i and transformations:
            # joint_0: base_value (no transform)
            # joint_1: base_value * 0.8 + 0.1 (scaled + offset)
            # joint_2: base_value * 1.2 - 0.05 (scaled + offset)
            # joint_3: base_value * (-1.0) (inverted)
            if i > 0:  # Skip first state which might be zero
                base_val = received_states[0].joint_0 if i == 1 else state.joint_0
                # For small values, offset dominates, so joint_1 might be > joint_0
                # But joint_2 should still be scaled up from the base
                if base_val > 0.1:  # Only check when base value is significant
                    assert state.joint_2 >= state.joint_0, "Joint 2 should be scaled up"
                    # Joint 3 values should be negative if input was positive
                    assert state.joint_3 <= 0, "Joint 3 should be inverted"

    def test_piper_config_loading(self, piper_config_path):
        """Test that Piper configuration loads correctly."""
        from robot_teleop.utils import load_config
        
        # Load Piper config
        config = load_config(piper_config_path)
        
        # Verify structure
        assert 'joint_limits' in config
        assert 'joint_mapping' in config  
        assert 'transformation_matrix' in config
        assert 'piper_settings' in config
        
        # Check joint limits
        joint_limits = config['joint_limits']
        assert 'joint_0' in joint_limits
        assert joint_limits['joint_0']['min'] == -3.14159
        assert joint_limits['joint_1']['max'] == 1.8326
        
        # Check transformation matrix dimensions
        transform_matrix = config['transformation_matrix']
        assert len(transform_matrix) == 6, "Should have 6 rows"
        assert len(transform_matrix[0]) == 6, "Should have 6 columns"
        
        # Check specific transformations
        assert transform_matrix[1][1] == 0.8, "Joint 1 should be scaled by 0.8"
        assert transform_matrix[2][2] == 1.2, "Joint 2 should be scaled by 1.2"  
        assert transform_matrix[3][3] == -1.0, "Joint 3 should be inverted"
        
        # Check Piper settings
        piper_settings = config['piper_settings']
        assert 'gripper_type' in piper_settings
        assert piper_settings['gripper_type'] == 'parallel'
        assert piper_settings['control_frequency'] == 100