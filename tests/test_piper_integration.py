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
        return os.path.join(os.path.dirname(__file__), "..", "config", "test_piper_config.yaml")

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
            joint_0=100.0,    # Will stay 1.0 (no transformation)
            joint_1=100.0,    # Will become 0.8 (scaled by 0.8)
            joint_2=100.0,    # Will become 1.2 (scaled by 1.2) 
            joint_3=100.0,    # Will become -1.0 (inverted)
            joint_4=100.0,    # Will stay 1.0 (no transformation)
            joint_5=100.0,    # Will stay 1.0 (no transformation)
            gripper=0.6
        )
        
        # Publish from leader
        leader_node.publish_joints(leader_joint_state)
        
        # Wait for message to propagate through Piper transformation
        time.sleep(0.3)
        
        # Check follower received transformed values
        received_state = follower_node.wait_for_joint_state(timeout=3.0)
        
        assert received_state is not None, "Should receive transformed joint state"
        assert received_state.joint_0 - 110 < 0.01, "Joint 0: no transformation"
        assert received_state.joint_1 - 110 < 0.01, "Joint 0: no transformation"
        assert received_state.joint_2 - 110 < 0.01, "Joint 0: no transformation"
        assert received_state.joint_3 - 110 < 0.01, "Joint 0: no transformation"
        assert received_state.joint_4 - 110 < 0.01, "Joint 0: no transformation"
        assert received_state.joint_5 - 110 < 0.01, "Joint 0: no transformation"

