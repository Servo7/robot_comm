"""
Extensive tests for Piper transformation pipeline with customizable YAML configs.
"""

import pytest
import time
import threading
import os
import tempfile
import yaml
import numpy as np

from robot_teleop import LeaderNode, MasterNode, FollowerNode, JointState


class TestPiperExtensive:
    """Extensive tests for Piper transformations with custom YAML configurations."""

    @pytest.fixture
    def create_temp_config(self):
        """Helper fixture to create temporary config files from YAML strings."""
        configs = []
        
        def _create_config(yaml_content):
            """Create a temporary config file from YAML string."""
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
                f.write(yaml_content)
                temp_path = f.name
            configs.append(temp_path)
            return temp_path
        
        yield _create_config
        
        # Cleanup all created configs
        for config_path in configs:
            try:
                os.unlink(config_path)
            except:
                pass

    @pytest.fixture
    def base_config_yaml(self):
        """Base configuration YAML string with safe joint limits."""
        return """
# Base joint limits - safe for all tests
joint_limits:
  joint_0:
    min: -3.14159
    max: 3.14159
    name: "Base Joint"
  joint_1:
    min: -3.14159
    max: 3.14159
    name: "Shoulder Joint"
  joint_2:
    min: -3.14159
    max: 3.14159
    name: "Elbow Joint"
  joint_3:
    min: -3.14159
    max: 3.14159
    name: "Wrist Roll"
  joint_4:
    min: -3.14159
    max: 3.14159
    name: "Wrist Pitch"
  joint_5:
    min: -3.14159
    max: 3.14159
    name: "Wrist Yaw"
"""

    @pytest.fixture
    def leader_node(self):
        """Create a leader node for testing."""
        leader = LeaderNode(publish_port=5580)
        yield leader
        leader.close()

    @pytest.fixture
    def follower_node(self):
        """Create a follower node for testing."""
        follower = FollowerNode(subscribe_port=5581)
        yield follower
        follower.close()

    def create_master_with_config(self, config_path):
        """Create a master node with specific config."""
        return MasterNode(
            subscribe_port=5580,
            publish_port=5581,
            config_path=config_path
        )

    @pytest.mark.integration
    def test_identity_transformation(self, base_config_yaml, create_temp_config, leader_node, follower_node):
        """Test identity transformation (no changes)."""
        # Create config with identity transformation
        test_yaml = base_config_yaml + """
# Identity mapping (no reordering)
joint_mapping:
  0: 0
  1: 1
  2: 2
  3: 3
  4: 4
  5: 5

# Identity matrix (no scaling)
transformation_matrix:
  - [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

# No offsets
joint_offsets:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0

# No gripper transformation
gripper_scale: 1.0
gripper_offset: 0.0
"""
        
        config_path = create_temp_config(test_yaml)
        master = self.create_master_with_config(config_path)
        
        # Start services
        master_thread = threading.Thread(target=master.start, daemon=True)
        master_thread.start()
        follower_node.start()
        time.sleep(0.5)
        
        # Send test data
        test_state = JointState(
            joint_0=0.5, joint_1=1.0, joint_2=-0.5,
            joint_3=0.8, joint_4=-0.3, joint_5=1.2,
            gripper=0.7
        )
        
        leader_node.publish_joints(test_state)
        time.sleep(0.3)
        
        # Check received values are identical
        received = follower_node.wait_for_joint_state(timeout=2.0)
        assert received is not None
        
        # All values should be unchanged
        assert abs(received.joint_0 - 0.5) < 0.01
        assert abs(received.joint_1 - 1.0) < 0.01
        assert abs(received.joint_2 - (-0.5)) < 0.01
        assert abs(received.joint_3 - 0.8) < 0.01
        assert abs(received.joint_4 - (-0.3)) < 0.01
        assert abs(received.joint_5 - 1.2) < 0.01
        assert abs(received.gripper - 0.7) < 0.01
        
        master.stop()

    @pytest.mark.integration
    def test_joint_reordering(self, base_config_yaml, create_temp_config, leader_node, follower_node):
        """Test joint reordering/mapping."""
        test_yaml = base_config_yaml + """
# Swap joints 0 and 1, 2 and 3, 4 and 5
joint_mapping:
  0: 1  # Leader joint 0 -> Follower joint 1
  1: 0  # Leader joint 1 -> Follower joint 0
  2: 3  # Leader joint 2 -> Follower joint 3
  3: 2  # Leader joint 3 -> Follower joint 2
  4: 5  # Leader joint 4 -> Follower joint 5
  5: 4  # Leader joint 5 -> Follower joint 4

# Identity matrix
transformation_matrix:
  - [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

gripper_scale: 1.0
gripper_offset: 0.0
"""
        
        config_path = create_temp_config(test_yaml)
        master = self.create_master_with_config(config_path)
        
        master_thread = threading.Thread(target=master.start, daemon=True)
        master_thread.start()
        follower_node.start()
        time.sleep(0.5)
        
        test_state = JointState(
            joint_0=0.1, joint_1=0.2, joint_2=0.3,
            joint_3=0.4, joint_4=0.5, joint_5=0.6,
            gripper=0.5
        )
        
        leader_node.publish_joints(test_state)
        time.sleep(0.3)
        
        received = follower_node.wait_for_joint_state(timeout=2.0)
        assert received is not None
        
        # Check reordering: joints should be swapped
        assert abs(received.joint_0 - 0.2) < 0.01  # Was joint_1
        assert abs(received.joint_1 - 0.1) < 0.01  # Was joint_0
        assert abs(received.joint_2 - 0.4) < 0.01  # Was joint_3
        assert abs(received.joint_3 - 0.3) < 0.01  # Was joint_2
        assert abs(received.joint_4 - 0.6) < 0.01  # Was joint_5
        assert abs(received.joint_5 - 0.5) < 0.01  # Was joint_4
        assert abs(received.gripper - 0.5) < 0.01
        
        master.stop()

    @pytest.mark.integration
    def test_scaling_transformation(self, base_config_yaml, create_temp_config, leader_node, follower_node):
        """Test scaling transformations."""
        test_yaml = base_config_yaml + """
# No reordering
joint_mapping:
  0: 0
  1: 1
  2: 2
  3: 3
  4: 4
  5: 5

# Different scales for each joint
transformation_matrix:
  - [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]   # Joint 0: 50%
  - [0.0, 2.0, 0.0, 0.0, 0.0, 0.0]   # Joint 1: 200%
  - [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]   # Joint 2: 100% (no change)
  - [0.0, 0.0, 0.0, -1.0, 0.0, 0.0]  # Joint 3: inverted
  - [0.0, 0.0, 0.0, 0.0, 0.8, 0.0]   # Joint 4: 80%
  - [0.0, 0.0, 0.0, 0.0, 0.0, 1.5]   # Joint 5: 150%

gripper_scale: 0.5
gripper_offset: 0.0
"""
        
        config_path = create_temp_config(test_yaml)
        master = self.create_master_with_config(config_path)
        
        master_thread = threading.Thread(target=master.start, daemon=True)
        master_thread.start()
        follower_node.start()
        time.sleep(0.5)
        
        test_state = JointState(
            joint_0=1.0, joint_1=0.5, joint_2=0.8,
            joint_3=0.6, joint_4=1.0, joint_5=0.4,
            gripper=0.8
        )
        
        leader_node.publish_joints(test_state)
        time.sleep(0.3)
        
        received = follower_node.wait_for_joint_state(timeout=2.0)
        assert received is not None
        
        # Check scaled values
        assert abs(received.joint_0 - 0.5) < 0.01   # 1.0 * 0.5
        assert abs(received.joint_1 - 1.0) < 0.01   # 0.5 * 2.0
        assert abs(received.joint_2 - 0.8) < 0.01   # 0.8 * 1.0
        assert abs(received.joint_3 - (-0.6)) < 0.01 # 0.6 * -1.0
        assert abs(received.joint_4 - 0.8) < 0.01   # 1.0 * 0.8
        assert abs(received.joint_5 - 0.6) < 0.01   # 0.4 * 1.5
        assert abs(received.gripper - 0.4) < 0.01   # 0.8 * 0.5
        
        master.stop()

    @pytest.mark.integration
    def test_offset_transformation(self, base_config_yaml, create_temp_config, leader_node, follower_node):
        """Test offset transformations."""
        test_yaml = base_config_yaml + """
# Identity mapping and matrix
joint_mapping:
  0: 0
  1: 1
  2: 2
  3: 3
  4: 4
  5: 5

transformation_matrix:
  - [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

# Apply different offsets
joint_offsets:
  - 0.1    # Add 0.1
  - -0.2   # Subtract 0.2
  - 0.5    # Add 0.5
  - -0.3   # Subtract 0.3
  - 0.0    # No offset
  - 0.4    # Add 0.4

gripper_scale: 1.0
gripper_offset: 0.2
"""
        
        config_path = create_temp_config(test_yaml)
        master = self.create_master_with_config(config_path)
        
        master_thread = threading.Thread(target=master.start, daemon=True)
        master_thread.start()
        follower_node.start()
        time.sleep(0.5)
        
        test_state = JointState(
            joint_0=0.5, joint_1=0.5, joint_2=0.5,
            joint_3=0.5, joint_4=0.5, joint_5=0.5,
            gripper=0.3
        )
        
        leader_node.publish_joints(test_state)
        time.sleep(0.3)
        
        received = follower_node.wait_for_joint_state(timeout=2.0)
        assert received is not None
        
        # Check offset values
        assert abs(received.joint_0 - 0.6) < 0.01   # 0.5 + 0.1
        assert abs(received.joint_1 - 0.3) < 0.01   # 0.5 - 0.2
        assert abs(received.joint_2 - 1.0) < 0.01   # 0.5 + 0.5
        assert abs(received.joint_3 - 0.2) < 0.01   # 0.5 - 0.3
        assert abs(received.joint_4 - 0.5) < 0.01   # 0.5 + 0.0
        assert abs(received.joint_5 - 0.9) < 0.01   # 0.5 + 0.4
        assert abs(received.gripper - 0.5) < 0.01   # 0.3 + 0.2
        
        master.stop()

    @pytest.mark.integration
    def test_combined_transformations(self, base_config_yaml, create_temp_config, leader_node, follower_node):
        """Test all transformations combined: mapping + scaling + offsets."""
        test_yaml = base_config_yaml + """
# Reorder joints
joint_mapping:
  0: 2  # 0 -> 2
  1: 0  # 1 -> 0
  2: 1  # 2 -> 1
  3: 3  # 3 -> 3 (same)
  4: 4  # 4 -> 4 (same)
  5: 5  # 5 -> 5 (same)

# Scale after reordering
transformation_matrix:
  - [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # Scale by 2
  - [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]   # Scale by 0.5
  - [0.0, 0.0, 1.5, 0.0, 0.0, 0.0]   # Scale by 1.5
  - [0.0, 0.0, 0.0, -1.0, 0.0, 0.0]  # Invert
  - [0.0, 0.0, 0.0, 0.0, 0.8, 0.0]   # Scale by 0.8
  - [0.0, 0.0, 0.0, 0.0, 0.0, 1.2]   # Scale by 1.2

# Add offsets after scaling
joint_offsets:
  - 0.1
  - -0.1
  - 0.2
  - 0.0
  - 0.05
  - -0.15

# Gripper transformations
gripper_scale: 1.5
gripper_offset: -0.2
"""
        
        config_path = create_temp_config(test_yaml)
        master = self.create_master_with_config(config_path)
        
        master_thread = threading.Thread(target=master.start, daemon=True)
        master_thread.start()
        follower_node.start()
        time.sleep(0.5)
        
        test_state = JointState(
            joint_0=0.3,  # -> joint 2
            joint_1=0.4,  # -> joint 0
            joint_2=0.5,  # -> joint 1
            joint_3=0.6,  # -> joint 3
            joint_4=0.7,  # -> joint 4
            joint_5=0.8,  # -> joint 5
            gripper=0.4
        )
        
        leader_node.publish_joints(test_state)
        time.sleep(0.3)
        
        received = follower_node.wait_for_joint_state(timeout=2.0)
        assert received is not None
        
        # Expected transformations:
        # joint_0 gets value from leader joint_1 (0.4) * 2.0 + 0.1 = 0.9
        # joint_1 gets value from leader joint_2 (0.5) * 0.5 - 0.1 = 0.15
        # joint_2 gets value from leader joint_0 (0.3) * 1.5 + 0.2 = 0.65
        # joint_3 gets value from leader joint_3 (0.6) * -1.0 + 0.0 = -0.6
        # joint_4 gets value from leader joint_4 (0.7) * 0.8 + 0.05 = 0.61
        # joint_5 gets value from leader joint_5 (0.8) * 1.2 - 0.15 = 0.81
        # gripper: 0.4 * 1.5 - 0.2 = 0.4
        
        assert abs(received.joint_0 - 0.9) < 0.01
        assert abs(received.joint_1 - 0.15) < 0.01
        assert abs(received.joint_2 - 0.65) < 0.01
        assert abs(received.joint_3 - (-0.6)) < 0.01
        assert abs(received.joint_4 - 0.61) < 0.01
        assert abs(received.joint_5 - 0.81) < 0.01
        assert abs(received.gripper - 0.4) < 0.01
        
        master.stop()

    @pytest.mark.integration
    def test_gripper_clamping(self, base_config_yaml, create_temp_config, leader_node, follower_node):
        """Test that gripper values are properly clamped to [0, 1]."""
        test_yaml = base_config_yaml + """
# Identity for joints
joint_mapping:
  0: 0
  1: 1
  2: 2
  3: 3
  4: 4
  5: 5

transformation_matrix:
  - [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

# Extreme gripper transformation to test clamping
gripper_scale: 3.0
gripper_offset: 0.5
"""
        
        config_path = create_temp_config(test_yaml)
        master = self.create_master_with_config(config_path)
        
        master_thread = threading.Thread(target=master.start, daemon=True)
        master_thread.start()
        follower_node.start()
        time.sleep(0.5)
        
        # Test upper clamping
        test_state = JointState(gripper=0.4)  # 0.4 * 3.0 + 0.5 = 1.7 -> should clamp to 1.0
        leader_node.publish_joints(test_state)
        time.sleep(0.3)
        
        received = follower_node.wait_for_joint_state(timeout=2.0)
        assert received is not None
        assert received.gripper == 1.0  # Clamped to maximum
        
        # Test lower clamping
        test_yaml2 = base_config_yaml + """
gripper_scale: 0.5
gripper_offset: -0.5
"""
        config_path2 = create_temp_config(test_yaml2)
        master.stop()
        master = self.create_master_with_config(config_path2)
        
        master_thread = threading.Thread(target=master.start, daemon=True)
        master_thread.start()
        time.sleep(0.5)
        
        test_state = JointState(gripper=0.3)  # 0.3 * 0.5 - 0.5 = -0.35 -> should clamp to 0.0
        leader_node.publish_joints(test_state)
        time.sleep(0.3)
        
        received = follower_node.wait_for_joint_state(timeout=2.0)
        assert received is not None
        assert received.gripper == 0.0  # Clamped to minimum
        
        master.stop()

    @pytest.mark.integration
    def test_inverted_gripper(self, base_config_yaml, create_temp_config, leader_node, follower_node):
        """Test inverted gripper logic (0=open, 1=closed vs 0=closed, 1=open)."""
        test_yaml = base_config_yaml + """
# Identity for joints
joint_mapping:
  0: 0
  1: 1
  2: 2
  3: 3
  4: 4
  5: 5

transformation_matrix:
  - [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

# Invert gripper
gripper_scale: -1.0
gripper_offset: 1.0
"""
        
        config_path = create_temp_config(test_yaml)
        master = self.create_master_with_config(config_path)
        
        master_thread = threading.Thread(target=master.start, daemon=True)
        master_thread.start()
        follower_node.start()
        time.sleep(0.5)
        
        # Test fully closed becomes fully open
        test_state = JointState(gripper=0.0)  # 0.0 * -1.0 + 1.0 = 1.0
        leader_node.publish_joints(test_state)
        time.sleep(0.3)
        
        received = follower_node.wait_for_joint_state(timeout=2.0)
        assert received is not None
        assert abs(received.gripper - 1.0) < 0.01
        
        # Test fully open becomes fully closed
        test_state = JointState(gripper=1.0)  # 1.0 * -1.0 + 1.0 = 0.0
        leader_node.publish_joints(test_state)
        time.sleep(0.3)
        
        received = follower_node.wait_for_joint_state(timeout=2.0)
        assert received is not None
        assert abs(received.gripper - 0.0) < 0.01
        
        # Test middle value
        test_state = JointState(gripper=0.3)  # 0.3 * -1.0 + 1.0 = 0.7
        leader_node.publish_joints(test_state)
        time.sleep(0.3)
        
        received = follower_node.wait_for_joint_state(timeout=2.0)
        assert received is not None
        assert abs(received.gripper - 0.7) < 0.01
        
        master.stop()