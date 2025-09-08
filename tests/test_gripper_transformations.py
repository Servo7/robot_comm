"""
Tests for gripper transformations including scaling and offset.
"""

import pytest
import numpy as np
import tempfile
import os

from robot_teleop.utils import transform_joints
from robot_teleop.joint_state import JointState
from robot_teleop import MasterNode


class TestGripperTransformations:
    """Test gripper transformation functionality."""

    def test_gripper_scaling(self):
        """Test gripper scaling transformation."""
        joint_state = JointState(
            joint_0=0.5, joint_1=0.5, joint_2=0.5,
            joint_3=0.5, joint_4=0.5, joint_5=0.5,
            gripper=0.5  # Half open
        )
        
        # Scale gripper by 1.5 (150%)
        joints, transformed_gripper = transform_joints(
            joint_state,
            gripper_scale=1.5
        )
        
        assert abs(transformed_gripper - 0.75) < 0.01  # 0.5 * 1.5 = 0.75

    def test_gripper_offset(self):
        """Test gripper offset transformation."""
        joint_state = JointState(
            joint_0=0.5, joint_1=0.5, joint_2=0.5,
            joint_3=0.5, joint_4=0.5, joint_5=0.5,
            gripper=0.5
        )
        
        # Add offset of 0.2
        joints, transformed_gripper = transform_joints(
            joint_state,
            gripper_offset=0.2
        )
        
        assert abs(transformed_gripper - 0.7) < 0.01  # 0.5 + 0.2 = 0.7

    def test_gripper_scale_and_offset(self):
        """Test combined gripper scaling and offset."""
        joint_state = JointState(
            joint_0=0.5, joint_1=0.5, joint_2=0.5,
            joint_3=0.5, joint_4=0.5, joint_5=0.5,
            gripper=0.4
        )
        
        # Scale by 2.0 then add 0.1
        joints, transformed_gripper = transform_joints(
            joint_state,
            gripper_scale=2.0,
            gripper_offset=0.1
        )
        
        # 0.4 * 2.0 + 0.1 = 0.9
        assert abs(transformed_gripper - 0.9) < 0.01

    def test_gripper_clamping_upper(self):
        """Test that gripper values are clamped to [0, 1] range."""
        joint_state = JointState(
            joint_0=0.5, joint_1=0.5, joint_2=0.5,
            joint_3=0.5, joint_4=0.5, joint_5=0.5,
            gripper=0.8
        )
        
        # Scale and offset that would exceed 1.0
        joints, transformed_gripper = transform_joints(
            joint_state,
            gripper_scale=2.0,   # 0.8 * 2.0 = 1.6
            gripper_offset=0.5   # 1.6 + 0.5 = 2.1
        )
        
        # Should be clamped to 1.0
        assert transformed_gripper == 1.0

    def test_gripper_clamping_lower(self):
        """Test that gripper values are clamped to [0, 1] range."""
        joint_state = JointState(
            joint_0=0.5, joint_1=0.5, joint_2=0.5,
            joint_3=0.5, joint_4=0.5, joint_5=0.5,
            gripper=0.2
        )
        
        # Offset that would go below 0
        joints, transformed_gripper = transform_joints(
            joint_state,
            gripper_scale=0.5,    # 0.2 * 0.5 = 0.1
            gripper_offset=-0.3   # 0.1 - 0.3 = -0.2
        )
        
        # Should be clamped to 0.0
        assert transformed_gripper == 0.0

    def test_gripper_with_joint_transformations(self):
        """Test gripper transformation alongside joint transformations."""
        joint_state = JointState(
            joint_0=1.0, joint_1=1.0, joint_2=1.0,
            joint_3=1.0, joint_4=1.0, joint_5=1.0,
            gripper=0.6
        )
        
        # Apply all transformations
        transform_matrix = np.diag([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])  # Scale joints by 0.5
        joint_offsets = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # Add offsets to joints
        
        joints, transformed_gripper = transform_joints(
            joint_state,
            transformation_matrix=transform_matrix,
            joint_offsets=joint_offsets,
            gripper_scale=1.5,
            gripper_offset=-0.2
        )
        
        # Check joint transformations
        expected_joints = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6]  # (1.0 * 0.5) + 0.1
        np.testing.assert_array_almost_equal(joints, expected_joints, decimal=6)
        
        # Check gripper transformation: 0.6 * 1.5 - 0.2 = 0.7
        assert abs(transformed_gripper - 0.7) < 0.01

    def test_backward_compatibility_with_list(self):
        """Test that list input still works (no gripper transformation)."""
        joint_values = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        
        result = transform_joints(
            joint_values,
            gripper_scale=2.0,  # Should be ignored for list input
            gripper_offset=0.1   # Should be ignored for list input
        )
        
        # Should return just the list, not a tuple
        assert isinstance(result, list)
        assert result == joint_values

    def test_master_node_with_gripper_config(self):
        """Test that MasterNode correctly applies gripper transformations from config."""
        # Create config with gripper settings
        config_content = """
joint_limits:
  joint_0: {min: -3.14, max: 3.14}
  joint_1: {min: -3.14, max: 3.14}
  joint_2: {min: -3.14, max: 3.14}
  joint_3: {min: -3.14, max: 3.14}
  joint_4: {min: -3.14, max: 3.14}
  joint_5: {min: -3.14, max: 3.14}

gripper_scale: 0.8
gripper_offset: 0.1

transformation_matrix:
  - [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
"""
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(config_content)
            temp_path = f.name
        
        try:
            # Create master node with config
            master = MasterNode(
                subscribe_port=5570,
                publish_port=5571,
                config_path=temp_path
            )
            
            # Verify gripper settings were loaded
            assert master.gripper_scale == 0.8
            assert master.gripper_offset == 0.1
            
            # Test processing
            input_state = JointState(
                joint_0=0.5, joint_1=0.5, joint_2=0.5,
                joint_3=0.5, joint_4=0.5, joint_5=0.5,
                gripper=0.5
            )
            
            processed_state = master.process_joint_state(input_state)
            
            # Gripper should be: 0.5 * 0.8 + 0.1 = 0.5
            assert abs(processed_state.gripper - 0.5) < 0.01
            
            master.stop()
            
        finally:
            os.unlink(temp_path)

    def test_gripper_transformation_order(self):
        """Test that gripper transformations happen in correct order: scale then offset."""
        joint_state = JointState(
            joint_0=0.5, joint_1=0.5, joint_2=0.5,
            joint_3=0.5, joint_4=0.5, joint_5=0.5,
            gripper=0.5
        )
        
        # Test order: (gripper * scale) + offset
        joints, transformed_gripper = transform_joints(
            joint_state,
            gripper_scale=2.0,   # First: 0.5 * 2.0 = 1.0
            gripper_offset=-0.3  # Then: 1.0 - 0.3 = 0.7
        )
        
        assert abs(transformed_gripper - 0.7) < 0.01

    def test_gripper_edge_cases(self):
        """Test edge cases for gripper transformations."""
        # Test with gripper at 0 (fully closed)
        joint_state = JointState(gripper=0.0)
        joints, gripper = transform_joints(joint_state, gripper_scale=2.0, gripper_offset=0.5)
        assert abs(gripper - 0.5) < 0.01  # 0 * 2.0 + 0.5 = 0.5
        
        # Test with gripper at 1 (fully open)
        joint_state = JointState(gripper=1.0)
        joints, gripper = transform_joints(joint_state, gripper_scale=0.5, gripper_offset=-0.2)
        assert abs(gripper - 0.3) < 0.01  # 1.0 * 0.5 - 0.2 = 0.3
        
        # Test with scale of 0 (always results in offset)
        joint_state = JointState(gripper=0.7)
        joints, gripper = transform_joints(joint_state, gripper_scale=0.0, gripper_offset=0.4)
        assert abs(gripper - 0.4) < 0.01  # 0.7 * 0.0 + 0.4 = 0.4

    def test_gripper_negative_scale(self):
        """Test gripper with negative scale (inversion)."""
        joint_state = JointState(gripper=0.3)
        
        # Negative scale inverts the gripper
        joints, gripper = transform_joints(
            joint_state,
            gripper_scale=-1.0,   # Invert: 0.3 * -1.0 = -0.3
            gripper_offset=1.0    # Add offset: -0.3 + 1.0 = 0.7
        )
        
        assert abs(gripper - 0.7) < 0.01