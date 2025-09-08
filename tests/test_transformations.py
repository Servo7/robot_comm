"""
Tests for joint transformations including mapping, scaling, and offsets.
"""

import pytest
import numpy as np
import tempfile
import os

from robot_teleop.utils import transform_joints
from robot_teleop.joint_state import JointState


class TestTransformations:
    """Test joint transformation functionality with mapping and offsets."""

    def test_joint_mapping_reorder(self):
        """Test joint mapping with reordering."""
        input_joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
        # Swap joints 0 and 1, keep others same
        mapping = {0: 1, 1: 0, 2: 2, 3: 3, 4: 4, 5: 5}
        
        result = transform_joints(input_joints, joint_mapping=mapping)
        expected = [0.2, 0.1, 0.3, 0.4, 0.5, 0.6]  # 0 and 1 swapped
        
        assert result == expected

    def test_joint_mapping_complex_reorder(self):
        """Test complex joint reordering."""
        input_joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        
        # Rotate all joints: 0->2, 1->0, 2->1, others stay
        mapping = {0: 2, 1: 0, 2: 1, 3: 3, 4: 4, 5: 5}
        
        result = transform_joints(input_joints, joint_mapping=mapping)
        expected = [2.0, 3.0, 1.0, 4.0, 5.0, 6.0]  # Rotated
        
        assert result == expected

    def test_joint_offsets_simple(self):
        """Test simple joint offsets."""
        input_joints = [0.5, 1.0, -0.5, 0.0, 0.3, -0.2]
        offsets = [0.1, -0.2, 0.0, 0.5, -0.1, 0.3]
        
        result = transform_joints(input_joints, joint_offsets=offsets)
        expected = [0.6, 0.8, -0.5, 0.5, 0.2, 0.1]  # input + offsets
        
        np.testing.assert_array_almost_equal(result, expected, decimal=6)

    def test_joint_offsets_with_joint_state(self):
        """Test joint offsets with JointState input."""
        joint_state = JointState(
            joint_0=1.0, joint_1=0.5, joint_2=-0.3,
            joint_3=0.8, joint_4=0.0, joint_5=-0.7,
            gripper=0.5
        )
        offsets = [0.1, 0.2, -0.1, 0.0, 0.3, 0.05]
        
        result = transform_joints(joint_state, joint_offsets=offsets)
        # When input is JointState, returns tuple (joints, gripper)
        joints, gripper = result
        expected = [1.1, 0.7, -0.4, 0.8, 0.3, -0.65]  # joint values + offsets
        
        np.testing.assert_array_almost_equal(joints, expected, decimal=6)
        assert abs(gripper - 0.5) < 0.01  # Gripper unchanged with default scale/offset

    def test_transformation_matrix_with_offsets(self):
        """Test transformation matrix combined with offsets."""
        input_joints = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        
        # Scale by 0.5, then add offsets
        transform_matrix = np.diag([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        offsets = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
        result = transform_joints(
            input_joints, 
            transformation_matrix=transform_matrix,
            joint_offsets=offsets
        )
        
        # First scale: [1,1,1,1,1,1] * 0.5 = [0.5,0.5,0.5,0.5,0.5,0.5]
        # Then add offsets: [0.5,0.5,0.5,0.5,0.5,0.5] + [0.1,0.2,0.3,0.4,0.5,0.6]
        expected = [0.6, 0.7, 0.8, 0.9, 1.0, 1.1]
        
        # With list input, should return list (not tuple)
        assert isinstance(result, list)
        np.testing.assert_array_almost_equal(result, expected, decimal=6)

    def test_mapping_matrix_offsets_combined(self):
        """Test all three transformations together."""
        input_joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        
        # 1. Mapping: swap 0 and 1
        mapping = {0: 1, 1: 0, 2: 2, 3: 3, 4: 4, 5: 5}
        
        # 2. Matrix: scale by different factors
        transform_matrix = np.diag([0.8, 1.2, 0.5, 2.0, 0.1, 1.0])
        
        # 3. Offsets: add different values
        offsets = [0.1, -0.1, 0.5, -0.2, 1.0, 0.0]
        
        result = transform_joints(
            input_joints,
            transformation_matrix=transform_matrix,
            joint_mapping=mapping,
            joint_offsets=offsets
        )
        
        # Step by step:
        # Input: [1, 2, 3, 4, 5, 6]
        # 1. After mapping: [2, 1, 3, 4, 5, 6]  (swapped 0 and 1)
        # 2. After matrix: [1.6, 1.2, 1.5, 8.0, 0.5, 6.0]  (scaled)
        # 3. After offsets: [1.7, 1.1, 2.0, 7.8, 1.5, 6.0]  (added offsets)
        expected = [1.7, 1.1, 2.0, 7.8, 1.5, 6.0]
        
        np.testing.assert_array_almost_equal(result, expected, decimal=6)

    def test_piper_transformation_sequence(self):
        """Test the actual Piper transformation sequence."""
        # Simulate Piper config transformations
        input_joints = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        
        # Identity mapping (no reordering)
        mapping = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5}
        
        # Piper transformation matrix
        transform_matrix = np.array([
            [1.0,  0.0,  0.0,  0.0,  0.0,  0.0],   # Joint 0: no change
            [0.0,  0.8,  0.0,  0.0,  0.0,  0.0],   # Joint 1: scale by 0.8
            [0.0,  0.0,  1.2,  0.0,  0.0,  0.0],   # Joint 2: scale by 1.2
            [0.0,  0.0,  0.0, -1.0,  0.0,  0.0],   # Joint 3: invert
            [0.0,  0.0,  0.0,  0.0,  1.0,  0.0],   # Joint 4: no change
            [0.0,  0.0,  0.0,  0.0,  0.0,  1.0],   # Joint 5: no change
        ])
        
        # Piper offsets
        offsets = [0.0, 0.1, -0.05, 0.0, 0.02, 0.0]
        
        result = transform_joints(
            input_joints,
            transformation_matrix=transform_matrix,
            joint_mapping=mapping,
            joint_offsets=offsets
        )
        
        # Expected: [1.0, 0.8, 1.2, -1.0, 1.0, 1.0] + offsets
        expected = [1.0, 0.9, 1.15, -1.0, 1.02, 1.0]
        
        # With list input, should return list
        assert isinstance(result, list)
        np.testing.assert_array_almost_equal(result, expected, decimal=6)

    def test_mirror_transformation(self):
        """Test mirroring transformation."""
        input_joints = [0.5, 0.3, -0.2, 0.8, -0.4, 0.1]
        
        # Mirror by inverting specific joints
        transform_matrix = np.array([
            [-1.0, 0.0,  0.0,  0.0,  0.0,  0.0],   # Flip base
            [0.0,  1.0,  0.0,  0.0,  0.0,  0.0],   # Keep shoulder
            [0.0,  0.0, -1.0,  0.0,  0.0,  0.0],   # Flip elbow
            [0.0,  0.0,  0.0,  1.0,  0.0,  0.0],   # Keep wrist roll
            [0.0,  0.0,  0.0,  0.0, -1.0,  0.0],   # Flip wrist pitch
            [0.0,  0.0,  0.0,  0.0,  0.0,  1.0],   # Keep wrist yaw
        ])
        
        result = transform_joints(input_joints, transformation_matrix=transform_matrix)
        expected = [-0.5, 0.3, 0.2, 0.8, 0.4, 0.1]  # Flipped specific joints
        
        np.testing.assert_array_almost_equal(result, expected, decimal=6)

    def test_calibration_offsets(self):
        """Test calibration offsets for zero position differences."""
        input_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # All zeros
        
        # Calibration offsets (different zero positions)
        calibration_offsets = [
            0.087,   # 5 degrees for base
            -0.175,  # -10 degrees for shoulder
            0.262,   # 15 degrees for elbow
            -0.087,  # -5 degrees for wrist roll
            0.175,   # 10 degrees for wrist pitch
            0.0      # No offset for wrist yaw
        ]
        
        result = transform_joints(input_joints, joint_offsets=calibration_offsets)
        expected = calibration_offsets  # Should equal the offsets
        
        np.testing.assert_array_almost_equal(result, expected, decimal=6)

    def test_transformation_order(self):
        """Test that transformations are applied in correct order: mapping -> matrix -> offsets."""
        input_joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        
        # Simple test: swap first two, scale by 2, add 1
        mapping = {0: 1, 1: 0, 2: 2, 3: 3, 4: 4, 5: 5}
        matrix = np.eye(6) * 2  # Scale all by 2
        offsets = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # Add 1 to all
        
        result = transform_joints(
            input_joints,
            transformation_matrix=matrix,
            joint_mapping=mapping,
            joint_offsets=offsets
        )
        
        # Step by step:
        # 1. Input: [1, 2, 3, 4, 5, 6]
        # 2. Mapping: [2, 1, 3, 4, 5, 6] (swap first two)
        # 3. Matrix: [4, 2, 6, 8, 10, 12] (multiply by 2)
        # 4. Offsets: [5, 3, 7, 9, 11, 13] (add 1)
        expected = [5.0, 3.0, 7.0, 9.0, 11.0, 13.0]
        
        np.testing.assert_array_almost_equal(result, expected, decimal=6)

    def test_empty_transformations(self):
        """Test with no transformations applied."""
        input_joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
        result = transform_joints(input_joints)  # No transformations
        
        assert result == input_joints  # Should be unchanged

    def test_invalid_offset_length(self):
        """Test behavior with invalid offset length."""
        input_joints = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        invalid_offsets = [0.1, 0.2, 0.3]  # Only 3 offsets for 6 joints
        
        # Should log warning but still return original joints
        result = transform_joints(input_joints, joint_offsets=invalid_offsets)
        assert result == input_joints