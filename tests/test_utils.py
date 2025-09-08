"""
Tests for utility functions including joint limits validation.
"""

import pytest
import numpy as np
import tempfile
import os

from robot_teleop.utils import (
    validate_joint_limits,
    transform_joints,
    serialize_joint_message,
    deserialize_joint_message,
    load_config,
)
from robot_teleop.joint_state import JointState


class TestJointLimitsValidation:
    """Test joint limits validation functionality."""

    @pytest.fixture
    def basic_limits(self):
        """Basic joint limits for testing."""
        return {
            'joint_0': {'min': -1.0, 'max': 1.0},
            'joint_1': {'min': -0.5, 'max': 0.5},
            'joint_2': {'min': -2.0, 'max': 2.0},
            'joint_3': {'min': -3.14, 'max': 3.14},
            'joint_4': {'min': -0.3, 'max': 0.3},
            'joint_5': {'min': -3.14, 'max': 3.14},
        }

    def test_validate_within_limits(self, basic_limits):
        """Test validation of joints within limits."""
        joint_values = [0.5, 0.2, 1.0, 1.5, 0.1, -1.0]
        
        valid, violations = validate_joint_limits(joint_values, basic_limits)
        
        assert valid is True
        assert len(violations) == 0

    def test_validate_exceeding_limits(self, basic_limits):
        """Test validation of joints exceeding limits."""
        joint_values = [1.5, 0.8, 1.0, 1.5, 0.5, -1.0]  # joint_0, joint_1, joint_4 exceed
        
        valid, violations = validate_joint_limits(joint_values, basic_limits)
        
        assert valid is False
        assert len(violations) == 3
        assert any("joint_0" in v for v in violations)
        assert any("joint_1" in v for v in violations)  
        assert any("joint_4" in v for v in violations)

    def test_validate_joint_state(self, basic_limits):
        """Test validation of JointState object."""
        joint_state = JointState(
            joint_0=0.5, joint_1=0.2, joint_2=1.0,
            joint_3=1.5, joint_4=0.1, joint_5=-1.0,
            gripper=0.5
        )
        
        valid, violations = validate_joint_limits(joint_state, basic_limits)
        
        assert valid is True
        assert len(violations) == 0

    def test_validate_joint_state_exceeding(self, basic_limits):
        """Test validation of JointState exceeding limits."""
        joint_state = JointState(
            joint_0=1.5,   # Exceeds max 1.0
            joint_1=0.8,   # Exceeds max 0.5
            joint_2=1.0,   # OK
            joint_3=1.5,   # OK
            joint_4=0.5,   # Exceeds max 0.3
            joint_5=-1.0,  # OK
            gripper=0.5
        )
        
        valid, violations = validate_joint_limits(joint_state, basic_limits)
        
        assert valid is False
        assert len(violations) == 3

    def test_validate_empty_limits(self):
        """Test validation with no limits configured."""
        joint_values = [10.0, -10.0, 5.0, -5.0, 2.0, -2.0]  # Extreme values
        
        valid, violations = validate_joint_limits(joint_values, {})
        
        assert valid is True  # Should pass with no limits
        assert len(violations) == 0


class TestJointTransformation:
    """Test joint transformation functionality."""

    def test_transform_no_changes(self):
        """Test transformation with no changes applied."""
        joint_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
        result = transform_joints(joint_values)
        
        assert result == joint_values

    def test_transform_with_mapping(self):
        """Test transformation with joint mapping."""
        joint_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        mapping = {0: 1, 1: 0, 2: 2, 3: 3, 4: 4, 5: 5}  # Swap 0 and 1
        
        result = transform_joints(joint_values, joint_mapping=mapping)
        
        expected = [0.2, 0.1, 0.3, 0.4, 0.5, 0.6]  # 0 and 1 swapped
        assert result == expected

    def test_transform_with_matrix(self):
        """Test transformation with transformation matrix."""
        joint_values = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        # Scale matrix - multiply each joint by 0.5
        matrix = np.diag([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        
        result = transform_joints(joint_values, transformation_matrix=matrix)
        
        expected = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
        assert result == expected

    def test_transform_joint_state(self):
        """Test transformation with JointState input."""
        joint_state = JointState(
            joint_0=0.1, joint_1=0.2, joint_2=0.3,
            joint_3=0.4, joint_4=0.5, joint_5=0.6,
            gripper=0.8
        )
        mapping = {0: 1, 1: 0, 2: 2, 3: 3, 4: 4, 5: 5}  # Swap 0 and 1
        
        joints, gripper = transform_joints(joint_state, joint_mapping=mapping)
        
        expected = [0.2, 0.1, 0.3, 0.4, 0.5, 0.6]  # 0 and 1 swapped
        assert joints == expected


class TestMessageSerialization:
    """Test message serialization/deserialization."""

    def test_serialize_joint_state(self):
        """Test serializing JointState to message."""
        joint_state = JointState(
            joint_0=0.1, joint_1=0.2, joint_2=0.3,
            joint_3=0.4, joint_4=0.5, joint_5=0.6,
            gripper=0.8, timestamp=12345.0
        )
        
        message = serialize_joint_message(joint_state)
        
        assert message['joint_0'] == 0.1
        assert message['gripper'] == 0.8
        assert message['timestamp'] == 12345.0

    def test_serialize_list_backward_compatibility(self):
        """Test serializing list for backward compatibility."""
        joint_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
        message = serialize_joint_message(joint_values)
        
        # Should convert to JointState format
        assert 'joint_0' in message
        assert message['joint_0'] == 0.1
        assert message['gripper'] == 0.0  # Default

    def test_deserialize_joint_state_format(self):
        """Test deserializing new JointState format."""
        message = {
            'joint_0': 0.1, 'joint_1': 0.2, 'joint_2': 0.3,
            'joint_3': 0.4, 'joint_4': 0.5, 'joint_5': 0.6,
            'gripper': 0.8, 'timestamp': 12345.0
        }
        
        joint_data, timestamp = deserialize_joint_message(message)
        
        assert isinstance(joint_data, JointState)
        assert joint_data.joint_0 == 0.1
        assert joint_data.gripper == 0.8
        assert timestamp == 12345.0

    def test_deserialize_old_format(self):
        """Test deserializing old list format."""
        message = {
            'joints': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            'timestamp': 12345.0
        }
        
        joint_data, timestamp = deserialize_joint_message(message)
        
        assert isinstance(joint_data, list)
        assert joint_data == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        assert timestamp == 12345.0

    def test_roundtrip_serialization(self):
        """Test that serialization is lossless."""
        original = JointState(
            joint_0=0.1, joint_1=0.2, joint_2=0.3,
            joint_3=0.4, joint_4=0.5, joint_5=0.6,
            gripper=0.8, timestamp=12345.0
        )
        
        # Serialize and deserialize
        message = serialize_joint_message(original)
        restored_data, restored_timestamp = deserialize_joint_message(message)
        
        assert isinstance(restored_data, JointState)
        assert restored_data.joint_0 == original.joint_0
        assert restored_data.gripper == original.gripper
        assert restored_timestamp == original.timestamp


class TestConfigLoading:
    """Test configuration file loading."""

    def test_load_valid_config(self):
        """Test loading valid configuration file."""
        config_content = """
joint_limits:
  joint_0:
    min: -1.0
    max: 1.0
    name: "Base rotation"
  joint_1:
    min: -0.5
    max: 0.5
    name: "Shoulder pitch"

joint_mapping:
  0: 0
  1: 1

transformation_matrix:
  - [1.0, 0.0]
  - [0.0, 1.0]
"""
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(config_content)
            temp_path = f.name
        
        try:
            config = load_config(temp_path)
            
            assert 'joint_limits' in config
            assert 'joint_mapping' in config
            assert 'transformation_matrix' in config
            assert config['joint_limits']['joint_0']['min'] == -1.0
            
        finally:
            os.unlink(temp_path)

    def test_load_nonexistent_config(self):
        """Test loading non-existent configuration file."""
        with pytest.raises(FileNotFoundError):
            load_config("/nonexistent/file.yaml")