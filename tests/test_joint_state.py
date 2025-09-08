"""
Unit tests for JointState class.
"""

import pytest
import time
from robot_teleop.joint_state import JointState


class TestJointState:
    """Test JointState functionality."""

    def test_default_creation(self):
        """Test creating JointState with default values."""
        js = JointState()
        
        assert js.joint_0 == 0.0
        assert js.joint_1 == 0.0
        assert js.joint_2 == 0.0
        assert js.joint_3 == 0.0
        assert js.joint_4 == 0.0
        assert js.joint_5 == 0.0
        assert js.gripper == 0.0
        assert js.timestamp > 0  # Should auto-set timestamp

    def test_custom_creation(self):
        """Test creating JointState with custom values."""
        timestamp = time.time()
        js = JointState(
            joint_0=0.1, joint_1=0.2, joint_2=0.3,
            joint_3=0.4, joint_4=0.5, joint_5=0.6,
            gripper=0.8, timestamp=timestamp
        )
        
        assert js.joint_0 == 0.1
        assert js.joint_1 == 0.2
        assert js.joint_2 == 0.3
        assert js.joint_3 == 0.4
        assert js.joint_4 == 0.5
        assert js.joint_5 == 0.6
        assert js.gripper == 0.8
        assert js.timestamp == timestamp

    def test_from_list(self):
        """Test creating JointState from list."""
        joint_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        js = JointState.from_list(joint_values, gripper=0.7)
        
        assert js.to_list() == joint_values
        assert js.gripper == 0.7
        assert js.timestamp > 0

    def test_from_list_invalid_length(self):
        """Test creating JointState from invalid list length."""
        with pytest.raises(ValueError, match="Expected 6 joint values"):
            JointState.from_list([1, 2, 3])  # Only 3 values

    def test_from_dict(self):
        """Test creating JointState from dictionary."""
        data = {
            'joint_0': 0.1, 'joint_1': 0.2, 'joint_2': 0.3,
            'joint_3': 0.4, 'joint_4': 0.5, 'joint_5': 0.6,
            'gripper': 0.8, 'timestamp': 12345.0
        }
        js = JointState.from_dict(data)
        
        assert js.joint_0 == 0.1
        assert js.gripper == 0.8
        assert js.timestamp == 12345.0

    def test_to_list(self):
        """Test converting JointState to list."""
        js = JointState(
            joint_0=0.1, joint_1=0.2, joint_2=0.3,
            joint_3=0.4, joint_4=0.5, joint_5=0.6
        )
        
        expected = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        assert js.to_list() == expected

    def test_to_dict(self):
        """Test converting JointState to dictionary."""
        js = JointState(
            joint_0=0.1, joint_1=0.2, joint_2=0.3,
            joint_3=0.4, joint_4=0.5, joint_5=0.6,
            gripper=0.8, timestamp=12345.0
        )
        
        data = js.to_dict()
        assert data['joint_0'] == 0.1
        assert data['gripper'] == 0.8
        assert data['timestamp'] == 12345.0

    def test_to_full_list(self):
        """Test converting JointState to full list including gripper."""
        js = JointState(
            joint_0=0.1, joint_1=0.2, joint_2=0.3,
            joint_3=0.4, joint_4=0.5, joint_5=0.6,
            gripper=0.8
        )
        
        expected = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8]
        assert js.to_full_list() == expected

    def test_copy(self):
        """Test copying JointState."""
        js1 = JointState(joint_0=0.1, gripper=0.5, timestamp=12345.0)
        js2 = js1.copy()
        
        assert js1.joint_0 == js2.joint_0
        assert js1.gripper == js2.gripper
        assert js1.timestamp == js2.timestamp
        assert js1 is not js2  # Different objects

    def test_apply_joint_values(self):
        """Test applying new joint values while preserving gripper."""
        js = JointState(gripper=0.8, timestamp=12345.0)
        new_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        
        js_new = js.apply_joint_values(new_values)
        
        assert js_new.to_list() == new_values
        assert js_new.gripper == 0.8  # Preserved
        assert js_new.timestamp > js.timestamp  # Updated

    def test_apply_joint_values_invalid(self):
        """Test applying invalid number of joint values."""
        js = JointState()
        
        with pytest.raises(ValueError, match="Expected 6 joint values"):
            js.apply_joint_values([1, 2, 3])

    def test_get_age(self):
        """Test getting age of JointState."""
        # Create with old timestamp
        old_timestamp = time.time() - 5.0
        js = JointState(timestamp=old_timestamp)
        
        age = js.get_age()
        assert 4.9 < age < 5.1  # Should be approximately 5 seconds

    def test_string_representation(self):
        """Test string representations."""
        js = JointState(
            joint_0=0.1, joint_1=0.2, joint_2=0.3,
            joint_3=0.4, joint_4=0.5, joint_5=0.6,
            gripper=0.8
        )
        
        str_repr = str(js)
        assert "JointState([" in str_repr
        assert "gripper=0.800" in str_repr
        
        repr_str = repr(js)
        assert "JointState(joint_0=0.100" in repr_str

    def test_roundtrip_serialization(self):
        """Test that serialization/deserialization is lossless."""
        original = JointState(
            joint_0=0.1, joint_1=0.2, joint_2=0.3,
            joint_3=0.4, joint_4=0.5, joint_5=0.6,
            gripper=0.8, timestamp=12345.0
        )
        
        # Convert to dict and back
        data = original.to_dict()
        restored = JointState.from_dict(data)
        
        assert original.joint_0 == restored.joint_0
        assert original.joint_1 == restored.joint_1
        assert original.joint_2 == restored.joint_2
        assert original.joint_3 == restored.joint_3
        assert original.joint_4 == restored.joint_4
        assert original.joint_5 == restored.joint_5
        assert original.gripper == restored.gripper
        assert original.timestamp == restored.timestamp