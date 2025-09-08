"""
JointState data structure for robot teleoperation.
Provides a structured way to handle joint positions and gripper state.
"""

import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass


@dataclass
class JointState:
    """
    Structured representation of robot joint state.
    
    Attributes:
        joint_0: Base rotation (radians)
        joint_1: Shoulder pitch (radians)  
        joint_2: Elbow pitch (radians)
        joint_3: Wrist roll (radians)
        joint_4: Wrist pitch (radians)
        joint_5: Wrist yaw (radians)
        gripper: Gripper state (0.0 = closed, 1.0 = open)
        timestamp: Unix timestamp when state was captured
    """
    joint_0: float = 0.0
    joint_1: float = 0.0
    joint_2: float = 0.0
    joint_3: float = 0.0
    joint_4: float = 0.0
    joint_5: float = 0.0
    gripper: float = 0.0
    timestamp: float = 0.0
    
    def __post_init__(self):
        """Set timestamp if not provided."""
        if self.timestamp == 0.0:
            self.timestamp = time.time()
    
    @classmethod
    def from_list(cls, joint_values: List[float], gripper: float = 0.0, timestamp: Optional[float] = None) -> 'JointState':
        """
        Create JointState from list of joint values.
        
        Args:
            joint_values: List of 6 joint values
            gripper: Gripper state (default: 0.0)
            timestamp: Optional timestamp (uses current time if None)
            
        Returns:
            JointState instance
            
        Raises:
            ValueError: If joint_values doesn't contain exactly 6 values
        """
        if len(joint_values) != 6:
            raise ValueError(f"Expected 6 joint values, got {len(joint_values)}")
        
        return cls(
            joint_0=float(joint_values[0]),
            joint_1=float(joint_values[1]),
            joint_2=float(joint_values[2]),
            joint_3=float(joint_values[3]),
            joint_4=float(joint_values[4]),
            joint_5=float(joint_values[5]),
            gripper=float(gripper),
            timestamp=timestamp or time.time()
        )
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'JointState':
        """
        Create JointState from dictionary.
        
        Args:
            data: Dictionary containing joint state data
            
        Returns:
            JointState instance
        """
        return cls(
            joint_0=float(data.get('joint_0', 0.0)),
            joint_1=float(data.get('joint_1', 0.0)),
            joint_2=float(data.get('joint_2', 0.0)),
            joint_3=float(data.get('joint_3', 0.0)),
            joint_4=float(data.get('joint_4', 0.0)),
            joint_5=float(data.get('joint_5', 0.0)),
            gripper=float(data.get('gripper', 0.0)),
            timestamp=float(data.get('timestamp', time.time()))
        )
    
    def to_list(self) -> List[float]:
        """
        Convert to list of joint values (excluding gripper and timestamp).
        
        Returns:
            List of 6 joint values
        """
        return [
            self.joint_0,
            self.joint_1, 
            self.joint_2,
            self.joint_3,
            self.joint_4,
            self.joint_5
        ]
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert to dictionary for serialization.
        
        Returns:
            Dictionary representation
        """
        return {
            'joint_0': self.joint_0,
            'joint_1': self.joint_1,
            'joint_2': self.joint_2,
            'joint_3': self.joint_3,
            'joint_4': self.joint_4,
            'joint_5': self.joint_5,
            'gripper': self.gripper,
            'timestamp': self.timestamp
        }
    
    def to_full_list(self) -> List[float]:
        """
        Convert to list including gripper state.
        
        Returns:
            List of [joint_0, joint_1, ..., joint_5, gripper]
        """
        return self.to_list() + [self.gripper]
    
    def copy(self) -> 'JointState':
        """Create a copy of this JointState."""
        return JointState(
            joint_0=self.joint_0,
            joint_1=self.joint_1,
            joint_2=self.joint_2,
            joint_3=self.joint_3,
            joint_4=self.joint_4,
            joint_5=self.joint_5,
            gripper=self.gripper,
            timestamp=self.timestamp
        )
    
    def apply_joint_values(self, joint_values: List[float]) -> 'JointState':
        """
        Create new JointState with updated joint values, preserving gripper and updating timestamp.
        
        Args:
            joint_values: New joint values
            
        Returns:
            New JointState with updated values
        """
        if len(joint_values) != 6:
            raise ValueError(f"Expected 6 joint values, got {len(joint_values)}")
        
        return JointState(
            joint_0=float(joint_values[0]),
            joint_1=float(joint_values[1]),
            joint_2=float(joint_values[2]),
            joint_3=float(joint_values[3]),
            joint_4=float(joint_values[4]),
            joint_5=float(joint_values[5]),
            gripper=self.gripper,  # Preserve gripper
            timestamp=time.time()  # Update timestamp
        )
    
    def get_age(self) -> float:
        """
        Get age of this joint state in seconds.
        
        Returns:
            Age in seconds since timestamp
        """
        return time.time() - self.timestamp
    
    def __str__(self) -> str:
        """String representation."""
        joints_str = ", ".join([f"{j:6.3f}" for j in self.to_list()])
        return f"JointState([{joints_str}], gripper={self.gripper:.3f}, age={self.get_age():.3f}s)"
    
    def __repr__(self) -> str:
        """Detailed string representation."""
        return (f"JointState(joint_0={self.joint_0:.3f}, joint_1={self.joint_1:.3f}, "
                f"joint_2={self.joint_2:.3f}, joint_3={self.joint_3:.3f}, "
                f"joint_4={self.joint_4:.3f}, joint_5={self.joint_5:.3f}, "
                f"gripper={self.gripper:.3f}, timestamp={self.timestamp})")