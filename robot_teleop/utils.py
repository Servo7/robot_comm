import yaml
import numpy as np
import logging
from typing import Dict, List, Tuple, Optional, Any, Union
from .joint_state import JointState

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def load_config(config_path: str) -> Dict[str, Any]:
    """
    Load configuration from YAML file.
    
    Args:
        config_path: Path to YAML configuration file
        
    Returns:
        Dictionary containing configuration
    """
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        logger.info(f"Loaded configuration from {config_path}")
        return config
    except Exception as e:
        logger.error(f"Failed to load config from {config_path}: {e}")
        raise


def validate_joint_limits(
    joint_values: Union[List[float], JointState],
    joint_limits: Dict[str, Dict[str, float]]
) -> Tuple[bool, List[str]]:
    """
    Validate joint values against configured limits.
    
    Args:
        joint_values: List of joint values or JointState to validate
        joint_limits: Dictionary containing min/max limits for each joint
        
    Returns:
        Tuple of (valid, violations) where valid is True if all joints are within limits
        and violations is a list of violation messages
    """
    violations = []
    
    # Convert JointState to list if needed
    if isinstance(joint_values, JointState):
        values_to_check = joint_values.to_list()
    else:
        values_to_check = joint_values
    
    for i, value in enumerate(values_to_check):
        joint_name = f"joint_{i}"
        
        if joint_name in joint_limits:
            min_val = joint_limits[joint_name].get('min', -np.pi)
            max_val = joint_limits[joint_name].get('max', np.pi)
            
            if value < min_val or value > max_val:
                violations.append(
                    f"{joint_name}: value {value:.3f} outside limits [{min_val:.3f}, {max_val:.3f}]"
                )
    
    return len(violations) == 0, violations


def transform_joints(
    joint_values: Union[List[float], JointState],
    transformation_matrix: Optional[np.ndarray] = None,
    joint_mapping: Optional[Dict[int, int]] = None,
    joint_offsets: Optional[List[float]] = None,
    gripper_scale: float = 1.0,
    gripper_offset: float = 0.0
) -> Union[List[float], Tuple[List[float], float]]:
    """
    Transform joint values between different robot configurations.
    
    Args:
        joint_values: Input joint values (List or JointState)
        transformation_matrix: Optional transformation matrix to apply
        joint_mapping: Optional mapping of joint indices (e.g., {0: 2, 1: 0, 2: 1} to reorder)
        joint_offsets: Optional offsets to add to each joint (e.g., [0.1, -0.2, 0.0, ...])
        gripper_scale: Scale factor for gripper (default 1.0)
        gripper_offset: Offset to add to gripper value (default 0.0)
        
    Returns:
        If input is List: returns transformed joint values as list
        If input is JointState: returns tuple of (transformed joints, transformed gripper)
    """
    # Convert JointState to list if needed
    is_joint_state = isinstance(joint_values, JointState)
    if is_joint_state:
        values_to_transform = joint_values.to_list()
        original_gripper = joint_values.gripper
    else:
        values_to_transform = joint_values
        original_gripper = None
    
    transformed = np.array(values_to_transform)
    
    # Apply joint mapping if provided
    if joint_mapping:
        mapped = np.zeros_like(transformed)
        for src_idx, dst_idx in joint_mapping.items():
            if src_idx < len(transformed) and dst_idx < len(mapped):
                mapped[dst_idx] = transformed[src_idx]
        transformed = mapped
    
    # Apply transformation matrix if provided
    if transformation_matrix is not None:
        if transformation_matrix.shape[0] == len(transformed):
            transformed = transformation_matrix @ transformed
        else:
            logger.warning(
                f"Transformation matrix shape {transformation_matrix.shape} doesn't match "
                f"joint count {len(transformed)}"
            )
    
    # Apply joint offsets if provided
    if joint_offsets is not None:
        offsets_array = np.array(joint_offsets)
        if len(offsets_array) == len(transformed):
            transformed = transformed + offsets_array
        else:
            logger.warning(
                f"Joint offsets length {len(offsets_array)} doesn't match "
                f"joint count {len(transformed)}"
            )
    
    transformed_joints = transformed.tolist()
    
    # If input was JointState, also transform gripper
    if is_joint_state:
        # Apply gripper transformation: scale then offset
        transformed_gripper = (original_gripper * gripper_scale) + gripper_offset
        # Clamp gripper to valid range [0, 1]
        transformed_gripper = max(0.0, min(1.0, transformed_gripper))
        return transformed_joints, transformed_gripper
    
    return transformed_joints 


def serialize_joint_message(joint_state: Union[JointState, List[float]], timestamp: Optional[float] = None) -> Dict:
    """
    Serialize joint state into a message dictionary.
    
    Args:
        joint_state: JointState object or list of joint values (for backward compatibility)
        timestamp: Optional timestamp (uses current time if not provided)
        
    Returns:
        Dictionary containing joint state and metadata
    """
    import time
    
    if isinstance(joint_state, JointState):
        # Use JointState's built-in serialization
        data = joint_state.to_dict()
        if timestamp is not None:
            data['timestamp'] = timestamp
        return data
    else:
        # Backward compatibility: convert list to JointState
        js = JointState.from_list(joint_state, timestamp=timestamp or time.time())
        return js.to_dict()


def deserialize_joint_message(message: Dict) -> Tuple[Union[JointState, List[float]], float]:
    """
    Deserialize a joint message dictionary.
    
    Args:
        message: Dictionary containing joint state and metadata
        
    Returns:
        Tuple of (JointState or joint_values, timestamp) depending on message format
    """
    # Check if this is a new format with individual joint fields
    if 'joint_0' in message:
        # New format: return JointState
        joint_state = JointState.from_dict(message)
        return joint_state, joint_state.timestamp
    else:
        # Old format: return list for backward compatibility
        return message.get('joints', []), message.get('timestamp', 0.0)