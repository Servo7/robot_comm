"""
Pytest configuration and shared fixtures.
"""

import pytest
import time
import tempfile
import os
from typing import Generator


@pytest.fixture
def sample_joint_limits() -> dict:
    """Sample joint limits configuration for testing."""
    return {
        'joint_0': {'min': -1.0, 'max': 1.0, 'name': 'Base rotation'},
        'joint_1': {'min': -0.5, 'max': 0.5, 'name': 'Shoulder pitch'},
        'joint_2': {'min': -1.5708, 'max': 1.5708, 'name': 'Elbow pitch'},
        'joint_3': {'min': -3.14159, 'max': 3.14159, 'name': 'Wrist roll'},
        'joint_4': {'min': -0.3, 'max': 0.3, 'name': 'Wrist pitch'},
        'joint_5': {'min': -3.14159, 'max': 3.14159, 'name': 'Wrist yaw'},
    }


@pytest.fixture
def temp_config_file(sample_joint_limits) -> Generator[str, None, None]:
    """Create a temporary configuration file for testing."""
    config_content = f"""
joint_limits:
  joint_0:
    min: {sample_joint_limits['joint_0']['min']}
    max: {sample_joint_limits['joint_0']['max']}
    name: "{sample_joint_limits['joint_0']['name']}"
  joint_1:
    min: {sample_joint_limits['joint_1']['min']}
    max: {sample_joint_limits['joint_1']['max']}
    name: "{sample_joint_limits['joint_1']['name']}"
  joint_2:
    min: {sample_joint_limits['joint_2']['min']}
    max: {sample_joint_limits['joint_2']['max']}
    name: "{sample_joint_limits['joint_2']['name']}"
  joint_3:
    min: {sample_joint_limits['joint_3']['min']}
    max: {sample_joint_limits['joint_3']['max']}
    name: "{sample_joint_limits['joint_3']['name']}"
  joint_4:
    min: {sample_joint_limits['joint_4']['min']}
    max: {sample_joint_limits['joint_4']['max']}
    name: "{sample_joint_limits['joint_4']['name']}"
  joint_5:
    min: {sample_joint_limits['joint_5']['min']}
    max: {sample_joint_limits['joint_5']['max']}
    name: "{sample_joint_limits['joint_5']['name']}"

# Optional joint mapping for testing transformations
joint_mapping:
  0: 0
  1: 1
  2: 2
  3: 3
  4: 4
  5: 5

# Optional transformation matrix (identity matrix for testing)
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
    
    yield temp_path
    
    # Cleanup
    try:
        os.unlink(temp_path)
    except OSError:
        pass  # File might already be deleted


@pytest.fixture
def restrictive_config_file() -> Generator[str, None, None]:
    """Create a configuration file with very restrictive limits for testing blocking."""
    config_content = """
joint_limits:
  joint_0:
    min: -0.1
    max: 0.1
    name: "Base rotation - VERY RESTRICTIVE"
  joint_1:
    min: -0.05
    max: 0.05
    name: "Shoulder pitch - VERY RESTRICTIVE"
  joint_2:
    min: -0.2
    max: 0.2
    name: "Elbow pitch - RESTRICTIVE"
  joint_3:
    min: -0.1
    max: 0.1
    name: "Wrist roll - VERY RESTRICTIVE"
  joint_4:
    min: -0.05
    max: 0.05
    name: "Wrist pitch - VERY RESTRICTIVE"
  joint_5:
    min: -0.1
    max: 0.1
    name: "Wrist yaw - VERY RESTRICTIVE"
"""
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        f.write(config_content)
        temp_path = f.name
    
    yield temp_path
    
    # Cleanup
    try:
        os.unlink(temp_path)
    except OSError:
        pass


def pytest_configure(config):
    """Configure pytest with custom markers."""
    config.addinivalue_line(
        "markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')"
    )
    config.addinivalue_line(
        "markers", "integration: marks tests as integration tests"
    )
    config.addinivalue_line(
        "markers", "unit: marks tests as unit tests"
    )


def pytest_collection_modifyitems(config, items):
    """Automatically mark tests based on their location/name."""
    for item in items:
        # Mark integration tests
        if "test_integration" in item.nodeid:
            item.add_marker(pytest.mark.integration)
        
        # Mark unit tests
        if any(name in item.nodeid for name in ["test_joint_state", "test_utils"]):
            item.add_marker(pytest.mark.unit)
        
        # Mark slow tests (integration tests are typically slow)
        if "test_integration" in item.nodeid:
            item.add_marker(pytest.mark.slow)