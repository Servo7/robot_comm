# Robot Teleoperation Library

Python library for master-follower robot teleoperation using ZMQ messaging. This system enables safe, real-time communication between leader and follower robots with built-in safety checks and transformation capabilities.

## Features

- **ZMQ Pub/Sub Architecture**: Reliable network communication between robots
- **Safety Validation**: Configurable joint limits with automatic blocking of unsafe commands
- **Joint Transformation**: Support for mapping and transforming joints between different robot models
- **Modular Design**: Easy integration with existing robot control systems
- **High Performance**: Supports up to 100Hz control loops
- **Clear Logging**: Comprehensive logging of messages, violations, and statistics

## Architecture

```
Leader Robot → [Publish Joint States] → Master Node → [Transform & Validate] → [Publish Commands] → Follower Robot
   (Jetson)                                (Anywhere)                                               (Laptop)
```

### Components

1. **Leader Node**: Publishes joint states from the leader robot (runs on Jetson with lerobot)
2. **Master Node**: Subscribes to leader, transforms data, validates safety, publishes to follower
3. **Follower Node**: Subscribes to processed commands (runs on laptop controlling follower robot)

## Installation

### Prerequisites

- Python 3.7+
- Network connectivity between machines

### Install from source

```bash
# Clone the repository
git clone https://github.com/yourusername/robot-teleop.git
cd robot-teleop

# Install dependencies
pip install -r requirements.txt

# Install the package
pip install -e .
```

## Quick Start

### 1. Configure Joint Limits

Edit `config/joint_limits.yaml` to set safe operating ranges for your robot:

```yaml
joint_limits:
  joint_0:
    min: -3.14159  # -π
    max: 3.14159   # π
    name: "Base rotation"
```

### 2. Start the Master Node

The master node provides safety validation and transformation:

```bash
python examples/run_master.py --leader-address <JETSON_IP> --leader-port 5555
```

### 3. Start the Leader Node (on Jetson)

```bash
# With simulated data for testing
python examples/run_leader.py --simulate --rate 100

# In production (integrate with your robot)
python examples/run_leader.py --address 0.0.0.0 --port 5555
```

### 4. Start the Follower Node (on Laptop)

```bash
python examples/run_follower.py --address <MASTER_IP> --port 5556
```

## API Usage

### Leader Node (Publishing Joint States)

```python
from robot_teleop import LeaderNode

# Create leader node
leader = LeaderNode(publish_port=5555)

# Publish joint values
joint_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
leader.publish_joints(joint_values)

# Or use continuous publishing with callback
def get_robot_joints():
    # Read from your robot hardware
    return robot.get_joint_positions()

leader.publish_loop(get_robot_joints, rate=100.0)
```

### Master Node (Safety & Transformation)

```python
from robot_teleop import MasterNode

# Create master with configuration
master = MasterNode(
    subscribe_port=5555,
    subscribe_address="192.168.1.100",  # Leader IP
    publish_port=5556,
    config_path="config/joint_limits.yaml"
)

# Start processing
master.start()
```

### Follower Node (Receiving Commands)

```python
from robot_teleop import FollowerNode

# Create follower node
follower = FollowerNode(subscribe_port=5556)

# Start receiving in background
follower.start()

# Get latest joint values
joints = follower.get_latest_joints()
if joints:
    robot.set_joint_positions(joints)

# Or use callback mode
def handle_joints(joint_values, timestamp):
    robot.set_joint_positions(joint_values)

follower.subscribe_with_callback(handle_joints, rate=100.0)
```

## Configuration

### Joint Limits

The `config/joint_limits.yaml` file defines safety limits for each joint:

```yaml
joint_limits:
  joint_0:
    min: -3.14159
    max: 3.14159
    name: "Base rotation"
```

### Joint Mapping

Optionally remap joints between different robot configurations:

```yaml
joint_mapping:
  0: 2  # Leader joint 0 → Follower joint 2
  1: 0  # Leader joint 1 → Follower joint 0
  2: 1  # Leader joint 2 → Follower joint 1
```

### Transformation Matrix

Apply linear transformations to joint values:

```yaml
transformation_matrix:
  - [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
  # ... 6x6 matrix
```

## Network Configuration

### Running on Same Machine (Testing)

```bash
# Terminal 1: Leader
python examples/run_leader.py --simulate

# Terminal 2: Master
python examples/run_master.py

# Terminal 3: Follower
python examples/run_follower.py
```

### Running Across Network

```bash
# On Jetson (Leader)
python examples/run_leader.py --address 0.0.0.0 --port 5555

# On Server (Master)
python examples/run_master.py \
    --leader-address 192.168.1.100 \
    --leader-port 5555 \
    --follower-address 0.0.0.0 \
    --follower-port 5556

# On Laptop (Follower)
python examples/run_follower.py \
    --address 192.168.1.200 \
    --port 5556
```

## Integration with LeRobot

On the Jetson with lerobot:

```python
from robot_teleop import LeaderNode
from lerobot import Robot

# Initialize robot and leader node
robot = Robot()
leader = LeaderNode(publish_port=5555, publish_address="0.0.0.0")

# Publish loop
def get_joints():
    return robot.get_joint_positions()

leader.publish_loop(get_joints, rate=100.0)
```

## Safety Features

1. **Joint Limit Validation**: Automatically blocks commands exceeding configured limits
2. **Message Logging**: Tracks blocked messages with detailed violation reasons
3. **Statistics Tracking**: Monitor message flow and blocking rate
4. **Timestamp Validation**: Track message age for latency monitoring

## Troubleshooting

### No Messages Received

1. Check network connectivity: `ping <IP_ADDRESS>`
2. Verify ports are not blocked by firewall
3. Ensure topic names match between nodes
4. Check that master node is running

### Joint Limits Blocking All Messages

1. Review `config/joint_limits.yaml` settings
2. Check master node logs for specific violations
3. Temporarily disable limits with `--no-limits` flag (not recommended for production)

### High Latency

1. Reduce publishing rate if network is saturated
2. Check network quality between machines
3. Consider running master node closer to follower

## License

MIT License - See LICENSE file for details

## Contributing

Contributions are welcome! Please submit pull requests or open issues for bugs and feature requests.
