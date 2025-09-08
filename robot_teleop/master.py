import zmq
import json
import logging
import time
import numpy as np
from typing import Dict, List, Optional, Any
from .utils import (
    deserialize_joint_message,
    serialize_joint_message,
    validate_joint_limits,
    transform_joints,
    load_config
)
from .joint_state import JointState

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MasterNode:
    """
    Master node that subscribes to leader, transforms data, and publishes to follower.
    Can run anywhere and acts as the safety and transformation layer.
    """
    
    def __init__(self,
                 subscribe_port: int = 5555,
                 subscribe_address: str = "localhost",
                 publish_port: int = 5556,
                 publish_address: str = "*",
                 subscribe_topic: str = "leader_joints",
                 publish_topic: str = "follower_commands",
                 joint_limits: Optional[Dict[str, Any]] = None,
                 config_path: Optional[str] = None):
        """
        Initialize the master node.
        
        Args:
            subscribe_port: Port to subscribe to leader joints
            subscribe_address: Address of the leader node
            publish_port: Port to publish follower commands
            publish_address: Address to bind publisher to
            subscribe_topic: Topic to subscribe to from leader
            publish_topic: Topic to publish for follower
            joint_limits: Dictionary of joint limits or None
            config_path: Optional path to configuration file
        """
        self.subscribe_port = subscribe_port
        self.subscribe_address = subscribe_address
        self.publish_port = publish_port
        self.publish_address = publish_address
        self.subscribe_topic = subscribe_topic
        self.publish_topic = publish_topic
        
        # Load configuration
        if config_path:
            config = load_config(config_path)
            self.joint_limits = config.get('joint_limits', {})
            self.joint_mapping = config.get('joint_mapping')
            self.transformation_matrix = config.get('transformation_matrix')
            self.joint_offsets = config.get('joint_offsets')
            if self.transformation_matrix:
                self.transformation_matrix = np.array(self.transformation_matrix)
        else:
            self.joint_limits = joint_limits or {}
            self.joint_mapping = None
            self.transformation_matrix = None
            self.joint_offsets = None
        
        # Statistics
        self.messages_received = 0
        self.messages_published = 0
        self.messages_blocked = 0
        
        # Setup ZMQ
        self.context = zmq.Context()
        
        # Subscriber for leader
        self.subscriber = self.context.socket(zmq.SUB)
        connect_address = f"tcp://{subscribe_address}:{subscribe_port}"
        self.subscriber.connect(connect_address)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, subscribe_topic)
        logger.info(f"Master subscribing to {connect_address} on topic '{subscribe_topic}'")
        
        # Publisher for follower
        self.publisher = self.context.socket(zmq.PUB)
        bind_address = f"tcp://{publish_address}:{publish_port}"
        self.publisher.bind(bind_address)
        logger.info(f"Master publishing on {bind_address}")
        
        # Give sockets time to initialize
        time.sleep(0.1)
        
        self.running = False
    
    def process_joint_state(self, joint_state: JointState) -> Optional[JointState]:
        """
        Process joint state: transform and validate.
        
        Args:
            joint_state: Input JointState from leader
            
        Returns:
            Processed JointState or None if validation fails
        """
        # Apply transformation to joint values
        transformed_joints = transform_joints(
            joint_state,
            self.transformation_matrix,
            self.joint_mapping,
            self.joint_offsets
        )
        
        # Create new JointState with transformed joints
        transformed_state = joint_state.apply_joint_values(transformed_joints)
        
        # Validate limits
        if self.joint_limits:
            valid, violations = validate_joint_limits(transformed_state, self.joint_limits)
            
            if not valid:
                logger.warning(f"Joint limits violated, blocking message:")
                for violation in violations:
                    logger.warning(f"  - {violation}")
                self.messages_blocked += 1
                return None
        
        return transformed_state
    
    def start(self):
        """Start the master node processing loop."""
        self.running = True
        logger.info("Master node starting...")
        
        try:
            while self.running:
                try:
                    # Poll for messages with timeout
                    if self.subscriber.poll(timeout=100):  # 100ms timeout
                        message = self.subscriber.recv_string()
                        
                        # Remove topic prefix
                        if message.startswith(self.subscribe_topic):
                            json_data = message[len(self.subscribe_topic):].strip()
                            data = json.loads(json_data)
                            
                            joint_data, timestamp = deserialize_joint_message(data)
                            self.messages_received += 1
                            
                            # Convert to JointState if it's a list (backward compatibility)
                            if isinstance(joint_data, list):
                                joint_state = JointState.from_list(joint_data, timestamp=timestamp)
                            else:
                                joint_state = joint_data
                            
                            logger.debug(f"Received joint state from leader: {joint_state}")
                            
                            # Process joint state
                            processed_state = self.process_joint_state(joint_state)
                            
                            if processed_state is not None:
                                # Publish to follower
                                follower_message = serialize_joint_message(processed_state)
                                topic_message = f"{self.publish_topic} {json.dumps(follower_message)}"
                                self.publisher.send_string(topic_message)
                                
                                self.messages_published += 1
                                logger.debug(f"Published joint state to follower: {processed_state}")
                            
                            # Log statistics periodically
                            if self.messages_received % 100 == 0:
                                self.log_statistics()
                                
                except zmq.ZMQError as e:
                    if self.running:
                        logger.error(f"ZMQ error: {e}")
                except json.JSONDecodeError as e:
                    logger.error(f"Failed to decode message: {e}")
                except Exception as e:
                    logger.error(f"Error in processing loop: {e}")
                    
        except KeyboardInterrupt:
            logger.info("Master node interrupted")
        finally:
            self.close()
    
    def log_statistics(self):
        """Log processing statistics."""
        logger.info(
            f"Statistics - Received: {self.messages_received}, "
            f"Published: {self.messages_published}, "
            f"Blocked: {self.messages_blocked} "
            f"({100.0 * self.messages_blocked / max(1, self.messages_received):.1f}%)"
        )
    
    def stop(self):
        """Stop the master node."""
        self.running = False
        logger.info("Master node stopping...")
    
    def close(self):
        """Clean up ZMQ resources."""
        self.log_statistics()
        self.subscriber.close()
        self.publisher.close()
        self.context.term()
        logger.info("Master node closed")