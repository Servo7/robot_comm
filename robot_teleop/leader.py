import zmq
import json
import logging
import time
from typing import List, Optional, Union
from .utils import serialize_joint_message
from .joint_state import JointState

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class LeaderNode:
    """
    Leader node that publishes joint states from the leader robot.
    Runs on the Jetson with lerobot.
    """
    
    def __init__(self, 
                 publish_port: int = 5555,
                 publish_address: str = "*",
                 topic: str = "leader_joints"):
        """
        Initialize the leader node.
        
        Args:
            publish_port: Port to publish joint states on
            publish_address: Address to bind to (default "*" for all interfaces)
            topic: Topic name for messages
        """
        self.publish_port = publish_port
        self.publish_address = publish_address
        self.topic = topic
        
        # Setup ZMQ publisher
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)
        
        bind_address = f"tcp://{publish_address}:{publish_port}"
        self.publisher.bind(bind_address)
        
        logger.info(f"Leader node publishing on {bind_address}")
        
        # Give socket time to initialize
        time.sleep(0.1)
    
    def publish_joints(self, joint_state: Union[JointState, List[float]], gripper: float = 0.0, timestamp: Optional[float] = None):
        """
        Publish joint state to subscribers.
        
        Args:
            joint_state: JointState object or list of joint values
            gripper: Gripper state if joint_state is a list (ignored if joint_state is JointState)
            timestamp: Optional timestamp (uses current time if not provided)
        """
        try:
            # Convert list to JointState if needed
            if isinstance(joint_state, list):
                js = JointState.from_list(joint_state, gripper, timestamp)
            else:
                js = joint_state
                if timestamp is not None:
                    js.timestamp = timestamp
            
            message = serialize_joint_message(js)
            
            # Create topic-prefixed message for ZMQ pub/sub filtering
            topic_message = f"{self.topic} {json.dumps(message)}"
            self.publisher.send_string(topic_message)
            
            logger.debug(f"Published joint state: {js}")
            
        except Exception as e:
            logger.error(f"Failed to publish joint state: {e}")
    
    def publish_loop(self, get_joints_callback, gripper: float = 0,  rate: float = 100.0):
        """
        Continuous publishing loop.
        
        Args:
            get_joints_callback: Function that returns JointState or list of joint values
            rate: Publishing rate in Hz
        """
        period = 1.0 / rate
        logger.info(f"Starting publish loop at {rate} Hz")
        
        try:
            while True:
                start_time = time.time()
                
                # Get current joint state from callback
                joint_data, gripper = get_joints_callback()
                
                if joint_data is not None:
                    self.publish_joints(joint_data, gripper)
                
                # Maintain rate
                elapsed = time.time() - start_time
                if elapsed < period:
                    time.sleep(period - elapsed)
                    
        except KeyboardInterrupt:
            logger.info("Publish loop interrupted")
        finally:
            self.close()
    
    def close(self):
        """Clean up ZMQ resources."""
        self.publisher.close()
        self.context.term()
        logger.info("Leader node closed")