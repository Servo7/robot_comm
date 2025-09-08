import zmq
import json
import logging
import time
from typing import List, Optional, Tuple, Union
from threading import Thread, Lock
from .utils import deserialize_joint_message
from .joint_state import JointState

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class FollowerNode:
    """
    Follower node that subscribes to processed commands.
    Runs on the laptop controlling the follower robot.
    """
    
    def __init__(self, 
                 subscribe_port: int = 5556,
                 subscribe_address: str = "localhost",
                 topic: str = "follower_commands"):
        """
        Initialize the follower node.
        
        Args:
            subscribe_port: Port to subscribe for commands
            subscribe_address: Address of the master node
            topic: Topic name to subscribe to
        """
        self.subscribe_port = subscribe_port
        self.subscribe_address = subscribe_address
        self.topic = topic
        
        # Latest joint state
        self.latest_joint_state = None
        self.lock = Lock()
        
        # Setup ZMQ subscriber
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        
        connect_address = f"tcp://{subscribe_address}:{subscribe_port}"
        self.subscriber.connect(connect_address)
        
        # Subscribe to specific topic
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, topic)
        
        logger.info(f"Follower node subscribing to {connect_address} on topic '{topic}'")
        
        # Start background thread for receiving
        self.running = False
        self.receive_thread = None
    
    def start(self):
        """Start the background receiving thread."""
        if not self.running:
            self.running = True
            self.receive_thread = Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            logger.info("Follower node started")
    
    def stop(self):
        """Stop the background receiving thread."""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        logger.info("Follower node stopped")
    
    def _receive_loop(self):
        """Background thread for receiving messages."""
        while self.running:
            try:
                # Use polling to allow clean shutdown
                if self.subscriber.poll(timeout=100):  # 100ms timeout
                    message = self.subscriber.recv_string()
                    
                    # Remove topic prefix
                    if message.startswith(self.topic):
                        json_data = message[len(self.topic):].strip()
                        data = json.loads(json_data)
                        
                        joint_data, timestamp = deserialize_joint_message(data)
                        
                        # Convert to JointState if it's a list (backward compatibility)
                        if isinstance(joint_data, list):
                            joint_state = JointState.from_list(joint_data, timestamp=timestamp)
                        else:
                            joint_state = joint_data
                        
                        with self.lock:
                            self.latest_joint_state = joint_state
                        
                        logger.debug(f"Received joint state: {joint_state}")
                        
            except zmq.ZMQError as e:
                if self.running:
                    logger.error(f"ZMQ error in receive loop: {e}")
            except json.JSONDecodeError as e:
                logger.error(f"Failed to decode message: {e}")
            except Exception as e:
                logger.error(f"Error in receive loop: {e}")
    
    def get_latest_joint_state(self) -> Optional[JointState]:
        """
        Get the latest received joint state.
        
        Returns:
            Latest JointState or None if no data received yet
        """
        with self.lock:
            return self.latest_joint_state.copy() if self.latest_joint_state else None
    
    def get_latest_joints(self) -> Optional[List[float]]:
        """
        Get the latest received joint values (backward compatibility).
        
        Returns:
            Latest joint values or None if no data received yet
        """
        joint_state = self.get_latest_joint_state()
        return joint_state.to_list() if joint_state else None
    
    def get_latest_joints_with_timestamp(self) -> Tuple[Optional[List[float]], Optional[float]]:
        """
        Get the latest received joint values with timestamp (backward compatibility).
        
        Returns:
            Tuple of (joint_values, timestamp) or (None, None) if no data
        """
        joint_state = self.get_latest_joint_state()
        if joint_state:
            return joint_state.to_list(), joint_state.timestamp
        return None, None
    
    def get_latest_joint_state_with_gripper(self) -> Optional[JointState]:
        """
        Get the latest received joint state with gripper information.
        
        Returns:
            Latest JointState with gripper or None if no data
        """
        return self.get_latest_joint_state()
    
    def wait_for_joint_state(self, timeout: float = 5.0) -> Optional[JointState]:
        """
        Wait for joint state to be received.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            JointState or None if timeout
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            joint_state = self.get_latest_joint_state()
            if joint_state is not None:
                return joint_state
            time.sleep(0.01)
        
        logger.warning(f"Timeout waiting for joint state after {timeout}s")
        return None
    
    def wait_for_joints(self, timeout: float = 5.0) -> Optional[List[float]]:
        """
        Wait for joint values to be received (backward compatibility).
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            Joint values or None if timeout
        """
        joint_state = self.wait_for_joint_state(timeout)
        return joint_state.to_list() if joint_state else None
    
    def subscribe_with_callback(self, callback, rate: float = 100.0, use_joint_state: bool = True):
        """
        Subscribe with a callback function that's called for each received message.
        
        Args:
            callback: Function called with (JointState) if use_joint_state=True, 
                     or (joint_values, timestamp) if use_joint_state=False
            rate: Maximum callback rate in Hz
            use_joint_state: If True, pass JointState to callback; if False, pass (joints, timestamp)
        """
        period = 1.0 / rate
        last_timestamp = None
        
        logger.info(f"Starting callback subscription at max {rate} Hz (joint_state={use_joint_state})")
        
        try:
            while True:
                joint_state = self.get_latest_joint_state()
                
                if joint_state is not None and joint_state.timestamp != last_timestamp:
                    if use_joint_state:
                        callback(joint_state)
                    else:
                        # Backward compatibility
                        callback(joint_state.to_list(), joint_state.timestamp)
                    last_timestamp = joint_state.timestamp
                
                time.sleep(period)
                
        except KeyboardInterrupt:
            logger.info("Callback subscription interrupted")
    
    def close(self):
        """Clean up ZMQ resources."""
        self.stop()
        self.subscriber.close()
        self.context.term()
        logger.info("Follower node closed")