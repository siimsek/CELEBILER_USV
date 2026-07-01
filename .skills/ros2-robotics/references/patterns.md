# ROS2 Robotics

## Patterns

### **Node Structure**
#### **Name**
ROS2 Node Architecture
#### **Description**
Proper node structure with lifecycle management
#### **When**
Creating a new ROS2 node
#### **Pattern**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from typing import Optional
import threading

class RobotController(LifecycleNode):
    """
    Lifecycle-managed robot controller node.

    Lifecycle states: unconfigured -> inactive -> active -> finalized
    Transitions: configure, activate, deactivate, cleanup, shutdown

    Use lifecycle nodes for:
    - Controlled startup/shutdown sequences
    - Resource management
    - Coordinated system bringup
    """

    def __init__(self):
        super().__init__('robot_controller')

        # Declare parameters with defaults
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensor_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        # These will be created in on_configure
        self._laser_sub = None
        self._cmd_pub = None
        self._timer = None

        self.get_logger().info('Node created (unconfigured)')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure node resources."""
        try:
            # Get parameters
            self._max_speed = self.get_parameter('max_speed').value
            sensor_topic = self.get_parameter('sensor_topic').value
            cmd_topic = self.get_parameter('cmd_topic').value

            # Create QoS profile for sensors
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            # Create subscribers and publishers
            self._laser_sub = self.create_subscription(
                LaserScan,
                sensor_topic,
                self._laser_callback,
                sensor_qos
            )

            self._cmd_pub = self.create_publisher(
                Twist,
                cmd_topic,
                10
            )

            self.get_logger().info('Configured successfully')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Configuration failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate node - start processing."""
        self._timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info('Activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate node - stop processing."""
        if self._timer:
            self._timer.cancel()
            self._timer = None
        # Stop robot
        self._publish_stop()
        self.get_logger().info('Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clean up resources."""
        self._laser_sub = None
        self._cmd_pub = None
        self.get_logger().info('Cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown node."""
        self._publish_stop()
        self.get_logger().info('Shut down')
        return TransitionCallbackReturn.SUCCESS

    def _laser_callback(self, msg: LaserScan):
        """Process laser scan data."""
        # Process sensor data
        pass

    def _control_loop(self):
        """Main control loop."""
        cmd = Twist()
        # Compute control
        self._cmd_pub.publish(cmd)

    def _publish_stop(self):
        """Send stop command."""
        if self._cmd_pub:
            cmd = Twist()
            self._cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **Service Client Pattern**
#### **Name**
Async Service Client
#### **Description**
Proper async service client with timeout and error handling
#### **Pattern**
```python
from rclpy.node import Node
from std_srvs.srv import SetBool
from rclpy.callback_groups import ReentrantCallbackGroup
import asyncio

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client')

        # Use reentrant callback group for async operations
        self._callback_group = ReentrantCallbackGroup()

        self._client = self.create_client(
            SetBool,
            '/enable_robot',
            callback_group=self._callback_group
        )

        # Wait for service with timeout
        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available')

    async def call_service_async(self, enable: bool) -> bool:
        """Call service with timeout and error handling."""
        if not self._client.service_is_ready():
            self.get_logger().warning('Service not ready')
            return False

        request = SetBool.Request()
        request.data = enable

        try:
            # Call with timeout
            future = self._client.call_async(request)
            response = await asyncio.wait_for(
                asyncio.wrap_future(future),
                timeout=5.0
            )
            return response.success

        except asyncio.TimeoutError:
            self.get_logger().error('Service call timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return False
```

### **QoS Profile Pattern**
#### **Name**
QoS Profile Selection
#### **Description**
Choose appropriate QoS for different use cases
#### **Pattern**
```python
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy,
    HistoryPolicy, qos_profile_sensor_data, qos_profile_services_default
)

# Sensor data (camera, lidar, IMU)
# - BEST_EFFORT: Don't block on lost packets
# - VOLATILE: Don't need old messages
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
# Or use predefined:
# qos_profile_sensor_data

# Commands (cmd_vel, setpoint)
# - RELIABLE: Must deliver
# - TRANSIENT_LOCAL: New subscribers get last message
cmd_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# State/latched topics (map, robot state)
# - RELIABLE + TRANSIENT_LOCAL
# - New subscribers get latest state
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Services (default)
# qos_profile_services_default
```

### **Transform Management**
#### **Name**
TF2 Transform Handling
#### **Description**
Proper TF2 usage with buffering and error handling
#### **Pattern**
```python
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
import tf_transformations

class TransformManager(Node):
    def __init__(self):
        super().__init__('transform_manager')

        # Broadcaster for publishing transforms
        self._broadcaster = TransformBroadcaster(self)

        # Static broadcaster for fixed transforms
        self._static_broadcaster = StaticTransformBroadcaster(self)

        # Transform listener with buffer
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publish static transforms once
        self._publish_static_transforms()

    def _publish_static_transforms(self):
        """Publish transforms that never change."""
        # Sensor mount offset
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_link'
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        t.transform.rotation.w = 1.0

        self._static_broadcaster.sendTransform(t)

    def lookup_transform_safe(
        self,
        target_frame: str,
        source_frame: str,
        timeout_sec: float = 1.0
    ):
        """Safely look up transform with error handling."""
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec)
            )
            return transform
        except TransformException as e:
            self.get_logger().warning(
                f'Could not get transform {source_frame} -> {target_frame}: {e}'
            )
            return None
```

## Anti-Patterns

### **Topic Name Typo**
#### **Problem**
Mistyped topic name causes silent failure - no error
#### **Solution**
Use constants, remapping, and check with ros2 topic list

### **Blocking Callbacks**
#### **Problem**
Long operations in callback block executor
#### **Solution**
Use async callbacks or separate threads

### **QoS Mismatch**
#### **Problem**
Incompatible QoS causes no connection, no error
#### **Solution**
Check compatibility, use ros2 topic info --verbose
