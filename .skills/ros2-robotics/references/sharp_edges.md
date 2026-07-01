# Ros2 Robotics - Sharp Edges

## Topic Name Typo Causes Silent Failure

### **Severity**: critical
### **Summary**
Mistyped topic name creates no connection and no error
### **Symptoms**
- Publisher publishes but subscriber receives nothing
- Node appears to work but no data flows
- ros2 topic list shows separate topics
### **Why**
ROS2 doesn't validate topic names at compile or launch time.
If you type '/odom' in one place and '\odom' in another,
you get two separate topics with no warning.

Common typos:
- Missing leading slash: 'cmd_vel' vs '/cmd_vel'
- Underscore vs no underscore: 'laser_scan' vs 'laserscan'
- Namespace issues: '/robot1/cmd_vel' vs '/cmd_vel'

This is the #1 debugging issue for ROS beginners.

### **Solution**
```python
# Use constants for topic names
class Topics:
    CMD_VEL = '/cmd_vel'
    ODOM = '/odom'
    SCAN = '/scan'

self.pub = self.create_publisher(Twist, Topics.CMD_VEL, 10)
self.sub = self.create_subscription(Twist, Topics.CMD_VEL, callback, 10)

# Use remapping in launch files for flexibility
# Verify with: ros2 topic list
# Check connections: ros2 topic info /topic_name --verbose
```

## QoS Mismatch Causes Silent Connection Failure

### **Severity**: critical
### **Summary**
Publisher and subscriber QoS incompatibility prevents connection
### **Symptoms**
- ros2 topic list shows topic exists
- ros2 topic info shows both pub and sub
- No data flows, no error messages
### **Why**
ROS2 enforces QoS compatibility at connection time.
If publisher offers less than subscriber requires, no connection.

Most common mismatch:
- Publisher: BEST_EFFORT (sensor default)
- Subscriber: RELIABLE (default for many packages)

Result: Silent failure, no error message.

### **Solution**
```python
# Check QoS with verbose info
# ros2 topic info /scan --verbose

# Match publisher's QoS
from rclpy.qos import qos_profile_sensor_data
self.sub = self.create_subscription(
    LaserScan, '/scan', callback,
    qos_profile_sensor_data  # Matches sensor publishers
)

# Or explicitly set compatible QoS
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

## Blocking Operations in Callbacks

### **Severity**: high
### **Summary**
Long operations block all other callbacks
### **Why**
ROS2 executors are single-threaded by default.
If one callback takes 1 second, ALL other callbacks wait.

This means:
- 100Hz control loop becomes 1Hz
- Watchdogs trigger
- Robot stops responding

### **Solution**
```python
# Option 1: Use callback groups
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, MultiThreadedExecutor

self._sensor_group = MutuallyExclusiveCallbackGroup()
self._compute_group = MutuallyExclusiveCallbackGroup()

self.sub = self.create_subscription(
    ..., callback_group=self._sensor_group
)
self.timer = self.create_timer(
    0.01, self.control_callback,
    callback_group=self._compute_group
)

# Use MultiThreadedExecutor
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()

# Option 2: Async callbacks (ROS2 Humble+)
async def callback(self, msg):
    result = await asyncio.to_thread(self.expensive_computation, msg)
```

## Transform Lookup Without Timeout

### **Severity**: high
### **Summary**
Missing TF transform blocks forever
### **Why**
lookup_transform with default timeout blocks forever if
the transform doesn't exist.

### **Solution**
```python
# Always use timeout
try:
    transform = self.tf_buffer.lookup_transform(
        'map', 'base_link',
        rclpy.time.Time(),
        timeout=rclpy.duration.Duration(seconds=1.0)
    )
except TransformException as e:
    self.get_logger().warning(f'Transform failed: {e}')
    return  # Handle gracefully

# Check if transform exists before lookup
if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
    transform = self.tf_buffer.lookup_transform(...)
```

## Using Parameters Without Declaration

### **Severity**: medium
### **Summary**
get_parameter fails if parameter not declared first

### **Solution**
```python
# Declare parameter first
self.declare_parameter('max_speed', 1.0)  # With default
speed = self.get_parameter('max_speed').value

# Or declare without default (requires YAML)
self.declare_parameter('max_speed')

# Bulk declaration
self.declare_parameters('', [
    ('max_speed', 1.0),
    ('min_speed', 0.1),
    ('topic_name', '/cmd_vel')
])
```

## Transform Lookup at Time Zero

### **Severity**: high
### **Summary**
Requesting latest transform can get stale data

### **Solution**
```python
# Use message timestamp for transform lookup
def sensor_callback(self, msg):
    try:
        # Use sensor message timestamp
        transform = self.tf_buffer.lookup_transform(
            'map',
            msg.header.frame_id,
            msg.header.stamp,  # Sensor timestamp, not Time(0)
            timeout=Duration(seconds=0.1)
        )
    except TransformException as e:
        self.get_logger().warning(f'TF lookup failed: {e}')
```
