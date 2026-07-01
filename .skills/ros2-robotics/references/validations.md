# Ros2 Robotics - Validations

## Parameter Usage Without Declaration

### **Severity**: warning
### **Type**: regex
### **Pattern**
- get_parameter\(['"][^'"]+['"]\)(?![\s\S]{0,200}declare_parameter)
### **Message**
Declare parameters before using them to avoid ParameterNotDeclaredException.
### **Fix Action**
Add self.declare_parameter('name', default_value) in __init__

## Transform Lookup Without Timeout

### **Severity**: warning
### **Type**: regex
### **Pattern**
- lookup_transform\([^)]*\)(?![\s\S]{0,50}timeout)
### **Message**
Add timeout to lookup_transform to avoid blocking forever.
### **Fix Action**
Add timeout=Duration(seconds=1.0) parameter

## Potential Blocking Operation in Callback

### **Severity**: info
### **Type**: regex
### **Pattern**
- def.*callback.*:\s*[^}]*time\.sleep
- def.*callback.*:\s*[^}]*requests\.
- def.*callback.*:\s*[^}]*input\(
### **Message**
Avoid blocking operations in callbacks. Use async or separate threads.

## Hardcoded Topic Name (No Constant)

### **Severity**: info
### **Type**: regex
### **Pattern**
- create_publisher\([^,]+,\s*['"]/
- create_subscription\([^,]+,\s*['"]/
### **Message**
Consider using constants or parameters for topic names.

## Default QoS Used for Sensor Subscription

### **Severity**: info
### **Type**: regex
### **Pattern**
- create_subscription\(.*Scan.*,\s*['"][^'"]+['"],\s*\w+,\s*\d+\)
- create_subscription\(.*Image.*,\s*['"][^'"]+['"],\s*\w+,\s*\d+\)
### **Message**
Use sensor QoS profile for sensor topics (BEST_EFFORT reliability).
### **Fix Action**
Use qos_profile_sensor_data or custom BEST_EFFORT profile

## Transform Operations Without Exception Handling

### **Severity**: warning
### **Type**: regex
### **Pattern**
- lookup_transform(?![\s\S]{0,100}try:|except)
- tf_buffer\.transform(?![\s\S]{0,100}try:|except)
### **Message**
Wrap TF operations in try/except TransformException.

## Lifecycle Node Missing Transition Callbacks

### **Severity**: info
### **Type**: regex
### **Pattern**
- class.*LifecycleNode(?![\s\S]{0,500}on_configure)
### **Message**
Implement lifecycle transition callbacks (on_configure, on_activate, etc.).

## Heavy Compute with Single-Threaded Executor

### **Severity**: info
### **Type**: regex
### **Pattern**
- rclpy\.spin\(node\)(?![\s\S]{0,200}MultiThreadedExecutor)
### **Message**
Consider MultiThreadedExecutor for nodes with heavy computation.

## Missing Shutdown Handler

### **Severity**: info
### **Type**: regex
### **Pattern**
- def main.*rclpy\.spin(?![\s\S]{0,200}finally:|shutdown)
### **Message**
Add finally block with rclpy.shutdown() for clean exit.
