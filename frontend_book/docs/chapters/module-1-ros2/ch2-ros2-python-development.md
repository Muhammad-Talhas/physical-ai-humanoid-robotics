# Chapter 2: ROS 2 Development with Python (rclpy)

## Learning Objectives

By the end of this chapter, students will be able to:
1. Write basic ROS 2 nodes in Python using the rclpy library
2. Implement publishers and subscribers to exchange messages
3. Create and use services for request/reply interactions
4. Apply best practices for modular robot software development
5. Bridge Python AI agents to ROS controllers

## Writing ROS 2 Nodes in Python

ROS 2 provides Python client libraries through rclpy (ROS Client Library for Python). This allows you to write ROS 2 nodes in Python, which is particularly useful for rapid prototyping and AI applications.

### Basic Node Structure

A minimal ROS 2 Python node follows this structure:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers and Subscribers

### Publishers

A publisher node sends messages to a topic. Here's an example of a publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Subscribers

A subscriber node receives messages from a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

### Complete Publisher-Subscriber Example

Here's how to create both nodes and run them together:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    talker = Talker()
    listener = Listener()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass

    talker.destroy_node()
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services

Services provide a request/reply interaction model. Here's an example:

### Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: %d' % response.sum)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bridging Python AI Agents to ROS Controllers

One of the key advantages of using Python with ROS 2 is the ability to easily integrate AI libraries and frameworks. Here's an example of how to bridge an AI component with ROS:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np  # Example AI library

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Publishers for AI outputs
        self.ai_output_pub = self.create_publisher(String, 'ai_decision', 10)

        # Subscribers for sensor data
        self.sensor_sub = self.create_subscription(
            String, 'sensor_data', self.sensor_callback, 10)

        # Timer for AI processing
        self.ai_timer = self.create_timer(0.1, self.ai_processing_callback)

        # Initialize AI model (example)
        self.ai_model = self.initialize_ai_model()

    def initialize_ai_model(self):
        # Initialize your AI model here
        # This could be a neural network, decision tree, etc.
        return {"initialized": True}

    def sensor_callback(self, msg):
        # Process incoming sensor data
        self.get_logger().info(f"Received sensor data: {msg.data}")
        # Store data for AI processing
        self.last_sensor_data = msg.data

    def ai_processing_callback(self):
        if hasattr(self, 'last_sensor_data'):
            # Process data with AI model
            decision = self.process_with_ai(self.last_sensor_data)

            # Publish AI decision
            ai_msg = String()
            ai_msg.data = decision
            self.ai_output_pub.publish(ai_msg)

    def process_with_ai(self, sensor_data):
        # Example AI processing
        # In real implementation, this would use actual AI models
        return f"AI decision based on: {sensor_data}"

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AIBridgeNode()
    rclpy.spin(ai_bridge)
    ai_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Modular Robot Software

### 1. Node Design Principles

- Keep nodes focused on a single responsibility
- Use meaningful node names that reflect their function
- Initialize resources in the constructor
- Properly clean up resources in destroy_node()

### 2. Parameter Management

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('robot_name', 'turtlebot')

        # Access parameters
        self.rate = self.get_parameter('publish_rate').value
        self.robot_name = self.get_parameter('robot_name').value
```

### 3. Error Handling

```python
def safe_publish(self, msg):
    try:
        self.publisher.publish(msg)
    except Exception as e:
        self.get_logger().error(f'Failed to publish message: {e}')
```

### 4. Logging Best Practices

- Use appropriate log levels (debug, info, warn, error, fatal)
- Include relevant context in log messages
- Don't log sensitive information

## Summary

This chapter covered the fundamentals of ROS 2 development with Python using rclpy. You learned how to create nodes, implement publishers and subscribers, work with services, and bridge AI agents with ROS controllers. These skills form the foundation for building complex robotic applications.

## Exercises

1. Create a ROS 2 node that publishes the current time to a topic.
2. Implement a subscriber that calculates the average of received numeric values.
3. Design a service that takes a string and returns its reverse.
4. Create a simple AI bridge that takes sensor data and returns a basic decision.

## References

1. ROS 2 Documentation. (2023). "Python Client Library (rclpy)". Retrieved from https://docs.ros.org/en/rolling/How-To-Guides/Using-RCLPY-With-ROS2.html
2. Open Robotics. (2023). "ROS 2 Tutorials - Writing a Simple Python Publisher and Subscriber". Retrieved from https://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
3. Quigley, M., et al. (2021). "Programming Robots with ROS". O'Reilly Media.
4. Majumdar, A., et al. (2020). "Robot Learning from Demonstration with Human Feedback". IEEE Transactions on Robotics.
5. Sutton, R. S., & Barto, A. G. (2018). "Reinforcement Learning: An Introduction". MIT Press.

<!-- Integration placeholders -->
<!-- 🔘 Personalization button -->
<!-- 🌐 Urdu translation button -->
<!-- 🤖 RAG chatbot query examples -->