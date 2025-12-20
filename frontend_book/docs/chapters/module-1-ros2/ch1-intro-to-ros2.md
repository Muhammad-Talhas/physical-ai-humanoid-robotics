# Chapter 1: Introduction to ROS 2 Architecture

## Learning Objectives

By the end of this chapter, students will be able to:
1. Explain the core concepts of ROS 2 architecture including nodes, topics, services, and actions
2. Understand how DDS (Data Distribution Service) enables communication in ROS 2
3. Compare and contrast ROS 1 and ROS 2 architectural approaches
4. Identify the key components of a ROS 2 system and their roles in robotic communication

## ROS 2 Architecture Overview

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

![ROS 2 Architecture](/assets/module-1/ros2-architecture-diagram.svg)

### Core Architecture Components

The fundamental building blocks of ROS 2 include:

- **Nodes**: A node is a process that performs computation. Nodes are combined together to form a ROS graph.
- **Topics**: Topics are named buses over which nodes exchange messages.
- **Services**: Services provide a request/reply interaction model between nodes.
- **Actions**: Actions are a more advanced form of services that support long-running requests with feedback.

## Nodes, Topics, Services, and Actions

![Node-Topic-Service Communication](/assets/module-1/node-topic-service-communication.svg)

### Nodes

In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS program. They are designed to be modular, so each node is responsible for a specific task.

Example of a basic node structure:
```python
import rclpy
from rclpy.node import Node

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

### Topics and Message Passing

Topics are named buses over which nodes exchange messages. ROS 2 uses a publish/subscribe model for topics, where nodes can publish messages to a topic and other nodes can subscribe to that topic to receive messages.

The communication is asynchronous and loosely coupled - publishers don't know who is subscribing, and subscribers don't know who is publishing.

### Services

Services provide a request/reply interaction model. A service client sends a request message to a service server, which processes the request and sends back a response.

### Actions

Actions are used for long-running tasks that require:
- Feedback during execution
- Ability to cancel the task
- Status information

## DDS-Based Communication

ROS 2 uses DDS (Data Distribution Service) as its communication middleware. DDS provides:

- **Reliability**: Guaranteed delivery of messages
- **Real-time performance**: Deterministic behavior for time-critical applications
- **Scalability**: Support for large numbers of nodes and messages
- **Quality of Service (QoS)**: Configurable policies for reliability, durability, etc.

### DDS vs ROS 1 Communication

In ROS 1, all communication went through a central master node, which could become a single point of failure. ROS 2's DDS-based approach provides:

- Decentralized communication
- Better fault tolerance
- Improved performance for distributed systems
- Native support for multiple programming languages

## ROS 2 vs ROS 1 Comparison

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Master-based | DDS-based |
| Languages | Python, C++ | Python, C++ Java, etc. |
| Platforms | Linux-focused | Multi-platform |
| Real-time | Limited | Better support |
| Security | Limited | Built-in support |

## Summary

This chapter introduced the fundamental concepts of ROS 2 architecture. Understanding these core components is essential for developing robotic applications using ROS 2. The next chapter will explore practical development with Python and rclpy.

## Exercises

1. Explain the difference between a topic and a service in ROS 2.
2. Describe a scenario where you would use actions instead of services.
3. What are the advantages of DDS-based communication over the ROS 1 master-based approach?

## References

1. ROS 2 Documentation. (2023). "Concepts - About ROS 2". Retrieved from https://docs.ros.org/en/rolling/Concepts/About-ROS-2.html
2. Pradeep, K. M., & Sridharan, M. (2021). "ROS 2 for Absolute Beginners". Apress.
3. Quigley, M., Gerkey, B., & Smart, W. (2021). "Programming Robots with ROS". O'Reilly Media.
4. DDS Consortium. (2023). "DDS Specification". Object Management Group.
5. Faconti, P., et al. (2019). "ROS 2 Design Overview". Open Robotics.

<!-- Integration placeholders -->
<!-- 🔘 Personalization button -->
<!-- 🌐 Urdu translation button -->
<!-- 🤖 RAG chatbot query examples -->