---
sidebar_position: 2
---

# Module 1 — The Robotic Nervous System (ROS 2)

This module introduces ROS 2 as the middleware that forms the "nervous system" of modern robots. You will learn the core communication primitives, how to structure robot software using nodes, and how to bridge Python-based AI agents into robotic systems using `rclpy`. We also cover robot description with URDF for humanoid platforms and provide hands-on exercises to build practical skills.

## Why ROS 2?

- ROS 2 provides a distributed, real-time-aware framework for connecting sensors, controllers, planners, and user interfaces.
- It standardizes inter-process communication, enabling modular software where independent nodes exchange messages safely and predictably.
- ROS 2 supports multiple languages (C++, Python) and tools (RViz, Gazebo), and it is widely used in research and industry for robot development.

ROS 2 is particularly important for humanoid robotics because humanoids require tightly integrated perception, planning, and whole-body control that run across multiple CPUs, real-time controllers, and simulation environments.

## Core Concepts

### Nodes

- A node is a single process that performs computation. In ROS 2, systems are built by composing many nodes that each handle a responsibility (sensor driver, estimator, planner, controller).
- Nodes are the primary unit of modularity and deployment.

Example: a simple Python node using `rclpy`:

```python
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello from ROS 2 node')

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics (Publish / Subscribe)

- Topics implement asynchronous, many-to-many message passing. Publishers publish messages; subscribers receive them.
- Topics are ideal for streaming sensor data (camera images, LiDAR point clouds, IMU) and actuator commands.

Publisher example (Python):

```python
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello world'
        self.pub.publish(msg)
```

Subscriber example (Python):

```python
class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'chatter', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

### Services

- Services provide synchronous request/reply interactions. Use services when a client needs a deterministic response (e.g., asking a localisation server for a pose, requesting a map segment).

Service server (Python):

```python
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

Client usage is similar: create a client, wait for service, send request, and wait for result.

### Actions

- Actions are for long-running tasks that publish feedback and may be preempted (e.g., navigation goals, trajectory execution).
- Actions provide a goal/result/feedback protocol and are implemented with `rclpy.action` in Python.

Action server skeleton (Python):

```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(self, Fibonacci, 'fibonacci', self.execute_callback)

    def execute_callback(self, goal_handle):
        # compute fibonacci sequence and publish feedback
        # set result when complete
        pass
```

Refer to official ROS 2 documentation for full action server/client patterns.

## Bridging Python AI Agents with ROS 2 (`rclpy`)

Python is commonly used to develop AI models (PyTorch, TensorFlow). `rclpy` lets you embed these models in ROS 2 nodes so that perception and decision-making run alongside robot middleware.

Example pattern: a perception node that runs a neural network and publishes detected objects.

```python
import torch
from sensor_msgs.msg import Image
from my_package.msg import Detections

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector')
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(Detections, 'detections', 10)
        self.model = torch.load('best_model.pt')

    def image_cb(self, img_msg):
        # convert ROS Image to tensor, run model, publish results
        detections = self.run_model_on_image(img_msg)
        self.pub.publish(detections)

    def run_model_on_image(self, img_msg):
        # preprocessing, inference, postprocessing
        pass
```

Design notes:

- Keep heavy computation off the critical control loop or run it on a separate process/GPU node to avoid blocking low-latency controllers.
- Use message queues and appropriate QoS for lossy topics (e.g., image streams vs. critical state).
- Consider `composition` (composable nodes) when you want to co-locate multiple components in one process for efficiency.

## Understanding URDF for Humanoid Robots

URDF (Unified Robot Description Format) describes a robot's kinematic and inertial properties. For humanoids, URDF models the links (torso, limbs), joints (hip, knee, ankle), sensors, and visual/ collision geometry.

Key URDF elements:

- `<link>`: defines mass, inertia, collision and visual geometry.
- `<joint>`: connects links and defines joint type (revolute, continuous, fixed), axis, and limits.
- `<transmission>`: maps joint commands to actuators (used in hardware interfaces).

Simple URDF snippet (hip joint):

```xml
<robot name="humanoid_simple">
  <link name="torso"/>
  <link name="left_upper_leg"/>

  <joint name="left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.0 0.1 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="150" velocity="2.0"/>
  </joint>
</robot>
```

Tooling:

- Use `xacro` for parameterized URDF generation.
- Visualize models in `rviz2` and simulate them in Gazebo or Ignition.

## Hands-on Exercises

Exercise 1 — ROS 2 Quickstart

- Install ROS 2 (recommended distribution: check course instructions).
- Create a Python package with `ros2 pkg create --build-type ament_python my_robot_pkg`.
- Implement a `talker` and `listener` node using the examples above and verify communication with `ros2 topic list` and `ros2 topic echo`.

Exercise 2 — Service and Action

- Implement the `AddTwoInts` service server and client and test remote calls with `ros2 service call`.
- Implement a simple Fibonacci action server and client; observe feedback from the server as the action executes.

Exercise 3 — Deploy a Python AI agent node

- Train or reuse a small image classifier (or download a pre-trained model) and place it in a node that subscribes to a `/camera/image_raw` topic and publishes detections.
- Benchmark inference latency and move heavy inference to a separate process or GPU if needed.

Exercise 4 — URDF & Visualization

- Build a minimal humanoid URDF (torso, two legs, two arms) using `xacro` parameters for link lengths.
- Visualize the model in `rviz2` and verify joint names and frames.

Exercise 5 — Sim-to-Real Introduction

- Launch a humanoid model in Gazebo (or Ignition) and implement a controller node that sends joint trajectories.
- Observe differences between simulation and reality; document sources of mismatch and strategies for mitigation (sensor noise, latency, actuator dynamics).

## Tips and Best Practices

- Separate concerns: keep perception, planning, and control modular and connected via well-defined topics/services/actions.
- Use appropriate QoS settings for different data types (sensor streams vs. control commands).
- Profile CPU, memory, and network usage early—humanoid systems are resource intensive.
- Write tests and integration checks for each node; use `ros2 launch` for reproducible system brings-up.

## Further Reading and Resources

- ROS 2 official documentation: https://docs.ros.org
- ROS 2 tutorials: https://index.ros.org/doc/ros2/Tutorials/
- URDF and xacro guide: https://wiki.ros.org/urdf
- Composable nodes and lifecycle management in ROS 2

---

Proceed to Module 2 to explore robotic hardware, actuators, and sensor integration in depth.
