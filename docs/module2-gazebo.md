---
sidebar_position: 3
---

# Module 2 — The Digital Twin (Gazebo & Unity)

Digital twins provide virtual replicas of physical robots and their environments. In robotics, digital twins let us iterate faster, run large-scale experiments safely, and perform sim-to-real validation before deploying to hardware. This module covers physics simulation fundamentals in Gazebo, high-fidelity rendering and human-robot interaction in Unity, realistic sensor simulation (LiDAR, depth cameras, IMUs), and hands-on exercises to build practical simulation skills.

## What is a Digital Twin?

- A digital twin is a software model that mirrors the structure, dynamics, and state of a physical system.
- For robots, this includes kinematics/dynamics, sensors, actuators, controllers, and environment models.
- Benefits: accelerated development, reproducible testing, safety validation, and data generation for learning.

## Physics Simulation in Gazebo

Gazebo (and Ignition Gazebo) provide a physics-enabled environment where robots interact with gravity, contacts, and rigid-body dynamics.

### Key Physics Concepts

- Gravity: set by world plugins or SDF world definitions; critical for locomotion and falls.
- Collisions: collision shapes (primitive or mesh) and contact parameters (friction, restitution) determine interactions.
- Solvers and timestep: physics engines (ODE, Bullet, DART, Simbody) and their contact/constraint solvers affect stability and realism.

### Example — Minimal SDF world with gravity and ground plane

```xml
<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='default'>
    <gravity>0 0 -9.81</gravity>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

### Gazebo Plugins and Controllers

- Plugins extend Gazebo with sensor emulation, custom actuators, and bridges to ROS 2.
- Use `ros_ign_bridge` or `gazebo_ros` plugins to expose topics and services to ROS 2 nodes.
- Real-time controllers and hardware-in-the-loop (HIL) setups let you test control strategies on either the simulator or real actuators.

### Practical Notes

- Use simplified collision geometry for performance and detailed visual meshes for rendering.
- Tune friction and contact parameters to match the robot's real-world behavior.
- Adjust solver iterations and timestep to trade off between performance and stability.

## High-Fidelity Rendering and HRI in Unity

Unity provides high-fidelity rendering, physics (PhysX), and extensive tooling for human-robot interaction (HRI) scenarios and UX testing.

Advantages of Unity:

- Realistic lighting, materials, and animations for user studies and perception dataset creation.
- Easy authoring of interactive scenes and UI for human-in-the-loop experiments.
- Integration paths: ROS-TCP-Connector, ROS# (for older setups), and Unity ML-Agents for learning in Unity.

Example pattern: connect a Unity scene to ROS 2 using the ROS-TCP-Connector, stream camera images and joint states, and receive joint commands from a ROS 2 controller.

### Unity tips for robotics

- Use URDF importer plugins to bring robot geometry into Unity.
- Sync physics timesteps between Unity and ROS 2 when using closed-loop controllers.
- Use lightweight sensor shaders or compute shaders for depth, segmentation, and synthetic LiDAR generation.

## Simulating Sensors

Accurate sensor simulation is crucial for training perception models and validating algorithms.

### LiDAR

- Simulated LiDAR provides point clouds and must approximate scanning patterns, angular resolution, range falloff, and noise.
- In Gazebo use `gazebo_ros_pkgs` sensors or third-party LiDAR plugins that support ray-tracing and configurable noise models.

Example configuration snippet (SDF sensor):

```xml
<sensor name='hokuyo' type='gpu_ray'>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <min_angle>-1.5708</min_angle>
        <max_angle>1.5708</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
    </range>
  </ray>
</sensor>
```

### Depth Cameras & RGB Cameras

- Simulated cameras provide `sensor_msgs/Image` messages in ROS 2 and can generate RGB, depth, and semantic segmentation streams.
- Configure intrinsic parameters (fx, fy, cx, cy), resolution, and noise. For domain-randomization, vary lighting, textures, and sensor noise.

### IMUs and Odometry

- Simulated IMUs produce angular velocity and linear acceleration with configurable bias and noise.
- Odometry can be derived from joint states or ground-truth pose; add realistic drift and latency to emulate real sensors.

Example IMU noise settings (SDF):

```xml
<imu>
  <topic>imu/data</topic>
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>
  </noise>
</imu>
```

## Bridging Simulation and ROS 2

- Use `ros_ign_bridge`, `ros_gz`, or `gazebo_ros_pkgs` to translate simulation topics to ROS 2 messages.
- Launch files (ROS 2) or world files (SDF) should declare the robot model, sensors, controllers, and bridges.

Example ROS 2 launch snippet (Python):

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': '<robot>...</robot>'}]),
    ])
```

## Hands-on Exercises

Exercise 1 — Gazebo Quickstart

- Install Gazebo or Ignition Gazebo (see course install guide).
- Launch a sample world and spawn a simple robot model. Use `ros2 topic list` and `ros2 topic echo` to inspect sensor data.

Exercise 2 — Configure and Tune Physics

- Modify the robot's collision geometry and friction coefficients; observe how walking or balancing controllers behave.
- Experiment with different physics engines (where supported) and solver settings; document differences.

Exercise 3 — LiDAR and Camera Simulation

- Add a simulated LiDAR to your robot and record point clouds while the robot navigates a scene.
- Create a camera with depth and RGB streams, then train a small perception model on synthetic data. Test the model on held-out simulated scenes.

Exercise 4 — Unity Scene for HRI

- Import a humanoid model into Unity (URDF importer or FBX), create a simple environment, and use the ROS-TCP-Connector to stream joint states and camera images.
- Implement a simple UI to request robot actions and observe HRI behavior under different lighting/appearance settings.

Exercise 5 — Sim-to-Real Checklist

- Compare simulated sensor outputs with logged real-world sensor data. Identify parameter mismatches (noise, timing, calibration).
- Implement domain-randomization strategies (textures, lighting, sensor noise) and evaluate whether models trained in simulation generalize to real-world recordings.

## Best Practices

- Keep separate visual and collision meshes for performance vs. realism.
- Parameterize worlds and robots with `xacro`/templates so you can sweep parameters for robustness testing.
- Use headless simulation for large-scale data generation and GPU-enabled rendering for high-fidelity images.
- Log deterministic seeds and metadata for reproducibility of experiments.

## Further Resources

- Gazebo / Ignition docs: https://gazebosim.org
- ROS 2 + Gazebo integration guide
- Unity Robotics Hub and ROS-TCP-Connector: https://github.com/Unity-Technologies/ROS-TCP-Connector
- Unity ML-Agents for training in Unity

---

Proceed to Module 3 for kinematics and dynamics of humanoid systems.
