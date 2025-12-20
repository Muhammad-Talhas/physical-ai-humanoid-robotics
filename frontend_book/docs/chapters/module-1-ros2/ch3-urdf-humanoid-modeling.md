# Chapter 3: Robot Modeling with URDF for Humanoids

## Learning Objectives

By the end of this chapter, students will be able to:
1. Understand the structure and syntax of URDF (Unified Robot Description Format)
2. Create links and joints to define robot kinematic structures
3. Implement transmissions for actuator control
4. Model humanoid robots with proper joint configurations
5. Prepare URDF files for simulation and control in ROS 2

## URDF Structure and Syntax

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including links, joints, and their relationships.

### Basic URDF Structure

![URDF Structure Example](/assets/module-1/urdf-structure-example.svg)

A minimal URDF file follows this structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.25 0" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### URDF Components

1. **Robot Element**: The root element that defines the robot name
2. **Links**: Represent rigid bodies with visual, collision, and inertial properties
3. **Joints**: Define connections between links with specific degrees of freedom
4. **Materials**: Define visual appearance properties

## Links and Joints

### Links

Links represent rigid bodies in the robot model. Each link can have multiple properties:

- **Visual**: Defines how the link appears in simulation
- **Collision**: Defines collision geometry for physics simulation
- **Inertial**: Defines mass and inertia properties for dynamics

```xml
<link name="link_name">
  <!-- Visual properties -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Available geometries: box, cylinder, sphere, mesh -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- Collision properties -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial properties -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

### Joints

Joints define the kinematic relationships between links. Common joint types include:

- **Fixed**: No degrees of freedom
- **Revolute**: Single axis rotation with limits
- **Continuous**: Single axis rotation without limits
- **Prismatic**: Single axis translation with limits
- **Floating**: 6 degrees of freedom

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>
```

## Transmissions

Transmissions define how actuators (motors) connect to joints and links. They specify the mechanical relationship between actuators and joints, including gear ratios, reduction, and safety limits.

### Basic Transmission Structure

```xml
<transmission name="wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Hardware Interfaces

ROS 2 supports different hardware interfaces:
- **EffortJointInterface**: Controls joint effort/torque
- **VelocityJointInterface**: Controls joint velocity
- **PositionJointInterface**: Controls joint position

## Modeling Humanoid Robots

![Humanoid Robot Model](/assets/module-1/humanoid-model-diagram.svg)

Humanoid robots require special attention to anthropomorphic joint configurations and kinematic chains. The typical humanoid structure includes:

- **Trunk**: Base of the robot body
- **Head**: With neck joint for orientation
- **Arms**: With shoulder, elbow, and wrist joints
- **Legs**: With hip, knee, and ankle joints
- **Hands/Feet**: With multiple degrees of freedom for manipulation

### Humanoid URDF Example

A complete humanoid URDF model can be found in the [simple_humanoid.urdf](./simple_humanoid.urdf) file in this chapter's directory. This example demonstrates a basic humanoid structure with torso, head, arms, and legs with appropriate joint configurations.

The model includes:
- A base link representing the robot's core
- A torso connected with a fixed joint
- A head with neck joint for orientation
- Symmetric arms with shoulder, elbow joints
- Symmetric legs with hip, knee joints
- Proper inertial properties for each link
- Appropriate joint limits for safe operation

## Preparing URDFs for Simulation and Control

### URDF with Gazebo Integration

To use URDF models in Gazebo simulation, additional Gazebo-specific tags are needed:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>
```

### Launch Files for URDF Visualization

Create a launch file to visualize the URDF model in RViz:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the urdf file path
    urdf_file = os.path.join(
        get_package_share_directory('your_package_name'),
        'urdf',
        'simple_humanoid.urdf'
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    # Joint State Publisher node (for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_gui': True
        }]
    )

    # RViz2 node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('your_package_name'), 'rviz', 'urdf_viewer.rviz')]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])
```

### URDF Best Practices

1. **Use Xacro**: For complex robots, use Xacro (XML Macros) to simplify URDF creation
2. **Proper Inertial Properties**: Accurate inertial properties are crucial for simulation
3. **Collision vs Visual**: Use simple geometries for collision to improve performance
4. **Joint Limits**: Always specify appropriate joint limits for safety
5. **Consistent Units**: Use consistent units (SI) throughout the model
6. **Naming Conventions**: Use clear, consistent naming for links and joints

## Summary

This chapter covered the fundamentals of robot modeling with URDF for humanoid robots. You learned how to create links and joints to define robot kinematic structures, implement transmissions for actuator control, and model humanoid robots with proper joint configurations. You also learned how to prepare URDF files for simulation and control in ROS 2, including integration with Gazebo and visualization in RViz.

## Exercises

1. Create a simple URDF file for a 2-wheeled robot with proper links, joints, and inertial properties.
2. Design a URDF model for a robotic arm with 3 degrees of freedom (shoulder, elbow, wrist).
3. Implement a humanoid leg model with hip, knee, and ankle joints using appropriate joint types and limits.
4. Create a launch file that visualizes your URDF model in RViz with joint state publisher.

## References

1. ROS Documentation. (2023). "URDF: Unified Robot Description Format". Retrieved from http://wiki.ros.org/urdf
2. ROS 2 Documentation. (2023). "Working with URDF in ROS 2". Retrieved from https://docs.ros.org/en/rolling/Tutorials/URDF/Working-with-URDF.html
3. Corke, P. (2017). "Robotics, Vision and Control: Fundamental Algorithms In MATLAB". Springer.
4. Siciliano, B., & Khatib, O. (2016). "Springer Handbook of Robotics". Springer.
5. Craig, J. J. (2005). "Introduction to Robotics: Mechanics and Control". Pearson.

<!-- Integration placeholders -->
<!-- 🔘 Personalization button -->
<!-- 🌐 Urdu translation button -->
<!-- 🤖 RAG chatbot query examples -->