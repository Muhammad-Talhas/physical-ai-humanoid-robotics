# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

## Overview
This guide provides step-by-step instructions for setting up the simulation environment and beginning with Module 2 content. The module focuses on digital twins using Gazebo for physics simulation and Unity for high-fidelity rendering and human-robot interaction scenarios.

## Prerequisites
Before starting Module 2, ensure you have:
- Completed Module 1: The Robotic Nervous System (ROS 2) or have equivalent knowledge
- A computer running Windows, macOS, or Linux
- Basic familiarity with command-line tools
- Understanding of fundamental robotics concepts

## Environment Setup

### 1. Install ROS 2 (if not already installed)
Follow the official ROS 2 installation guide for your operating system:
- **Ubuntu**: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- **Windows**: https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html
- **macOS**: https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html

### 2. Install Gazebo Garden
Gazebo Garden is the recommended version for this module:

**Ubuntu:**
```bash
sudo apt install gazebo
```

**Windows/macOS:** Follow the binary installation guide at http://gazebosim.org/docs/garden/install

### 3. Install Unity (Personal Edition)
1. Go to https://unity.com/products/unity-personal
2. Download and install Unity Hub
3. Through Unity Hub, install the latest LTS version of Unity
4. Create a free Unity account when prompted

### 4. Set up Docusaurus Documentation
Ensure you can access the textbook content:
```bash
cd frontend_book
npm install
npm start
```

## First Simulation Exercise

### 1. Launch Basic Gazebo Environment
Open a terminal and source your ROS 2 installation:
```bash
# For Ubuntu/Humble
source /opt/ros/humble/setup.bash

# Launch a basic empty world
gz sim -v 4 empty.sdf
```

### 2. Create a Simple Robot Model
Create a basic URDF file to represent a simple robot:

**simple_robot.urdf:**
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

### 3. Load Robot into Gazebo
```bash
# Launch Gazebo with your robot
gz sim -v 4 -r simple_robot.sdf
```

## ROS 2 Integration

### 1. Launch Robot State Publisher
In a new terminal (with ROS 2 sourced):
```bash
# Publish static transforms
ros2 run robot_state_publisher robot_state_publisher simple_robot.urdf
```

### 2. Verify Integration
Check that ROS 2 can communicate with Gazebo:
```bash
# List available topics
ros2 topic list

# Check for Gazebo-specific topics like /clock, /model_states, etc.
```

## Unity Setup for HRI

### 1. Create New Unity Project
1. Open Unity Hub
2. Click "New Project"
3. Select "3D (Built-in Render Pipeline)" template
4. Name your project "RobotHRI_Simulation"
5. Create the project

### 2. Install Robotics Packages
In Unity:
1. Go to Window → Package Manager
2. Install "ROS-TCP-Connector" package
3. Install "Visual Scripting" (for HRI prototyping)

### 3. Basic HRI Scene
1. Create a simple environment with primitive shapes
2. Add a basic robot model (or use Unity's primitive shapes as placeholder)
3. Set up a simple interaction (e.g., button press to move robot)

## Documentation Navigation

### 1. Access Module Content
The Module 2 content is organized as follows:
- **Chapter 1**: Introduction to Digital Twins in Robotics
- **Chapter 2**: Physics Simulation with Gazebo
- **Chapter 3**: Sensor Simulation and Human–Robot Interaction

### 2. Key Features of the Textbook Interface
- **Navigation sidebar**: Access all chapters and sections
- **Code blocks**: Copyable code examples
- **Diagrams**: Interactive diagrams where applicable
- **Cross-references**: Links to related concepts in other modules
- **Exercises**: Interactive or simulation-based exercises

### 3. Finding Simulation Examples
- Look for the 🤖 icon for simulation-specific examples
- Code examples are marked with language indicators (Python, XML, etc.)
- Exercises are marked with 📝 and include expected outcomes

## Troubleshooting Common Issues

### Gazebo Not Launching
- Ensure ROS 2 is properly sourced in your terminal
- Check that your system meets Gazebo's requirements
- Try running `gz --versions` to verify installation

### ROS 2 Communication Issues
- Verify that both Gazebo and ROS 2 terminals have the same ROS_DOMAIN_ID
- Check that the RMW implementation is consistent between processes
- Ensure network configuration allows local communication

### Unity Installation Problems
- Verify your Unity license is activated
- Check system requirements are met
- Restart Unity Hub if packages don't appear

### Docusaurus Documentation Issues
- Run `npm install` again if pages don't load properly
- Clear browser cache if content appears outdated
- Check that you're running the development server with `npm start`

## Next Steps

After completing this quickstart:

1. **Chapter 1**: Begin with the Introduction to Digital Twins to understand the theoretical foundation
2. **Chapter 2**: Work through Physics Simulation with Gazebo to gain hands-on experience
3. **Chapter 3**: Explore Sensor Simulation and Human–Robot Interaction to complete the module

Each chapter builds upon the previous, so follow them in sequence for optimal learning. The exercises in each chapter are designed to reinforce the concepts with practical simulation experience.

## Getting Help

- Check the FAQ section in the main textbook documentation
- Review the troubleshooting section at the end of each chapter
- Consult the official documentation for Gazebo and Unity
- Join the course discussion forum for peer support