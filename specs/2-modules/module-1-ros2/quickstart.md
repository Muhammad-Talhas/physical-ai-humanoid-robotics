# Quickstart Guide: Module 1 - The Robotic Nervous System (ROS 2)

## Overview
This guide provides a rapid introduction to setting up and working with Module 1 content for the Physical AI & Humanoid Robotics textbook. It covers the essential steps to get the Docusaurus documentation site running with the ROS 2 module content.

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Git for version control
- Basic understanding of ROS 2 concepts (optional but helpful)

## Setup Steps

### 1. Clone and Prepare the Repository
```bash
git clone <repository-url>
cd <repository-name>
npm install
```

### 2. Install ROS 2 Development Environment (for testing code examples)
```bash
# Install ROS 2 Humble Hawksbill (Ubuntu/Debian)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
```

### 3. Install Gazebo for Simulation
```bash
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo
```

### 4. Add Module 1 Content
Create the module directory and chapter files:
```bash
mkdir -p docs/chapters/module-1-ros2
mkdir -p docs/assets/module-1
touch docs/chapters/module-1-ros2/ch1-intro-to-ros2.md
touch docs/chapters/module-1-ros2/ch2-ros2-python-development.md
touch docs/chapters/module-1-ros2/ch3-urdf-humanoid-modeling.md
```

### 5. Configure Docusaurus Sidebar
Add the following to `sidebars.js`:
```javascript
module.exports = {
  // ... existing sidebar configuration
  module1: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'chapters/module-1-ros2/ch1-intro-to-ros2',
        'chapters/module-1-ros2/ch2-ros2-python-development',
        'chapters/module-1-ros2/ch3-urdf-humanoid-modeling',
      ],
    },
  ],
};
```

### 6. Update Docusaurus Configuration
Add the module to the main navigation in `docusaurus.config.js`:
```javascript
module.exports = {
  // ... existing configuration
  themeConfig: {
    // ... existing theme config
    navbar: {
      items: [
        // ... existing items
        {
          type: 'docSidebar',
          sidebarId: 'module1',
          label: 'Module 1',
        },
      ],
    },
  },
};
```

### 7. Run the Development Server
```bash
npm start
```

## Testing Code Examples

### 1. Create a Test Workspace
```bash
mkdir -p ~/ros2_textbook_ws/src
cd ~/ros2_textbook_ws
colcon build
source install/setup.bash
```

### 2. Validate Python Examples
For each Python code example in the textbook:
```bash
# Navigate to the example directory
cd ~/ros2_textbook_ws/src/<example_package>
# Run the example
python3 <example_file>.py
```

### 3. Test with Gazebo Simulation
```bash
# Launch Gazebo
ros2 launch gazebo_ros gazebo.launch.py
# Run your robot simulation nodes in separate terminals
```

## Validation Steps

### 1. Content Structure Validation
```bash
# Verify all required files exist
ls -la docs/chapters/module-1-ros2/
ls -la docs/assets/module-1/
```

### 2. Build Validation
```bash
npm run build
```

### 3. Link Validation
```bash
npm run serve
# Visit http://localhost:3000 and verify all navigation works
```

## Troubleshooting

### Common Issues
1. **Build fails**: Ensure all required packages are installed and Node.js version is 18+
2. **Missing navigation**: Verify sidebar configuration in `sidebars.js` and `docusaurus.config.js`
3. **Code examples don't work**: Check ROS 2 installation and source the setup.bash file

### Quick Fixes
- For Docusaurus issues: `npm install` to reinstall dependencies
- For ROS 2 issues: `source /opt/ros/humble/setup.bash` to ensure ROS 2 is sourced
- For broken links: Check file paths in sidebar configuration

## Next Steps
1. Complete the content for each chapter following the specification
2. Add diagrams and assets to the `docs/assets/module-1/` directory
3. Validate all code examples in a ROS 2 environment
4. Test the simulation labs in Gazebo
5. Review content for adherence to educational objectives