// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'chapters/module-1-ros2/ch1-intro-to-ros2',
        'chapters/module-1-ros2/ch2-ros2-python-development',
        'chapters/module-1-ros2/ch3-urdf-humanoid-modeling',
      ],
    },
    // Additional modules will be added here as they are developed
  ],
};

export default sidebars;