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
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'chapters/module-2-digital-twin/intro',
        'chapters/module-2-digital-twin/ch1-intro-to-digital-twins',
        'chapters/module-2-digital-twin/ch2-physics-simulation-gazebo',
        'chapters/module-2-digital-twin/ch3-sensor-simulation-unity',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'chapters/module-3-ai-robot-brain/intro',
        'chapters/module-3-ai-robot-brain/ch1-nvidia-isaac-sim-synthetic-data',
        'chapters/module-3-ai-robot-brain/ch2-isaac-ros-visual-slam',
        'chapters/module-3-ai-robot-brain/ch3-autonomous-navigation-nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'chapters/module-4-vision-language-action/intro',
        'chapters/module-4-vision-language-action/chapter-4-1-voice-to-action',
        'chapters/module-4-vision-language-action/chapter-4-2-cognitive-planning-llms',
        'chapters/module-4-vision-language-action/chapter-4-3-capstone-autonomous-humanoid',
      ],
    },
    // Additional modules will be added here as they are developed
  ],
};

export default sidebars;