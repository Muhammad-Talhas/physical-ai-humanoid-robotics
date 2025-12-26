import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './modules.module.css';

function ModuleCard({ title, description, to, color }) {
  return (
    <div className="col col--3">
      <Link to={to} className={styles.moduleCard}>
        <div className={styles.card} style={{ borderLeft: `4px solid ${color}` }}>
          <h3>{title}</h3>
          <p>{description}</p>
          <div className={styles.moduleLink}>
            Explore →
          </div>
        </div>
      </Link>
    </div>
  );
}

function ModulesPage() {
  const modules = [
    {
      title: "Module 1: The Robotic Nervous System",
      description: "ROS 2 fundamentals, Python development, and URDF modeling",
      to: "/docs/chapters/module-1-ros2/intro",
      color: "#4f46e5"
    },
    {
      title: "Module 2: The Digital Twin",
      description: "Gazebo and Unity simulation, physics engines, and sensor modeling",
      to: "/docs/chapters/module-2-digital-twin/intro",
      color: "#1cc88a"
    },
    {
      title: "Module 3: The AI-Robot Brain",
      description: "NVIDIA Isaac, visual SLAM, and autonomous navigation",
      to: "/docs/chapters/module-3-ai-robot-brain/intro",
      color: "#36b9cc"
    },
    {
      title: "Module 4: Vision-Language-Action",
      description: "Voice-to-action, cognitive planning, and LLM integration",
      to: "/docs/chapters/module-4-vision-language-action/intro",
      color: "#f59e0b"
    }
  ];

  return (
    <div className={styles.modulesPage}>
      <div className="container">
        <div className="row">
          <div className="col col--12 text--center padding--lg">
            <h1 className={styles.modulesTitle}>Explore All Modules</h1>
            <p className={styles.modulesSubtitle}>Choose a module to begin your learning journey in physical AI and humanoid robotics</p>
          </div>
        </div>

        <div className="row">
          {modules.map((module, index) => (
            <ModuleCard
              key={index}
              title={module.title}
              description={module.description}
              to={module.to}
              color={module.color}
            />
          ))}
        </div>

        <div className={styles.ctaSection}>
          <Link to="/" className={styles.backButton}>
            ← Back to Home
          </Link>
          <Link to="/docs/intro" className={styles.startLearningButton}>
            Start Learning →
          </Link>
        </div>
      </div>
    </div>
  );
}

export default function Modules() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`Modules - ${siteConfig.title}`}
      description="Explore all modules of the Physical AI and Humanoid Robotics textbook">
      <ModulesPage />
    </Layout>
  );
}