import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './about-author.module.css';

function AuthorPage() {
  return (
    <div className={styles.authorPage}>
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className={styles.authorContent}>
              <h1 className={styles.authorTitle}>About the Author</h1>
              <div className={styles.authorCard}>
                <div className={styles.authorImage}>
                  <div className={styles.placeholderImage}>👤</div>
                </div>
                <div className={styles.authorDetails}>
                  <h2 className={styles.authorName}>Talha Mehtab</h2>
                  <p className={styles.authorBio}>
                    Talha Mehtab is a renowned expert in the field of robotics and artificial intelligence.
                    With years of experience in developing cutting-edge robotic systems and AI-powered solutions,
                    he has made significant contributions to the advancement of humanoid robotics and embodied intelligence.
                  </p>

                  <h3 className={styles.sectionTitle}>Expertise and Background</h3>
                  <p className={styles.sectionContent}>
                    Talha specializes in:
                  </p>
                  <ul className={styles.expertiseList}>
                    <li>Physical AI and embodied intelligence</li>
                    <li>Humanoid robotics development</li>
                    <li>ROS 2 and advanced robotics frameworks</li>
                    <li>Digital twin technologies</li>
                    <li>Vision-language-action systems</li>
                  </ul>

                  <h3 className={styles.sectionTitle}>Research and Innovation</h3>
                  <p className={styles.sectionContent}>
                    His work focuses on bridging the gap between theoretical AI research and practical robotics applications,
                    particularly in the development of next-generation humanoid robots that can interact seamlessly with the physical world.
                    Through his research, he has pioneered approaches to integrating advanced neural networks with real-time robotic control systems.
                  </p>

                  <h3 className={styles.sectionTitle}>Vision for the Future</h3>
                  <p className={styles.sectionContent}>
                    Talha believes that the future of robotics lies in the convergence of advanced AI,
                    sophisticated sensorimotor systems, and ethical frameworks that ensure safe and beneficial human-robot interaction.
                    His book "Physical AI and Humanoid Robotics" represents a comprehensive guide to this exciting and rapidly evolving field.
                  </p>
                </div>
              </div>

              <div className={styles.ctaSection}>
                <Link to="/" className={styles.backButton}>
                  ← Back to Home
                </Link>
                <Link to="/docs/intro" className={styles.readBookButton}>
                  Read the Book →
                </Link>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function AboutAuthor() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`About the Author - ${siteConfig.title}`}
      description="Learn about Talha Mehtab, the author of Physical AI and Humanoid Robotics">
      <AuthorPage />
    </Layout>
  );
}