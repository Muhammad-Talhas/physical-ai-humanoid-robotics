import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { motion } from 'framer-motion';
import styles from './index.module.css';

// Navigation Component
function Navigation() {
  return (
    <nav className={styles.navbar}>
      <div className="container">
        <div className={styles.navContent}>
          <div className={styles.navBrand}>
            <Link to="/" className={styles.navLogo}>
              Physical AI
            </Link>
          </div>
          <div className={styles.navLinks}>
            <Link to="/about-author" className={styles.navLink}>
              About the Author
            </Link>
            <Link to="/modules" className={styles.navLink}>
              Modules
            </Link>
            <Link
              to="/docs/intro"
              className={clsx(styles.navLink, styles.navCta)}
            >
              Start Reading
            </Link>
          </div>
        </div>
      </div>
    </nav>
  );
}

// Hero Section Component
function HeroSection() {
  return (
    <section className={styles.heroSection}>
      <div className="container">
        <div className={styles.heroGrid}>
          <div className={styles.heroLeft}>
            <motion.div
              initial={{ opacity: 0, y: 30 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8 }}
            >
              <h1 className={styles.heroTitle}>
                Physical AI &<br />
                <span className={styles.heroGradient}>Humanoid Robotics</span>
              </h1>
              <div className={styles.heroSubtitleCard}>
                <p className={styles.heroSubtitle}>
                  The definitive guide to embodied intelligence, combining cutting-edge research with practical implementations for the next generation of AI-powered humanoid robots.
                </p>
              </div>
              <div className={styles.heroButtons}>
                <Link
                  to="/docs/intro"
                  className={clsx(styles.ctaButton, styles.primaryButton)}
                >
                  Start Reading
                </Link>
                <Link
                  to="/modules"
                  className={clsx(styles.ctaButton, styles.secondaryButton)}
                >
                  Explore Modules
                </Link>
              </div>
            </motion.div>
          </div>
          <div className={styles.heroRight}>
            <motion.div
              className={styles.bookCover}
              initial={{ opacity: 0, scale: 0.8 }}
              animate={{ opacity: 1, scale: 1 }}
              transition={{ duration: 0.8, delay: 0.2 }}
              whileHover={{ y: -10 }}
              animate={{ y: [0, -10, 0] }}
              transition={{ duration: 3, repeat: Infinity, ease: "easeInOut" }}
            >
              <div className={styles.bookCoverContent}>
                <div className={styles.bookIcon}>🤖</div>
                <h3 className={styles.bookTitle}>PHYSICAL AI</h3>
                <p className={styles.bookSubtitle}>HUMANOID ROBOTICS</p>
              </div>
            </motion.div>
          </div>
        </div>
      </div>
    </section>
  );
}

// Bento Grid Component
function BentoGrid() {
  const concepts = [
    {
      title: "Proprioception",
      description: "Understanding spatial awareness and body position in robotics",
      color: "from-blue-500 to-cyan-500",
      delay: 0.1
    },
    {
      title: "Neural Scaling",
      description: "Advanced techniques for scaling neural networks in physical systems",
      color: "from-purple-500 to-pink-500",
      delay: 0.2
    },
    {
      title: "Actuator Ethics",
      description: "Moral frameworks for AI-powered mechanical systems",
      color: "from-green-500 to-emerald-500",
      delay: 0.3
    },
    {
      title: "Vision-Language-Action",
      description: "Integrated perception and action systems",
      color: "from-orange-500 to-red-500",
      delay: 0.4
    },
    {
      title: "Digital Twins",
      description: "Real-time simulation and modeling of physical systems",
      color: "from-indigo-500 to-purple-500",
      delay: 0.5
    },
    {
      title: "ROS 2 Integration",
      description: "Modern robotics operating system implementations",
      color: "from-teal-500 to-blue-500",
      delay: 0.6
    }
  ];

  return (
    <section className={styles.bentoSection}>
      <div className="container">
        <motion.div
          initial={{ opacity: 0, y: 30 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
        >
          <h2 className={styles.sectionTitle}>Key Concepts</h2>
          <p className={styles.sectionSubtitle}>Explore the fundamental principles shaping the future of robotics</p>
        </motion.div>

        <div className={styles.bentoGrid}>
          {concepts.map((concept, index) => (
            <motion.div
              key={index}
              className={clsx(styles.bentoItem, styles[`bentoItem${index + 1}`])}
              initial={{ opacity: 0, scale: 0.8 }}
              whileInView={{ opacity: 1, scale: 1 }}
              transition={{ duration: 0.6, delay: concept.delay }}
              viewport={{ once: true }}
              whileHover={{ scale: 1.05, y: -5 }}
            >
              <div className={clsx(styles.bentoGradient, `bg-gradient-to-br ${concept.color}`)} />
              <div className={styles.bentoContent}>
                <h3 className={styles.bentoTitle}>{concept.title}</h3>
                <p className={styles.bentoDescription}>{concept.description}</p>
              </div>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Testimonials Component
function Testimonials() {
  const testimonials = [
    {
      quote: "A groundbreaking work that bridges the gap between theoretical AI and practical robotics applications.",
      author: "Dr. Sarah Chen",
      role: "MIT Robotics Lab"
    },
    {
      quote: "Essential reading for anyone serious about the future of embodied intelligence.",
      author: "Prof. James Rodriguez",
      role: "Stanford AI Research"
    },
    {
      quote: "The most comprehensive guide to humanoid robotics I've encountered in my 20 years in the field.",
      author: "Dr. Michael Thompson",
      role: "Google DeepMind"
    },
    {
      quote: "Exceptional technical depth combined with practical insights that will shape the industry.",
      author: "Dr. Lisa Park",
      role: "Boston Dynamics"
    }
  ];

  return (
    <section className={styles.testimonialsSection}>
      <div className="container">
        <motion.div
          initial={{ opacity: 0, y: 30 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
        >
          <h2 className={styles.sectionTitle}>Trusted by Industry Leaders</h2>
        </motion.div>

        <div className={styles.testimonialsGrid}>
          {testimonials.map((testimonial, index) => (
            <motion.div
              key={index}
              className={styles.testimonialCard}
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.6, delay: index * 0.1 }}
              viewport={{ once: true }}
              whileHover={{ y: -5 }}
            >
              <div className={styles.testimonialQuote}>
                <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor" className={styles.quoteIcon}>
                  <path d="M14.017 21v-7.391c0-5.704 3.731-9.57 8.983-10.609l.995 2.151c-2.432.917-3.995 3.638-3.995 5.849h4v10h-9.983zm-14.017 0v-7.391c0-5.704 3.748-9.57 9-10.609l.996 2.151c-2.433.917-3.996 3.638-3.996 5.849h3.983v10h-9.983z"/>
                </svg>
                <p className={styles.testimonialText}>{testimonial.quote}</p>
              </div>
              <div className={styles.testimonialAuthor}>
                <span className={styles.authorName}>{testimonial.author}</span>
                <span className={styles.authorRole}>{testimonial.role}</span>
              </div>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
}

// CTA Section Component
function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <motion.div
          className={styles.ctaContent}
          initial={{ opacity: 0, y: 30 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
        >
          <h2 className={styles.ctaTitle}>Ready to Transform Your Understanding?</h2>
          <p className={styles.ctaSubtitle}>Join thousands of robotics engineers and AI researchers advancing the field of physical AI.</p>
          <motion.div
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
          >
            <Link
              to="/docs/intro"
              className={clsx(styles.ctaButton, styles.primaryButton, styles.glowingButton)}
            >
              Order Your Copy
            </Link>
          </motion.div>
        </motion.div>
      </div>
    </section>
  );
}

// Footer Component
function Footer() {
  return (
    <footer className={styles.footer}>
      <div className="container">
        <div className={styles.footerContent}>
          <div className={styles.footerSection}>
            <h4>Physical AI & Humanoid Robotics</h4>
            <p className={styles.footerText}>The definitive guide to embodied intelligence.</p>
          </div>
          <div className={styles.footerSection}>
            <h4>Quick Links</h4>
            <ul className={styles.footerLinks}>
              <li><Link to="/docs/intro">About</Link></li>
              <li><Link to="/docs">Chapters</Link></li>
              <li><Link to="/docs">Resources</Link></li>
            </ul>
          </div>
          <div className={styles.footerSection}>
            <h4>Connect</h4>
            <ul className={styles.footerLinks}>
              <li><a href="#" target="_blank" rel="noopener noreferrer">GitHub</a></li>
              <li><a href="#" target="_blank" rel="noopener noreferrer">Twitter</a></li>
              <li><a href="#" target="_blank" rel="noopener noreferrer">LinkedIn</a></li>
            </ul>
          </div>
        </div>
        <div className={styles.footerBottom}>
          <p>&copy; 2025 Physical AI & Humanoid Robotics. All rights reserved.</p>
        </div>
      </div>
    </footer>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Physical AI and Humanoid Robotics`}
      description="A comprehensive textbook on embodied intelligence, covering ROS 2, Digital Twins, AI Robot Brains, and Vision-Language-Action systems"
    >
      <div className={styles.pageWrapper}>
        <Navigation />
        <HeroSection />
        <BentoGrid />
        <Testimonials />
        <CTASection />
        <Footer />
      </div>
    </Layout>
  );
}