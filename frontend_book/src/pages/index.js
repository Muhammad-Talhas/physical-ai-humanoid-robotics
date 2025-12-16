import React from 'react';
import {Redirect} from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function Home() {
  const {siteConfig} = useDocusaurusContext();
  return <Redirect to="/physical-ai-and-humanoid-robotic/docs/intro" />;
}

export default Home;