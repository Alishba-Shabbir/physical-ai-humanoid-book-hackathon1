import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Foundations of Physical AI',
    description: (
      <>
        Understand embodied intelligence and how AI systems interact with
        physical environments through perception, decision-making, and action.
      </>
    ),
  },
  {
    title: 'Robotic Middleware & Simulation',
    description: (
      <>
        Learn ROS 2, Gazebo, and digital twin workflows to design, simulate,
        and test humanoid robots before real-world deployment.
      </>
    ),
  },
  {
    title: 'Humanoid Intelligence & Capstone',
    description: (
      <>
        Integrate vision, language, and action systems to build an autonomous
        humanoid robot as a final capstone project.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">

        {/* Book Intro */}
        <div className="margin-bottom--lg">
          <Heading as="h2">
            Physical AI & Humanoid Robotics
          </Heading>
          <p>
            This AI-native textbook introduces students to intelligent systems
            operating in the physical world. The course bridges modern AI with
            robotics, simulation, and real-world deployment using industry tools.
          </p>
        </div>

        {/* Learning Outcomes */}
        <div className="margin-bottom--lg">
          <Heading as="h3">What You Will Learn</Heading>
          <ul>
            <li>Core principles of Physical AI and embodied intelligence</li>
            <li>ROS 2 for robotic control and communication</li>
            <li>Simulation with Gazebo and NVIDIA Isaac</li>
            <li>Vision-Language-Action integration for humanoid robots</li>
          </ul>
        </div>

        {/* Core Modules */}
        <div className="row margin-bottom--lg">
          {FeatureList.map((item, idx) => (
            <Feature key={idx} {...item} />
          ))}
        </div>

        {/* Call to Action */}
        <div className="text--center">
          <a
            className="button button--primary button--lg"
            href="/docs/intro"
          >
            Start Reading the Book
          </a>
        </div>

      </div>
    </section>
  );
}
