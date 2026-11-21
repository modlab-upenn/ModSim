import React, {useState} from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useBrokenLinks from '@docusaurus/useBrokenLinks';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import CodeBlock from '@theme/CodeBlock';
import ThemedImage from '@theme/ThemedImage';
import {DockingDemo, TopologyPlayground, ModelViewDemo} from '../components/home';
import styles from './index.module.css';

const repository = 'https://github.com/modlab-upenn/ModSim';
const clips = [
  {file: 'inspector_lattice.mp4', label: 'Lattice view', caption: 'Cubic-lattice view, M-Blocks rectangle to line: measured and target lattices as cubes pivot into place.'},
  {file: 'inspector_smores_planning.mp4', label: 'SMORES planner', caption: 'SMORES-EP online self-assembly: goals, generated routes, action states, and action history.'},
  {file: 'inspector_mblocks_planning.mp4', label: 'M-Blocks planner', caption: 'M-Blocks online planning: generated pivots, clearance checks, and action progress.'},
];

function Recording({file, label, caption}: {file: string; label: string; caption: string}): React.JSX.Element {
  return <figure className={styles.recording}>
    <video src={useBaseUrl(`/media/${file}`)} controls playsInline muted loop preload="metadata" aria-label={label} />
    <figcaption>{caption}</figcaption>
  </figure>;
}

function InspectorRecordings(): React.JSX.Element {
  const [active, setActive] = useState(0);
  const clip = clips[active];
  return <figure className={styles.recording}>
    <video key={clip.file} src={useBaseUrl(`/media/${clip.file}`)} controls playsInline muted loop preload="metadata"
      aria-label={`Runtime Inspector recording: ${clip.label}`} />
    <div className={styles.clipChoices} role="group" aria-label="Runtime Inspector recordings">
      {clips.map((item, index) => <button key={item.file} type="button"
        aria-pressed={index === active} onClick={() => setActive(index)}>{item.label}</button>)}
    </div>
    <figcaption>{clip.caption}</figcaption>
  </figure>;
}

function Architecture(): React.JSX.Element {
  return <figure className={styles.architecture} aria-label="ModSim architecture">
    <div className={styles.architectureFlow}>
      <div className={styles.architectureNode}>
        <span className={styles.step}>01 / Describe</span>
        <h3>Robot Pack</h3>
        <p>URDF and meshes<br />Modules and connectors<br />Compatibility and docking rules</p>
      </div>
      <span className={styles.flowArrow} aria-hidden="true">→</span>
      <div className={`${styles.architectureNode} ${styles.canonicalNode}`}>
        <span className={styles.step}>02 / Execute</span>
        <h3>ModSim runtime</h3>
        <p>Canonical world state<br />Docking lifecycle and events<br />Planners and controllers</p>
      </div>
      <span className={styles.flowArrow} aria-hidden="true">↔</span>
      <div className={styles.architectureNode}>
        <span className={styles.step}>03 / Observe</span>
        <h3>Backend and views</h3>
        <p>Mock or MuJoCo physics<br />Generated topology and lattice<br />Studio’s Runtime Inspector</p>
      </div>
    </div>
    <figcaption>The backend supplies observations and physical constraints. Model views are derived from ModSim’s canonical state.</figcaption>
  </figure>;
}

export default function Home(): React.JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  const libraryUrl = siteConfig.customFields?.robotPackLibraryUrl as string | null | undefined;
  const brokenLinks = useBrokenLinks();
  for (const id of ['abstract', 'problem', 'docking', 'method', 'robotpack-library', 'experiments', 'results', 'start', 'citation']) {
    brokenLinks.collectAnchor(id);
  }
  const lightLogo = useBaseUrl('/branding/light/logos/svg/modsim-horizontal.svg');
  const darkLogo = useBaseUrl('/branding/midnight/logos/svg/modsim-horizontal.svg');
  return <Layout title="Modular robotics simulation"
    description="Describe modular robots once, author Robot Packs in Studio, and explore docking, reconfiguration, and runtime state with ModSim.">
    <main className={styles.page}>
      <div className={styles.column}>
        <header className={styles.hero}>
          <p className={styles.eyebrow}>ModLab · University of Pennsylvania</p>
          <h1><ThemedImage className={styles.heroLogo} alt="ModSim" sources={{light: lightLogo, dark: darkLogo}} /></h1>
          <p className={styles.subtitle}>A semantic and physics framework for modular and multi-robot simulation.</p>
          <p className={styles.authors}>Angelina Wong · Akshay Ram Panyam · Nils Roede · Mark Yim</p>
          <p className={styles.intro}>Describe a robot once. Dock, disconnect, and reconfigure it at runtime—with logical connections and physical constraints kept in sync.</p>
          <div className={styles.heroActions}>
            <Link className="button button--primary" to="/docs/tutorials/getting-started/">Get started <span aria-hidden="true">↗</span></Link>
            <Link className="button button--outline button--secondary" to="/docs/">Read the documentation</Link>
            <Link className="button button--outline button--secondary" to={libraryUrl || '#robotpack-library'}>RobotPack Library</Link>
            <a className={styles.textLink} href={repository}>View code <span aria-hidden="true">↗</span></a>
          </div>
          <nav className={styles.contents} aria-label="On this page">
            <a href="#problem">Motivation</a><a href="#docking">Docking</a><a href="#method">Architecture</a>
            <a href="#robotpack-library">RobotPack Library</a><a href="#experiments">Experiments</a><a href="#results">Results</a><a href="#start">Installation</a><a href="#citation">Citation</a>
          </nav>
        </header>

        <div className={styles.capabilities}>
          <div><span>Describe</span><strong>URDF + semantic YAML</strong></div>
          <div><span>Connect</span><strong>Runtime docking and release</strong></div>
          <div><span>Inspect</span><strong>Physics, topology, and plans</strong></div>
        </div>

        <section className={styles.section} id="abstract" aria-labelledby="abstract-title">
          <p className={styles.eyebrow}>The framework</p>
          <h2 id="abstract-title">One description. A changing assembly.</h2>
          <p>ModSim makes modules, connectors, and their connection lifecycle a shared layer above the simulator. A validated Robot Pack combines imported mechanical assets with connector compatibility, acceptance tolerances, and docking policies. The runtime decides when a connection can form, asks the backend to create its constraint, and commits it after the backend confirms.</p>
          <p>Studio brings authoring and runtime inspection into one desktop application. The same pack also works with the CLI and Python API, so an interactive experiment can become a reproducible script.</p>
          <Recording file="studio_walkthrough_2x.mp4" label="Recorded ModSim Studio authoring walkthrough"
            caption="Studio authoring recording · 2× speed. Inspect visual and collision geometry, frames, joint axes, connectors, and the underlying YAML. The current Studio also includes the integrated Runtime workspace." />
        </section>

        <section className={styles.section} id="problem" aria-labelledby="problem-title">
          <h2 id="problem-title">Modular robots change shape while they run.</h2>
          <p>A modular robot’s mechanism changes as its modules dock into assemblies, release, and rearrange. A logical connection must stay consistent with the physical constraint that joins the bodies.</p>
          <p>ModSim separates the meaning of a connection from the engine that enforces it. Compatibility rules, connector-frame tolerances, lifecycle guards, and committed events stay in one place, while a backend supplies physical observations and constraints.</p>
        </section>

        <section className={styles.section} id="docking" aria-labelledby="docking-title">
          <p className={styles.eyebrow}>Try the connection rules</p>
          <h2 id="docking-title">What decides whether two modules dock?</h2>
          <p>Connector types declare an acceptance region, alignment tolerances, and a relative-speed limit. Try moving a module toward its partner and changing its orientation. This browser illustration explains the checks; the actual runtime also requires backend confirmation.</p>
          <DockingDemo />
          <Link className={styles.sectionLink} to="/docs/explanation/docking-lifecycle/">Follow the docking lifecycle <span aria-hidden="true">→</span></Link>
        </section>

        <section className={styles.section} id="method" aria-labelledby="method-title">
          <p className={styles.eyebrow}>Inside ModSim</p>
          <h2 id="method-title">Describe, connect, and inspect.</h2>
          <Architecture />
          <div className={styles.readingCopy}>
            <p><strong>Robot Packs</strong> keep URDF and meshes alongside strict YAML descriptions. Studio imports the mechanics, lets you author connectors and capabilities, and checks the data before a run.</p>
            <p><strong>Runtime and backends</strong> keep connection semantics in ModSim and physical integration in the adapter. The kinematic mock is included; MuJoCo adds contact, gravity, joint effort, and runtime welds and hinges.</p>
            <p><strong>Model views</strong> turn canonical state into reusable topology and lattice snapshots. Planners publish their intent separately so the Inspector can show the target next to the measured result.</p>
          </div>
          <Link className={styles.sectionLink} to="/docs/explanation/">Explore the architecture and UML <span aria-hidden="true">→</span></Link>
          <h3 className={styles.widgetHeading}>Connections become a topology.</h3>
          <p>Try assembling modules in the browser playground. It illustrates how adjacency and committed graph connections relate; it does not launch a simulator.</p>
          <TopologyPlayground />
          <h3 className={styles.widgetHeading}>Inspect what happened—and what comes next.</h3>
          <p>The Runtime Inspector combines generated model views, events, and planner decisions. These recordings show the desktop application in use.</p>
          <InspectorRecordings />
          <p className={styles.replayIntro}>Scrub recorded plans and runtime samples to compare connections, measured poses, and the planning sequence.</p>
          <ModelViewDemo />
        </section>

        <section className={styles.section} id="robotpack-library" aria-labelledby="robotpack-library-title">
          <p className={styles.eyebrow}>The community library</p>
          <h2 id="robotpack-library-title">RobotPack Library</h2>
          <p>Find a robot. Share what you build. The RobotPack Library is an online repository where the community can share complete Robot Packs and download them for simulation in ModSim.</p>
          <p>Keep the robot description and its assets together: semantic YAML, URDF, meshes, and mappings travel as one pack, so another builder can explore and use the robot you authored.</p>
          <div className={styles.principles}>
            <article>
              <span className={styles.smallNumber}>01 / Browse</span>
              <h3>Discover a robot.</h3>
              <p>Explore shared packs, inspect their YAML and supporting assets, preview meshes, and review validation reports before downloading the complete pack.</p>
            </article>
            <article>
              <span className={styles.smallNumber}>02 / Simulate</span>
              <h3>Open it in Studio.</h3>
              <p>Extract a downloaded pack and open its folder in Studio. Inspect the robot, then use the Runtime workspace to choose a backend and model view, validate the launch, and start your simulation.</p>
              <Link className={styles.sectionLink} to="/docs/how-to/run-robot-pack/">Follow the runtime guide <span aria-hidden="true">→</span></Link>
            </article>
            <article>
              <span className={styles.smallNumber}>03 / Share</span>
              <h3>Contribute your own.</h3>
              <p>Build and save your Robot Pack in Studio, then upload the complete folder or ZIP to the library. Share your robot with the community so others can use it in their own experiments.</p>
            </article>
          </div>
          <div className={`${styles.heroActions} ${styles.libraryActions}`}>
            {libraryUrl && <a className="button button--primary" href={libraryUrl}>Browse the RobotPack Library <span aria-hidden="true">↗</span></a>}
            <Link className={libraryUrl ? 'button button--outline button--secondary' : 'button button--primary'} to="/docs/how-to/">Build a RobotPack</Link>
          </div>
        </section>

        <section className={styles.section} id="experiments" aria-labelledby="experiments-title">
          <p className={styles.eyebrow}>Experiments</p>
          <h2 id="experiments-title">From a connector pair to a new configuration.</h2>
          <p>Example Robot Packs, named demonstrations, and reference tests make the experiments reproducible. Larger robot assets are supplied separately from the installed Generic Cube starter.</p>
          <article className={styles.experiment}>
            <div className={styles.experimentHeading}><span className={styles.smallNumber}>01</span><div><h3>SMORES-EP dock and undock</h3><span className={styles.experimentMeta}>Wheel effort · measured connector frames · release</span></div></div>
            <p>A differential-drive module approaches another module’s connector, commits a weld after acceptance, holds, releases, and reverses. The semantic graph and event log describe the same authoritative simulation.</p>
            <Link className={styles.sectionLink} to="/docs/how-to/run-robot-pack/">Start with the docking workflow <span aria-hidden="true">→</span></Link>
          </article>
          <article className={styles.experiment}>
            <div className={styles.experimentHeading}><span className={styles.smallNumber}>02</span><div><h3>SMORES-EP online self-assembly</h3><span className={styles.experimentMeta}>Assignment · routes · action scheduling</span></div></div>
            <p>Seven modules assemble toward a target topology using an online planner, wheel-driven routes, and measured docking checks. Planner status exposes stalls, retries, and time limits.</p>
            <Recording file="smores_online_assembly_6x.mp4" label="Recorded SMORES-EP online self-assembly"
              caption="SMORES-EP online assembly recording · 6× speed. MuJoCo and the Runtime Inspector observe the same run." />
            <Link className={styles.sectionLink} to="/docs/explanation/planning-and-scenarios/">Understand planning and execution <span aria-hidden="true">→</span></Link>
          </article>
          <article className={styles.experiment}>
            <div className={styles.experimentHeading}><span className={styles.smallNumber}>03</span><div><h3>M-Blocks planar reconfiguration</h3><span className={styles.experimentMeta}>Lattice planning · flywheel effort · landing checks</span></div></div>
            <p>Six cubes reconfigure from a rectangle into a line. The planner generates lattice pivots; the physical controller spins up a flywheel, pivots around an edge hinge, and checks the landing before continuing.</p>
            <Recording file="mblocks_planar_reconf_3x.mp4" label="Recorded M-Blocks planar reconfiguration"
              caption="Six-module planar reconfiguration recording · 3× speed. Generated moves are executed by the physical pivot controller." />
            <a className={styles.sectionLink} href={`${repository}/blob/main/docs/mblocks_planning.md`}>Reproduce the M-Blocks reference <span aria-hidden="true">↗</span></a>
          </article>
        </section>

        <section className={styles.section} id="results" aria-labelledby="results-title">
          <p className={styles.eyebrow}>What the examples demonstrate</p>
          <h2 id="results-title">Connections you can inspect and reproduce.</h2>
          <div className={styles.resultRows}>
            <div><h3>Commit after confirmation</h3><p>A logical connection is committed after backend success, with matching assembly and event updates.</p></div>
            <div><h3>Separate intent from measurement</h3><p>Target configurations and routes remain distinct from measured poses, connections, and landing diagnostics.</p></div>
            <div><h3>Expose the outcome</h3><p>Completion, failure, stopping, and an exhausted time budget remain visible in the Inspector and event history.</p></div>
          </div>
          <p className={styles.scopeNote}>Physical examples have documented controller and geometry limits. The <Link to="/docs/explanation/planning-and-scenarios/">planning chapter</Link> and repository tests describe supported reference cases and experimental routes.</p>
        </section>

        <section className={`${styles.section} ${styles.start}`} id="start" aria-labelledby="start-title">
          <p className={styles.eyebrow}>Your first Robot Pack</p>
          <h2 id="start-title">Open Studio. Start connecting.</h2>
          <p>The default Python package includes Studio and the mock backend. Setup creates your local library and installs Generic Cube.</p>
          <CodeBlock language="bash">{'python -m pip install modsim-robotics\nmodsim-robotics setup\nmodsim-robotics studio generic_cube'}</CodeBlock>
          <p>For physics, add <code>modsim-robotics[mujoco]</code>. In Studio, switch to <strong>Runtime</strong>, choose the backend and model view, validate the launch, and start.</p>
          <div className={styles.heroActions}>
            <Link className="button button--primary" to="/docs/installation/">Installation guide</Link>
            <Link className={styles.textLink} to="/docs/how-to/">Build a pack from your URDF <span aria-hidden="true">→</span></Link>
          </div>
        </section>

        <section className={styles.section} id="citation" aria-labelledby="citation-title">
          <p className={styles.eyebrow}>Research</p>
          <h2 id="citation-title">Cite ModSim</h2>
          <p>If ModSim is useful in your work, please cite the project:</p>
          <CodeBlock language="text">{`@misc{wong2026modsim,
  title = {ModSim: An Open-Source Framework for Dynamic
           Modular Robot Simulation},
  author = {Wong, Angelina and Panyam, Akshay Ram and
            Roede, Nils and Yim, Mark},
  year = {2026},
  howpublished = {\\url{https://github.com/modlab-upenn/ModSim}}
}`}</CodeBlock>
          <a className={styles.sectionLink} href={`${repository}/blob/main/docs/technical_report/modsim_technical_report.tex`}>Read the technical report source <span aria-hidden="true">↗</span></a>
        </section>
      </div>
    </main>
  </Layout>;
}
