import React, {useEffect, useId, useRef, useState} from 'react';
import {clamp, ease, EventTrace, svgPoint, useReducedMotion, type Point, type TraceEvent} from './shared';
import styles from './widgets.module.css';

type Pose = Point & {yaw: number};
type DemoState = Pose & {docked: boolean; velocity: number; events: TraceEvent[]};
const PX = 1.8;
const HALF = 45;
const FIXED = {x: 175, y: 130};
const TARGET = {x: 265, y: 130};
const START: Pose = {x: 470, y: 150, yaw: 10};
const INITIAL: DemoState = {...START, docked: false, velocity: 0, events: [{kind: 'Ready', detail: '2 modules · compatible fixed-face connectors'}]};

function connector(pose: Pose): Point {
  const angle = pose.yaw * Math.PI / 180;
  return {x: pose.x - HALF * Math.cos(angle), y: pose.y - HALF * Math.sin(angle)};
}
function intersects(pose: Pose): boolean {
  const angle = pose.yaw * Math.PI / 180, c = Math.cos(angle), s = Math.sin(angle);
  return [[1, 0], [0, 1], [c, s], [-s, c]].every(([nx, ny]) =>
    Math.abs((pose.x - FIXED.x) * nx + (pose.y - FIXED.y) * ny) <
    HALF * (Math.abs(nx) + Math.abs(ny) + Math.abs(nx * c + ny * s) + Math.abs(-nx * s + ny * c)) - 0.5);
}
function collisionFree(start: Pose, desired: Pose): Pose {
  const destination = {...desired, x: clamp(desired.x, 65, 575), y: clamp(desired.y, 65, 225)};
  if (!intersects(destination)) return destination;
  let low = 0, high = 1;
  for (let index = 0; index < 16; index++) {
    const ratio = (low + high) / 2;
    const pose = {x: start.x + (destination.x - start.x) * ratio, y: start.y + (destination.y - start.y) * ratio, yaw: start.yaw};
    if (intersects(pose)) high = ratio; else low = ratio;
  }
  return {x: start.x + (destination.x - start.x) * low, y: start.y + (destination.y - start.y) * low, yaw: start.yaw};
}

export function DockingDemo(): React.JSX.Element {
  const [state, setState] = useState<DemoState>(INITIAL);
  const current = useRef(state);
  const svg = useRef<SVGSVGElement>(null);
  const drag = useRef<{offset: Point; point: Point; at: number} | null>(null);
  const frame = useRef(0);
  const cooldown = useRef(false);
  const reduced = useReducedMotion();
  const marker = `dock-arrow-${useId().replace(/:/g, '')}`;
  useEffect(() => () => cancelAnimationFrame(frame.current), []);

  function update(pose: Pose, velocity = 0): void {
    let next = {...current.current, ...pose, velocity};
    const point = connector(pose);
    const distance = Math.hypot(point.x - FIXED.x - HALF, point.y - FIXED.y) / PX;
    if (distance > 30) cooldown.current = false;
    if (!next.docked && !cooldown.current && distance <= 6 && Math.abs(pose.yaw) <= 15 && velocity <= 50) {
      next = {...next, ...TARGET, yaw: 0, docked: true, velocity: 0, events: [...next.events,
        {kind: 'DockCandidateDetected', detail: 'module_0/front ↔ module_1/rear'},
        {kind: 'Acceptance', detail: 'position, axis alignment, and speed pass'},
        {kind: 'Constraint confirmed', detail: 'illustrated backend commit boundary'},
        {kind: 'DockCommitted', detail: 'fixed connection recorded'},
        {kind: 'AssemblyMerged', detail: '2 modules → 1 assembly'},
      ].slice(-30)};
      cancelAnimationFrame(frame.current);
      drag.current = null;
    }
    current.current = next;
    setState(next);
  }

  function animate(): void {
    cancelAnimationFrame(frame.current);
    if (current.current.docked) return;
    const from = {...current.current};
    const startTime = performance.now();
    const duration = reduced ? 1 : Math.max(1000, Math.hypot(from.x - TARGET.x, from.y - TARGET.y) / PX / 40 * 1000);
    let previous = connector(from), previousTime = startTime;
    const tick = (now: number) => {
      const progress = clamp((now - startTime) / duration, 0, 1), blend = ease(progress);
      const pose = collisionFree(current.current, {x: from.x + (TARGET.x - from.x) * blend, y: from.y + (TARGET.y - from.y) * blend, yaw: from.yaw * (1 - blend)});
      const point = connector(pose);
      const velocity = progress >= 1 || reduced ? 0 : Math.hypot(point.x - previous.x, point.y - previous.y) / PX / Math.max((now - previousTime) / 1000, 0.001);
      previous = point;
      previousTime = now;
      update(pose, velocity);
      if (progress < 1 && !current.current.docked) frame.current = requestAnimationFrame(tick);
    };
    frame.current = requestAnimationFrame(tick);
  }

  function reset(): void {
    cancelAnimationFrame(frame.current);
    drag.current = null;
    cooldown.current = false;
    current.current = INITIAL;
    setState(INITIAL);
  }
  function undock(): void {
    cancelAnimationFrame(frame.current);
    cooldown.current = true;
    const next = {...current.current, x: TARGET.x + 110, docked: false, events: [...current.current.events,
      {kind: 'UndockCommitted', detail: 'fixed constraint released'},
      {kind: 'AssemblySplit', detail: '1 assembly → 2 modules'},
      {kind: 'Retract', detail: 'scripted separation after release'},
    ].slice(-30)};
    current.current = next;
    setState(next);
  }
  const point = connector(state);
  const distance = Math.hypot(point.x - FIXED.x - HALF, point.y - FIXED.y) / PX;
  const status = state.docked ? 'docked' : distance <= 30 ? 'candidate' : 'free';
  const measurements = [
    ['Position', `${distance.toFixed(1)} mm`, '≤ 6 mm', distance <= 6],
    ['Axis alignment', `${Math.abs(state.yaw).toFixed(0)}°`, '≤ 15°', Math.abs(state.yaw) <= 15],
    ['Relative speed', `${state.velocity.toFixed(0)} mm/s`, '≤ 50 mm/s', state.velocity <= 50],
  ];

  return <figure className={styles.widget} aria-label="Interactive docking acceptance demonstration">
    <div className={styles.header}><strong>Try docking two modules</strong><span className={styles.badge} data-state={status} aria-live="polite">{status}</span></div>
    <div className={styles.stageScroll}>
    <svg ref={svg} className={styles.canvas} viewBox="0 0 640 280" role="group" aria-label="Move the right module toward the left module's front connector">
      <defs><marker id={marker} viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" fill="var(--modsim-muted)"/></marker></defs>
      {Array.from({length: 17}, (_, i) => <line key={`x${i}`} x1={i * 40} x2={i * 40} y1={0} y2={280} stroke="var(--modsim-border)" strokeOpacity={0.35}/>)}
      {Array.from({length: 8}, (_, i) => <line key={`y${i}`} y1={i * 40} y2={i * 40} x1={0} x2={640} stroke="var(--modsim-border)" strokeOpacity={0.35}/>)}
      <circle cx={FIXED.x + HALF} cy={FIXED.y} r={54} fill="none" stroke="var(--modsim-warning)" strokeDasharray="4 4" opacity={0.55}/>
      <rect x={FIXED.x + HALF - 10.8} y={FIXED.y - 10.8} width={21.6} height={21.6} fill="var(--modsim-accent)" fillOpacity={0.15} stroke="var(--modsim-accent)" strokeDasharray="3 3"/>
      <g transform={`translate(${FIXED.x} ${FIXED.y})`}>
        <rect x={-HALF} y={-HALF} width={90} height={90} rx={7} fill="var(--modsim-accent)"/>
        <text x={0} y={5} textAnchor="middle" fill="var(--modsim-accent-ink)" fontSize={13}>module_0</text>
        <rect x={HALF - 7} y={-14} width={7} height={28} fill="var(--modsim-accent-ink)"/>
        <text x={HALF - 9} y={32} textAnchor="end" fill="var(--modsim-accent-ink)" fontSize={11}>front</text>
        {!state.docked && <line x1={HALF + 8} x2={HALF + 32} y1={0} y2={0} stroke="var(--modsim-muted)" markerEnd={`url(#${marker})`}/>}
      </g>
      <g transform={`translate(${state.x} ${state.y}) rotate(${state.yaw})`} className={state.docked ? undefined : styles.interactive}
        role="button" tabIndex={0} aria-label="Moving module: drag, or use arrow keys to move and R to rotate" aria-disabled={state.docked}
        onKeyDown={event => {
          if (state.docked) return;
          const direction = {ArrowLeft: [-4, 0], ArrowRight: [4, 0], ArrowUp: [0, -4], ArrowDown: [0, 4]}[event.key];
          if (direction) { event.preventDefault(); update(collisionFree(state, {...state, x: state.x + direction[0], y: state.y + direction[1]})); }
          if (event.key.toLowerCase() === 'r') { event.preventDefault(); const pose = {...state, yaw: state.yaw >= 30 ? -30 : state.yaw + 5}; if (!intersects(pose)) update(pose); }
        }}
        onPointerDown={event => {
          if (state.docked || !svg.current) return;
          cancelAnimationFrame(frame.current);
          const p = svgPoint(svg.current, event.clientX, event.clientY);
          drag.current = {offset: {x: state.x - p.x, y: state.y - p.y}, point: p, at: performance.now()};
          event.currentTarget.setPointerCapture(event.pointerId);
        }}
        onPointerMove={event => {
          if (!drag.current || !svg.current || current.current.docked) return;
          const p = svgPoint(svg.current, event.clientX, event.clientY), now = performance.now();
          const speed = Math.hypot(p.x - drag.current.point.x, p.y - drag.current.point.y) / PX / Math.max((now - drag.current.at) / 1000, 0.001);
          update(collisionFree(current.current, {...current.current, x: p.x + drag.current.offset.x, y: p.y + drag.current.offset.y}), speed);
          if (drag.current) { drag.current.point = p; drag.current.at = now; }
        }}
        onPointerUp={() => { drag.current = null; if (!current.current.docked) update(current.current, 0); }}
        onPointerCancel={() => { drag.current = null; if (!current.current.docked) update(current.current, 0); }}>
        <rect x={-HALF} y={-HALF} width={90} height={90} rx={7} fill={state.docked ? 'var(--modsim-accent)' : 'var(--modsim-blue)'}/>
        <text x={0} y={5} textAnchor="middle" fill="var(--modsim-accent-ink)" fontSize={13}>module_1</text>
        <rect x={-HALF} y={-14} width={7} height={28} fill="var(--modsim-accent-ink)"/>
        <text x={-HALF + 9} y={32} fill="var(--modsim-accent-ink)" fontSize={11}>rear</text>
        {!state.docked && <line x1={-HALF - 8} x2={-HALF - 32} y1={0} y2={0} stroke="var(--modsim-muted)" markerEnd={`url(#${marker})`}/>}
      </g>
    </svg>
    </div>
    <div className={styles.controls}>
      <label className={styles.control}>Rotate module <input aria-label="Rotate moving module" type="range" min={-30} max={30} step={1} value={state.yaw} disabled={state.docked} onChange={event => {
        cancelAnimationFrame(frame.current);
        const pose = {...current.current, yaw: Number(event.target.value)};
        if (intersects(pose)) pose.x = Math.max(pose.x, FIXED.x + HALF * 2.5);
        update(pose);
      }}/><output>{state.yaw.toFixed(0)}°</output></label>
      <button type="button" className={`${styles.controlButton} ${styles.primary}`} onClick={animate} disabled={state.docked}>Animate</button>
      <button type="button" className={styles.controlButton} onClick={undock} disabled={!state.docked}>Undock</button>
      <button type="button" className={styles.controlButton} onClick={reset}>Reset</button>
    </div>
    <div className={styles.detailGrid}>
      <div className={styles.tableWrap}><table className={styles.criteria}><thead><tr><th>Criterion</th><th>Measured</th><th>Limit</th><th>Pass</th></tr></thead><tbody>
        {measurements.map(([name, value, limit, pass]) => <tr key={String(name)}><td>{name}</td><td>{value}</td><td>{limit}</td><td className={pass ? styles.pass : styles.fail}>{pass ? '✓' : '×'}</td></tr>)}
      </tbody></table></div>
      <EventTrace events={state.events}/>
    </div>
    <figcaption className={styles.note}>Browser illustration of compatible, auto-latching connectors with nominal alignment. Drag or use arrow keys; R rotates. The 2D view omits roll and additional policy guards. It illustrates acceptance and commit ordering without running a physics backend.</figcaption>
  </figure>;
}
