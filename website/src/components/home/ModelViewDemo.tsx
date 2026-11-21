import React, {useId, useMemo, useState} from 'react';
import {latticeData, smoresData, topologyData, type Cell, type PortPair, type Pose2D, type RecordedAction} from './replayData';
import {clamp, ease, EventTrace, usePlayback, type Point, type TraceEvent} from './shared';
import styles from './widgets.module.css';

type View = 'topology' | 'lattice' | 'planning';
type ReplayResult = {picture: React.JSX.Element; title: string; meta: string; events: TraceEvent[]};
const COLORS = ['#2F5DA8', '#C2410C', '#7C3AED', '#15803D', '#BE185D', '#0E7490', '#B91C1C'];
const TOPOLOGY_DURATION = 20;
const LATTICE_DURATION = 1 + latticeData.actions.length * 1.7 + 2.8;
const PLANNING_DURATION = 45;
const edgeKey = (pair: PortPair): string => [...pair].sort().join('|');
const moduleId = (port: string): string => port.split('/')[0];
const shortId = (id: string): string => id.replace('module_', 'm').replace('block_', 'b');

function topologyLayouts(): Record<string, Point>[] {
  const initial: Record<string, Point> = {};
  const points = [[80, 80], [170, 170], [80, 260], [305, 170], [440, 170], [530, 80], [530, 260]];
  topologyData.modules.forEach((id, index) => { initial[id] = {x: points[index][0], y: points[index][1]}; });
  const layouts = [initial];
  // A stable graph layout, independent of measured module poses. Only the
  // rewiring and mover membership come from the recorded plan.
  for (let stage = 1; stage <= topologyData.actions.length; stage++) {
    const previous = layouts[stage - 1];
    const progress = stage / topologyData.actions.length;
    const next: Record<string, Point> = {};
    topologyData.chain.forEach((id, index) => {
      const target = {x: 58 + index * 82, y: 170};
      next[id] = {x: previous[id].x + (target.x - previous[id].x) * progress, y: previous[id].y + (target.y - previous[id].y) * progress};
    });
    layouts.push(next);
  }
  return layouts;
}
const TOPOLOGY_LAYOUTS = topologyLayouts();

function Edge({pair, positions, color = 'var(--modsim-accent)', width = 3}: {pair: PortPair; positions: Record<string, Point>; color?: string; width?: number}): React.JSX.Element {
  const a = positions[moduleId(pair[0])], b = positions[moduleId(pair[1])];
  const dx = b.x - a.x, dy = b.y - a.y, length = Math.max(1, Math.hypot(dx, dy));
  return <g>
    <line x1={a.x} y1={a.y} x2={b.x} y2={b.y} stroke={color} strokeWidth={width}/>
    {pair.map((port, index) => {
      const mix = index === 0 ? 0.25 : 0.75;
      return <text key={index} x={a.x + dx * mix + dy / length * 10} y={a.y + dy * mix - dx / length * 10} fontSize={11} textAnchor="middle" fill="var(--modsim-ink)" stroke="var(--modsim-reading-bg)" strokeWidth={4} paintOrder="stroke">{port.split('/')[1]}</text>;
    })}
  </g>;
}

function topologyFrame(time: number): ReplayResult {
  const phase = clamp(Math.floor((time - 1) / 4), 0, topologyData.actions.length);
  const local = clamp((time - 1 - phase * 4) / 4, 0, 1);
  let edges = topologyData.initial;
  const events: TraceEvent[] = [{kind: 'Recorded plan', detail: '7 SMORES-EP modules · Driver → Snake'}];
  for (let i = 0; i < phase; i++) {
    edges = edges.filter(pair => edgeKey(pair) !== edgeKey(topologyData.actions[i].undock)).concat([topologyData.actions[i].dock]);
    events.push({kind: 'UndockCommitted', detail: topologyData.actions[i].undock.join(' ↔ ')}, {kind: 'DockCommitted', detail: topologyData.actions[i].dock.join(' ↔ ')});
  }
  const active = time >= 1 && phase < topologyData.actions.length ? topologyData.actions[phase] : null;
  const progress = active ? ease(clamp((local - 0.15) / 0.65, 0, 1)) : 0;
  const positions: Record<string, Point> = {};
  const from = TOPOLOGY_LAYOUTS[phase], to = TOPOLOGY_LAYOUTS[Math.min(phase + 1, topologyData.actions.length)];
  for (const id of topologyData.modules) positions[id] = {x: from[id].x + (to[id].x - from[id].x) * progress, y: from[id].y + (to[id].y - from[id].y) * progress};
  if (active && local > 0.12) {
    edges = edges.filter(pair => edgeKey(pair) !== edgeKey(active.undock));
    events.push({kind: 'UndockCommitted', detail: active.undock.join(' ↔ ')});
  }
  if (active && local >= 0.82) {
    edges = edges.concat([active.dock]);
    events.push({kind: 'DockCommitted', detail: active.dock.join(' ↔ ')});
  }
  const target: Record<string, Point> = {};
  topologyData.chain.forEach((id, index) => { target[id] = {x: 755, y: 62 + index * 45}; });
  return {
    title: time < 1 ? 'Driver configuration · 7 modules' : active ? `Action ${phase + 1}/4 · ${active.label}` : 'Target topology reached · Snake',
    meta: `${edges.length} committed connections · node positions are graph layout`,
    events,
    picture: <>
      <line x1={620} x2={620} y1={0} y2={400} stroke="var(--modsim-border)"/>
      <text x={18} y={25} fontSize={15} fontWeight={600} fill="var(--modsim-ink)">Topology replay</text>
      <text x={650} y={25} fontSize={15} fontWeight={600} fill="var(--modsim-ink)">Target topology</text>
      {edges.map(pair => <Edge key={edgeKey(pair)} pair={pair} positions={positions}/>)}
      {topologyData.modules.map((id, index) => <g key={id}><circle cx={positions[id].x} cy={positions[id].y} r={19} fill={COLORS[index]} stroke={active?.movers.includes(id) ? 'var(--modsim-ink)' : 'var(--modsim-surface)'} strokeWidth={2}/><text x={positions[id].x} y={positions[id].y + 35} fontSize={13} textAnchor="middle" fill="var(--modsim-ink)">{shortId(id)}</text></g>)}
      {topologyData.target.map(pair => <Edge key={edgeKey(pair)} pair={pair} positions={target} width={2}/>)}
      {topologyData.modules.map((id, index) => <g key={id}><circle cx={target[id].x} cy={target[id].y} r={10} fill={COLORS[index]}/><text x={target[id].x + 22} y={target[id].y + 5} fontSize={12} fill="var(--modsim-ink)">{shortId(id)}</text></g>)}
    </>,
  };
}

const LATTICE_CELLS = [...Object.values(latticeData.initial), ...latticeData.goal, ...latticeData.actions.flatMap(action => [action[2], action[3], ...action[6]])];
const LX = Math.min(...LATTICE_CELLS.map(cell => cell[0]));
const LY = Math.min(...LATTICE_CELLS.map(cell => cell[1]));
const LROWS = Math.max(...LATTICE_CELLS.map(cell => cell[0])) - LX + 1;
const LCOLS = Math.max(...LATTICE_CELLS.map(cell => cell[1])) - LY + 1;
const LS = Math.min(820 / LCOLS, 290 / LROWS);
const latticePoint = ([x, y]: Cell): Point => ({x: (900 - LCOLS * LS) / 2 + (y - LY + 0.5) * LS, y: 60 + (290 - LROWS * LS) / 2 + (x - LX + 0.5) * LS});

function latticeFrame(time: number): ReplayResult {
  const phase = clamp(Math.floor((time - 1) / 1.7), 0, latticeData.actions.length);
  const local = Math.max(0, time - 1 - phase * 1.7);
  const cells = {...latticeData.initial};
  const events: TraceEvent[] = [{kind: 'Recorded plan', detail: `6 cubes · rectangle → line · ${latticeData.actions.length} pivots`}];
  for (let i = 0; i < phase; i++) {
    const action = latticeData.actions[i]; cells[action[0]] = action[3];
    events.push({kind: 'Pivot', detail: `${shortId(action[0])} about ${shortId(action[1])} → (${action[3].join(', ')})`});
  }
  const active = time >= 1 && phase < latticeData.actions.length ? latticeData.actions[phase] : null;
  const progress = ease(clamp((local - 0.25) / 1, 0, 1));
  return {
    title: time < 1 ? 'Initial 2 × 3 rectangle' : active ? `Pivot ${phase + 1}/${latticeData.actions.length} · ${shortId(active[0])} about ${shortId(active[1])}` : 'Planner target reached · line',
    meta: active ? `${Math.abs(active[5]) * 90}° rotation · ${active[7]} · fixed anchor ${latticeData.anchor}` : 'Plan-only replay · lattice coordinates retained',
    events,
    picture: <>
      <text x={18} y={25} fontSize={15} fontWeight={600} fill="var(--modsim-ink)">Cubic lattice · top view</text>
      {Array.from({length: LROWS}, (_, x) => Array.from({length: LCOLS}, (_, y) => {
        const p = latticePoint([x + LX, y + LY]);
        return <rect key={`${x},${y}`} x={p.x - LS / 2} y={p.y - LS / 2} width={LS} height={LS} fill="none" stroke="var(--modsim-border)" strokeOpacity={0.6}/>;
      }))}
      {active && local < 1.35 && active[6].map(cell => { const p = latticePoint(cell); return <rect key={cell.join(',')} x={p.x - LS / 2 + 1} y={p.y - LS / 2 + 1} width={LS - 2} height={LS - 2} fill="var(--modsim-warning)" fillOpacity={0.17}/>; })}
      {latticeData.goal.map(cell => { const p = latticePoint(cell); return <rect key={cell.join(',')} x={p.x - LS / 2 + 4} y={p.y - LS / 2 + 4} width={LS - 8} height={LS - 8} rx={5} fill="none" stroke="#7C3AED" strokeWidth={2} strokeDasharray="5 4"/>; })}
      {Object.entries(cells).map(([id, cell]) => {
        const p = latticePoint(cell), moving = active?.[0] === id;
        const pivot = active ? latticePoint([active[4][0] / 2, active[4][1] / 2]) : p;
        return <g key={id} transform={moving ? `rotate(${-active[5] * 90 * progress} ${pivot.x} ${pivot.y})` : undefined}>
          <rect x={p.x - LS / 2 + 3} y={p.y - LS / 2 + 3} width={LS - 6} height={LS - 6} rx={5} fill="var(--modsim-accent)" stroke={active?.[1] === id ? 'var(--modsim-warning)' : 'var(--modsim-reading-bg)'} strokeWidth={active?.[1] === id ? 3 : 1}/>
          <text x={p.x} y={p.y + 5} textAnchor="middle" fontSize={Math.max(13, LS * 0.23)} fill="var(--modsim-accent-ink)">{shortId(id)}</text>
        </g>;
      })}
      {active && <circle cx={latticePoint([active[4][0] / 2, active[4][1] / 2]).x} cy={latticePoint([active[4][0] / 2, active[4][1] / 2]).y} r={4} fill="var(--modsim-warning)"/>}
      <text x={450} y={385} textAnchor="middle" fontSize={13} fill="var(--modsim-muted)">Dashed violet: target cells · amber: required clearance · dot: pivot edge</text>
    </>,
  };
}

type Sample = {time: number; poses: Pose2D[]; actions: RecordedAction[]; revision: number | null; replans: number; detail: string};
const SAMPLES: Sample[] = [];
let lastActions: RecordedAction[] = [];
for (const [time, poses, actions, revision, replans, detail] of smoresData.samples) {
  if (actions !== 0) lastActions = actions;
  SAMPLES.push({time, poses, actions: lastActions, revision, replans, detail: smoresData.details[detail]});
}
const SIM_DURATION = SAMPLES[SAMPLES.length - 1].time;
const ALL_POSES = [...SAMPLES.flatMap(sample => sample.poses), ...Object.values(smoresData.plans).flatMap(plan => Object.values(plan.targets)), ...smoresData.routes.flat()];
const XMIN = Math.min(...ALL_POSES.map(pose => pose[0])) - 0.1;
const XMAX = Math.max(...ALL_POSES.map(pose => pose[0])) + 0.1;
const YMIN = Math.min(...ALL_POSES.map(pose => pose[1])) - 0.1;
const YMAX = Math.max(...ALL_POSES.map(pose => pose[1])) + 0.1;
const SCALE = Math.min(830 / (XMAX - XMIN), 330 / (YMAX - YMIN));
const screen = (x: number, y: number): Point => ({x: (900 - (XMAX - XMIN) * SCALE) / 2 + (x - XMIN) * SCALE, y: 50 + (360 - (YMAX - YMIN) * SCALE) / 2 + (YMAX - y) * SCALE});
const wrap = (angle: number): number => Math.atan2(Math.sin(angle), Math.cos(angle));
function centre(pose: Pose2D): Point { return {x: pose[0] + Math.cos(pose[2]) * smoresData.footprint_cx, y: pose[1] + Math.sin(pose[2]) * smoresData.footprint_cx}; }
function footprint(pose: Pose2D, grow = 0): string {
  const p = centre(pose), c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  const [hx, hy] = smoresData.footprint_half;
  return [[hx + grow, hy + grow], [-hx - grow, hy + grow], [-hx - grow, -hy - grow], [hx + grow, -hy - grow]].map(([x, y]) => {
    const point = screen(p.x + x * c - y * s, p.y + x * s + y * c);
    return `${point.x.toFixed(2)},${point.y.toFixed(2)}`;
  }).join(' ');
}

function planningFrame(time: number): ReplayResult {
  const simulationTime = clamp(time / 42, 0, 1) * SIM_DURATION;
  let low = 0, high = SAMPLES.length - 1;
  while (high - low > 1) { const mid = (low + high) >> 1; if (SAMPLES[mid].time <= simulationTime) low = mid; else high = mid; }
  const a = SAMPLES[low], b = SAMPLES[high], sample = simulationTime >= b.time ? b : a;
  const ratio = b.time > a.time ? clamp((simulationTime - a.time) / (b.time - a.time), 0, 1) : 0;
  const poses = a.poses.map((pose, index): Pose2D => [pose[0] + (b.poses[index][0] - pose[0]) * ratio, pose[1] + (b.poses[index][1] - pose[1]) * ratio, pose[2] + wrap(b.poses[index][2] - pose[2]) * ratio]);
  const plan = sample.revision === null ? null : smoresData.plans[sample.revision];
  const events: TraceEvent[] = [{kind: 'Recording', detail: 'Historical headless SMORES-EP assembly run'}];
  smoresData.events.filter(event => event[0] <= simulationTime).forEach(([t, kind, count]) => events.push({kind: kind === 'dock' ? 'DockCommitted' : 'UndockCommitted', detail: `t=${t.toFixed(1)} s · ${count} connections`}));
  return {
    title: sample.detail,
    meta: `t=${simulationTime.toFixed(1)} s recorded · plan r${sample.revision ?? '—'} · ${sample.replans} replans`,
    events,
    picture: <>
      <text x={18} y={25} fontSize={15} fontWeight={600} fill="var(--modsim-ink)">Measured poses and planner intent</text>
      {Array.from({length: Math.ceil((XMAX - XMIN) * 10)}, (_, i) => { const x = XMIN + i / 10; const p = screen(x, YMIN), q = screen(x, YMAX); return <line key={`x${i}`} x1={p.x} x2={q.x} y1={p.y} y2={q.y} stroke="var(--modsim-border)" strokeOpacity={0.55}/>; })}
      {Array.from({length: Math.ceil((YMAX - YMIN) * 10)}, (_, i) => { const y = YMIN + i / 10; const p = screen(XMIN, y), q = screen(XMAX, y); return <line key={`y${i}`} x1={p.x} x2={q.x} y1={p.y} y2={q.y} stroke="var(--modsim-border)" strokeOpacity={0.55}/>; })}
      {plan && Object.entries(plan.targets).map(([id, pose]) => <polygon key={`goal:${id}`} points={footprint(pose)} fill="none" stroke={COLORS[smoresData.ids.indexOf(id)]} strokeDasharray="7 5" strokeWidth={1.6}/>)}
      {smoresData.ids.map((id, index) => {
        const trail = SAMPLES.slice(0, low + 1).filter((_, n) => n % 3 === 0).map(frame => { const p = centre(frame.poses[index]); const q = screen(p.x, p.y); return `${q.x.toFixed(1)},${q.y.toFixed(1)}`; }).join(' ');
        return <polyline key={`trail:${id}`} points={trail} fill="none" stroke={COLORS[index]} strokeWidth={1.8} strokeOpacity={0.55}/>;
      })}
      {sample.actions.filter(action => ['navigating', 'aligning', 'retreating'].includes(action[2]) && action[3] >= 0).map(([id, , , routeIndex, waypoint]) => {
        const route = smoresData.routes[routeIndex];
        if (!route?.length) return null;
        let end = clamp(waypoint, 0, route.length - 1);
        while (end < route.length - 1 && route[end][3] === route[end + 1][3] && route[end][3] !== 0 && Math.abs(wrap(route[end + 1][2] - route[end][2])) <= 0.52) end++;
        return <polygon key={`subgoal:${id}`} points={footprint([route[end][0], route[end][1], route[end][2]], 0.005)} fill={COLORS[smoresData.ids.indexOf(id)]} fillOpacity={0.06} stroke={COLORS[smoresData.ids.indexOf(id)]} strokeWidth={2} strokeDasharray="3 3"/>;
      })}
      {smoresData.ids.map((id, index) => {
        const p = centre(poses[index]), q = screen(p.x, p.y), front = screen(p.x + Math.cos(poses[index][2]) * 0.035, p.y + Math.sin(poses[index][2]) * 0.035);
        return <g key={id}><polygon points={footprint(poses[index])} fill={COLORS[index]} fillOpacity={0.93} stroke="var(--modsim-reading-bg)" strokeWidth={1.2}/><line x1={q.x} y1={q.y} x2={front.x} y2={front.y} stroke="#FFFFFF" strokeWidth={2}/><text x={q.x} y={q.y - 22} textAnchor="middle" fontSize={12} fill="var(--modsim-ink)" stroke="var(--modsim-reading-bg)" strokeWidth={4} paintOrder="stroke">{shortId(id)}</text></g>;
      })}
      <text x={450} y={430} textAnchor="middle" fontSize={13} fill="var(--modsim-muted)">Solid: measured paths · long dashes: goals · short dashes: current route subgoals</text>
    </>,
  };
}

const VIEWS: {id: View; label: string; duration: number; note: string}[] = [
  {id: 'topology', label: 'Topology · SMORES-EP', duration: TOPOLOGY_DURATION, note: 'Replays four actions from the recorded Driver-to-Snake plan. Nodes represent modules; edges represent committed connections, labelled by their endpoint connector faces. Graph positions and transition timing are illustrative.'},
  {id: 'lattice', label: 'Lattice · M-Blocks', duration: LATTICE_DURATION, note: 'Replays 21 pivots from a previously captured six-cube planar planner output. The drawing shows planned lattice motion and required clearance; it does not execute the physics controller.'},
  {id: 'planning', label: 'Online planner · SMORES-EP', duration: PLANNING_DURATION, note: 'Replays 658 sampled frames from a historical MuJoCo SMORES-EP assembly recording. Measured poses are interpolated for display; goals and subgoals are planner intent. This is recorded data, not a live simulation.'},
];

export function ModelViewDemo(): React.JSX.Element {
  const [view, setView] = useState<View>('topology');
  const selected = VIEWS.find(item => item.id === view)!;
  const playback = usePlayback(selected.duration);
  const id = `model-replay-${useId().replace(/:/g, '')}`;
  const frame = useMemo(() => view === 'topology' ? topologyFrame(playback.time) : view === 'lattice' ? latticeFrame(playback.time) : planningFrame(playback.time), [view, playback.time]);
  function select(next: View): void { playback.setPlaying(false); playback.setTime(0); setView(next); }
  return <figure className={styles.widget} aria-label="Recorded model-view and planner replays">
    <div className={styles.tabs} role="tablist" aria-label="Choose a recorded model-view replay">
      {VIEWS.map((item, index) => <button key={item.id} id={`${id}-${item.id}`} className={`${styles.controlButton} ${styles.tab}`} type="button" role="tab" aria-selected={view === item.id} aria-controls={`${id}-panel`} tabIndex={view === item.id ? 0 : -1} onClick={() => select(item.id)}
        onKeyDown={event => {
          const direction = event.key === 'ArrowRight' ? 1 : event.key === 'ArrowLeft' ? -1 : 0;
          if (!direction && !['Home', 'End'].includes(event.key)) return;
          event.preventDefault();
          const next = event.key === 'Home' ? 0 : event.key === 'End' ? VIEWS.length - 1 : (index + direction + VIEWS.length) % VIEWS.length;
          select(VIEWS[next].id);
          document.getElementById(`${id}-${VIEWS[next].id}`)?.focus();
        }}>{item.label}</button>)}
    </div>
    <div id={`${id}-panel`} role="tabpanel" aria-labelledby={`${id}-${view}`}>
      <div className={styles.status}>{frame.title}<span>{frame.meta}</span></div>
      <div className={styles.stageScroll}><svg className={styles.canvas} viewBox={`0 0 900 ${view === 'planning' ? 450 : 400}`} role="img" aria-label={selected.label}>{frame.picture}</svg></div>
      <div className={styles.controls}>
        <button type="button" className={`${styles.controlButton} ${styles.primary}`} onClick={() => { if (playback.time >= selected.duration) playback.setTime(0); playback.setPlaying(!playback.playing); }}>{playback.playing ? 'Pause replay' : 'Play replay'}</button>
        <button type="button" className={styles.controlButton} onClick={() => { playback.setPlaying(false); playback.setTime(0); }}>Restart replay</button>
        <input className={styles.timeline} type="range" min={0} max={selected.duration} step={0.05} value={playback.time} aria-label="Replay time" onChange={event => { playback.setPlaying(false); playback.setTime(Number(event.target.value)); }}/>
        <span className={styles.time}>{playback.time.toFixed(1)} / {selected.duration.toFixed(1)} s</span>
      </div>
      <EventTrace events={frame.events} live={false}/>
    </div>
    <figcaption className={styles.note}>{selected.note} Choose Play or use the timeline; on small screens, scroll the drawing horizontally.</figcaption>
  </figure>;
}
