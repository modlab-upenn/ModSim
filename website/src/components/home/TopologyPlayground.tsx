import React, {useRef, useState} from 'react';
import {EventTrace, svgPoint, type Point, type TraceEvent} from './shared';
import styles from './widgets.module.css';

type Face = 'right' | 'bottom' | 'left' | 'pan';
type Module = Point & {rotation: number};
type Connection = {id: number; a: number; fa: Face; b: number; fb: Face};
type State = {modules: Module[]; connections: Connection[]; blocked: string[]; events: TraceEvent[]; nextId: number};
const FACES: Face[] = ['right', 'bottom', 'left', 'pan'];
const LOCAL: Record<Face, Point> = {right: {x: 1, y: 0}, bottom: {x: 0, y: 1}, left: {x: -1, y: 0}, pan: {x: 0, y: -1}};
const COLORS = ['#2F5DA8', '#C2410C', '#7C3AED', '#15803D', '#BE185D'];
const SIZE = 56;
const HALF = SIZE / 2;

function initialState(): State {
  return {
    modules: [{x: 110, y: 120, rotation: 0}, {x: 166, y: 120, rotation: 0}, {x: 330, y: 92, rotation: 1}, {x: 230, y: 300, rotation: 0}, {x: 440, y: 270, rotation: 3}],
    connections: [{id: 0, a: 0, fa: 'right', b: 1, fb: 'left'}], blocked: [], nextId: 1,
    events: [{kind: 'Ready', detail: '5 modules · 1 committed connection · 4 assemblies'}],
  };
}
function components(state: State): number[] {
  const parents = state.modules.map((_, index) => index);
  const find = (index: number): number => parents[index] === index ? index : (parents[index] = find(parents[index]));
  state.connections.forEach(connection => { parents[find(connection.a)] = find(connection.b); });
  return parents.map((_, index) => find(index));
}
function groupFor(state: State, index: number): number[] {
  const groups = components(state);
  return groups.flatMap((group, other) => group === groups[index] ? [other] : []);
}
function direction(module: Module, face: Face): Point {
  let {x, y} = LOCAL[face];
  for (let i = 0; i < (module.rotation % 4 + 4) % 4; i++) [x, y] = [-y, x];
  return {x, y};
}
function facePoint(module: Module, face: Face): Point {
  const d = direction(module, face);
  return {x: module.x + d.x * HALF, y: module.y + d.y * HALF};
}
function engaged(state: State, module: number, face: Face): boolean {
  return state.connections.some(c => c.a === module && c.fa === face || c.b === module && c.fb === face);
}
function key(a: number, fa: Face, b: number, fb: Face): string {
  return [`${a}:${fa}`, `${b}:${fb}`].sort().join('|');
}
function collision(state: State, group: number[], positions: Point[]): boolean {
  return positions.some(position => position.x < 30 || position.x > 530 || position.y < 44 || position.y > 388 ||
    state.modules.some((other, index) => !group.includes(index) && Math.abs(position.x - other.x) < SIZE - 0.5 && Math.abs(position.y - other.y) < SIZE - 0.5));
}
function candidates(state: State, group: number[]) {
  const result: {a: number; fa: Face; b: number; fb: Face; distance: number; delta: Point}[] = [];
  for (const a of group) for (const fa of FACES) {
    if (engaged(state, a, fa)) continue;
    const da = direction(state.modules[a], fa), pa = facePoint(state.modules[a], fa);
    state.modules.forEach((other, b) => {
      if (group.includes(b)) return;
      for (const fb of FACES) {
        if (engaged(state, b, fb) || state.blocked.includes(key(a, fa, b, fb))) continue;
        const db = direction(other, fb), pb = facePoint(other, fb);
        const distance = Math.hypot(pa.x - pb.x, pa.y - pb.y);
        if (da.x + db.x === 0 && da.y + db.y === 0 && distance < 12) {
          result.push({a, fa, b, fb, distance, delta: {x: pb.x - pa.x, y: pb.y - pa.y}});
        }
      }
    });
  }
  return result.sort((a, b) => a.distance - b.distance);
}

export function TopologyPlayground(): React.JSX.Element {
  const [state, setState] = useState<State>(initialState);
  const current = useRef(state);
  const svg = useRef<SVGSVGElement>(null);
  const drag = useRef<{index: number; group: number[]; positions: Point[]; pointer: Point; delta: Point} | null>(null);
  const [selected, setSelected] = useState(0);
  function update(next: State): void { current.current = next; setState(next); }

  function undock(id: number): void {
    const previous = current.current, connection = previous.connections.find(c => c.id === id);
    if (!connection) return;
    const next = {...previous, connections: previous.connections.filter(c => c.id !== id), blocked: [...previous.blocked, key(connection.a, connection.fa, connection.b, connection.fb)]};
    const split = new Set(components(next)).size > new Set(components(previous)).size;
    update({...next, events: [...previous.events, {kind: 'UndockCommitted', detail: `m${connection.a + 1}/${connection.fa} ↔ m${connection.b + 1}/${connection.fb}`},
      {kind: split ? 'AssemblySplit' : 'Loop remains', detail: split ? 'Move a module away to separate the released faces.' : 'Another committed path still connects these modules.'}].slice(-30)});
  }

  function moveGroup(group: number[], positions: Point[]): void {
    const previous = current.current;
    if (collision(previous, group, positions)) return;
    const modules = previous.modules.map((module, index) => {
      const member = group.indexOf(index);
      return member < 0 ? module : {...module, ...positions[member]};
    });
    const blocked = previous.blocked.filter(pair => {
      const ends = pair.split('|').map(end => end.split(':'));
      const a = facePoint(modules[Number(ends[0][0])], ends[0][1] as Face);
      const b = facePoint(modules[Number(ends[1][0])], ends[1][1] as Face);
      return Math.hypot(a.x - b.x, a.y - b.y) <= 20;
    });
    update({...previous, modules, blocked});
  }

  function finishMove(group: number[]): void {
    const previous = current.current, candidate = candidates(previous, group)[0];
    if (!candidate) return;
    const positions = group.map(index => ({x: previous.modules[index].x + candidate.delta.x, y: previous.modules[index].y + candidate.delta.y}));
    if (collision(previous, group, positions)) return;
    moveGroup(group, positions);
    let next = current.current;
    for (const pair of candidates(next, group).filter(pair => pair.distance < 1)) {
      if (engaged(next, pair.a, pair.fa) || engaged(next, pair.b, pair.fb)) continue;
      const merging = components(next)[pair.a] !== components(next)[pair.b];
      next = {...next, nextId: next.nextId + 1, connections: [...next.connections, {id: next.nextId, a: pair.a, fa: pair.fa, b: pair.b, fb: pair.fb}],
        events: [...next.events, {kind: 'DockCommitted', detail: `m${pair.a + 1}/${pair.fa} ↔ m${pair.b + 1}/${pair.fb}`},
          {kind: merging ? 'AssemblyMerged' : 'Loop closure', detail: merging ? 'Connected modules now move as one group.' : 'Both modules were already in the same assembly.'}].slice(-30)};
    }
    update(next);
  }

  function rotate(index: number): void {
    const previous = current.current, group = groupFor(previous, index), pivot = previous.modules[index];
    const positions = group.map(i => ({x: pivot.x - (previous.modules[i].y - pivot.y), y: pivot.y + (previous.modules[i].x - pivot.x)}));
    if (collision(previous, group, positions)) return;
    const modules = previous.modules.map((module, i) => {
      const member = group.indexOf(i);
      return member < 0 ? module : {...module, ...positions[member], rotation: (module.rotation + 1) % 4};
    });
    update({...previous, modules});
  }

  const groups = components(state), count = new Set(groups).size;
  const graph = state.modules.map((_, index) => ({x: 730 + Math.cos(-Math.PI / 2 + index * Math.PI * 2 / 5) * 112, y: 210 + Math.sin(-Math.PI / 2 + index * Math.PI * 2 / 5) * 125}));
  const near = drag.current ? candidates(state, drag.current.group) : [];
  return <figure className={styles.widget} aria-label="Interactive module and topology playground">
    <div className={styles.header}><strong>Move the modules. Watch the graph.</strong><span className={styles.muted}>{state.modules.length} modules · {state.connections.length} connections · {count} assemblies</span></div>
    <div className={styles.stageScroll}>
      <svg ref={svg} className={styles.canvas} viewBox="0 0 900 420" role="group" aria-label="Physical workspace on the left and generated topology on the right"
        onPointerMove={event => {
          const active = drag.current;
          if (!active || !svg.current) return;
          const p = svgPoint(svg.current, event.clientX, event.clientY);
          let desired = {x: p.x - active.pointer.x, y: p.y - active.pointer.y};
          const positions = (delta: Point) => active.positions.map(position => ({x: position.x + delta.x, y: position.y + delta.y}));
          if (collision(current.current, active.group, positions(desired))) {
            let low = 0, high = 1;
            for (let i = 0; i < 14; i++) {
              const value = (low + high) / 2;
              const delta = {x: active.delta.x + (desired.x - active.delta.x) * value, y: active.delta.y + (desired.y - active.delta.y) * value};
              if (collision(current.current, active.group, positions(delta))) high = value; else low = value;
            }
            desired = {x: active.delta.x + (desired.x - active.delta.x) * low, y: active.delta.y + (desired.y - active.delta.y) * low};
          }
          active.delta = desired;
          moveGroup(active.group, positions(desired));
        }}
        onPointerUp={() => { const active = drag.current; drag.current = null; if (active) finishMove(active.group); }}
        onPointerCancel={() => { drag.current = null; }}>
        {Array.from({length: 21}, (_, index) => <line key={`x${index}`} x1={index * 28} x2={index * 28} y1={38} y2={420} stroke="var(--modsim-border)" strokeOpacity={0.45}/>)}
        {Array.from({length: 15}, (_, index) => <line key={`y${index}`} x1={0} x2={560} y1={index * 28 + 38} y2={index * 28 + 38} stroke="var(--modsim-border)" strokeOpacity={0.45}/>)}
        <rect x={560} width={340} height={420} fill="var(--modsim-surface)"/><line x1={560} x2={560} y1={0} y2={420} stroke="var(--modsim-border)"/>
        <text x={16} y={24} fontSize={15} fontWeight={600} fill="var(--modsim-ink)">Workspace · top view</text>
        <text x={580} y={24} fontSize={15} fontWeight={600} fill="var(--modsim-ink)">Generated topology</text>
        {near.map((pair, index) => {
          const p = facePoint(state.modules[pair.a], pair.fa);
          return <circle key={index} cx={p.x} cy={p.y} r={13} fill="var(--modsim-accent)" fillOpacity={0.2} stroke="var(--modsim-accent)" strokeDasharray="3 3"/>;
        })}
        {state.modules.map((module, index) => <g key={index} transform={`translate(${module.x} ${module.y}) rotate(${module.rotation * 90})`} className={styles.interactive} role="button" tabIndex={0}
          aria-label={`Module m${index + 1}. Arrow keys move its assembly; R rotates.`}
          onPointerDown={event => {
            if (!svg.current) return;
            event.preventDefault();
            setSelected(index);
            const group = groupFor(current.current, index), pointer = svgPoint(svg.current, event.clientX, event.clientY);
            drag.current = {index, group, positions: group.map(i => ({...current.current.modules[i]})), pointer, delta: {x: 0, y: 0}};
            svg.current.setPointerCapture(event.pointerId);
          }}
          onDoubleClick={() => rotate(index)}
          onKeyDown={event => {
            if (event.key.toLowerCase() === 'r') { event.preventDefault(); rotate(index); return; }
            const direction = {ArrowLeft: [-8, 0], ArrowRight: [8, 0], ArrowUp: [0, -8], ArrowDown: [0, 8]}[event.key];
            if (!direction) return;
            event.preventDefault();
            const group = groupFor(current.current, index);
            moveGroup(group, group.map(i => ({x: current.current.modules[i].x + direction[0], y: current.current.modules[i].y + direction[1]})));
            finishMove(group);
          }}>
          <rect x={-HALF + 1} y={-HALF + 1} width={SIZE - 2} height={SIZE - 2} rx={6} fill={COLORS[index]} stroke={selected === index ? 'var(--modsim-ink)' : 'var(--modsim-surface)'} strokeWidth={selected === index ? 3 : 1}/>
          {FACES.map(face => {
            const d = LOCAL[face];
            return <rect key={face} x={d.x ? d.x * 23 - 2 : -10} y={d.y ? d.y * 23 - 2 : -10} width={d.x ? 4 : 20} height={d.y ? 4 : 20} rx={1} fill={engaged(state, index, face) ? 'var(--modsim-teal)' : '#FFFFFF'}/>;
          })}
          <text x={0} y={5} textAnchor="middle" fontSize={14} fill="#FFFFFF" transform={`rotate(${-module.rotation * 90})`}>m{index + 1}</text>
        </g>)}
        {state.connections.map(connection => {
          const p = facePoint(state.modules[connection.a], connection.fa), a = graph[connection.a], b = graph[connection.b];
          return <g key={connection.id} className={styles.joint} role="button" tabIndex={0} aria-label={`Undock m${connection.a + 1} from m${connection.b + 1}`} onClick={() => undock(connection.id)} onKeyDown={event => { if (['Enter', ' '].includes(event.key)) { event.preventDefault(); undock(connection.id); } }}>
            <line x1={a.x} y1={a.y} x2={b.x} y2={b.y} stroke="var(--modsim-accent)" strokeWidth={4}/>
            <line x1={a.x} y1={a.y} x2={b.x} y2={b.y} stroke="transparent" strokeWidth={18}/>
            <circle cx={p.x} cy={p.y} r={8} fill="var(--modsim-surface)" stroke="var(--modsim-accent)" strokeWidth={2}/>
            <path d={`M${p.x - 3} ${p.y - 3}l6 6m0 -6l-6 6`} stroke="var(--modsim-accent)" strokeWidth={1.5}/>
            <text x={a.x * 0.73 + b.x * 0.27} y={a.y * 0.73 + b.y * 0.27 - 9} textAnchor="middle" fontSize={11} fill="var(--modsim-ink)" paintOrder="stroke" stroke="var(--modsim-surface)" strokeWidth={4}>{connection.fa}</text>
            <text x={a.x * 0.27 + b.x * 0.73} y={a.y * 0.27 + b.y * 0.73 - 9} textAnchor="middle" fontSize={11} fill="var(--modsim-ink)" paintOrder="stroke" stroke="var(--modsim-surface)" strokeWidth={4}>{connection.fb}</text>
          </g>;
        })}
        {graph.map((point, index) => <g key={index}><circle cx={point.x} cy={point.y} r={20} fill={COLORS[index]} stroke="var(--modsim-surface)" strokeWidth={2}/><text x={point.x} y={point.y + 5} textAnchor="middle" fontSize={14} fill="#FFFFFF">m{index + 1}</text></g>)}
        <text x={730} y={400} fontSize={13} fill="var(--modsim-muted)" textAnchor="middle">Edges are committed connections.</text>
      </svg>
    </div>
    <div className={styles.controls}>
      <label className={styles.control}>Selected module<select value={selected} onChange={event => setSelected(Number(event.target.value))} aria-label="Selected playground module">{state.modules.map((_, index) => <option key={index} value={index}>m{index + 1}</option>)}</select></label>
      <button className={styles.controlButton} type="button" onClick={() => rotate(selected)}>Rotate 90°</button>
      <button className={styles.controlButton} type="button" disabled={!state.connections.length} onClick={() => { for (const connection of [...current.current.connections]) undock(connection.id); }}>Undock all</button>
      <button className={styles.controlButton} type="button" onClick={() => { drag.current = null; update(initialState()); }}>Reset</button>
    </div>
    <EventTrace events={state.events}/>
    <figcaption className={styles.note}>Illustrative 2D browser playground. Drag an assembly, double-click or press R to rotate, and click a connection to release it. Compatible opposing free faces snap together on release. This model illustrates topology and assembly membership; it does not run ModSim or MuJoCo physics. On small screens, scroll the drawing horizontally.</figcaption>
  </figure>;
}
