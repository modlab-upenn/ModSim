import React, {useEffect, useRef, useState} from 'react';
import styles from './widgets.module.css';

export type Point = {x: number; y: number};
export type TraceEvent = {kind: string; detail: string};
export const clamp = (value: number, minimum: number, maximum: number): number => Math.max(minimum, Math.min(maximum, value));
export const ease = (value: number): number => value < 0.5 ? 2 * value * value : 1 - (-2 * value + 2) ** 2 / 2;

export function useReducedMotion(): boolean {
  const [reduced, setReduced] = useState(false);
  useEffect(() => {
    const media = window.matchMedia('(prefers-reduced-motion: reduce)');
    const update = () => setReduced(media.matches);
    update();
    media.addEventListener('change', update);
    return () => media.removeEventListener('change', update);
  }, []);
  return reduced;
}

export function svgPoint(svg: SVGSVGElement, clientX: number, clientY: number): Point {
  const matrix = svg.getScreenCTM();
  if (!matrix) return {x: 0, y: 0};
  const point = svg.createSVGPoint();
  point.x = clientX;
  point.y = clientY;
  return point.matrixTransform(matrix.inverse());
}

export function EventTrace({events, live = true}: {events: TraceEvent[]; live?: boolean}): React.JSX.Element {
  const target = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (target.current) target.current.scrollTop = target.current.scrollHeight;
  }, [events.length]);
  return <div ref={target} className={styles.log} role="log" aria-label="Demonstration event trace" aria-live={live ? 'polite' : 'off'}>
    {events.map((event, index) => <div key={`${index}:${event.kind}`}><span className={styles.event}>{event.kind}</span>{' '}{event.detail}</div>)}
  </div>;
}

export function usePlayback(duration: number) {
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  useEffect(() => {
    if (!playing) return;
    let frame = 0;
    let previous = performance.now();
    const tick = (now: number) => {
      const elapsed = Math.min((now - previous) / 1000, 0.1);
      previous = now;
      setTime(value => Math.min(duration, value + elapsed));
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    const visibility = () => { if (document.hidden) setPlaying(false); };
    document.addEventListener('visibilitychange', visibility);
    return () => {
      cancelAnimationFrame(frame);
      document.removeEventListener('visibilitychange', visibility);
    };
  }, [playing, duration]);
  useEffect(() => { if (time >= duration) setPlaying(false); }, [time, duration]);
  return {time, setTime, playing, setPlaying};
}
