import React, {useEffect, useMemo, useRef, useState, type ReactNode} from 'react';
import ErrorBoundary from '@docusaurus/ErrorBoundary';
import {ErrorBoundaryErrorMessageFallback, useColorMode} from '@docusaurus/theme-common';
import {useMermaidConfig, useMermaidRenderResult} from '@docusaurus/theme-mermaid/client';
import type {Props} from '@theme/Mermaid';
import type {MermaidConfig, RenderResult} from 'mermaid';

const tokens = {
  background: '--modsim-bg',
  surface: '--modsim-surface',
  panel: '--modsim-surface-2',
  border: '--modsim-border',
  ink: '--modsim-ink',
  muted: '--modsim-muted',
  accent: '--modsim-accent',
  teal: '--modsim-secondary-accent',
};

type Palette = Record<keyof typeof tokens, string>;
type Mode = 'light' | 'dark';

function kindOf(source: string): string {
  if (/^\s*classDiagram/m.test(source)) return 'UML class diagram';
  if (/^\s*sequenceDiagram/m.test(source)) return 'Sequence diagram';
  if (/^\s*stateDiagram/m.test(source)) return 'State diagram';
  return 'Architecture flowchart';
}

function Canvas({result, fit}: {result: RenderResult; fit: boolean}): ReactNode {
  const element = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (element.current) result.bindFunctions?.(element.current);
  }, [result]);
  return <div ref={element} className={`docusaurus-mermaid-container modsim-diagram-canvas${fit ? ' modsim-diagram-fit' : ''}`}
    dangerouslySetInnerHTML={{__html: result.svg}} />;
}

function Diagram({value, palette, mode}: Props & {palette: Palette; mode: Mode}): ReactNode {
  const defaults = useMermaidConfig();
  const config = useMemo<MermaidConfig>(() => ({
    ...defaults,
    theme: 'base',
    fontFamily: 'IBM Plex Sans, sans-serif',
    themeVariables: {
      darkMode: mode === 'dark',
      background: palette.surface,
      primaryColor: palette.panel,
      primaryTextColor: palette.ink,
      primaryBorderColor: palette.accent,
      secondaryColor: palette.surface,
      secondaryTextColor: palette.ink,
      secondaryBorderColor: palette.teal,
      tertiaryColor: palette.background,
      tertiaryTextColor: palette.ink,
      tertiaryBorderColor: palette.border,
      mainBkg: palette.panel,
      textColor: palette.ink,
      lineColor: palette.muted,
      nodeBorder: palette.accent,
      clusterBkg: palette.background,
      clusterBorder: palette.border,
      edgeLabelBackground: palette.surface,
      actorBkg: palette.panel,
      actorBorder: palette.accent,
      actorTextColor: palette.ink,
      actorLineColor: palette.muted,
      signalColor: palette.ink,
      signalTextColor: palette.ink,
      labelBoxBkgColor: palette.surface,
      labelBoxBorderColor: palette.border,
      labelTextColor: palette.ink,
      loopTextColor: palette.ink,
      noteBkgColor: palette.panel,
      noteBorderColor: palette.teal,
      noteTextColor: palette.ink,
      activationBkgColor: palette.panel,
      activationBorderColor: palette.teal,
      classText: palette.ink,
      fontFamily: 'IBM Plex Sans, sans-serif',
      fontSize: '15px',
    },
    flowchart: {useMaxWidth: false, htmlLabels: false, curve: 'linear', nodeSpacing: 35, rankSpacing: 50},
    sequence: {useMaxWidth: false, diagramMarginX: 24, diagramMarginY: 20, actorMargin: 42, messageMargin: 32},
    class: {useMaxWidth: false, hideEmptyMembersBox: true},
    state: {useMaxWidth: false},
  }), [defaults, mode, palette]);
  const result = useMermaidRenderResult({text: value, config});
  const dialog = useRef<HTMLDialogElement>(null);
  const inline = useRef<HTMLDivElement>(null);
  const [expanded, setExpanded] = useState(false);
  const [fit, setFit] = useState(false);
  const [height, setHeight] = useState(0);
  const kind = kindOf(value);
  const download = result ? `data:image/svg+xml;charset=utf-8,${encodeURIComponent(result.svg)}` : undefined;
  function expand() {
    setHeight(inline.current?.offsetHeight ?? 0);
    setExpanded(true);
    dialog.current?.showModal();
  }
  return <figure className="modsim-diagram" aria-label={kind}>
    <div className="modsim-diagram-toolbar">
      <span>{kind}</span>
      <div>
        <button type="button" onClick={() => setFit(!fit)} aria-pressed={fit} disabled={!result}>
          {fit ? 'Actual size' : 'Fit width'}
        </button>
        <button type="button" onClick={expand} disabled={!result}>Expand</button>
        <a href={download} download={`modsim-${kind.toLowerCase().replaceAll(' ', '-')}.svg`}>Download SVG</a>
      </div>
    </div>
    <div ref={inline} style={expanded ? {height} : undefined}>
      {!expanded && (result ? <Canvas result={result} fit={fit} /> : <p className="modsim-diagram-loading">Rendering diagram…</p>)}
    </div>
    <dialog ref={dialog} className="modsim-diagram-dialog" aria-label={`Expanded ${kind.toLowerCase()}`} onClose={() => setExpanded(false)}>
      <div className="modsim-diagram-toolbar">
        <span>{kind}</span>
        <div>
          <button type="button" onClick={() => setFit(!fit)} aria-pressed={fit}>
            {fit ? 'Actual size' : 'Fit width'}
          </button>
          <button type="button" onClick={() => dialog.current?.close()}>Close diagram</button>
        </div>
      </div>
      {expanded && result && <Canvas result={result} fit={fit} />}
    </dialog>
  </figure>;
}

function ThemedDiagram(props: Props): ReactNode {
  const {colorMode} = useColorMode();
  const [palette, setPalette] = useState<{mode: Mode; values: Palette} | null>(null);
  useEffect(() => {
    const frame = requestAnimationFrame(() => {
      const styles = getComputedStyle(document.documentElement);
      const values = Object.fromEntries(Object.entries(tokens).map(([name, token]) =>
        [name, styles.getPropertyValue(token).trim()])) as Palette;
      setPalette({mode: colorMode, values});
    });
    return () => cancelAnimationFrame(frame);
  }, [colorMode]);
  return palette?.mode === colorMode ?
    <Diagram key={colorMode} {...props} palette={palette.values} mode={colorMode} /> :
    <p className="modsim-diagram-loading">Preparing diagram…</p>;
}

export default function Mermaid(props: Props): ReactNode {
  return <ErrorBoundary fallback={(params) => <ErrorBoundaryErrorMessageFallback {...params} />}>
    <ThemedDiagram {...props} />
  </ErrorBoundary>;
}
