"use client";

import { useState } from "react";
import { COLUMNS, columnOf, formatDate, journey, trackLabel, type JourneyEvent } from "@/lib/journey";
import "./journey.css";

// Map geometry (SVG units). The SVG scales to the container width.
const GUTTER = 190;
const COL_W = 120;
const TOP = 52;
const LANE_H = 128;
const LANE_Y = 48; // lane line, from the top of its row
const DEAD_Y = 40; // dropped ideas hang this far below the line
const R = 9;
const WIDTH = GUTTER + COLUMNS * COL_W;
const HEIGHT = TOP + journey.tracks.length * LANE_H;

type Placed = JourneyEvent & { x: number; y: number; lane: number; index: number };

/** Spread the events of one lane and one day evenly across that day's column. */
function place(): Placed[] {
  const out: Placed[] = [];
  journey.tracks.forEach((track, lane) => {
    const perColumn = new Map<number, JourneyEvent[]>();
    for (const e of journey.events.filter((ev) => ev.track === track.id)) {
      const c = columnOf(e);
      perColumn.set(c, [...(perColumn.get(c) ?? []), e]);
    }
    for (const [col, list] of perColumn) {
      list.forEach((e, i) => {
        const pad = 16;
        const x = GUTTER + col * COL_W + pad + ((i + 0.5) / list.length) * (COL_W - pad * 2);
        const y = TOP + lane * LANE_H + LANE_Y + (e.status === "abandoned" ? DEAD_Y : 0);
        out.push({ ...e, x, y, lane, index: out.length });
      });
    }
  });
  return out;
}

const PLACED = place();
const BY_ID = new Map(PLACED.map((p) => [p.id, p]));
// The last stop on each lane's main road gets the "now" pulse.
const LAST_ON_LANE = new Set(
  journey.tracks.map((t) => PLACED.filter((p) => p.track === t.id && p.status === "done").sort((a, b) => b.x - a.x)[0]?.id),
);

function Station({ p, selected, onSelect }: { p: Placed; selected: boolean; onSelect: () => void }) {
  return (
    <g
      className={`jmap__station jmap__station--${p.status}${selected ? " is-selected" : ""}`}
      style={{ ["--i" as string]: p.index }}
      transform={`translate(${p.x} ${p.y})`}
      role="button"
      tabIndex={0}
      aria-label={`${p.title}, ${formatDate(p.date)}${p.status === "abandoned" ? ", dropped" : ""}`}
      aria-pressed={selected}
      onClick={onSelect}
      onKeyDown={(e) => {
        if (e.key === "Enter" || e.key === " ") {
          e.preventDefault();
          onSelect();
        }
      }}
    >
      <title>{p.title}</title>
      <circle className="jmap__hit" r={R + 8} />
      {LAST_ON_LANE.has(p.id) && <circle className="jmap__pulse" r={R} />}
      <circle className="jmap__dot" r={R} />
      {p.status === "done" && <path className="jmap__tick" d="M-3.6 0.2 -1 2.8 3.8 -2.6" />}
      {p.status === "abandoned" && <path className="jmap__cross" d="M-3.4 -3.4 3.4 3.4M3.4 -3.4 -3.4 3.4" />}
    </g>
  );
}

/** Subway-style map of the project: one coloured line per track, one column per day. */
export function JourneyMap() {
  const [selectedId, setSelectedId] = useState("h-link");
  const selected = BY_ID.get(selectedId) ?? PLACED[0];
  const replaced = selected.replaces ? BY_ID.get(selected.replaces) : undefined;
  const replacedBy = PLACED.find((p) => p.replaces === selected.id);

  const lines = journey.tracks.flatMap((track, lane) => {
    const road = PLACED.filter((p) => p.track === track.id && p.status !== "abandoned").sort((a, b) => a.x - b.x);
    return road.slice(1).map((p, i) => ({ key: `${track.id}-${p.id}`, track: track.id, x1: road[i].x, x2: p.x, y: TOP + lane * LANE_H + LANE_Y }));
  });
  const branches = PLACED.filter((p) => p.replaces && BY_ID.has(p.replaces)).map((p) => ({ from: BY_ID.get(p.replaces as string) as Placed, to: p }));

  return (
    <div className="jmap">
      <svg className="jmap__svg" viewBox={`0 0 ${WIDTH} ${HEIGHT}`} role="group" aria-label="Map of the project journey. Each dot is a step; select one for details. The same steps are listed as text below.">
        {journey.tracks.map((t, lane) => (
          <g key={t.id} className={`jmap__lane-band lane--${t.id}`}>
            <rect x={0} y={TOP + lane * LANE_H} width={WIDTH} height={LANE_H - 8} rx="18" className="jmap__band" />
            <text className="jmap__lane" x={18} y={TOP + lane * LANE_H + LANE_Y + 5}>
              {t.label}
            </text>
          </g>
        ))}
        {journey.days.map((d, c) => (
          <text key={d.date} className={`jmap__col${c === COLUMNS - 1 ? " jmap__col--now" : ""}`} x={GUTTER + c * COL_W + COL_W / 2} y={TOP - 20} textAnchor="middle">
            {c === COLUMNS - 1 ? "Today" : formatDate(d.date)}
          </text>
        ))}
        {lines.map((l) => (
          <line key={l.key} className={`jmap__line lane--${l.track}`} pathLength={1} x1={l.x1} x2={l.x2} y1={l.y} y2={l.y} />
        ))}
        {PLACED.filter((p) => p.status === "abandoned").map((p) => (
          <line key={`stub-${p.id}`} className="jmap__stub" x1={p.x} x2={p.x} y1={p.y} y2={p.y - DEAD_Y} />
        ))}
        {branches.map(({ from, to }) => (
          <path key={`br-${to.id}`} className={`jmap__branch lane--${to.track}`} d={`M${from.x} ${from.y} C${from.x + 44} ${from.y} ${to.x - 44} ${to.y} ${to.x} ${to.y}`} />
        ))}
        {PLACED.map((p) => (
          <g key={p.id} className={`lane--${p.track}`}>
            <Station p={p} selected={p.id === selected.id} onSelect={() => setSelectedId(p.id)} />
          </g>
        ))}
      </svg>

      <div className={`jmap__detail lane--${selected.track}${selected.status === "abandoned" ? " jmap__detail--dropped" : ""}`} aria-live="polite">
        <p className="jmap__detail-meta">
          {trackLabel(selected.track)} · {formatDate(selected.date, true)}
          {selected.status === "abandoned" && <span className="jmap__chip">Dropped</span>}
        </p>
        <h3 className="jmap__detail-title">{selected.title}</h3>
        <p className="jmap__detail-note">{selected.note}</p>
        {selected.why && (
          <p className="jmap__detail-why">
            <strong>Why we dropped it.</strong> {selected.why}
          </p>
        )}
        {selected.lesson && (
          <p className="jmap__detail-why">
            <strong>What we learned.</strong> {selected.lesson}
          </p>
        )}
        {replacedBy && (
          <button type="button" className="jmap__link" onClick={() => setSelectedId(replacedBy.id)}>
            Replaced by: {replacedBy.title} →
          </button>
        )}
        {replaced && (
          <button type="button" className="jmap__link" onClick={() => setSelectedId(replaced.id)}>
            ← Replaced: {replaced.title}
          </button>
        )}
      </div>

      <ul className="jmap__legend" aria-label="Legend">
        <li><span className="jmap__key jmap__key--done" aria-hidden="true" />Done</li>
        <li><span className="jmap__key jmap__key--abandoned" aria-hidden="true" />Tried and dropped</li>
        <li><span className="jmap__key jmap__key--now" aria-hidden="true" />Latest step on a track</li>
      </ul>
    </div>
  );
}
