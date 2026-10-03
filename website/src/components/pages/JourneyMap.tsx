"use client";

import { useMemo, useState } from "react";
import { StatusBadge } from "@/components/ui/Tag";
import { COLUMNS, columnLabel, columnOf, formatWhen, journey, type JourneyEvent } from "@/lib/journey";
import "./journey.css";

// Map geometry (SVG units). The SVG scales to the container width.
const GUTTER = 176;
const COL_W = 100;
const TOP = 44;
const LANE_H = 124;
const LANE_Y = 46; // lane line, from the top of its row
const DEAD_Y = 36; // abandoned stations hang this far below the line
const R = 8;
const WIDTH = GUTTER + COLUMNS * COL_W;
const HEIGHT = TOP + journey.tracks.length * LANE_H;

type Placed = JourneyEvent & { x: number; y: number; lane: number };

/** Spread the events of one lane and one column evenly across that column. */
function place(): Placed[] {
  const out: Placed[] = [];
  journey.tracks.forEach((track, lane) => {
    const mine = journey.events.filter((e) => e.track === track.id);
    const perColumn = new Map<number, JourneyEvent[]>();
    for (const e of mine) {
      const c = columnOf(e);
      perColumn.set(c, [...(perColumn.get(c) ?? []), e]);
    }
    for (const [col, list] of perColumn) {
      list.forEach((e, i) => {
        const pad = 14;
        const x = GUTTER + col * COL_W + pad + ((i + 0.5) / list.length) * (COL_W - pad * 2);
        const y = TOP + lane * LANE_H + LANE_Y + (e.status === "abandoned" ? DEAD_Y : 0);
        out.push({ ...e, x, y, lane });
      });
    }
  });
  return out;
}

function Station({ p, selected, onSelect }: { p: Placed; selected: boolean; onSelect: () => void }) {
  return (
    <g
      className={`jmap__station jmap__station--${p.status}${selected ? " is-selected" : ""}`}
      transform={`translate(${p.x} ${p.y})`}
      role="button"
      tabIndex={0}
      aria-label={`${p.title}, ${formatWhen(p)}, ${p.status.replace("-", " ")}`}
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
      <circle className="jmap__hit" r={R + 7} />
      {p.status === "in-progress" && <circle className="jmap__pulse" r={R} />}
      <circle className="jmap__dot" r={R} />
      {p.status === "done" && <path className="jmap__tick" d="M-3.4 0.2 -1 2.6 3.6 -2.4" />}
      {p.status === "abandoned" && <path className="jmap__cross" d="M-3.2 -3.2 3.2 3.2M3.2 -3.2 -3.2 3.2" />}
    </g>
  );
}

/** Subway-style map of the project: one line per track, time left to right. */
export function JourneyMap() {
  const placed = useMemo(place, []);
  const byId = useMemo(() => new Map(placed.map((p) => [p.id, p])), [placed]);
  const [selectedId, setSelectedId] = useState(() => placed.find((p) => p.status === "in-progress")?.id ?? placed[0].id);
  const selected = byId.get(selectedId) ?? placed[0];

  // Lane lines join the stations that are on the main road; abandoned ones branch off.
  const lines = journey.tracks.flatMap((track, lane) => {
    const road = placed.filter((p) => p.track === track.id && p.status !== "abandoned").sort((a, b) => a.x - b.x);
    return road.slice(1).map((p, i) => ({ key: `${track.id}-${p.id}`, x1: road[i].x, x2: p.x, y: TOP + lane * LANE_H + LANE_Y, planned: p.status === "planned" }));
  });
  // A dead end is joined to the station that replaced it.
  const branches = placed.filter((p) => p.replaces && byId.has(p.replaces)).map((p) => ({ from: byId.get(p.replaces as string) as Placed, to: p }));

  return (
    <div className="jmap">
      <div className="jmap__scroll" aria-hidden={false}>
        <svg className="jmap__svg" viewBox={`0 0 ${WIDTH} ${HEIGHT}`} role="group" aria-label="Map of the project journey. Each dot is a step; select one for details. A text list follows below.">
          <rect className="jmap__future" x={GUTTER + (COLUMNS - 2) * COL_W} y={TOP - 8} width={2 * COL_W} height={HEIGHT - TOP + 8} rx="14" />
          {Array.from({ length: COLUMNS }, (_, c) => (
            <text key={c} className="jmap__col" x={GUTTER + c * COL_W + COL_W / 2} y={TOP - 18} textAnchor="middle">
              {columnLabel(c)}
            </text>
          ))}
          {journey.tracks.map((t, lane) => (
            <text key={t.id} className="jmap__lane" x={0} y={TOP + lane * LANE_H + LANE_Y + 5}>
              {t.label}
            </text>
          ))}
          {lines.map((l) => (
            <line key={l.key} className={`jmap__line${l.planned ? " jmap__line--planned" : ""}`} x1={l.x1} x2={l.x2} y1={l.y} y2={l.y} />
          ))}
          {placed
            .filter((p) => p.status === "abandoned")
            .map((p) => (
              <line key={`stub-${p.id}`} className="jmap__stub" x1={p.x} x2={p.x} y1={p.y} y2={p.y - DEAD_Y} />
            ))}
          {branches.map(({ from, to }) => (
            <path key={`br-${to.id}`} className="jmap__branch" d={`M${from.x} ${from.y} C${from.x + 40} ${from.y} ${to.x - 40} ${to.y} ${to.x} ${to.y}`} />
          ))}
          {placed.map((p) => (
            <Station key={p.id} p={p} selected={p.id === selected.id} onSelect={() => setSelectedId(p.id)} />
          ))}
        </svg>
      </div>

      <div className="jmap__detail" aria-live="polite">
        <div className="jmap__detail-head">
          <StatusBadge status={selected.status === "abandoned" ? "planned" : selected.status}>{selected.status === "abandoned" ? "Replaced" : undefined}</StatusBadge>
          <span className="jmap__detail-when">
            {journey.tracks.find((t) => t.id === selected.track)?.label} · {formatWhen(selected)}
          </span>
        </div>
        <h3 className="jmap__detail-title">{selected.title}</h3>
        <p className="jmap__detail-note">{selected.note}</p>
        {selected.replaces && byId.get(selected.replaces) && <p className="jmap__detail-note jmap__detail-note--muted">Replaced: {byId.get(selected.replaces)?.title}</p>}
      </div>

      <ul className="jmap__legend" aria-label="Legend">
        <li><span className="jmap__key jmap__key--done" aria-hidden="true" />Done</li>
        <li><span className="jmap__key jmap__key--in-progress" aria-hidden="true" />In progress</li>
        <li><span className="jmap__key jmap__key--planned" aria-hidden="true" />Planned</li>
        <li><span className="jmap__key jmap__key--abandoned" aria-hidden="true" />Tried and replaced</li>
      </ul>
    </div>
  );
}
