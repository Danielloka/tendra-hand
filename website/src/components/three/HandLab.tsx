"use client";

import dynamic from "next/dynamic";
import { useSearchParams } from "next/navigation";
import { useCallback, useEffect, useRef, useState } from "react";
import { Tag } from "@/components/ui/Tag";
import { type HandState, JOINT_IDS, type JointId, createHandState } from "@/lib/handState";
import { setTheme, useTheme } from "@/lib/theme";
import { HAND_MODEL_URL } from "./model";

// three.js stays out of the server bundle and the first page load.
const HandCanvas = dynamic(() => import("./HandCanvas"), { ssr: false });

/*
 * Hand lab (/dev/hand): sliders for every HandState field, presets that match
 * the homepage story, and a joint sequence. Used to tune the placeholder and
 * to check the real GLB once HAND_MODEL_URL is set.
 */

type Values = Omit<HandState, "rotation" | "joints"> & { rotation: HandState["rotation"]; joints: Record<JointId, number> };

function snapshot(s: HandState): Values {
  return { ...s, rotation: { ...s.rotation }, joints: { ...s.joints } };
}

function neutral(): HandState {
  return { ...createHandState(), idle: 0, rim: 0.3 };
}

const joints = (v: Partial<Record<JointId, number>>) => ({ ...createHandState().joints, ...v });

/** Presets mirror the homepage story (PLAN.md, "Scroll-animation plan"). */
const PRESETS: Record<string, () => HandState> = {
  hero: () => createHandState(),
  intro: () => ({ ...neutral(), idle: 0.2, rim: 0.6, rotation: { x: 0, y: -0.5, z: 0 } }),
  tendons: () => ({ ...neutral(), tendons: 1, rotation: { x: 0, y: Math.PI, z: 0 } }),
  joints: () => ({
    ...neutral(),
    labels: 1,
    rotation: { x: 0.25, y: -0.35, z: 0 },
    joints: joints({ index_mcp_flex: 0.55, index_pip: 0.6, index_dip: 0.5, thumb_cmc_rot: 0.6, thumb_cmc_flex: 0.4, thumb_mcp: 0.5, thumb_ip: 0.5 }),
  }),
  exploded: () => ({ ...neutral(), explode: 1, rotation: { x: 0.1, y: -0.6, z: 0 } }),
  electronics: () => ({ ...neutral(), wireframe: 1, zoom: 0.85, rotation: { x: 0, y: -0.5, z: 0 } }),
  fist: () => ({
    ...neutral(),
    rotation: { x: 0.15, y: -0.6, z: 0 },
    joints: Object.fromEntries(JOINT_IDS.map((id) => [id, id === "index_mcp_abd" ? 0 : 1])) as Record<JointId, number>,
  }),
};

/** The story's joint section: index MCP → PIP → DIP, then thumb CMC → MCP → IP, then release. */
const SEQUENCE: JointId[][] = [["index_mcp_flex"], ["index_pip"], ["index_dip"], ["thumb_cmc_rot", "thumb_cmc_flex"], ["thumb_mcp"], ["thumb_ip"]];

const FINGERS: { name: string; real: boolean; ids: JointId[] }[] = [
  { name: "Thumb", real: true, ids: ["thumb_cmc_rot", "thumb_cmc_flex", "thumb_mcp", "thumb_ip"] },
  { name: "Index", real: true, ids: ["index_mcp_abd", "index_mcp_flex", "index_pip", "index_dip"] },
  { name: "Middle", real: false, ids: ["middle_mcp_flex", "middle_pip", "middle_dip"] },
  { name: "Ring", real: false, ids: ["ring_mcp_flex", "ring_pip", "ring_dip"] },
  { name: "Little", real: false, ids: ["little_mcp_flex", "little_pip", "little_dip"] },
];

const ease = (t: number) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);

/** Copies every number from `to` into `state`, blended by t (0 = from, 1 = to). */
function blend(state: HandState, from: Values, to: HandState, t: number) {
  const mix = (a: number, b: number) => a + (b - a) * t;
  for (const k of ["idle", "rim", "tendons", "labels", "explode", "wireframe", "zoom", "shiftY"] as const) state[k] = mix(from[k], to[k]);
  for (const k of ["x", "y", "z"] as const) state.rotation[k] = mix(from.rotation[k], to.rotation[k]);
  for (const id of JOINT_IDS) state.joints[id] = mix(from.joints[id], to.joints[id]);
}

export default function HandLab() {
  // ?preset=joints · ?theme=dark · ?mode=static (for screenshots and quick links)
  const params = useSearchParams();
  const [initialPreset] = useState(() => (PRESETS[params.get("preset") ?? ""] ? params.get("preset")! : "hero"));
  const [state] = useState(() => {
    const s = createHandState();
    blend(s, snapshot(s), PRESETS[initialPreset](), 1);
    return s;
  });
  const [values, setValues] = useState<Values>(() => snapshot(state));
  const [mode, setMode] = useState<"animated" | "static">(() => (params.get("mode") === "static" ? "static" : "animated"));
  const [active, setActive] = useState(initialPreset);
  const [copied, setCopied] = useState(false);
  const anim = useRef(0);
  const theme = useTheme();

  const sync = useCallback(() => setValues(snapshot(state)), [state]);

  /** Runs `step(t)` for `ms` milliseconds on requestAnimationFrame (cancels any running animation). */
  const animate = useCallback(
    (ms: number, step: (t: number) => void) => {
      cancelAnimationFrame(anim.current);
      const start = performance.now();
      const tick = (now: number) => {
        const t = Math.min(1, (now - start) / ms);
        step(t);
        sync();
        if (t < 1) anim.current = requestAnimationFrame(tick);
      };
      anim.current = requestAnimationFrame(tick);
    },
    [sync],
  );
  useEffect(() => () => cancelAnimationFrame(anim.current), []);

  const applyPreset = useCallback(
    (name: string) => {
      const target = PRESETS[name]();
      const from = snapshot(state);
      setActive(name);
      animate(900, (t) => blend(state, from, target, ease(t)));
    },
    [state, animate],
  );

  const playSequence = useCallback(() => {
    const base = { ...PRESETS.joints(), joints: joints({}) };
    const from = snapshot(state);
    setActive("sequence");
    const clamp01 = (v: number) => Math.min(1, Math.max(0, v));
    animate(7000, (t) => {
      blend(state, from, base, clamp01(t * 12)); // settle into the view first
      state.labels = 1;
      // One joint group after another (5%-71% of the time), then all release (80%-100%).
      SEQUENCE.forEach((ids, i) => {
        const bend = ease(clamp01((t - 0.05 - i * 0.11) / 0.11));
        const release = ease(clamp01((t - 0.8) / 0.2));
        for (const id of ids) state.joints[id] = 0.7 * bend * (1 - release);
      });
    });
  }, [state, animate]);

  const themeParam = params.get("theme");
  useEffect(() => {
    if (themeParam === "dark" || themeParam === "light") setTheme(themeParam);
  }, [themeParam]);

  const setNumber = (update: (s: HandState) => void) => {
    cancelAnimationFrame(anim.current);
    update(state);
    setActive("");
    sync();
  };

  const copyState = async () => {
    try {
      await navigator.clipboard.writeText(JSON.stringify(snapshot(state), null, 2));
      setCopied(true);
      setTimeout(() => setCopied(false), 1500);
    } catch {
      /* clipboard blocked: nothing to do */
    }
  };

  return (
    <div className="grid gap-8 lg:grid-cols-[minmax(0,1.35fr)_minmax(0,1fr)]">
      <div className="lg:sticky lg:top-[calc(var(--nav-h)+1rem)] lg:self-start">
        <div className="h-[60svh] overflow-hidden rounded-xl border border-border bg-bg lg:h-[calc(100svh-var(--nav-h)-2rem)]">
          <HandCanvas key={mode} state={state} mode={mode} />
        </div>
        <p className="caption mt-3">
          Model: {HAND_MODEL_URL ? <code>{HAND_MODEL_URL}</code> : "procedural placeholder (set HAND_MODEL_URL in src/components/three/model.ts)"} · Mode: {mode}
          {mode === "static" && " (renders once; switch back to see slider changes)"}
        </p>
      </div>

      <div className="grid content-start gap-5">
        <Group title="Presets">
          <div className="flex flex-wrap gap-2">
            {Object.keys(PRESETS).map((name) => (
              <button key={name} type="button" className={`btn btn--sm ${active === name ? "btn--primary" : "btn--secondary"}`} onClick={() => applyPreset(name)}>
                {name}
              </button>
            ))}
            <button type="button" className={`btn btn--sm ${active === "sequence" ? "btn--primary" : "btn--secondary"}`} onClick={playSequence}>
              Play joint sequence
            </button>
          </div>
          <div className="mt-4 flex flex-wrap gap-2">
            <button type="button" className="btn btn--sm btn--ghost" onClick={() => setMode(mode === "animated" ? "static" : "animated")}>
              Mode: {mode}
            </button>
            <button type="button" className="btn btn--sm btn--ghost" onClick={() => setTheme(theme === "dark" ? "light" : "dark")}>
              Theme: {theme}
            </button>
            <button type="button" className="btn btn--sm btn--ghost" onClick={copyState}>
              {copied ? "Copied" : "Copy state as JSON"}
            </button>
          </div>
        </Group>

        <Group title="View">
          {(["x", "y", "z"] as const).map((axis) => (
            <Slider key={axis} label={`rotation.${axis}`} min={-Math.PI} max={Math.PI} value={values.rotation[axis]} onChange={(v) => setNumber((s) => void (s.rotation[axis] = v))} />
          ))}
          <Slider label="zoom" min={0.5} max={2} value={values.zoom} onChange={(v) => setNumber((s) => void (s.zoom = v))} />
          <Slider label="shiftY" min={-0.5} max={0.5} value={values.shiftY} onChange={(v) => setNumber((s) => void (s.shiftY = v))} />
          {(["idle", "rim", "tendons", "labels", "explode", "wireframe"] as const).map((k) => (
            <Slider key={k} label={k} value={values[k]} onChange={(v) => setNumber((s) => void (s[k] = v))} />
          ))}
        </Group>

        {FINGERS.map((f) => (
          <Group key={f.name} title={f.name} tag={f.real ? <Tag tone="success">On the prototype</Tag> : <Tag>Planned</Tag>}>
            {f.ids.map((id) => (
              <Slider key={id} label={id} value={values.joints[id]} onChange={(v) => setNumber((s) => void (s.joints[id] = v))} />
            ))}
          </Group>
        ))}
      </div>
    </div>
  );
}

function Group({ title, tag, children }: { title: string; tag?: React.ReactNode; children: React.ReactNode }) {
  return (
    // section--alt: grey panel, and secondary buttons inside switch to the surface colour.
    <section className="section--alt rounded-lg p-5">
      <div className="mb-3 flex items-center justify-between gap-3">
        <h2 className="text-h4">{title}</h2>
        {tag}
      </div>
      {children}
    </section>
  );
}

function Slider({ label, value, onChange, min = 0, max = 1 }: { label: string; value: number; onChange: (v: number) => void; min?: number; max?: number }) {
  return (
    <label className="grid grid-cols-[minmax(0,9.5rem)_minmax(0,1fr)_3.25rem] items-center gap-3 py-1">
      <span className="truncate font-mono text-[0.8125rem] text-text-muted">{label}</span>
      <input type="range" min={min} max={max} step={0.001} value={value} onChange={(e) => onChange(Number(e.target.value))} style={{ accentColor: "var(--accent)" }} />
      <span className="text-right font-mono text-[0.8125rem] tabular-nums">{value.toFixed(2)}</span>
    </label>
  );
}
