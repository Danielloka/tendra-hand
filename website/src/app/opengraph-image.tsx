import { ImageResponse } from "next/og";
import home from "@content/home.json";
import { siteConfig } from "@/lib/site";

export const alt = `${siteConfig.name}: ${siteConfig.tagline}`;
export const size = { width: 1200, height: 630 };
export const contentType = "image/png";

// CSS variables don't exist in a generated PNG, so the light-theme token
// values are copied here (assets/css/tokens.css: --bg, --bg-alt, --text,
// --text-muted, --accent, --accent-soft, --border).
const C = { bg: "#ffffff", bgAlt: "#f5f5f7", text: "#1d1d1f", muted: "#6e6e73", accent: "#0066cc", accentSoft: "#e8f1fd", border: "#e3e3e8" };

/** Simple line drawing of a hand: palm, four fingers and a thumb, with tendon lines. */
function HandMark() {
  const fingers = [
    { x: 118, top: 40, h: 150 },
    { x: 168, top: 20, h: 170 },
    { x: 218, top: 34, h: 156 },
    { x: 268, top: 64, h: 126 },
  ];
  return (
    <svg width="380" height="400" viewBox="-40 0 380 400" fill="none">
      <rect x="44" y="190" width="36" height="150" rx="18" fill={C.bg} stroke={C.text} strokeWidth="8" transform="rotate(-38 62 265)" />
      <line x1="40" y1="222" x2="112" y2="316" stroke={C.accent} strokeWidth="4" strokeLinecap="round" />
      <rect x="100" y="178" width="200" height="190" rx="56" fill={C.bg} stroke={C.text} strokeWidth="8" />
      {fingers.map((f) => (
        <g key={f.x}>
          <rect x={f.x - 18} y={f.top} width="36" height={f.h + 30} rx="18" fill={C.bg} stroke={C.text} strokeWidth="8" />
          <line x1={f.x} y1={f.top + 22} x2={f.x} y2="330" stroke={C.accent} strokeWidth="4" strokeLinecap="round" />
          <circle cx={f.x} cy={f.top + f.h * 0.35} r="5" fill={C.text} />
          <circle cx={f.x} cy={f.top + f.h * 0.7} r="5" fill={C.text} />
        </g>
      ))}
    </svg>
  );
}

export default function OpenGraphImage() {
  return new ImageResponse(
    (
      <div style={{ width: "100%", height: "100%", display: "flex", background: C.bg, padding: 72, fontFamily: "sans-serif" }}>
        <div style={{ display: "flex", flexDirection: "column", justifyContent: "space-between", flex: 1, paddingRight: 40 }}>
          <div style={{ display: "flex", alignItems: "center", gap: 16, fontSize: 32, fontWeight: 600, color: C.text }}>
            <div style={{ width: 36, height: 36, borderRadius: 11, background: C.accent }} />
            {siteConfig.name}
          </div>
          <div style={{ display: "flex", flexDirection: "column", gap: 24 }}>
            <div style={{ fontSize: 76, fontWeight: 700, lineHeight: 1.04, letterSpacing: -3, color: C.text }}>{home.hero.title}</div>
            <div style={{ fontSize: 30, lineHeight: 1.35, color: C.muted }}>{siteConfig.tagline}</div>
          </div>
          <div style={{ display: "flex", gap: 12, fontSize: 22, color: C.muted }}>
            {["3D-printed", "Tendon-driven", "Open source"].map((t) => (
              <div key={t} style={{ padding: "8px 20px", borderRadius: 999, background: C.bgAlt, border: `1px solid ${C.border}` }}>
                {t}
              </div>
            ))}
          </div>
        </div>
        <div style={{ display: "flex", alignItems: "center", justifyContent: "center", width: 400, borderRadius: 40, background: C.accentSoft }}>
          <HandMark />
        </div>
      </div>
    ),
    size,
  );
}
