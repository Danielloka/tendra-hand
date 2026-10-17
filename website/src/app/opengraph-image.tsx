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

/** The logo's hand ("Wave"); same geometry as scripts/make-brand.py. Called as a function: satori only accepts plain SVG elements inside <svg>. */
function brandHand() {
  const fingers = [[21.5, 40.88, 15.12], [21.5, 20, 18.68], [30.5, 39.62, 16.38], [30.5, 17, 20.42], [39.5, 40.46, 15.54], [39.5, 19, 19.26], [48.5, 42.98, 13.02], [48.5, 25, 15.78]];
  const thumb = [[15.3, 40, 16], [15.3, 26.8, 11]];
  const rect = ([x, y, h]: number[]) => <rect key={`${x}-${y}`} x={x} y={y} width="7" height={h} rx="3.5" />;
  return (
    <g transform="translate(3.95 -4.48) scale(0.9524)">
      <g transform="rotate(14 36 40)">
        {fingers.map(rect)}
        <g transform="rotate(-38 18.8 56.5)">{thumb.map(rect)}</g>
      </g>
    </g>
  );
}

export default function OpenGraphImage() {
  return new ImageResponse(
    (
      <div style={{ width: "100%", height: "100%", display: "flex", background: C.bg, padding: 72, fontFamily: "sans-serif" }}>
        <div style={{ display: "flex", flexDirection: "column", justifyContent: "space-between", flex: 1, paddingRight: 40 }}>
          <div style={{ display: "flex", alignItems: "center", gap: 16, fontSize: 32, fontWeight: 600, color: C.text }}>
            <svg width="44" height="44" viewBox="0 0 64 64">
              <defs>
                <linearGradient id="g" x1="0" y1="0" x2="1" y2="1">
                  <stop offset="0" stopColor={C.accent} />
                  <stop offset="1" stopColor="#5aa9ff" />
                </linearGradient>
              </defs>
              <rect width="64" height="64" rx="14" fill="url(#g)" />
              <g fill="#fff" transform="translate(8.32 8.32) scale(0.74)">
                {brandHand()}
              </g>
            </svg>
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
          <svg width="300" height="300" viewBox="0 0 64 64">
            <g fill={C.accent}>{brandHand()}</g>
          </svg>
        </div>
      </div>
    ),
    size,
  );
}
