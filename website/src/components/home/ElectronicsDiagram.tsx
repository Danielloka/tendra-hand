import type { DiagramData, DiagramNode } from "./content";

/* Simple line icons (24×24, stroke = currentColor). */
const ICONS: Record<DiagramNode["id"], React.ReactNode> = {
  pc: (
    <>
      <rect x="3" y="4" width="18" height="12" rx="2" />
      <path d="M8 20h8M12 16v4" />
    </>
  ),
  esp32: (
    <>
      <rect x="6" y="6" width="12" height="12" rx="2" />
      <path d="M9 6V3M12 6V3M15 6V3M9 21v-3M12 21v-3M15 21v-3M6 9H3M6 12H3M6 15H3M21 9h-3M21 12h-3M21 15h-3" />
    </>
  ),
  motors: (
    <>
      <circle cx="12" cy="12" r="7.5" />
      <circle cx="12" cy="12" r="2.5" />
      <path d="M12 4.5v2M12 17.5v2M4.5 12h2M17.5 12h2" />
    </>
  ),
};

/**
 * PC → ESP32-S3 → motors, from content/home.json `diagram`. Plain HTML/SVG, fully
 * readable without JS or animation; ScrollStory draws the connectors on
 * (`data-diagram-link`) and fades the nodes in (`data-diagram-node`) as it scrolls in.
 */
export function ElectronicsDiagram({ diagram }: { diagram: DiagramData }) {
  return (
    <figure className="diagram" data-diagram="">
      <ol className="diagram__flow" role="list">
        {diagram.nodes.map((node, i) => (
          <li key={node.id} className="diagram__step">
            <div className="diagram__node" data-diagram-node="">
              <span className="diagram__icon" aria-hidden="true">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round">
                  {ICONS[node.id]}
                </svg>
              </span>
              <span className="diagram__text">
                <span className="diagram__label">{node.label}</span>
                <span className="diagram__detail">{node.detail}</span>
              </span>
            </div>
            {i < diagram.nodes.length - 1 && (
              <svg className="diagram__link" data-diagram-link="" viewBox="0 0 16 40" fill="none" aria-hidden="true">
                <path d="M8 2v34" pathLength={1} />
                <path d="M3.5 31.5 8 36l4.5-4.5" pathLength={1} />
              </svg>
            )}
          </li>
        ))}
      </ol>
      <figcaption className="caption diagram__caption">{diagram.caption}</figcaption>
    </figure>
  );
}
