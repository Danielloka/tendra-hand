import type { DiagramData, DiagramNode } from "./content";

/* Simple line icons (24×24, stroke = currentColor). */
const ICONS: Record<DiagramNode["id"], React.ReactNode> = {
  see: (
    <>
      <path d="M2.5 12S6 5.5 12 5.5 21.5 12 21.5 12 18 18.5 12 18.5 2.5 12 2.5 12Z" />
      <circle cx="12" cy="12" r="3" />
    </>
  ),
  think: (
    <>
      <circle cx="12" cy="12" r="2.5" />
      <circle cx="5" cy="6" r="1.75" />
      <circle cx="19" cy="6" r="1.75" />
      <circle cx="5" cy="18" r="1.75" />
      <circle cx="19" cy="18" r="1.75" />
      <path d="m6.4 7.1 3.7 3.2M17.6 7.1l-3.7 3.2M6.4 16.9l3.7-3.2M17.6 16.9l-3.7-3.2" />
    </>
  ),
  act: (
    <>
      <path d="M8 13V5.5a1.5 1.5 0 0 1 3 0V12M11 11V4a1.5 1.5 0 0 1 3 0v7M14 11V5.5a1.5 1.5 0 0 1 3 0V14a7 7 0 0 1-7 7h-.5a6 6 0 0 1-4.6-2.2L3.2 15.6a1.5 1.5 0 0 1 2.3-1.9L8 16" />
    </>
  ),
};

/**
 * Sense → think → act, from content/home.json `diagram` (the "Intelligence" chapter). Plain HTML/SVG, fully
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
