import { JsonLd } from "./JsonLd";
import { siteGraph } from "./schemas";

/** Organization + WebSite + project schema. Render once in the root layout. */
export function SiteJsonLd() {
  return <JsonLd data={siteGraph()} />;
}
