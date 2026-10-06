import type { MDXComponents } from "mdx/types";
import Link from "next/link";
import { BomTable } from "@/components/pages/BomTable";
import { RoadmapTimeline } from "@/components/pages/RoadmapTimeline";
import { StatusBadge } from "@/components/ui/Tag";
import { Callout } from "./Callout";
import { Card, CardGrid } from "./CardGrid";
import { CodeBlock } from "./CodeBlock";
import { H2, H3, H4 } from "./Heading";
import { Figure, Video } from "./Media";
import { Step, Steps } from "./Steps";

// Components available in every .mdx file without importing them (see PLAN.md →
// "MDX components"). Render MDX inside <div className="prose"> for the long-form styles.
export const mdxComponents: MDXComponents = {
  // Markdown element overrides
  h2: H2,
  h3: H3,
  h4: H4,
  pre: CodeBlock,
  a: ({ href = "", ...props }) => (href.startsWith("/") ? <Link href={href} {...props} /> : <a href={href} {...props} />),
  // Wide tables scroll sideways on phones instead of breaking the layout.
  table: (props) => (
    <div className="prose-table">
      <table {...props} />
    </div>
  ),

  // Components
  Callout,
  Figure,
  Video,
  StatusBadge,
  CardGrid,
  Card,
  Steps,
  Step,

  // Data-driven (Pages Agent): render content/data/bom.json and content/roadmap.json
  BomTable,
  Roadmap: RoadmapTimeline,
};
