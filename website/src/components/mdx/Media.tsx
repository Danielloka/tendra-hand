import Image from "next/image";
import { Placeholder as UiPlaceholder } from "@/components/ui/Placeholder";

type FigureProps = { src: string; alt: string; caption?: React.ReactNode; width?: number; height?: number };

/**
 * Image with an optional caption. Pass the real pixel size when you know it so
 * the page doesn't jump while loading; otherwise 16:10 is assumed.
 */
export function Figure({ src, alt, caption, width = 1600, height = 1000 }: FigureProps) {
  return (
    <figure className="prose-figure">
      <Image src={src} alt={alt} width={width} height={height} sizes="(min-width: 1024px) 720px, 100vw" style={{ width: "100%", height: "auto" }} />
      {caption && <figcaption className="caption">{caption}</figcaption>}
    </figure>
  );
}

/** Video that only downloads when played: muted, inline on phones, with controls. */
export function Video({ src, poster, caption }: { src: string; poster?: string; caption?: React.ReactNode }) {
  return (
    <figure className="prose-figure">
      <video src={src} poster={poster} controls muted playsInline preload="none" />
      {caption && <figcaption className="caption">{caption}</figcaption>}
    </figure>
  );
}

/** Stand-in for missing media. Always add a {/* TODO: … *\/} next to it in MDX. */
export function Placeholder({ label, ratio, caption }: { label: string; ratio?: string; caption?: React.ReactNode }) {
  return (
    <figure className="prose-figure">
      <UiPlaceholder label={label} ratio={ratio} />
      {caption && <figcaption className="caption">{caption}</figcaption>}
    </figure>
  );
}
