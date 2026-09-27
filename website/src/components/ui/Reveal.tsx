"use client";

import { useEffect, useRef } from "react";

type Props = {
  as?: "div" | "p" | "h1" | "h2" | "h3" | "section" | "li" | "span" | "header";
  delay?: number; // ms
  className?: string;
  children: React.ReactNode;
} & Record<string, unknown>;

/**
 * Fade/slide-in on first view, using the design system's [data-reveal] states
 * (base.css). Content stays visible without JS and with reduced motion.
 */
export function Reveal({ as = "div", delay = 0, className, children, ...rest }: Props) {
  const ref = useRef<HTMLDivElement>(null);
  const Tag = as as "div"; // every allowed tag takes the same props we pass

  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    const io = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          el.classList.add("is-visible");
          io.disconnect();
        }
      },
      { rootMargin: "0px 0px -10% 0px" },
    );
    io.observe(el);
    return () => io.disconnect();
  }, []);

  return (
    <Tag ref={ref} data-reveal="" className={className} style={delay ? ({ "--reveal-delay": `${delay}ms` } as React.CSSProperties) : undefined} {...rest}>
      {children}
    </Tag>
  );
}
