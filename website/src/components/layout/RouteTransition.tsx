"use client";

import { usePathname } from "next/navigation";
import "./layout.css";

/**
 * Quick fade + small rise when a page is entered (see layout.css). The homepage
 * only fades, so nothing moves under its pinned 3D canvas. None with reduced motion.
 */
export function RouteTransition({ children }: { children: React.ReactNode }) {
  const home = usePathname() === "/";
  return <div className={home ? "route-in route-in--fade" : "route-in"}>{children}</div>;
}
