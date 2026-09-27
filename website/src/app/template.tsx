import { RouteTransition } from "@/components/layout/RouteTransition";

// Templates re-mount on every navigation, which replays the entrance animation.
export default function Template({ children }: { children: React.ReactNode }) {
  return <RouteTransition>{children}</RouteTransition>;
}
