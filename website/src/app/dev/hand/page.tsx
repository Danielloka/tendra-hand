import type { Metadata } from "next";
import { Suspense } from "react";
import HandLab from "@/components/three/HandLab";

export const metadata: Metadata = {
  title: "Hand lab",
  robots: { index: false, follow: false },
};

/** Developer page: every HandState field on a slider. Not linked, not in the sitemap. */
export default function HandLabPage() {
  return (
    <div className="container py-8">
      <header className="mb-6 grid max-w-3xl gap-2">
        <p className="kicker">Developer tools</p>
        <h1 className="text-h3">Hand lab</h1>
        <p className="caption">
          Every number the scroll story can change, on a slider. Use it to tune the placeholder hand, and to check the real model once it is
          exported (see <code>public/models/README.md</code>). Links: <code>?preset=joints</code>, <code>?theme=dark</code>, <code>?mode=static</code>.
        </p>
      </header>
      <Suspense fallback={null}>
        <HandLab />
      </Suspense>
    </div>
  );
}
