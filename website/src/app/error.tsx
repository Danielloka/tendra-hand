"use client";

import Link from "next/link";
import { useEffect } from "react";
import { Footer } from "@/components/layout/Footer";
import { Nav } from "@/components/layout/Nav";
import { PageHero } from "@/components/pages/PageHero";
import { mainNav } from "@/lib/site";

/** Shown when a page throws while rendering. Same look as the 404 page, with a retry button. */
export default function Error({ error, reset }: { error: Error & { digest?: string }; reset: () => void }) {
  useEffect(() => {
    console.error(error);
  }, [error]);

  return (
    <>
      <Nav items={mainNav} />
      <main id="main" className="not-found">
        <PageHero
          narrow
          kicker="Something went wrong"
          title="This page dropped what it was holding."
          lead="An error stopped the page from loading. Try again, or head back to the homepage."
          actions={
            <>
              <button type="button" className="btn btn--primary" onClick={reset}>
                Try again
              </button>
              <Link className="btn btn--secondary" href="/">
                Go to the homepage
              </Link>
            </>
          }
        />
      </main>
      <Footer items={mainNav} />
    </>
  );
}
