import type { Metadata } from "next";
import Link from "next/link";
import { Footer } from "@/components/layout/Footer";
import { Nav } from "@/components/layout/Nav";
import { PageHero } from "@/components/pages/PageHero";
import { mainNav } from "@/lib/site";

export const metadata: Metadata = {
  title: "Page not found",
  description: "This page doesn't exist. Head back to the homepage or look through the docs.",
};

export default function NotFound() {
  return (
    <>
      <Nav items={mainNav} />
      <main id="main" className="not-found">
        <PageHero
          narrow
          kicker="Error 404"
          title="This page slipped through our fingers."
          lead="It may have moved, or it never existed. Try the homepage or the docs instead."
          actions={
            <>
              <Link className="btn btn--primary" href="/">
                Go to the homepage
              </Link>
              <Link className="btn btn--secondary" href="/docs">
                Read the docs
              </Link>
            </>
          }
        />
      </main>
      <Footer items={mainNav} />
    </>
  );
}
