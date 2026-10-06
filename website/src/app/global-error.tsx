"use client";

import "./globals.css";

/** Last-resort boundary: replaces the root layout when it fails, so it brings its own <html> and <body>. */
export default function GlobalError({ reset }: { error: Error & { digest?: string }; reset: () => void }) {
  return (
    <html lang="en">
      <body>
        <main id="main" className="container grid min-h-[60vh] content-center">
          <div className="grid max-w-2xl gap-4 py-16">
            <p className="kicker">Something went wrong</p>
            <h1 className="h2">The site could not load.</h1>
            <p className="lead">Please try again in a moment.</p>
            <div>
              <button type="button" className="btn btn--primary" onClick={reset}>
                Try again
              </button>
            </div>
          </div>
        </main>
      </body>
    </html>
  );
}
