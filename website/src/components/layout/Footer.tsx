import Link from "next/link";
import { siteConfig } from "@/lib/site";

export function Footer() {
  const { footer } = siteConfig;
  return (
    <footer id="site-footer" className="border-t border-border bg-bg-alt py-12">
      <div className="container grid gap-8 md:grid-cols-[2fr_1fr_1fr]">
        <div className="grid content-start gap-3">
          <Link className="nav__brand" href="/">
            <span className="nav__brand-mark" aria-hidden="true" />
            {siteConfig.name}
          </Link>
          <p className="caption">{footer.note}</p>
          <p className="caption">{footer.contact}</p>
        </div>
        <nav aria-label="Footer" className="grid content-start gap-2 text-small">
          {siteConfig.nav.map((item) => (
            <Link key={item.href} href={item.href} className="text-text-muted hover:text-text">
              {item.label}
            </Link>
          ))}
        </nav>
        <div className="grid content-start gap-2 text-small">
          <a href={siteConfig.github}>GitHub</a>
          {footer.licenses.map((l) => (
            <a key={l.href} href={l.href} className="text-text-muted hover:text-text">
              {l.label}
            </a>
          ))}
        </div>
      </div>
    </footer>
  );
}
