import Image from "next/image";
import { githubUrl, siteConfig } from "@/lib/site";
import { CopyEmail } from "./CopyEmail";

type Item = (typeof siteConfig.contact.items)[number];

/** X logo (Simple Icons, CC0). */
const XLogo = () => (
  <svg viewBox="0 0 24 24" aria-hidden="true" fill="currentColor">
    <path d="M18.901 1.153h3.68l-8.04 9.19L24 22.846h-7.406l-5.8-7.584-6.638 7.584H.474l8.6-9.83L0 1.154h7.594l5.243 6.932ZM17.61 20.644h2.039L6.486 3.24H4.298Z" />
  </svg>
);

const InstaLogo = () => (
  <svg viewBox="0 0 24 24" aria-hidden="true" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round">
    <rect x="3" y="3" width="18" height="18" rx="5.5" />
    <circle cx="12" cy="12" r="4.2" />
    <circle cx="17.4" cy="6.6" r="0.4" fill="currentColor" />
  </svg>
);

const MailLogo = () => (
  <svg viewBox="0 0 24 24" aria-hidden="true" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
    <rect x="3" y="5" width="18" height="14" rx="2.5" />
    <path d="M3.5 7l8.5 6.5L20.5 7" />
  </svg>
);

/** Small Tendra mark used as the profile picture in the X and Instagram previews. */
const Avatar = ({ ring = false }: { ring?: boolean }) => (
  <span className={`contact-avatar${ring ? " contact-avatar--ring" : ""}`} aria-hidden="true">
    {/* eslint-disable-next-line @next/next/no-img-element -- tiny static icon */}
    <img src="/brand/tendra-icon-512.png" alt="" width={48} height={48} />
  </span>
);

const INSTA_TILES = [
  "photos/2026-09-29-v0-fingers-three-quarter.jpg",
  "photos/2026-09-29-v0-forearm-motors-spools.jpg",
  "photos/2026-09-29-v0-printed-parts-box.jpg",
];

function XCard({ item }: { item: Item }) {
  return (
    <a className="contact-card contact-card--x" href={item.href} target="_blank" rel="noopener noreferrer">
      <div className="contact-card__top">
        <span className="contact-card__logo"><XLogo /></span>
        <span className="contact-card__cta">Follow</span>
      </div>
      <div className="contact-x__post" aria-hidden="true">
        <div className="contact-x__head">
          <Avatar />
          <div>
            <strong>Tendra Hand</strong>
            <span>{item.handle}</span>
          </div>
        </div>
        <p>Building an open robotic hand, one joint at a time. Updates here as it happens.</p>
      </div>
      <div className="contact-card__foot">
        <h2>{item.label}</h2>
        <p className="contact-card__handle">{item.handle}</p>
        <p className="contact-card__blurb">{item.blurb}</p>
      </div>
    </a>
  );
}

function InstaCard({ item }: { item: Item }) {
  return (
    <a className="contact-card contact-card--insta" href={item.href} target="_blank" rel="noopener noreferrer">
      <div className="contact-card__top">
        <span className="contact-card__logo"><InstaLogo /></span>
        <span className="contact-card__cta">Follow</span>
      </div>
      <div className="contact-insta__profile" aria-hidden="true">
        <div className="contact-insta__head">
          <Avatar ring />
          <strong>{item.handle.replace(/^@/, "")}</strong>
        </div>
        <div className="contact-insta__grid">
          {INSTA_TILES.map((f) => (
            <Image key={f} src={`/media/${f}`} alt="" width={160} height={160} sizes="7rem" />
          ))}
        </div>
      </div>
      <div className="contact-card__foot">
        <h2>{item.label}</h2>
        <p className="contact-card__handle">{item.handle}</p>
        <p className="contact-card__blurb">{item.blurb}</p>
      </div>
    </a>
  );
}

function MailCard({ item }: { item: Item }) {
  const href = `${item.href}?subject=${encodeURIComponent("Hello Tendra")}`;
  return (
    <div className="contact-card contact-card--mail">
      <div className="contact-card__top">
        <span className="contact-card__logo"><MailLogo /></span>
        <CopyEmail address={item.handle} />
      </div>
      <div className="contact-mail__window" aria-hidden="true">
        <div className="contact-mail__bar">
          <span /><span /><span />
          <em>New message</em>
        </div>
        <dl>
          <div><dt>To</dt><dd>{item.handle}</dd></div>
          <div><dt>Subject</dt><dd>Hello Tendra</dd></div>
        </dl>
        <p className="contact-mail__draft">Hi! I have an idea for the hand…<span className="contact-mail__caret" /></p>
      </div>
      <div className="contact-card__foot">
        <h2>{item.label}</h2>
        <p className="contact-card__handle">{item.handle}</p>
        <p className="contact-card__blurb">{item.blurb}</p>
        <a className="btn btn--primary contact-mail__send" href={href}>Write an email</a>
      </div>
    </div>
  );
}

const CARDS: Record<string, (p: { item: Item }) => React.ReactNode> = { x: XCard, instagram: InstaCard, email: MailCard };

/** Contact cards from site.json `contact.items`, each styled like its platform. */
export function ContactCards() {
  const { items } = siteConfig.contact;
  return (
    <div className="container contact-cards">
      <div className="contact-cards__grid">
        {items.filter((it) => it.href && CARDS[it.id]).map((item) => {
          const Card = CARDS[item.id];
          return <Card key={item.id} item={item} />;
        })}
      </div>
      <div className="contact-cards__note">
        <p>
          <strong>Everything is open.</strong> The designs, the code and the research are on{" "}
          <a href={githubUrl}>GitHub</a>, free to use and build on.
        </p>
        <a className="btn btn--secondary" href={githubUrl}>View on GitHub</a>
      </div>
    </div>
  );
}
