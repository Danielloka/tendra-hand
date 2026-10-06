import type { Metadata, Viewport } from "next";
import { Inter, JetBrains_Mono } from "next/font/google";
import { SiteJsonLd } from "@/components/seo/SiteJsonLd";
import { siteConfig, siteUrl } from "@/lib/site";
import "./globals.css";

const inter = Inter({ subsets: ["latin"], variable: "--font-inter", display: "swap" });
const mono = JetBrains_Mono({ subsets: ["latin"], variable: "--font-jetbrains", display: "swap", preload: false, weight: ["400", "500"] });

const defaultTitle = `${siteConfig.name}: a human hand, rebuilt in the open`;

// Site-wide defaults. Every public page sets its own canonical (alternates) and og:url, so they are
// not set here: a root canonical would be inherited by the 404 and /dev pages and point them at "/".
export const metadata: Metadata = {
  metadataBase: new URL(siteUrl),
  title: { default: defaultTitle, template: `%s · ${siteConfig.name}` },
  description: siteConfig.description,
  applicationName: siteConfig.name,
  keywords: [
    "Tendra Hand",
    "robotic hand",
    "open-source robotic hand",
    "tendon-driven hand",
    "3D-printed robotic hand",
    "dexterous hand",
    "humanoid hand",
    "robot hand DOF",
    "ESP32-S3",
    "Feetech SCS0009",
    "MuJoCo",
    "reinforcement learning",
    "teleoperation",
    "open hardware",
  ],
  authors: [{ name: "Tendra Hand contributors", url: siteConfig.github }],
  creator: "Tendra Hand",
  category: "technology",
  robots: { index: true, follow: true, googleBot: { index: true, follow: true, "max-image-preview": "large", "max-snippet": -1, "max-video-preview": -1 } },
  openGraph: { type: "website", siteName: siteConfig.name, locale: "en_US", title: defaultTitle, description: siteConfig.description },
  twitter: { card: "summary_large_image", title: defaultTitle, description: siteConfig.description },
};

export const viewport: Viewport = {
  themeColor: [
    { media: "(prefers-color-scheme: light)", color: "#ffffff" },
    { media: "(prefers-color-scheme: dark)", color: "#000000" },
  ],
};

// Runs before first paint: marks JS as available (for scroll reveals) and
// applies the saved theme so there is no flash. Same key as the style guide.
// "motion" = the homepage story animates (see src/lib/scroll/motion.ts; same storage key).
const bootScript = `var d=document.documentElement;d.classList.add("js");var m=!matchMedia("(prefers-reduced-motion: reduce)").matches;try{if(localStorage.getItem("tendra-theme")==="dark")d.dataset.theme="dark";if(localStorage.getItem("tendra-motion")==="on")m=true}catch(e){}if(m)d.classList.add("motion")`;

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en" className={`${inter.variable} ${mono.variable}`} suppressHydrationWarning>
      <head>
        <script dangerouslySetInnerHTML={{ __html: bootScript }} />
      </head>
      <body>
        <a className="skip-link" href="#main">
          Skip to content
        </a>
        <SiteJsonLd />
        {children}
      </body>
    </html>
  );
}
