import { githubUrl, siteConfig } from "@/lib/site";
import { docsLoc, mainLoc } from "./urls";

/** schema.org builders. Pure functions, no React. */

export const LICENSES = {
  code: "https://www.apache.org/licenses/LICENSE-2.0",
  hardware: "https://spdx.org/licenses/CERN-OHL-S-2.0.html",
  docs: "https://creativecommons.org/licenses/by/4.0/",
};

const author = { "@type": "Person", name: "Danielloka", url: "https://github.com/Danielloka" };

const orgId = () => `${mainLoc("/")}#organization`;
const siteId = () => `${mainLoc("/")}#website`;

/** Organization + WebSite + the open-source project itself. Safe to render on every page. */
export function siteGraph() {
  return {
    "@context": "https://schema.org",
    "@graph": [
      {
        "@type": "Organization",
        "@id": orgId(),
        name: siteConfig.name,
        url: mainLoc("/"),
        logo: mainLoc("/icon.svg"),
        description: siteConfig.description,
        sameAs: [githubUrl],
        founder: author,
      },
      {
        "@type": "WebSite",
        "@id": siteId(),
        url: mainLoc("/"),
        name: siteConfig.name,
        description: siteConfig.description,
        inLanguage: "en",
        publisher: { "@id": orgId() },
      },
      {
        "@type": "SoftwareSourceCode",
        "@id": `${mainLoc("/")}#source`,
        name: `${siteConfig.name}: firmware, software, simulation and CAD`,
        description: siteConfig.description,
        codeRepository: githubUrl,
        url: mainLoc("/project"),
        programmingLanguage: ["Python", "C++"],
        runtimePlatform: "ESP32-S3, MuJoCo",
        license: [LICENSES.code, LICENSES.hardware, LICENSES.docs],
        author,
        keywords: "robotic hand, tendon-driven, open source, 3D printed, humanoid, dexterous manipulation, MuJoCo, ESP32",
      },
    ],
  };
}

export type Crumb = { name: string; url: string };

export const breadcrumbs = (items: Crumb[]) => ({
  "@context": "https://schema.org",
  "@type": "BreadcrumbList",
  itemListElement: items.map((c, i) => ({ "@type": "ListItem", position: i + 1, name: c.name, item: c.url })),
});

export const docsCrumbs = (...rest: Crumb[]): Crumb[] => [
  { name: siteConfig.name, url: mainLoc("/") },
  { name: "Docs", url: docsLoc("/docs") },
  ...rest,
];

export function techArticle(o: { title: string; description?: string; url: string; updated?: string; published?: string; image?: string; keywords?: string[]; section?: string }) {
  return {
    "@context": "https://schema.org",
    "@type": "TechArticle",
    headline: o.title,
    description: o.description,
    url: o.url,
    mainEntityOfPage: o.url,
    inLanguage: "en",
    datePublished: o.published ?? o.updated,
    dateModified: o.updated ?? o.published,
    image: [o.image ?? mainLoc("/opengraph-image")],
    keywords: o.keywords?.join(", "),
    articleSection: o.section,
    author,
    publisher: { "@id": orgId() },
    isPartOf: { "@id": siteId() },
    license: LICENSES.docs,
    about: { "@type": "Thing", name: siteConfig.name },
  };
}

export const faqPage = (qa: { q: string; a: string }[]) => ({
  "@context": "https://schema.org",
  "@type": "FAQPage",
  mainEntity: qa.map(({ q, a }) => ({ "@type": "Question", name: q, acceptedAnswer: { "@type": "Answer", text: a } })),
});

export const imageGallery = (o: { name: string; description?: string; url: string; images: { src: string; alt?: string; caption?: string }[] }) => ({
  "@context": "https://schema.org",
  "@type": "ImageGallery",
  name: o.name,
  description: o.description,
  url: o.url,
  license: LICENSES.docs,
  isPartOf: { "@id": siteId() },
  image: o.images.map((i) => ({
    "@type": "ImageObject",
    contentUrl: mainLoc(i.src),
    caption: i.caption,
    description: i.alt,
    license: LICENSES.docs,
  })),
});

export const webPage = (o: { name: string; description?: string; url: string; type?: string }) => ({
  "@context": "https://schema.org",
  "@type": o.type ?? "WebPage",
  name: o.name,
  description: o.description,
  url: o.url,
  inLanguage: "en",
  isPartOf: { "@id": siteId() },
});

export const collectionPage = (o: { name: string; description?: string; url: string; items: { name: string; url: string }[] }) => ({
  ...webPage({ ...o, type: "CollectionPage" }),
  mainEntity: { "@type": "ItemList", itemListElement: o.items.map((it, i) => ({ "@type": "ListItem", position: i + 1, name: it.name, url: it.url })) },
});
