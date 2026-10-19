import { getLogPosts, getPage, type PageSlug } from "@/components/pages/content";
import { getDocsTree, getAllDocSlugs, getDoc, docHref } from "@/lib/docs";
import roadmap from "@content/roadmap.json";
import { githubUrl, siteConfig } from "@/lib/site";
import { readMdx } from "./mdxText";
import { docsLoc, mainLoc } from "./urls";

/**
 * Text for /llms.txt (llmstxt.org: a short map of the site) and /llms-full.txt (all docs and
 * log posts as one Markdown file). Built from the same content the pages use, at build time.
 */

/** Facts an assistant can quote. Keep in sync with CLAUDE.md "Project" and content/home.json. */
const KEY_FACTS = [
  `What it is: ${siteConfig.name} is an open-source, 3D-printed, tendon-driven robotic hand that aims to have the same joints as a human hand, each driven independently.`,
  "Goal: a robot that can do everyday tasks (cook, clean, use tools) as well as a person, understanding and carrying out multi-step tasks; the hand is the focus and the first step.",
  "Tendons: each joint has one antagonistic cord loop on a spool on its motor; one motor both bends and straightens the joint. Motors sit in the palm or forearm, not in the fingers.",
  "V0 (built, prototype): thumb + index finger, 8 joints, 8 x 28BYJ-48 stepper motors with ULN2003 drivers, ESP32-S3 controller over USB.",
  "V1 (designed, not built yet): the full five-finger hand, 20 joints on 16 Feetech SCS0009 smart servos in the forearm (each fingertip joint follows the middle joint through a small linkage).",
  "Software: Python library `tendra` (Hand API with simulation and real-hardware backends), MuJoCo simulation and digital twin, webcam hand tracking, reinforcement-learning grasp training, ESP32 firmware (PlatformIO, C++).",
  "Licenses: code Apache-2.0; hardware (CAD, print files, electronics) CERN-OHL-S-2.0; docs and research CC BY 4.0.",
  "Status: early-stage research project, built in public by an individual maker. The V0 prototype is being brought up; V1 is designed and simulated (including grasp learning in simulation), not built.",
  "Milestones: a hand as capable as a human one (in progress), then senses (touch, vision), learned skills, understanding tasks, and a two-armed robot working in a real home (all planned).",
  `Source: ${githubUrl}`,
  `Not to be confused with the unrelated open-source project "Tendra H1" (a different robotic hand).`,
];

const pageEntry = async (slug: PageSlug, url: string) => {
  const fm = (await getPage(slug)).frontmatter;
  return `- [${fm.title}](${url}): ${fm.description ?? fm.lead ?? ""}`;
};

export async function buildLlmsTxt(): Promise<string> {
  const groups = await getDocsTree();
  const posts = await getLogPosts();
  const L: string[] = [];
  L.push(`# ${siteConfig.name}`, "", `> ${siteConfig.description}`, "");
  L.push("## Key facts", "", ...KEY_FACTS.map((f) => `- ${f}`), "");
  L.push("## Project", "");
  L.push(`- [Home](${mainLoc("/")}): ${siteConfig.tagline} Overview with a 3D hand, the story of where the project is going, and the milestones.`);
  L.push(await pageEntry("project", mainLoc("/project")));
  L.push(`- [Journey](${mainLoc("/journey")}): Where the project is on its five-step roadmap, the big steps built so far (first prototype, full hand design, digital twin, learning to grasp) and the ideas that were dropped, with the lesson from each.`);
  L.push(await pageEntry("gallery", mainLoc("/gallery")));
  L.push(await pageEntry("contact", mainLoc("/contact")), "");
  L.push("## Reference", "");
  L.push(await pageEntry("hardware", docsLoc("/docs/hardware")));
  L.push(await pageEntry("software", docsLoc("/docs/software")));
  L.push(await pageEntry("log", docsLoc("/docs/log")), "");
  for (const g of groups) {
    L.push(`## Docs: ${g.title}`, "");
    for (const p of g.pages) L.push(`- [${p.title}](${docsLoc(p.href)}): ${p.description}`);
    L.push("");
  }
  L.push("## Build log", "");
  for (const p of posts) L.push(`- [${p.title}](${docsLoc(p.href)}): ${p.summary} (${p.date})`);
  L.push("", "## Optional", "");
  L.push(`- [Full text of the docs and build log](${mainLoc("/llms-full.txt")}): every doc and log post as one Markdown file.`);
  L.push(`- [Source code, CAD and research on GitHub](${githubUrl}): firmware, Python library, simulation, hardware files, research notes.`);
  L.push(`- [Sitemap](${mainLoc("/sitemap.xml")})`, "");
  return L.join("\n");
}

export async function buildLlmsFull(): Promise<string> {
  const L: string[] = [];
  L.push(`# ${siteConfig.name}: full text`, "", `> ${siteConfig.description}`, "");
  L.push("Content of the Tendra Hand docs, reference pages and build log as plain Markdown. Index: " + mainLoc("/llms.txt"), "");
  L.push("## Key facts", "", ...KEY_FACTS.map((f) => `- ${f}`), "");

  const section = (title: string, url: string, body: string, meta?: string) => {
    L.push("---", "", `# ${title}`, "", `URL: ${url}`);
    if (meta) L.push(meta);
    // demote headings by one level so each document sits under its own H1
    const shifted = body.replace(/^(#{1,5})(\s)/gm, "$1#$2");
    L.push("", shifted, "");
  };

  for (const [slug, url] of [
    ["project", mainLoc("/project")],
    ["hardware", docsLoc("/docs/hardware")],
    ["software", docsLoc("/docs/software")],
    ["contact", mainLoc("/contact")],
  ] as [PageSlug, string][]) {
    const { frontmatter: fm } = await getPage(slug);
    const raw = readMdx(`pages/${slug}`);
    const road =
      slug === "project"
        ? `## Roadmap\n\n${roadmap.milestones
            .map((m, i) => `${i + 1}. **${m.title}** (${m.status.replace(/-/g, " ")}): ${m.summary}`)
            .join("\n")}`
        : "";
    // the page's <Roadmap /> component has no text in the MDX: put the roadmap data under its heading
    const body = road ? raw.body.replace(/^## Roadmap\s*$/m, road) : raw.body;
    section(fm.title, url, `${fm.lead ? `${fm.lead}\n\n` : ""}${body}`);
  }
  for (const slug of getAllDocSlugs()) {
    const doc = await getDoc(slug);
    const raw = readMdx(`docs/${slug}`);
    const upd = raw.frontmatter.updated;
    section(doc!.frontmatter.title, docsLoc(docHref(slug)), `${doc!.frontmatter.description}\n\n${raw.body}`, upd ? `Updated: ${upd}` : undefined);
  }
  for (const p of [...(await getLogPosts())].reverse()) {
    section(p.title, docsLoc(p.href), `${p.summary}\n\n${readMdx(`log/${p.slug}`).body}`, `Date: ${p.date}`);
  }
  return L.join("\n");
}
