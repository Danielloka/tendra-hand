// Tell IndexNow search engines (Bing, Yandex, Naver, ...) which pages changed.
// Usage (from website/): node scripts/indexnow.mjs [--site https://tendrahand.com] [--since 2026-10-01] [--dry-run]
// Needs the key file public/<32 hex>.txt to be deployed on the site first.
import { readdirSync, readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const args = process.argv.slice(2);
const arg = (name) => {
  const i = args.indexOf(name);
  return i >= 0 ? args[i + 1] : undefined;
};
const dryRun = args.includes("--dry-run");
const site = (arg("--site") ?? "https://tendrahand.com").replace(/\/$/, "");
const since = arg("--since") ? new Date(arg("--since")) : null;
if (since && Number.isNaN(since.getTime())) {
  console.error("--since needs a date such as 2026-10-01");
  process.exit(1);
}

const publicDir = join(dirname(fileURLToPath(import.meta.url)), "..", "public");
const keyFiles = readdirSync(publicDir).filter((f) => /^[0-9a-f]{32}\.txt$/.test(f));
if (keyFiles.length !== 1) {
  console.error(`Expected exactly one <32 hex>.txt key file in public/, found ${keyFiles.length}.`);
  process.exit(1);
}
const key = keyFiles[0].replace(".txt", "");
if (readFileSync(join(publicDir, keyFiles[0]), "utf8").trim() !== key) {
  console.error("The key file's content must equal its file name (without .txt).");
  process.exit(1);
}

const sitemap = await fetch(`${site}/sitemap.xml`);
if (!sitemap.ok) {
  console.error(`Could not read ${site}/sitemap.xml (HTTP ${sitemap.status}).`);
  process.exit(1);
}
const xml = await sitemap.text();
let urlList = [];
for (const [, block] of xml.matchAll(/<url>([\s\S]*?)<\/url>/g)) {
  const loc = block.match(/<loc>\s*([^<\s]+)\s*<\/loc>/)?.[1];
  const lastmod = block.match(/<lastmod>\s*([^<\s]+)\s*<\/lastmod>/)?.[1];
  if (!loc) continue;
  if (since && lastmod && new Date(lastmod) <= since) continue;
  // With --since, URLs without <lastmod> are kept, since we cannot tell they are unchanged.
  urlList.push(loc);
}
if (!urlList.length) {
  console.log("No URLs to send.");
  process.exit(0);
}

const host = new URL(site).host;
const body = { host, key, keyLocation: `${site}/${key}.txt`, urlList };
console.log(`Host ${host}, key ${key}, ${urlList.length} URL(s)${since ? ` changed after ${since.toISOString()}` : ""}.`);

if (dryRun) {
  console.log("Dry run, nothing sent. Payload:");
  console.log(JSON.stringify(body, null, 2));
  process.exit(0);
}

const meanings = {
  200: "OK: URLs submitted.",
  202: "Accepted: received, key validation pending (normal for a new key).",
  400: "Bad request: invalid format.",
  403: "Forbidden: key not found or does not match. Is the key file deployed at keyLocation?",
  422: "Unprocessable: URLs do not belong to the host, or the key is invalid.",
  429: "Too many requests: slow down.",
};
const res = await fetch("https://api.indexnow.org/indexnow", {
  method: "POST",
  headers: { "Content-Type": "application/json; charset=utf-8" },
  body: JSON.stringify(body),
});
console.log(`HTTP ${res.status}: ${meanings[res.status] ?? res.statusText}`);
process.exit(res.status === 200 || res.status === 202 ? 0 : 1);
