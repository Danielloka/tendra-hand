import { execFileSync } from "node:child_process";

/** Shallow clones (some CI setups) give every file the tip commit's date, which would be wrong. */
let shallow: boolean | undefined;
const isShallow = () => {
  if (shallow === undefined) {
    try {
      shallow = execFileSync("git", ["rev-parse", "--is-shallow-repository"], { encoding: "utf8" }).trim() !== "false";
    } catch {
      shallow = true;
    }
  }
  return shallow;
};

const cache = new Map<string, string | undefined>();

/** ISO date of the last git commit that touched a file (path relative to website/), or undefined. Never "now". Build time only. */
export function gitLastModified(file: string): string | undefined {
  if (cache.has(file)) return cache.get(file);
  let out: string | undefined;
  try {
    if (!isShallow()) out = execFileSync("git", ["log", "-1", "--format=%cI", "--", file], { encoding: "utf8" }).trim() || undefined;
  } catch {
    out = undefined;
  }
  cache.set(file, out);
  return out;
}

/** Newest of several dates (ISO strings), ignoring missing ones. */
export const newest = (...dates: (string | undefined)[]) => dates.filter((d): d is string => !!d).sort().at(-1);
