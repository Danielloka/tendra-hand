import { docsAbsolute, docsUrl, siteUrl } from "@/lib/site";

/** Absolute address of a main-site path. */
export const mainLoc = (path: string) => `${siteUrl}${path === "/" ? "" : path}`;

/** Absolute public address of a docs path ("/docs/hardware"): the docs host once there is one, else the main host. */
export const docsLoc = (path: string) => (docsUrl ? docsAbsolute(path) : mainLoc(path));
