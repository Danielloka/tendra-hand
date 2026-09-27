import { buildSearchIndex } from "@/lib/docs";

// Built once at build time; DocsSearch fetches it on first focus.
export const dynamic = "force-static";

export async function GET() {
  return Response.json(await buildSearchIndex());
}
