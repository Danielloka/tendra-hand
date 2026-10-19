import NextLink from "next/link";
import type { ComponentProps } from "react";
import { resolveHref } from "@/lib/site";

/** next/link for in-app paths: sends docs paths to the docs host and main paths to the main site (see resolveHref). */
export default function Link({ href, ...props }: ComponentProps<typeof NextLink>) {
  return <NextLink href={typeof href === "string" ? resolveHref(href) : href} {...props} />;
}
