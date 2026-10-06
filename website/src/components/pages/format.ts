// Small helpers that are safe on both the server and the client.
/** "2026-09-26" → "26 September 2026" (the date is a calendar day, so no time zone shift). */
export function formatDate(iso: string): string {
  return new Intl.DateTimeFormat("en-GB", { day: "numeric", month: "long", year: "numeric", timeZone: "UTC" }).format(new Date(`${iso}T00:00:00Z`));
}
