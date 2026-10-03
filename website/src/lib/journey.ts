import data from "@content/journey.json";

export type JourneyStatus = "done" | "in-progress" | "planned" | "abandoned";
export type JourneyEvent = {
  id: string;
  track: string;
  /** ISO date for things that happened. Planned events use `when` instead. */
  date?: string;
  when?: "next" | "later";
  title: string;
  status: JourneyStatus;
  note: string;
  /** Id of the abandoned event this one replaced. */
  replaces?: string;
};
export type Journey = {
  kicker: string;
  title: string;
  lead: string;
  tracks: { id: string; label: string }[];
  events: JourneyEvent[];
};

export const journey = data as Journey;

/** One map column per day since the project started, then "Next" and "Later". */
export const START = "2026-09-26";
export const DAYS = 8;
export const COLUMNS = DAYS + 2;

const DAY_MS = 86_400_000;
const utc = (iso: string) => Date.parse(`${iso}T00:00:00Z`);

export function columnOf(e: JourneyEvent): number {
  if (e.when) return e.when === "next" ? DAYS : DAYS + 1;
  const day = Math.round((utc(e.date ?? START) - utc(START)) / DAY_MS);
  return Math.min(Math.max(day, 0), DAYS - 1);
}

export function columnLabel(col: number): string {
  if (col === DAYS) return "Next";
  if (col === DAYS + 1) return "Later";
  return new Date(utc(START) + col * DAY_MS).toLocaleDateString("en-GB", { day: "numeric", month: "short", timeZone: "UTC" });
}

export function formatWhen(e: JourneyEvent): string {
  if (e.when) return e.when === "next" ? "Next" : "Later";
  return new Date(utc(e.date ?? START)).toLocaleDateString("en-GB", { day: "numeric", month: "long", year: "numeric", timeZone: "UTC" });
}
