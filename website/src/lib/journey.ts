import data from "@content/journey.json";

export type JourneyStatus = "done" | "abandoned";
export type JourneyEvent = {
  id: string;
  track: string;
  date: string;
  title: string;
  status: JourneyStatus;
  note: string;
  /** Abandoned events: why we dropped it, and what we learned. */
  why?: string;
  lesson?: string;
  /** Id of the abandoned event this one replaced. */
  replaces?: string;
};
export type JourneyDay = { date: string; title: string; blurb: string };
export type Journey = {
  kicker: string;
  title: string;
  lead: string;
  tracks: { id: string; label: string }[];
  days: JourneyDay[];
  events: JourneyEvent[];
};

export const journey = data as Journey;

/** One map column per day, from the first day on. */
export const COLUMNS = journey.days.length;

const utc = (iso: string) => Date.parse(`${iso}T00:00:00Z`);

export function columnOf(e: JourneyEvent): number {
  const i = journey.days.findIndex((d) => d.date === e.date);
  return Math.max(i, 0);
}

export function formatDate(iso: string, long = false): string {
  return new Date(utc(iso)).toLocaleDateString("en-GB", long ? { day: "numeric", month: "long", year: "numeric", timeZone: "UTC" } : { day: "numeric", month: "short", timeZone: "UTC" });
}

export const trackLabel = (id: string) => journey.tracks.find((t) => t.id === id)?.label ?? id;
