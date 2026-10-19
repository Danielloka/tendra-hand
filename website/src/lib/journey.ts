import data from "@content/journey.json";

/** A step inside the current milestone: done, being worked on now, or still to come. */
export type NowStatus = "done" | "current" | "next";

export type Chapter = {
  id: string;
  /** Rough time, e.g. "September 2026". No day counts: the page tells the big story. */
  when: string;
  title: string;
  text: string;
  highlights: string[];
  image: { src: string; alt: string; width: number; height: number; fit?: "cover" | "contain" };
};

/** A dropped idea, told as a lesson. */
export type Lesson = { id: string; tried: string; why: string; lesson: string };

type Head = { kicker: string; title: string; lead?: string };

export type Journey = {
  kicker: string;
  title: string;
  lead: string;
  now: Head & {
    /** Id of the milestone in content/roadmap.json that we are working on. */
    milestone: string;
    text: string;
    steps: { label: string; status: NowStatus }[];
  };
  chaptersHead: Head;
  chapters: Chapter[];
  lessonsHead: Head;
  lessons: Lesson[];
  next: Head & {
    text: string;
    actions: { label: string; href: string; variant: "primary" | "secondary" }[];
  };
};

export const journey = data as Journey;
