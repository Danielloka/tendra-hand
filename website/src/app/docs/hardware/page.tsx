import { ContentPage } from "@/components/pages/ContentPage";
import { pageMetadata } from "@/components/pages/content";

export const generateMetadata = () => pageMetadata("hardware");

export default function HardwarePage() {
  return <ContentPage slug="hardware" toc />;
}
