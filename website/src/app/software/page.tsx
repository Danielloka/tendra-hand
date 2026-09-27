import { ContentPage } from "@/components/pages/ContentPage";
import { pageMetadata } from "@/components/pages/content";

export const generateMetadata = () => pageMetadata("software");

export default function SoftwarePage() {
  return <ContentPage slug="software" toc />;
}
