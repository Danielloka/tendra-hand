import { ContentPage } from "@/components/pages/ContentPage";
import { pageMetadata } from "@/components/pages/content";
import { githubUrl } from "@/lib/site";

export const generateMetadata = () => pageMetadata("contribute");

export default function ContributePage() {
  return (
    <ContentPage
      slug="contribute"
      actions={
        <>
          <a className="btn btn--primary btn--lg" href={githubUrl}>
            Open GitHub
          </a>
          <a className="btn btn--ghost" href={`${githubUrl}/issues`}>
            See open issues <span aria-hidden="true">›</span>
          </a>
        </>
      }
    />
  );
}
