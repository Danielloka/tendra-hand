"use client";

import { useState } from "react";

/** "Copy address" button for the email card. Without JS it is hidden (the mailto link still works). */
export function CopyEmail({ address }: { address: string }) {
  const [copied, setCopied] = useState(false);
  return (
    <button
      type="button"
      className="contact-mail__copy"
      onClick={async () => {
        try {
          await navigator.clipboard.writeText(address);
          setCopied(true);
          setTimeout(() => setCopied(false), 2000);
        } catch {
          /* clipboard blocked: the mailto link is still there */
        }
      }}
    >
      <span aria-live="polite">{copied ? "Copied" : "Copy address"}</span>
    </button>
  );
}
