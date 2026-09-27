"use client";

import { useState } from "react";

export function CopyButton({ text, getText, label = "Copy code", className = "" }: { text?: string; getText?: () => string; label?: string; className?: string }) {
  const [copied, setCopied] = useState(false);

  async function copy() {
    try {
      await navigator.clipboard.writeText(getText ? getText() : (text ?? ""));
      setCopied(true);
      setTimeout(() => setCopied(false), 1600);
    } catch {
      /* clipboard blocked: nothing to do */
    }
  }

  return (
    <button type="button" className={`btn btn--ghost code__copy ${className}`} onClick={copy} aria-label={copied ? "Copied" : label} data-copied={copied ? "" : undefined}>
      <span aria-hidden="true">{copied ? "Copied" : "Copy"}</span>
      <span className="visually-hidden" aria-live="polite">
        {copied ? "Copied to clipboard" : ""}
      </span>
    </button>
  );
}
