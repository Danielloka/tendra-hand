/**
 * Runs `fn` when the main thread is idle (or after `timeout` ms at the latest),
 * so heavy setup work doesn't land in the same frame as the first render.
 * Returns a cancel function.
 */
export function onIdle(fn: () => void, timeout = 1500): () => void {
  if ("requestIdleCallback" in window) {
    const id = window.requestIdleCallback(fn, { timeout });
    return () => window.cancelIdleCallback(id);
  }
  const id = setTimeout(fn, 200);
  return () => clearTimeout(id);
}
