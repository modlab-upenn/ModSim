/** Keep historical filename links valid on static hosts without redirect rules. */
function canonicalizeIndexPath(): void {
  if (typeof window === 'undefined') return;
  const destination = new URL(window.location.href);
  if (!destination.pathname.endsWith('/index.html')) return;
  destination.pathname = destination.pathname.slice(0, -'index.html'.length);
  window.location.replace(destination.href);
}

canonicalizeIndexPath();

export function onRouteUpdate(): void {
  canonicalizeIndexPath();
}
