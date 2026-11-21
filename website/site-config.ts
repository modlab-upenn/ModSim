/** Resolve the same static site for production, a preview repository, or localhost. */
export function siteLocation(environment: NodeJS.ProcessEnv): {url: string; baseUrl: string} {
  const url = environment.MODSIM_SITE_URL?.trim() || 'https://modlab-upenn.github.io';
  const baseUrl = environment.MODSIM_BASE_URL?.trim() || '/ModSim/';
  const parsed = new URL(url);
  if (!['http:', 'https:'].includes(parsed.protocol) || parsed.username || parsed.password ||
      parsed.pathname !== '/' || parsed.search || parsed.hash) {
    throw new Error('MODSIM_SITE_URL must be an HTTP(S) origin, for example https://YOUR-USERNAME.github.io.');
  }
  if (!baseUrl.startsWith('/') || !baseUrl.endsWith('/') || baseUrl.includes('//') ||
      /[?#\\\s]/.test(baseUrl) || baseUrl.split('/').some(part => part === '.' || part === '..')) {
    throw new Error('MODSIM_BASE_URL must be a slash-delimited path, for example /ModSim-preview/.');
  }
  return {url: parsed.origin, baseUrl};
}

/** Link to a separately hosted library only when its address is configured. */
export function robotPackLibraryUrl(environment: NodeJS.ProcessEnv): string | null {
  const value = environment.MODSIM_ROBOTPACK_LIBRARY_URL?.trim();
  if (!value) return null;
  const error = 'MODSIM_ROBOTPACK_LIBRARY_URL must be an absolute HTTP(S) URL without credentials.';
  let parsed: URL;
  try {
    parsed = new URL(value);
  } catch {
    throw new Error(error);
  }
  if (!/^https?:\/\//i.test(value) || !['http:', 'https:'].includes(parsed.protocol) ||
      parsed.username || parsed.password) {
    throw new Error(error);
  }
  return parsed.href;
}
