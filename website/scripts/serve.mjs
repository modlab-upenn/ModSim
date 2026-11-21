import {createServer} from 'node:http';
import {lstat} from 'node:fs/promises';
import {dirname, extname, relative, resolve, sep} from 'node:path';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';
import {loadSiteConfig} from '@docusaurus/core/lib/server/config.js';
import serveHandler from 'serve-handler';

const siteDir = resolve(dirname(fileURLToPath(import.meta.url)), '..');

async function fileInfo(filename) {
  try {
    return await lstat(filename);
  } catch (error) {
    if (error.code === 'ENOENT' || error.code === 'ENOTDIR') return undefined;
    throw error;
  }
}

function redirect(response, location) {
  response.writeHead(302, {Location: location});
  response.end();
}

async function main() {
  const {values} = parseArgs({
    options: {
      host: {type: 'string', default: '127.0.0.1'},
      port: {type: 'string', default: '3000'},
      dir: {type: 'string', default: 'build'},
      help: {type: 'boolean', short: 'h', default: false},
    },
  });
  if (values.help) {
    console.log('Usage: npm run docs:serve -- [--host 127.0.0.1] [--port 3000] [--dir build]');
    return;
  }
  const port = Number(values.port);
  const buildDir = resolve(siteDir, values.dir);
  if (!/^\d+$/.test(values.port) || !Number.isInteger(port) || port > 65535) {
    throw new Error('--port must be an integer between 0 and 65535.');
  }
  if (!(await fileInfo(resolve(buildDir, 'index.html')))?.isFile()) {
    throw new Error('Local documentation build is missing. Run npm --prefix website run docs:build first.');
  }
  // Use the pinned Docusaurus loader so the preview shares the central route config.
  const {siteConfig: {baseUrl, trailingSlash}} = await loadSiteConfig({siteDir});
  const server = createServer(async (request, response) => {
    try {
      const url = new URL(request.url ?? '/', 'http://localhost');
      if (!url.pathname.startsWith(baseUrl)) {
        redirect(response, baseUrl + url.search);
        return;
      }
      let requestPath = '/' + url.pathname.slice(baseUrl.length);
      const decodedPath = decodeURIComponent(requestPath);
      const filename = resolve(buildDir, '.' + decodedPath);
      const relativeFilename = relative(buildDir, filename);
      if (relativeFilename === '..' || relativeFilename.startsWith('..' + sep)) {
        response.writeHead(400);
        response.end('Invalid path.');
        return;
      }
      const info = await fileInfo(filename);
      const isFile = info?.isFile();
      const isRoute = info?.isDirectory() || !extname(decodedPath);
      // Serve real files first: extensions such as .woff2 must never gain a slash.
      if (!isFile && isRoute) {
        let canonicalPath = url.pathname;
        if (trailingSlash === true && !canonicalPath.endsWith('/')) canonicalPath += '/';
        if (trailingSlash === false && canonicalPath !== baseUrl) canonicalPath = canonicalPath.replace(/\/+$/, '');
        if (canonicalPath !== url.pathname) {
          redirect(response, canonicalPath + url.search);
          return;
        }
        const routePath = requestPath.replace(/\/+$/, '');
        for (const candidate of [`${routePath}/index.html`, `${routePath}.html`]) {
          if ((await fileInfo(resolve(buildDir, '.' + decodeURIComponent(candidate))))?.isFile()) {
            requestPath = candidate;
            break;
          }
        }
      }
      request.url = requestPath + url.search;
      // Route canonicalization stays above, where redirects retain the base URL.
      await serveHandler(request, response, {
        public: buildDir,
        cleanUrls: false,
        directoryListing: false,
      });
    } catch (error) {
      response.writeHead(error instanceof URIError ? 400 : 500);
      response.end(error instanceof URIError ? 'Invalid path encoding.' : 'Local preview failed.');
      console.error(error.message);
    }
  });
  await new Promise((resolveListen, reject) => {
    server.once('error', reject);
    server.listen(port, values.host, resolveListen);
  });
  const host = values.host.includes(':') ? `[${values.host}]` : values.host;
  console.log(`Serving local documentation at http://${host}:${server.address().port}${baseUrl}`);
}

main().catch((error) => {
  console.error(error.message);
  process.exitCode = 1;
});
