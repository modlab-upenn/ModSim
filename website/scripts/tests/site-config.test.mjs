import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import ts from 'typescript';

function loadTypeScript(relative, globals = {}) {
  const source = readFileSync(new URL(relative, import.meta.url), 'utf8');
  const output = ts.transpileModule(source, {
    compilerOptions: {module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022},
  }).outputText;
  const exports = {};
  vm.runInNewContext(output, {exports, URL, ...globals});
  return exports;
}

const {siteLocation} = loadTypeScript('../../site-config.ts');

test('the normal build retains the existing project URL', () => {
  const location = siteLocation({});
  assert.equal(location.url, 'https://modlab-upenn.github.io');
  assert.equal(location.baseUrl, '/ModSim/');
});

test('preview repositories and custom-domain roots need no source edits', () => {
  for (const baseUrl of ['/ModSim-preview/', '/preview/refactor/', '/']) {
    const location = siteLocation({MODSIM_SITE_URL: 'https://researcher.github.io/', MODSIM_BASE_URL: baseUrl});
    assert.equal(location.url, 'https://researcher.github.io');
    assert.equal(location.baseUrl, baseUrl);
  }
});

test('misconfigured origins and paths fail before asset URLs are generated', () => {
  for (const url of ['ftp://example.org', 'https://user:pass@example.org', 'https://example.org/project/', 'https://example.org/?token=1']) {
    assert.throws(() => siteLocation({MODSIM_SITE_URL: url}));
  }
  for (const baseUrl of ['ModSim/', '/ModSim', '//example.org/', '/../', '/bad path/', '/bad\\path/', '/?x=1/']) {
    assert.throws(() => siteLocation({MODSIM_BASE_URL: baseUrl}));
  }
});

test('historical index.html links retain the base path, query, and section', () => {
  for (const base of ['/ModSim/', '/ModSim-preview/', '/']) {
    for (const nested of ['', 'docs/how-to/']) {
      const navigations = [];
      loadTypeScript('../../src/clientModules/canonicalPaths.ts', {
        window: {location: {
          href: `https://example.org${base}${nested}index.html?from=old#docking`,
          replace: destination => navigations.push(destination),
        }},
      });
      assert.deepEqual(navigations, [`https://example.org${base}${nested}?from=old#docking`]);
    }
  }
});

test('normal routes do not redirect and client modules are safe without a browser', () => {
  const navigations = [];
  const hooks = loadTypeScript('../../src/clientModules/canonicalPaths.ts', {
    window: {location: {href: 'https://example.org/ModSim/docs/', replace: path => navigations.push(path)}},
  });
  hooks.onRouteUpdate();
  assert.deepEqual(navigations, []);
  assert.doesNotThrow(() => loadTypeScript('../../src/clientModules/canonicalPaths.ts'));
});
