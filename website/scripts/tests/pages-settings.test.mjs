import assert from 'node:assert/strict';
import {mkdtempSync, readFileSync, rmSync} from 'node:fs';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import test from 'node:test';
import {resolvePagesSettings} from '../pages-settings.mjs';

const baseline = {
  GITHUB_REPOSITORY: 'modlab-upenn/ModSim',
  GITHUB_EVENT_NAME: 'push',
  GITHUB_REF_NAME: 'main',
  GITHUB_REF_TYPE: 'branch',
};

test('default main and feature builds never publish without opt-in', () => {
  for (const branch of ['main', 'refactor', 'feature/website']) {
    const result = resolvePagesSettings({...baseline, GITHUB_REF_NAME: branch});
    assert.equal(result.publish, false);
    assert.equal(result.siteUrl, 'https://modlab-upenn.github.io');
    assert.equal(result.baseUrl, '/ModSim/');
  }
});

test('a preview repository derives its own independent Pages address', () => {
  const result = resolvePagesSettings({...baseline, GITHUB_REPOSITORY: 'Researcher/ModSim-preview'});
  assert.equal(result.siteUrl, 'https://researcher.github.io');
  assert.equal(result.baseUrl, '/ModSim-preview/');
  assert.equal(result.publish, false);
});

test('account sites and explicit custom domains can build at the root', () => {
  assert.equal(resolvePagesSettings({
    ...baseline, GITHUB_REPOSITORY: 'Researcher/researcher.github.io',
  }).baseUrl, '/');
  const custom = resolvePagesSettings({
    ...baseline, PAGES_SITE_URL: 'https://modsim.example.org/', PAGES_BASE_URL: '/',
  });
  assert.equal(custom.siteUrl, 'https://modsim.example.org');
  assert.equal(custom.baseUrl, '/');
});

test('automatic publishing is restricted to the configured branch', () => {
  assert.equal(resolvePagesSettings({...baseline, PAGES_AUTO_DEPLOY: 'true'}).publish, true);
  assert.equal(resolvePagesSettings({
    ...baseline, PAGES_AUTO_DEPLOY: 'true', GITHUB_REF_NAME: 'refactor',
  }).publish, false);
  assert.equal(resolvePagesSettings({
    ...baseline, PAGES_AUTO_DEPLOY: 'true', PAGES_DEPLOY_BRANCH: 'refactor',
    GITHUB_REF_NAME: 'refactor',
  }).publish, true);
});

test('manual publishing requires an explicit request on the deployment branch', () => {
  assert.equal(resolvePagesSettings({...baseline, GITHUB_EVENT_NAME: 'workflow_dispatch'}).publish,
    false);
  assert.equal(resolvePagesSettings({
    ...baseline, GITHUB_EVENT_NAME: 'workflow_dispatch', PAGES_PUBLISH_REQUESTED: 'true',
  }).publish, true);
  assert.throws(() => resolvePagesSettings({
    ...baseline, GITHUB_EVENT_NAME: 'workflow_dispatch', PAGES_PUBLISH_REQUESTED: 'true',
    GITHUB_REF_NAME: 'refactor',
  }), /restricted to the configured branch: main/);
});

test('pull requests, tags, and other events cannot publish', () => {
  for (const event of ['pull_request', 'pull_request_target', 'workflow_run', 'schedule']) {
    assert.equal(resolvePagesSettings({
      ...baseline, GITHUB_EVENT_NAME: event, PAGES_AUTO_DEPLOY: 'true',
      PAGES_PUBLISH_REQUESTED: 'true',
    }).publish, false);
  }
  assert.equal(resolvePagesSettings({
    ...baseline, GITHUB_REF_TYPE: 'tag', PAGES_AUTO_DEPLOY: 'true',
  }).publish, false);
});

test('invalid or credential-bearing settings fail before building', () => {
  for (const invalid of ['https://example.org/path', 'https://user:pass@example.org',
    'http://example.org', 'https://example.org/?token=test']) {
    assert.throws(() => resolvePagesSettings({...baseline, PAGES_SITE_URL: invalid}));
  }
  for (const invalid of ['ModSim/', '/ModSim', '//example.org/', '/../', '/ModSim/?x=1']) {
    assert.throws(() => resolvePagesSettings({...baseline, PAGES_BASE_URL: invalid}));
  }
  assert.throws(() => resolvePagesSettings({...baseline, PAGES_AUTO_DEPLOY: 'yes'}));
  assert.throws(() => resolvePagesSettings({...baseline, GITHUB_REPOSITORY: 'invalid'}));
});

test('the CLI passes the same preview URL to Docusaurus and the deployment gate', () => {
  const directory = mkdtempSync(join(tmpdir(), 'modsim-pages-settings-'));
  try {
    const environmentFile = join(directory, 'environment');
    const outputFile = join(directory, 'outputs');
    const result = spawnSync(process.execPath,
      [fileURLToPath(new URL('../pages-settings.mjs', import.meta.url))], {
        encoding: 'utf8',
        env: {
          ...process.env, ...baseline, PAGES_AUTO_DEPLOY: 'false', PAGES_PUBLISH_REQUESTED: 'false',
          PAGES_DEPLOY_BRANCH: 'refactor', PAGES_SITE_URL: '', PAGES_BASE_URL: '',
          GITHUB_REPOSITORY: 'researcher/ModSim-preview', GITHUB_REF_NAME: 'refactor',
          GITHUB_ENV: environmentFile, GITHUB_OUTPUT: outputFile, GITHUB_STEP_SUMMARY: '',
        },
      });
    assert.equal(result.status, 0, result.stderr);
    assert.equal(readFileSync(environmentFile, 'utf8'),
      'MODSIM_SITE_URL=https://researcher.github.io\nMODSIM_BASE_URL=/ModSim-preview/\n');
    assert.equal(readFileSync(outputFile, 'utf8'),
      'publish=false\nsite_url=https://researcher.github.io\nbase_url=/ModSim-preview/\n');
  } finally {
    rmSync(directory, {recursive: true, force: true});
  }
});
