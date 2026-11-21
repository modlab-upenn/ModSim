import {appendFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';

function flag(value, name) {
  const normalized = (value ?? '').trim().toLowerCase();
  if (!normalized || normalized === 'false') return false;
  if (normalized === 'true') return true;
  throw new Error(`${name} must be true or false.`);
}

export function resolvePagesSettings(environment) {
  const repository = environment.GITHUB_REPOSITORY ?? '';
  const match = /^([A-Za-z0-9_.-]+)\/([A-Za-z0-9_.-]+)$/.exec(repository);
  if (!match) throw new Error('GITHUB_REPOSITORY must be OWNER/REPOSITORY.');
  const [, owner, name] = match;
  const defaultOrigin = `https://${owner.toLowerCase()}.github.io`;
  const url = new URL(environment.PAGES_SITE_URL?.trim() || defaultOrigin);
  if (url.protocol !== 'https:' || url.pathname !== '/' || url.search || url.hash ||
      url.username || url.password) {
    throw new Error('PAGES_SITE_URL must be an HTTPS origin, without a path or credentials.');
  }
  const accountSite = name.toLowerCase() === `${owner.toLowerCase()}.github.io`;
  const baseUrl = environment.PAGES_BASE_URL?.trim() || (accountSite ? '/' : `/${name}/`);
  if (!/^\/(?:[A-Za-z0-9._~-]+\/)*$/.test(baseUrl) || baseUrl.split('/').some(
    part => part === '.' || part === '..',
  )) {
    throw new Error('PAGES_BASE_URL must be a site path beginning and ending with /.');
  }
  const deployBranch = environment.PAGES_DEPLOY_BRANCH?.trim() || 'main';
  const branch = environment.GITHUB_REF_NAME ?? '';
  if (/[\r\n]/.test(deployBranch + branch)) throw new Error('Branch names must fit on one line.');
  const event = environment.GITHUB_EVENT_NAME;
  const branchMatches = environment.GITHUB_REF_TYPE === 'branch' && branch === deployBranch;
  const requested = flag(environment.PAGES_PUBLISH_REQUESTED, 'Publish input');
  const automatic = flag(environment.PAGES_AUTO_DEPLOY, 'PAGES_AUTO_DEPLOY');
  if (event === 'workflow_dispatch' && requested && !branchMatches) {
    throw new Error(`Publishing is restricted to the configured branch: ${deployBranch}.`);
  }
  const publish = branchMatches && (
    (event === 'workflow_dispatch' && requested) || (event === 'push' && automatic)
  );
  return {siteUrl: url.origin, baseUrl, publish, deployBranch, branch};
}

function main() {
  const settings = resolvePagesSettings(process.env);
  if (process.env.GITHUB_ENV) {
    appendFileSync(process.env.GITHUB_ENV,
      `MODSIM_SITE_URL=${settings.siteUrl}\nMODSIM_BASE_URL=${settings.baseUrl}\n`);
  }
  if (process.env.GITHUB_OUTPUT) {
    appendFileSync(process.env.GITHUB_OUTPUT,
      `publish=${settings.publish}\nsite_url=${settings.siteUrl}\nbase_url=${settings.baseUrl}\n`);
  }
  if (process.env.GITHUB_STEP_SUMMARY) {
    appendFileSync(process.env.GITHUB_STEP_SUMMARY,
      `## Combined website build\n\n` +
      `- Site URL: ${settings.siteUrl}${settings.baseUrl}\n` +
      `- Configured deployment branch: \`${settings.deployBranch}\`\n` +
      `- Publish this build: **${settings.publish}**\n\n` +
      `Every successful build has a downloadable site-preview artifact. ` +
      `A build-only run leaves the live website unchanged.\n`);
  }
  process.stdout.write(`${JSON.stringify(settings)}\n`);
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    main();
  } catch (error) {
    console.error(error.message);
    process.exitCode = 1;
  }
}
