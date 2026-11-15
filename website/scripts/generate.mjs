import {spawnSync} from 'node:child_process';
import {existsSync} from 'node:fs';
import {dirname, resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {parseArgs} from 'node:util';

const siteDir = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const repoDir = resolve(siteDir, '..');

async function main() {
  const {values} = parseArgs({
    options: {
      api: {type: 'boolean', default: false},
      uml: {type: 'boolean', default: false},
      check: {type: 'boolean', default: false},
      help: {type: 'boolean', short: 'h', default: false},
    },
  });
  if (values.help) {
    console.log('Usage: npm run docs:generate -- [--api | --uml | --check]');
    console.log('Generates both by default. Uses .venv Python or MODSIM_DOCS_PYTHON.');
    return;
  }

  const localPython = resolve(repoDir, '.venv', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python');
  const python = process.env.MODSIM_DOCS_PYTHON ||
    (existsSync(localPython) ? localPython : process.platform === 'win32' ? 'python' : 'python3');
  const both = !values.api && !values.uml && !values.check;
  const jobs = [];
  if (both || values.api || values.check) jobs.push(['API documentation checks', ['tools/docs/check_api_docs.py']]);
  if (both || values.api) jobs.push(['Python API', ['website/api/build.py']]);
  if (both || values.uml) jobs.push(['UML diagrams', ['tools/docs/generate_diagrams.py']]);

  for (const [label, args] of jobs) {
    console.log(label === 'API documentation checks' ? 'Checking API documentation...' : `Generating ${label}...`);
    const result = spawnSync(python, args, {cwd: repoDir, stdio: 'inherit'});
    if (result.error) {
      throw new Error(`Cannot run ${python}: ${result.error.message}. Install the docs dependencies or set MODSIM_DOCS_PYTHON.`);
    }
    if (result.status !== 0) {
      throw new Error(`${label} failed. See Contribute → API documentation and documentation generation.`);
    }
  }
}

main().catch((error) => {
  console.error(error.message);
  process.exitCode = 1;
});
