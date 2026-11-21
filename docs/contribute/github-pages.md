---
title: Website preview and GitHub Pages
sidebar_label: Website and GitHub Pages
---

The homepage and manual are one Docusaurus application. A single build includes
the project page, browser demonstrations, documentation, generated Python API,
UML, and local media. GitHub Pages serves that static output. Python package
publication uses a separate workflow.

The workflow in `.github/workflows/pages.yml` builds branches and pull requests,
and provides a downloadable preview. **Publishing is off by default.** A local
build or a build-only workflow run leaves the existing public website unchanged.

## Review the combined website locally

Follow [Local development](local-development.md) and
[Documentation generation](documentation-generation.md) to install Node, the
locked Python docs group, and Graphviz. From the repository root:

```bash
npm --prefix website run docs:dev
```

Open `http://localhost:3000/ModSim/` for the homepage and
`http://localhost:3000/ModSim/docs/` for the manual. These pages share the same
navigation, theme switch, fonts, and branding. Saving a homepage component or
authored documentation page updates the live preview.

Before reviewing a deployable build, run:

```bash
node --test website/scripts/tests/*.test.mjs
npm --prefix website run docs:check
npm --prefix website run typecheck
npm --prefix website run docs:build
npm --prefix website run docs:serve -- --port 3197
```

Open `http://localhost:3197/ModSim/`. The production preview serves the exact
files in `website/build/`; rebuild after editing. Check the homepage, widgets,
videos, docs/API links, theme switching, and a direct refresh on a nested guide.
Use **Ctrl+C** to stop either local server.

## What the Pages workflow does

Relevant branch pushes and pull requests run **Website and Pages**. The job
installs the locked tools, tests publication settings, checks API documentation
and TypeScript, regenerates API/UML, and builds Docusaurus. It uploads the result
as `site-preview-RUN_ID-RUN_ATTEMPT`, retained for 14 days. Download it from the
workflow run's **Artifacts** section when reviewing a branch build.

An artifact is a download rather than a hosted preview URL. To serve an
extracted artifact, place its files under a directory matching the build's base
path, then serve the parent directory. For example, place a `/ModSim/` build in
`/tmp/modsim-preview/ModSim/`, then run:

```bash
python -m http.server 8000 --bind 127.0.0.1 --directory /tmp/modsim-preview
```

Visit `http://localhost:8000/ModSim/`. A preview-repository build instead uses
that repository's base path. The workflow summary records the URL used to build
the artifact.

Only a publication request on the configured deployment branch enables the
second job. That job uses GitHub's official Pages actions and the `github-pages`
environment to publish the already-built artifact. Pull requests and tags cannot
publish through this workflow. The build job has read-only repository access;
Pages write and identity-token permissions are limited to the deployment job.
See [GitHub's custom-workflow documentation](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages).

## Test a branch on GitHub Pages before merging main

GitHub provides one project Pages site per repository. Publishing `refactor`
from the original ModSim repository would use its existing public site address.
For an independent branch preview, use a separate public repository, such as
`YOUR-USERNAME/ModSim-preview`, or a fork with Pages enabled. This gives the
preview its own address while the original website keeps running.
See [GitHub Pages site types](https://docs.github.com/en/pages/getting-started-with-github-pages/what-is-github-pages#types-of-github-pages-sites).

1. Finish the local review and commit the website changes on `refactor`.
2. Create an empty public `ModSim-preview` repository on GitHub. Add it as another
   remote and push the source branch:

   ```bash
   git remote add pages-preview https://github.com/YOUR-USERNAME/ModSim-preview.git
   git push pages-preview refactor
   ```

   This pushes to the preview repository; it does not merge or publish the
   original repository's `main` branch.
3. In the **preview repository**, enable Actions if necessary and confirm that
   `refactor` is its default branch under **Settings → General → Default branch**.
   The first branch pushed to an empty repository will normally already be the
   default. For a fork or an existing repository with multiple branches, change
   the default to `refactor` if needed. GitHub only exposes manual workflow
   dispatch when the workflow file exists on the default branch; the workflow's
   push builds can run before that change. See
   [changing the default branch](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-branches-in-your-repository/changing-the-default-branch).
4. Open **Settings → Pages → Build and deployment** and select **GitHub Actions**
   as the source. Do this in the preview repository first.
5. Under **Settings → Secrets and variables → Actions → Variables**, add
   `PAGES_DEPLOY_BRANCH` with the value `refactor`. Leave `PAGES_AUTO_DEPLOY`
   unset, or set it to `false`, for explicit publication only.
6. Under **Settings → Environments → github-pages**, allow deployments from
   `refactor`. If that environment does not exist yet, create it with that name.
   Preserve any required-reviewer policy the organization uses.
7. Open **Actions → Website and Pages → Run workflow**, select `refactor`, and
   check **Publish this build**. A manual run with that box clear only builds
   and uploads the artifact.
8. Wait for both jobs to pass and follow the URL shown by the deployment. With
   the example repository, the homepage is
   `https://YOUR-USERNAME.github.io/ModSim-preview/` and the manual is under
   `/ModSim-preview/docs/`.

You can repeat the manual run after another preview push. To publish preview
pushes automatically, set `PAGES_AUTO_DEPLOY=true` in the preview repository;
only pushes to `PAGES_DEPLOY_BRANCH` can then deploy. That setting does not
affect the original repository.

Manual workflow controls and branch selection are described in
[GitHub's workflow-dispatch guide](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow).

### Site and branch variables

Configure these as repository **Variables**, not secrets:

| Variable | Default | Purpose |
| --- | --- | --- |
| `PAGES_DEPLOY_BRANCH` | `main` | The only branch allowed to publish. Use `refactor` in the preview repository. |
| `PAGES_AUTO_DEPLOY` | `false` | Set `true` to publish relevant pushes to the deployment branch automatically. |
| `PAGES_SITE_URL` | `https://OWNER.github.io` | Optional HTTPS origin for a custom domain. Omit a path. |
| `PAGES_BASE_URL` | `/REPOSITORY/` | Optional base path, beginning and ending with `/`. Account-site repositories default to `/`. |

The workflow derives its normal URL from the repository where it runs. A fork
or preview repository therefore does not need to reuse `/ModSim/`. It exports
`MODSIM_SITE_URL` and `MODSIM_BASE_URL` to the Docusaurus build. All website links,
media, font files, and generated documentation must respect that base path.

For a custom domain, configure and verify the domain in GitHub Pages settings,
then set the matching `PAGES_SITE_URL`. Set `PAGES_BASE_URL=/` when the site is
served at the domain root.

You can reproduce a preview repository's addressing locally on macOS/Linux:

```bash
export MODSIM_SITE_URL=https://YOUR-USERNAME.github.io
export MODSIM_BASE_URL=/ModSim-preview/
npm --prefix website run docs:build
npm --prefix website run docs:serve -- --port 3197
```

Open `http://localhost:3197/ModSim-preview/`. Keep the variables set while serving
that build. In PowerShell, set them with `$env:MODSIM_SITE_URL = "https://YOUR-USERNAME.github.io"`
and `$env:MODSIM_BASE_URL = "/ModSim-preview/"`. Clear the overrides before
building the normal `/ModSim/` preview again.

## Publish the reviewed site from main

After the local and branch preview reviews:

1. Record the original repository's current Pages source and deployment commit.
   Keep its existing `gh-pages` history available for the first-cutover rollback.
2. Merge the reviewed source changes into `main` through the usual pull request.
3. In the original repository, select **Settings → Pages → GitHub Actions**.
4. Set `PAGES_DEPLOY_BRANCH=main`. Leave `PAGES_AUTO_DEPLOY=false` for the first
   controlled publication. Allow `main` in the `github-pages` environment.
5. Run **Website and Pages** from `main` with **Publish this build** checked.
6. Verify the homepage, docs, installation guide, API, media, `quickstart.html`
   compatibility route, and a nested-page refresh at the public URL.
7. Enable `PAGES_AUTO_DEPLOY=true` when future reviewed `main` pushes should
   publish automatically.

For the ModSim project repository, defaults produce
`https://modlab-upenn.github.io/ModSim/`. The source remains in the normal code
branch; the workflow uploads static files directly to Pages. It does not push
generated output back into `gh-pages` or publish anything to PyPI.

### Roll back or diagnose a failed deployment

A failed build does not enter the deployment job. If a deployed change needs
reverting, revert its source change, build it again, and publish that reviewed
result. During the first migration, another rollback path is to restore the
recorded Pages source; if that source was `gh-pages` at the repository root,
select **Deploy from a branch → gh-pages → /(root)** and keep automatic Actions
publication disabled while investigating.

If a run only uploads an artifact, check the selected branch, **Publish this
build**, and `PAGES_AUTO_DEPLOY`. A requested manual publication from another
branch fails with the configured branch name. If deployment is refused, check
the Pages source, allowed Actions policy, and `github-pages` environment branch
rules. If HTML loads but assets or nested routes fail, compare the deployed
address with the build's origin/base path and reproduce that same path locally.

The local and build checks can be completed before a live deployment. Actual
GitHub Pages configuration, a hosted branch preview, and the final production
cutover each require performing the corresponding repository steps above.
