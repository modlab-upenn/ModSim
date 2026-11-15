---
title: Release process
sidebar_label: Release process
---

This is the maintainer path for distributing ModSim as an installable wheel and
source distribution. This branch provides local artifact checks and a release
workflow. The maintainer has confirmed creating a PyPI account, the GitHub
`pypi` environment, and a pending Trusted Publisher. These are user-reported
setup steps; a live publishing run has not independently verified them.
The release commit, integration into `main`, version tag, and first public
publication remain separate steps.

The intended result is one checked set of distribution files attached to a
GitHub Release and published to PyPI. End users can then install the package
without a repository checkout. The bundled Generic Cube is the starter pack;
an independent community library for larger packs remains separate work.

## Configure the publishing accounts

A maintainer needs permission to manage releases in `modlab-upenn/ModSim` and
authorization to publish the intended `modsim-robotics` project on PyPI. The
reported pending publisher uses the matching values below.

When maintaining or replacing that configuration, keep the PyPI publisher and
GitHub environment aligned:

| Publisher field | Value |
| --- | --- |
| GitHub owner | `modlab-upenn` |
| Repository | `ModSim` |
| Workflow filename | `release.yml` |
| Environment | `pypi` |

For a first project, PyPI supports a pending Trusted Publisher; for an existing
project, its owner adds the publisher in the project's publishing settings.
Trusted Publishing uses GitHub's OpenID Connect identity for publication rather
than a stored PyPI API token. The publish job needs `id-token: write` permission.
[PyPI publisher configuration](https://docs.pypi.org/trusted-publishers/adding-a-publisher/),
[first-project publisher setup](https://docs.pypi.org/trusted-publishers/creating-a-project-through-oidc/).

A pending publisher does not reserve the project name. Its settings must match
the repository, workflow, and environment used by the first publishing run.
Account and publisher settings are maintained outside the local documentation
build.

## Prepare a version

1. Finish and review the code and documentation for the release.
2. Set the version in `src/modsim_robotics/__init__.py`; Hatch reads it when building.
3. Check package metadata and the distribution contents. Include the bundled
   Generic Cube, and keep restricted or separately licensed example assets out
   of public artifacts.
4. Run the relevant core, MuJoCo, and Studio checks and the package checks.
5. Write release notes explaining changes, supported workflows, and limitations.
   The initial notes are in
   [`docs/releases/0.1.0.md`](https://github.com/modlab-upenn/ModSim/blob/main/docs/releases/0.1.0.md).

Keep the authored release body in `docs/releases/X.Y.Z.md` for the matching
package version. The release validation checks that this file exists and is
nonempty before any PyPI publication. Write it so it can be attached verbatim
to the GitHub Release; record temporary preparation and publication status in
this maintainer guide and the setup tracker.

The MuJoCo extra currently requires `mujoco>=3.3.3,<4`. Check the documented
minimum as part of compatibility verification when changing that bound.

Use a tag of the form `vX.Y.Z`, matching the package's `X.Y.Z` version. For
example, `v0.1.0` must identify code whose package version is `0.1.0`. A tag is a
reference to a specific commit; the release workflow checks that relationship.

## Build and promote the artifacts

The `release.yml` workflow is intended to run for version tags matching `v*`:

1. Check out the tagged commit, check the tag against its package version, and
   require that the commit belongs to the history of `main`. Validate the
   matching, nonempty authored release-note file before publication.
2. Run the required checks for that source state.
3. Build the wheel and source distribution.
4. Inspect the distributions and smoke-test installation from the built files.
5. Record SHA-256 checksums and retain that exact checked artifact set.
6. Verify the checksums and publish those files to PyPI, then attach the same
   distributions and checksum file to a GitHub Release with the authored notes.

The publication jobs should use the files from the checked build rather than
build them again. GitHub Release assets provide direct downloads, while PyPI
supports installation by the package name. GitHub's automatically generated
source archives are separate from the built wheel and source distribution.
[GitHub releases](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases),
[Python packaging CI workflow](https://packaging.python.org/en/latest/guides/publishing-package-distribution-releases-using-github-actions-ci-cd-workflows/).

The GitHub Release job passes the validated authored body with `--notes-file`
instead of substituting automatically generated notes. The early file check
prevents a missing release body from being discovered after PyPI publication.

Pushing a release tag is the publication trigger. Keep that action separate from
local artifact verification and ordinary documentation development.

Commit the intended release changes and integrate that commit into `main`
before creating the matching tag. The source-history check rejects a tag whose
commit is not reachable from `main`. Review the final build and package metadata
after cleanup; checks of an earlier working tree do not cover later changes.

The CI package job also runs on pull requests and pushes to `main`; it retains
checked files as Actions artifacts without publishing a release. Local
[development setup](development-setup.md#build-an-installable-distribution)
explains how to run the same installation checks before that point. The full
repository test suite uses example fixtures from a source checkout and is not
included in the source distribution.

## Retry a failed publication

Prefer retrying failed downstream jobs in the same workflow run: they download
the retained checked artifact set and verify its `SHA256SUMS`. A full workflow
rerun runs the build again, so it must not be assumed to produce the same bytes,
even when the source commit is unchanged.

If PyPI accepted some files before a failure, inspect those files and compare
their checksums with the retained artifact set before retrying. Do not use
`skip-existing` to conceal a mismatch. Recover the original checked files or
prepare a new version when uploaded files and a rebuilt artifact differ. A
retry must preserve the relationship between tested files, PyPI files, and
GitHub Release assets.

## Verify the published release

After publication, check the files attached to the GitHub Release and the
version shown on PyPI. In a fresh environment, install that published version,
run `modsim-robotics --version`, prepare the starter with `modsim-robotics setup`, and validate the
Generic Cube using the [installation guide](../how-to/installation.md).

Compare the GitHub Release distributions with the checked artifact hashes and
verify that the installed package metadata identifies the intended version and
documentation source. Until the Docusaurus site is deployed, package
documentation links point to the repository's `docs` directory on GitHub.

Record the tested Python versions and package version in the release notes.
Update installation examples when the supported version or starter data changes.
The first tag-triggered publication and live installation checks remain pending.
