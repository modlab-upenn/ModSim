# Website recordings

These six recordings were copied verbatim from the existing ModSim website's
`media/` directory on `origin/gh-pages`. Their filenames preserve the public
`/ModSim/media/…` URLs when the combined website and manual are published.

[`asset-manifest.json`](asset-manifest.json) records the immutable source commit,
Git blob, byte size, and SHA-256 hash of each copy. The recordings are existing
project assets; this migration does not add robot meshes or RobotPack archives
to the website.

| Recording | Content | Playback |
| --- | --- | --- |
| `studio_walkthrough_2x.mp4` | Studio authoring walkthrough | 2× |
| `inspector_lattice.mp4` | Runtime Inspector cubic-lattice view | As recorded |
| `inspector_smores_planning.mp4` | Runtime Inspector SMORES planning view | As recorded |
| `inspector_mblocks_planning.mp4` | Runtime Inspector M-Blocks planning view | As recorded |
| `smores_online_assembly_6x.mp4` | SMORES-EP online self-assembly | 6× |
| `mblocks_planar_reconf_3x.mp4` | M-Blocks planar reconfiguration | 3× |

The recordings show the interface and experiment state captured for the former
website. In particular, the Studio authoring recording predates the integrated
Robot Pack/Runtime workspace workflow. Current instructions belong in the
manual; captions should identify historical UI differences where relevant.

The former homepage's side-by-side docking teaser was an animated placeholder,
not another recording. It is not included in this asset set.

Keep media references relative to the configured site base URL so they also
work in a preview repository. Use descriptive video labels and visible playback
controls. The supplied `3x` and `6x` recordings represent accelerated playback,
not performance benchmarks.
