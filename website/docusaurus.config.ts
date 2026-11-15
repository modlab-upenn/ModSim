import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import prismTheme from './src/theme/prism';
import apiTableScroll from './src/rehype/apiTableScroll';

const config: Config = {
  title: 'ModSim documentation',
  tagline: 'Modular robotics simulation',
  favicon: 'favicon.svg',
  url: 'https://modlab-upenn.github.io',
  baseUrl: '/ModSim/',
  trailingSlash: true,
  onBrokenLinks: 'throw',
  staticDirectories: ['static', '.generated/site'],
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },
  markdown: {
    format: 'detect',
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: 'throw',
    },
  },
  themes: ['@docusaurus/theme-mermaid'],
  presets: [
    [
      'classic',
      {
        docs: {
          path: '../docs',
          routeBasePath: 'docs',
          sidebarPath: './sidebars.ts',
          rehypePlugins: [apiTableScroll],
          // Add existing guides deliberately as their links and navigation are reviewed.
          include: [
            'index.md',
            'how-to/**/*.md',
            'tutorials/getting-started.md',
            'explanation/**/*.md',
            'explanation/**/*.mdx',
            'reference/python-api.mdx',
            'reference/api/**/*.md',
            'contribute/index.md',
            'contribute/development-setup.md',
            'contribute/api-documentation.md',
            'contribute/local-development.md',
            'contribute/documentation-generation.md',
            'contribute/release-process.md',
          ],
          exclude: ['archive/**', 'technical_report/**'],
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],
  themeConfig: {
    mermaid: {
      theme: {light: 'neutral', dark: 'dark'},
    },
    colorMode: {
      defaultMode: 'light',
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'ModSim Docs',
      logo: {
        alt: 'ModSim modular robot mark',
        src: 'img/modsim-logo-light.svg',
        srcDark: 'img/modsim-logo-dark.svg',
        width: 32,
        height: 32,
      },
      items: [
        {type: 'docSidebar', sidebarId: 'documentation', label: 'Documentation', position: 'left'},
        {type: 'doc', docId: 'contribute/index', label: 'Contribute', position: 'left'},
        {href: 'https://modlab-upenn.github.io/ModSim/', label: 'Project website', position: 'right'},
        {
          href: 'https://github.com/modlab-upenn/ModSim',
          label: 'GitHub',
          position: 'right',
          className: 'modsim-github-link',
        },
      ],
    },
    prism: {
      theme: prismTheme,
      darkTheme: prismTheme,
      additionalLanguages: ['bash', 'python', 'toml', 'yaml'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
