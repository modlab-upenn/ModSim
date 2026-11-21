import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import prismTheme from './src/theme/prism';
import apiTableScroll from './src/rehype/apiTableScroll';
import {robotPackLibraryUrl, siteLocation} from './site-config';

const location = siteLocation(process.env);

const config: Config = {
  title: 'ModSim',
  tagline: 'Modular robotics simulation',
  favicon: 'favicon.svg',
  url: location.url,
  baseUrl: location.baseUrl,
  trailingSlash: true,
  onBrokenLinks: 'throw',
  onBrokenAnchors: 'throw',
  onDuplicateRoutes: 'throw',
  clientModules: [require.resolve('./src/clientModules/canonicalPaths.ts')],
  staticDirectories: ['static', '.generated/site'],
  customFields: {
    robotPackLibraryUrl: robotPackLibraryUrl(process.env),
  },
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
            'contribute/branding.md',
            'contribute/local-development.md',
            'contribute/documentation-generation.md',
            'contribute/release-process.md',
            'contribute/github-pages.md',
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
      logo: {
        alt: 'ModSim outlined wordmark',
        src: 'img/modsim-logo-light.svg',
        srcDark: 'img/modsim-logo-dark.svg',
        width: 160,
        height: 32,
      },
      items: [
        {to: '/', label: 'Home', position: 'left', activeBaseRegex: `^${location.baseUrl.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}$`},
        {type: 'docSidebar', sidebarId: 'documentation', label: 'Documentation', position: 'left'},
        {type: 'doc', docId: 'contribute/index', label: 'Contribute', position: 'left'},
        {
          href: 'https://github.com/modlab-upenn/ModSim',
          label: 'GitHub',
          position: 'right',
          className: 'modsim-github-link',
        },
      ],
    },
    footer: {
      style: 'light',
      links: [
        {title: 'Start building', items: [
          {label: 'Installation', to: '/docs/installation/'},
          {label: 'Getting started', to: '/docs/tutorials/getting-started/'},
          {label: 'Create a RobotPack', to: '/docs/how-to/'},
        ]},
        {title: 'Understand ModSim', items: [
          {label: 'Architecture and UML', to: '/docs/explanation/'},
          {label: 'Python API', to: '/docs/reference/python-api/'},
          {label: 'Contribute', to: '/docs/contribute/'},
        ]},
        {title: 'Project', items: [
          {label: 'Source on GitHub', href: 'https://github.com/modlab-upenn/ModSim'},
          {label: 'RobotPack Library', to: '/#robotpack-library'},
          {label: 'Experiments', to: '/#experiments'},
          {label: 'Citation', to: '/#citation'},
        ]},
      ],
      copyright: 'ModSim · ModLab, University of Pennsylvania · MIT License',
    },
    prism: {
      theme: prismTheme,
      darkTheme: prismTheme,
      additionalLanguages: ['bash', 'python', 'toml', 'yaml'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
