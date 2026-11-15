import type {PrismTheme} from 'prism-react-renderer';

// CSS variables keep code colors in the same shared palette as the website.
const prismTheme: PrismTheme = {
  plain: {
    color: 'var(--modsim-code-ink)',
    backgroundColor: 'var(--modsim-code-bg)',
  },
  styles: [
    {types: ['comment', 'prolog', 'doctype', 'cdata'], style: {color: 'var(--modsim-code-comment)'}},
    {types: ['string', 'attr-value', 'inserted'], style: {color: 'var(--modsim-code-string)'}},
    {types: ['keyword', 'selector', 'important'], style: {color: 'var(--modsim-code-keyword)'}},
    {types: ['boolean', 'number', 'constant'], style: {color: 'var(--modsim-code-number)'}},
    {types: ['function', 'class-name', 'tag'], style: {color: 'var(--modsim-code-function)'}},
  ],
};

export default prismTheme;
