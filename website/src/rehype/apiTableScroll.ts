import type {Element, Root} from 'hast';
import type {Plugin} from 'unified';

function wrapTables(parent: Root | Element): void {
  parent.children = parent.children.map((child) => {
    if (child.type !== 'element') return child;
    if (child.tagName === 'table') {
      return {
        type: 'element' as const,
        tagName: 'div',
        properties: {className: ['api-table-scroll']},
        children: [child],
      };
    }
    wrapTables(child);
    return child;
  });
}

const apiTableScroll: Plugin<[], Root> = () => (tree, file) => {
  if (!file.path.replaceAll('\\', '/').includes('/docs/reference/api/')) return;
  wrapTables(tree);
};

export default apiTableScroll;
