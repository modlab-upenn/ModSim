"""Static Python API extraction; never import ModSim, Studio, or a simulator."""

from __future__ import annotations

import ast
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "src" / "modsim_robotics"

# Only this local extension directory is imported; application packages stay out
# of sys.path and AutoAPI reads their source statically.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from scope import API_MODULES, PARSED_MODULES  # noqa: E402

DOCUMENTED_NAMES = {
    "modsim_robotics." + path.removesuffix(".py").replace("/", ".") for path in API_MODULES
}

project = "ModSim Python API"
author = "ModLab"
# Extract the version assignment as syntax, without importing the package.
_version_tree = ast.parse((PACKAGE / "__init__.py").read_text(encoding="utf-8"))
release = next(
    ast.literal_eval(node.value)
    for node in _version_tree.body
    if isinstance(node, ast.Assign)
    and any(isinstance(target, ast.Name) and target.id == "__version__" for target in node.targets)
)
version = release
needs_sphinx = "8.2"
extensions = ["sphinx.ext.napoleon", "autoapi.extension", "sphinx_markdown_builder"]
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_use_param = True
napoleon_use_rtype = True
source_suffix = {".rst": "restructuredtext"}
root_doc = "index"
# AutoAPI must keep parsed packages visible to generate their published child
# modules. Their package overview pages are not part of the public scope, so
# exclude only those exact index files from Sphinx's source discovery.
exclude_patterns = [
    "autoapi/modsim_robotics/" + path.removesuffix("/__init__.py") + "/index.rst"
    for path in sorted(PARSED_MODULES - API_MODULES)
    if path.endswith("/__init__.py")
]

# AutoAPI parses original Python source with Astroid rather than importing it.
# AutoAPI's generated rst is relative to the staged source tree, which build.py
# places under website/.generated/api/source rather than this authored directory.
autoapi_dirs = [str(PACKAGE)]
autoapi_ignore = [
    str(path)
    for path in sorted(PACKAGE.rglob("*.py"))
    if path.relative_to(PACKAGE).as_posix() not in PARSED_MODULES
]
autoapi_root = "autoapi"
autoapi_keep_files = True
autoapi_add_toctree_entry = False
autoapi_member_order = "bysource"
autoapi_options = ["members", "undoc-members", "show-inheritance", "show-module-summary"]
autoapi_python_class_content = "class"
autoapi_template_dir = str(Path(__file__).resolve().parent / "templates")

# Native Markdown is rendered by the existing Docusaurus content-docs plugin.
# Exact object IDs preserve Python-domain links, while Docusaurus owns headings.
markdown_anchor_sections = False
markdown_anchor_signatures = True
markdown_docinfo = False
markdown_uri_doc_suffix = ".md"
markdown_file_suffix = ".md"
markdown_flavor = "github"


def visit_abbreviation(translator, node):
    """Preserve Sphinx 9's keyword-only '*' marker in Python signatures.

    sphinx-markdown-builder 0.6.11 has no visitor for docutils.abbreviation,
    which Sphinx uses for this marker. Walk its text normally so the builder's
    escaping preserves literal '*' rather than dropping it with a warning.
    """


def depart_abbreviation(translator, node):
    """The abbreviation's text has already been rendered by the builder."""


def skip_helper_modules(app, what, name, obj, skip, options):
    """Resolve helper imports without adding their pages to this first slice."""
    if what == "module" and name not in DOCUMENTED_NAMES:
        return True
    return None


def setup(app):
    """Keep the source-reading boundary separate from the rendered reference."""
    from docutils import nodes
    from markdown import ApiMarkdownTranslator

    app.set_translator("markdown", ApiMarkdownTranslator, override=True)
    app.connect("autoapi-skip-member", skip_helper_modules)
    app.add_node(
        nodes.abbreviation,
        override=True,
        markdown=(visit_abbreviation, depart_abbreviation),
    )
