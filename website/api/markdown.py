"""Readable API entries built from resolved Sphinx nodes and AutoAPI metadata."""

from __future__ import annotations

from docutils import nodes
from sphinx import addnodes
from sphinx_markdown_builder.escape import escape_html_quote
from sphinx_markdown_builder.translator import MarkdownTranslator


def _shorten_type_names(node):
    """Shorten qualified identifier leaves while retaining resolved references."""
    result = node.deepcopy()
    for text in list(result.findall(nodes.Text)):
        value = text.astext()
        parts = value.split(".")
        if len(parts) > 1 and all(part.isidentifier() for part in parts):
            text.parent.replace(text, nodes.Text(parts[-1]))
    return result


def _inline(children):
    """Copy a semantic fragment into an independent inline node."""
    return nodes.inline("", "", *(child.deepcopy() for child in children))


def _table_cell(value):
    """Keep Markdown structure inside one table cell."""
    return value.strip().replace("|", r"\|").replace("\n\n", "<br/><br/>").replace("\n", " ")


class ApiMarkdownTranslator(MarkdownTranslator):
    """Render API identities separately from declarations and explanatory prose."""

    def _fragment(self, node, *, short_types=False):
        fragment = _shorten_type_names(node) if short_types else node.deepcopy()
        writer = MarkdownTranslator(self.document, self.builder)
        fragment.walkabout(writer)
        return writer.astext().strip()

    def _text(self, node):
        return _shorten_type_names(node).astext().strip()

    def _write(self, value):
        if value:
            self.add(value, prefix_eol=2, suffix_eol=2)

    def _table(self, headings, rows):
        lines = [
            "| " + " | ".join(headings) + " |",
            "| " + " | ".join("---" for _ in headings) + " |",
        ]
        lines.extend("| " + " | ".join(_table_cell(cell) for cell in row) + " |" for row in rows)
        self._write("\n".join(lines))

    def _object(self, signature):
        objects = self.builder.env.autoapi_all_objects
        return next((objects[key] for key in signature.get("ids", []) if key in objects), None)

    def _type_node(self, signature):
        returns = next(iter(signature.findall(addnodes.desc_returns)), None)
        if returns is not None:
            return _inline(returns.children)
        # Attribute and property types follow a ':' annotation, rather than ->.
        for annotation in signature.findall(addnodes.desc_annotation):
            parts = list(annotation.children)
            if parts and parts[0].astext() == ":":
                return _inline(parts[1:])
        return nodes.inline()

    def _parameters(self, signature, obj):
        parameter_list = next(iter(signature.findall(addnodes.desc_parameterlist)), None)
        if parameter_list is None or obj is None:
            return []
        source = obj.constructor if obj.type in ("class", "exception") else obj
        if source is None:
            return []
        arguments = list(source.obj.get("args", []))
        if obj.type in ("class", "exception"):
            arguments = [arg for arg in arguments if arg.name not in ("self", "cls")]
        parameters = [
            child for child in parameter_list if isinstance(child, addnodes.desc_parameter)
        ]
        if len(arguments) != len(parameters):
            raise ValueError(f"AutoAPI/Sphinx parameter mismatch for {obj.id}")
        result = []
        for arg, parameter in zip(arguments, parameters, strict=True):
            annotation = nodes.inline()
            parts = list(parameter.children)
            for index, part in enumerate(parts):
                if isinstance(part, addnodes.desc_sig_punctuation) and part.astext() == ":":
                    tail = []
                    for child in parts[index + 1 :]:
                        if isinstance(child, addnodes.desc_sig_operator) and child.astext() == "=":
                            break
                        tail.append(child)
                    annotation = _inline(tail)
                    break
            result.append({"arg": arg, "type": annotation})
        return result

    def _signature(self, signature, obj, kind, parameters):
        name = next(iter(signature.findall(addnodes.desc_name))).astext()
        result_type = self._type_node(signature)
        if kind in ("class", "exception") and not parameters:
            return f"class {name}:", result_type
        if kind in ("attribute", "data", "property"):
            declaration = name
            if result_type.astext().strip():
                declaration += f": {self._text(result_type)}"
            if obj is not None and obj.obj.get("value") is not None:
                declaration += f" = {obj.obj['value']}"
            return declaration, result_type
        args = []
        for item in parameters:
            arg = item["arg"]
            text = (arg.prefix or "") + (arg.name or "")
            if item["type"].astext().strip():
                text += f": {self._text(item['type'])}"
            if arg.default_value is not None:
                text += f" = {arg.default_value}"
            args.append(text)
        class_entry = kind in ("class", "exception")
        declaration_name = obj.constructor.name if class_entry else name
        if class_entry:
            args.insert(0, "cls" if declaration_name == "__new__" else "self")
        prefix = "def "
        suffix = f" -> {self._text(result_type)}" if result_type.astext().strip() else ""
        one_line = f"{prefix}{declaration_name}({', '.join(args)}){suffix}:"
        if len(one_line) > 88 or len(args) > 3:
            declaration = (
                f"{prefix}{declaration_name}(\n"
                + "\n".join(f"    {arg}," for arg in args)
                + f"\n){suffix}:"
            )
        else:
            declaration = one_line
        if class_entry:
            declaration = f"class {name}:\n" + "\n".join(
                "    " + line for line in declaration.splitlines()
            )
        if obj is not None:
            decorators = [
                item
                for item in obj.obj.get("properties", [])
                if item in ("classmethod", "staticmethod", "abstractmethod")
            ]
            declaration = "".join(f"@{item}\n" for item in decorators) + declaration
        return declaration, result_type

    def _field_entries(self, field):
        """Split Sphinx docfields at their semantic name/description separator."""
        body = field.children[1]
        items = (
            list(body.children[0])
            if body.children and isinstance(body.children[0], nodes.bullet_list)
            else [body]
        )
        result = []
        for item in items:
            paragraph = next(iter(item.findall(nodes.paragraph)), None)
            if paragraph is None:
                continue
            children = list(paragraph.children)
            separator = next(
                (
                    index
                    for index, child in enumerate(children)
                    if isinstance(child, nodes.Text) and child.astext() == " \u2013 "
                ),
                None,
            )
            if separator is None:
                result.append((nodes.inline(), item.deepcopy()))
                continue
            label = _inline(children[:separator])
            description = nodes.container()
            description += nodes.paragraph(
                "", "", *(child.deepcopy() for child in children[separator + 1 :])
            )
            siblings = list(item.children)
            if paragraph in siblings:
                description.extend(
                    child.deepcopy() for child in siblings[siblings.index(paragraph) + 1 :]
                )
            result.append((label, description))
        return result

    def _fields(self, content):
        return [
            field
            for child in content.children
            if isinstance(child, nodes.field_list)
            for field in child
        ]

    def _parameter_docs(self, fields):
        descriptions = {}
        for field in fields:
            if field.children[0].astext().lower() in (
                "parameters",
                "parameter",
                "keyword arguments",
            ):
                for label, description in self._field_entries(field):
                    name = next(iter(label.findall(addnodes.literal_strong)), None)
                    if name is None:
                        name = next(iter(label.findall(nodes.strong)), None)
                    if name is not None:
                        descriptions[name.astext().lstrip("*")] = self._fragment(description)
        return descriptions

    def _parameter_table(self, parameters, fields):
        rows = []
        descriptions = self._parameter_docs(fields)
        for item in parameters:
            arg = item["arg"]
            if arg.name is None:
                continue
            default = (
                f"`{arg.default_value}`"
                if arg.default_value is not None
                else "Variadic"
                if arg.prefix in ("*", "**")
                else "Required"
            )
            rows.append(
                (
                    f"`{arg.prefix or ''}{arg.name}`",
                    self._fragment(item["type"], short_types=True) or "—",
                    default,
                    descriptions.get(arg.name, ""),
                )
            )
        if rows:
            self._write("**Parameters**")
            self._table(("Name", "Type", "Default", "Description"), rows)

    def _return_fields(self, fields, result_type, *, property_type=False):
        descriptions = [
            self._fragment(field.children[1])
            for field in fields
            if field.children[0].astext().lower() in ("returns", "return")
        ]
        type_fields = [
            self._fragment(field.children[1], short_types=True)
            for field in fields
            if field.children[0].astext().lower() == "return type"
        ]
        annotation = (
            self._fragment(result_type, short_types=True) if result_type.astext().strip() else ""
        )
        if annotation or descriptions or type_fields:
            self._write("**Type**" if property_type else "**Returns**")
            self._table(
                ("Type", "Description"),
                [(annotation or " ".join(type_fields) or "—", "\n\n".join(descriptions))],
            )

    def _other_fields(self, fields):
        handled = {
            "parameters",
            "parameter",
            "keyword arguments",
            "returns",
            "return",
            "return type",
        }
        for field in fields:
            label = field.children[0].astext()
            if label.lower() in handled:
                continue
            self._write(f"**{label}**")
            if label.lower() in ("raises", "raise"):
                rows = [
                    (self._fragment(name, short_types=True), self._fragment(description))
                    for name, description in self._field_entries(field)
                ]
                self._table(("Exception", "When raised"), rows)
            else:
                field.children[1].walkabout(self)

    def _attributes(self, entries):
        self._write("**Attributes**")
        rows = []
        for entry in entries:
            signature = next(child for child in entry if isinstance(child, addnodes.desc_signature))
            obj = self._object(signature)
            name = next(iter(signature.findall(addnodes.desc_name))).astext()
            anchors = "".join(
                f'<a id="{escape_html_quote(anchor)}"></a>' for anchor in signature.get("ids", [])
            )
            annotation = self._fragment(self._type_node(signature), short_types=True) or "—"
            content = next(child for child in entry if isinstance(child, addnodes.desc_content))
            value = obj.obj.get("value") if obj is not None else None
            rows.append(
                (
                    f"{anchors}`{name}`",
                    annotation,
                    f"`{value}`" if value is not None else "—",
                    self._fragment(content),
                )
            )
        if all(row[2] == "—" for row in rows):
            self._table(("Name", "Type", "Description"), [(row[0], row[1], row[3]) for row in rows])
        else:
            self._table(("Name", "Type", "Value", "Description"), rows)

    def visit_desc(self, node):
        """Write one short entry, preserving all canonical object identities."""
        if node.get("domain") != "py":
            return super().visit_desc(node)
        signatures = [child for child in node if isinstance(child, addnodes.desc_signature)]
        content = next(child for child in node if isinstance(child, addnodes.desc_content))
        kind = node.get("objtype", "")
        for signature in signatures:
            obj = self._object(signature)
            parameters = self._parameters(signature, obj)
            declaration, result_type = self._signature(signature, obj, kind, parameters)
            for anchor in signature.get("ids", []):
                self._add_anchor(anchor)
            name = next(iter(signature.findall(addnodes.desc_name))).astext()
            callable_entry = kind in ("method", "function")
            level = 2 if kind in ("class", "exception", "data") else 3
            self._write(f"{'#' * level} `{name}{'()' if callable_entry else ''}`")
            self._write(f'```python title="Python"\n{declaration}\n```')
        fields = self._fields(content)
        attributes = [
            child
            for child in content
            if isinstance(child, addnodes.desc) and child.get("objtype") == "attribute"
        ]
        attributes_written = False
        sections_written = False
        for child in content:
            if child in attributes:
                if not attributes_written:
                    self._attributes(attributes)
                    attributes_written = True
                continue
            if isinstance(child, (nodes.field_list, nodes.rubric)) and not sections_written:
                self._parameter_table(parameters, fields)
                if callable_entry or kind == "property":
                    self._return_fields(fields, result_type, property_type=kind == "property")
                self._other_fields(fields)
                sections_written = True
            if not isinstance(child, nodes.field_list):
                child.walkabout(self)
        if not sections_written:
            self._parameter_table(parameters, fields)
            if callable_entry or kind == "property":
                self._return_fields(fields, result_type, property_type=kind == "property")
            self._other_fields(fields)
        raise nodes.SkipNode

    def visit_rubric(self, node):
        self._write(f"**{node.astext()}**")
        raise nodes.SkipNode
