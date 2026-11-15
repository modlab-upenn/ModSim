{% if obj.display and is_own_page %}
{{ obj.name.removeprefix('modsim_robotics.') }}
{{ '=' * obj.name.removeprefix('modsim_robotics.')|length }}

.. py:module:: {{ obj.name }}

Module: ``{{ obj.name }}``

{% if obj.docstring %}
.. autoapi-nested-parse::

   {{ obj.docstring|indent(3) }}

{% endif %}
{% set visible_children = obj.children|selectattr('display')|list %}
{% for child in visible_children|rejectattr('type', 'equalto', 'exception') %}
{{ child.render() }}
{% endfor %}
{% for child in visible_children|selectattr('type', 'equalto', 'exception') %}
{{ child.render() }}
{% endfor %}
{% endif %}
