{{ name | escape}}
{{ underline }}

.. currentmodule:: {{ module }}

.. autoclass:: {{ objname }}

    {% block attributes %}
    {% if attributes %}
    .. rubric:: Attributes

    .. autosummary::
    {% for item in attributes %}
        ~{{ name }}.{{ item }}
    {%- endfor %}
    {% endif %}
    {% endblock %}
    
    {% block methods %}
    {% if methods %}
    .. rubric:: Methods

    .. autosummary::
    {% for item in methods %}
        ~{{ name }}.{{ item }}
    {%- endfor %}

    .. rubric:: Details

    {% for item in methods %}
    .. automethod:: {{ item }}
    {%- endfor %}

    {% endif %}
    {% endblock %}

    