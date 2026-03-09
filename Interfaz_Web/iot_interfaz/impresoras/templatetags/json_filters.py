from django import template
import json

register = template.Library()

@register.filter
def parse_json(value):
    try:
        return json.loads(value)
    except (ValueError, TypeError):
        return {}