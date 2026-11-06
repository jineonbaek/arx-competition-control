"""Small shared contracts and strict JSON at external boundaries."""
import json
import math


class AgentError(Exception):
    def __init__(self, code, message, *, retryable=False):
        super().__init__(message)
        self.code, self.message, self.retryable = code, message, retryable


def strict_json(data):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("Duplicate JSON property")
            result[key] = value
        return result

    def constant(_):
        raise ValueError("Non-finite JSON number")

    return json.loads(data, object_pairs_hook=pairs, parse_constant=constant)


def validate(value, schema, path="arguments"):
    kind = schema.get("type")
    valid = {"object": lambda: isinstance(value, dict),
             "array": lambda: isinstance(value, list),
             "string": lambda: isinstance(value, str),
             "boolean": lambda: type(value) is bool,
             "integer": lambda: type(value) is int,
             "number": lambda: type(value) in (int, float) and math.isfinite(value)}
    if kind not in valid or not valid[kind]():
        raise AgentError("invalid_arguments", f"{path} must be {kind}")
    if "enum" in schema and value not in schema["enum"]:
        raise AgentError("invalid_arguments", f"{path} is not an allowed value")
    if kind == "object":
        props = schema.get("properties", {})
        if set(schema.get("required", [])) - value.keys() or value.keys() - props.keys():
            raise AgentError("invalid_arguments", f"{path} has missing or unknown fields")
        for key, item in value.items():
            validate(item, props[key], f"{path}.{key}")
    if kind in ("array", "string"):
        lower, upper = ("minItems", "maxItems") if kind == "array" else ("minLength", "maxLength")
        if not schema.get(lower, 0) <= len(value) <= schema.get(upper, 100000):
            raise AgentError("invalid_arguments", f"{path} has invalid length")
    if kind == "array":
        for index, item in enumerate(value):
            validate(item, schema["items"], f"{path}[{index}]")
        if schema.get("uniqueItems") and len({json.dumps(v, sort_keys=True) for v in value}) != len(value):
            raise AgentError("invalid_arguments", f"{path} has duplicate entries")
    if kind in ("integer", "number"):
        if value < schema.get("minimum", -math.inf) or value > schema.get("maximum", math.inf):
            raise AgentError("invalid_arguments", f"{path} is outside its bounds")


def object_schema(properties=None, required=None):
    properties = properties or {}
    return {"type": "object", "properties": properties,
            "required": list(properties) if required is None else required,
            "additionalProperties": False}
