"""Bounded Qwen-compatible HTTP transport; never log credentials or partial answers."""
import json
import time
from urllib.error import HTTPError
from urllib.request import Request

from .contracts import AgentError, strict_json
from .http import open_request


def assistant_message(message, finish):
    if finish not in ('stop', 'tool_calls') or not isinstance(message, dict):
        raise ValueError('Model reply unfinished or filtered')
    result = {'role': 'assistant', 'content': message.get('content') or ''}
    if not isinstance(result['content'], str):
        raise ValueError('Invalid assistant content')
    if message.get('reasoning_content') is not None:
        if not isinstance(message['reasoning_content'], str):
            raise ValueError('Invalid reasoning protocol field')
        result['reasoning_content'] = message['reasoning_content']
    calls = message.get('tool_calls') or []
    if not isinstance(calls, list) or len(calls) > 32:
        raise ValueError('Invalid tool call list')
    seen = set()
    for call in calls:
        function = call['function']
        if call.get('type') != 'function' or not isinstance(call.get('id'), str) or not call['id'] or call['id'] in seen or not isinstance(function.get('name'), str) or not isinstance(function.get('arguments'), str):
            raise ValueError('Invalid tool call')
        seen.add(call['id'])
    if (finish == 'tool_calls') != bool(calls):
        raise ValueError('Finish reason and tool calls disagree')
    if calls:
        result['tool_calls'] = calls
    return result


class QwenClient:
    def __init__(self, config, profile='cloud'):
        self.config, self.profile_name = config, profile

    def complete(self, messages, tools):
        profile = self.config.profile(self.profile_name)
        payload = {'model': profile['model'], 'messages': messages,
                   'stream': profile.get('stream', False), 'enable_thinking': profile.get('enable_thinking', False),
                   'max_tokens': self.config.max_output_tokens}
        if tools:
            payload.update(tools=tools, tool_choice='auto', parallel_tool_calls=(self.profile_name == 'cloud'))
        headers = {'Content-Type': 'application/json', 'Accept': 'text/event-stream' if payload['stream'] else 'application/json'}
        if profile['api_key']:
            headers['Authorization'] = 'Bearer ' + profile['api_key']
        request = Request(profile['base_url'] + '/chat/completions', data=json.dumps(payload, allow_nan=False).encode(), headers=headers)
        try:
            with open_request(request, self.config.api_timeout_s) as response:
                if payload['stream']:
                    return self._stream(response)
                body = response.read(4_000_001)
                if len(body) > 4_000_000:
                    raise ValueError('Oversized model reply')
                choice = strict_json(body)['choices'][0]
                return assistant_message(choice['message'], choice['finish_reason'])
        except HTTPError as error:
            # Do not echo provider bodies: they may include prompts or credentials.
            raise AgentError('model_http_error', f'Model API returned HTTP {error.code}', retryable=error.code in (429, 500, 502, 503, 504)) from None
        except OSError:
            raise AgentError('model_unavailable', 'Model API connection failed; no automatic retry', retryable=True) from None
        except (ValueError, KeyError, IndexError, TypeError, AttributeError):
            raise AgentError('model_protocol_error', 'Model reply was incomplete or invalid; no tools executed') from None

    def _stream(self, response):
        deadline = time.monotonic() + self.config.api_timeout_s
        message, calls, finish, size = {'content': '', 'reasoning_content': ''}, {}, None, 0
        for raw in response:
            size += len(raw)
            if size > 4_000_000 or len(raw) > 1_000_000 or time.monotonic() > deadline:
                raise ValueError('Stream budget exceeded')
            line = raw.decode('utf-8').strip()
            if not line.startswith('data:'):
                continue
            data = line[5:].strip()
            if data == '[DONE]':
                break
            event = strict_json(data)
            if not event.get('choices'):
                continue
            choice = event['choices'][0]
            delta = choice.get('delta', {})
            for field in ('content', 'reasoning_content'):
                if delta.get(field) is not None:
                    message[field] += delta[field]
            for chunk in delta.get('tool_calls') or []:
                index = chunk['index']
                if type(index) is not int or not 0 <= index < 32:
                    raise ValueError('Invalid tool index')
                call = calls.setdefault(index, {'id': '', 'type': 'function', 'function': {'name': '', 'arguments': ''}})
                if chunk.get('id'):
                    call['id'] += chunk['id']
                if chunk.get('type') and chunk['type'] != 'function':
                    raise ValueError('Invalid tool type')
                for field in ('name', 'arguments'):
                    if chunk.get('function', {}).get(field):
                        call['function'][field] += chunk['function'][field]
            if choice.get('finish_reason'):
                finish = choice['finish_reason']
        if calls:
            if set(calls) != set(range(len(calls))):
                raise ValueError('Non-contiguous tool indexes')
            message['tool_calls'] = [calls[i] for i in sorted(calls)]
        return assistant_message(message, finish)
