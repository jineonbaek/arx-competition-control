"""Battery-stage prompt on the existing serial Agent runtime; no default actuators."""
from pathlib import Path
from .runtime import AgentRuntime
from .battery_tools import BatteryExecutionTools


class BatteryRuntime(AgentRuntime):
    prompt_path=Path(__file__).parent/'prompts/battery.md'

    def __init__(self,config,store,tools=None,**kwargs):
        if not isinstance(tools,BatteryExecutionTools) or tools.store is not store or tools.config is not config:
            raise ValueError('Explicit matching battery tool registry required')
        super().__init__(config,store,tools=tools,**kwargs)
        self.tools.progress=lambda stage:self.progress({'event':stage}) if self.progress else None
