"""Measured grip-center translation and the agreed CAD tool-axis convention."""
import copy
import hashlib
import json
from pathlib import Path
from .grasp_geometry import rigid

TCP_FILE=Path(__file__).resolve().parents[1]/'config/right-grip-tcp.json'

def load_pick_setup(*,timing_limits,alignment_tolerance_deg,j6_tolerance_deg):
    blob=TCP_FILE.read_bytes();data=json.loads(blob)
    if (data['parent_frame']!='right_link26' or data['tool_frame']!='grip_reference'
            or data['translation_unit']!='m'):
        raise ValueError('Right-wrist metric grip-center calibration required')
    rigid(data['T_wrist_from_tool'])
    setup={k:data[k] for k in ('parent_frame','tool_frame','translation_unit','T_wrist_from_tool')}
    setup.update(calibration_id=data['calibration_id']+':'+hashlib.sha256(blob).hexdigest(),
        timing_limits=copy.deepcopy(timing_limits),alignment_tolerance_deg=alignment_tolerance_deg,
        j6_tolerance_deg=j6_tolerance_deg)
    return setup
