"""Bounded terminal continuation and identity-bound completion proof; no I/O."""
import math

KEYS=('route_id','mission_id','point_id','sdk_forward','duration_s')
class TerminalForward:
 def __init__(self,permit,started):
  if not isinstance(permit,dict) or permit.get('authorized') is not True:raise ValueError('terminal_forward_not_authorized')
  if any(not isinstance(permit.get(k),str) or not permit[k] for k in KEYS[:3]):raise ValueError('terminal_forward_identity_invalid')
  sdk,seconds=permit.get('sdk_forward'),permit.get('duration_s')
  if (type(sdk) not in (int,float) or type(seconds) not in (int,float)
      or not math.isfinite(sdk) or not math.isfinite(seconds)
      or (sdk,seconds) not in ((.225,4.),(.30,6.))):raise ValueError('terminal_forward_policy_invalid')
  self.spec={k:permit[k] for k in KEYS};self.started=started;self.end=started+seconds
 def command(self,permit,now):
  if (not isinstance(permit,dict) or permit.get('authorized') is not True
      or any(permit.get(k)!=self.spec[k] for k in KEYS)):raise ValueError('terminal_forward_permit_lost')
  return (float(self.spec['sdk_forward']),0.,0.) if now<self.end else (0.,0.,0.)
 def receipt(self,now):
  if now<self.end:raise ValueError('terminal_forward_incomplete')
  return dict(self.spec,status='completed',output_ended=True)

def receipt_matches(receipt,expected):
 return (isinstance(receipt,dict) and receipt.get('status')=='completed'
         and receipt.get('output_ended') is True and all(receipt.get(k)==expected.get(k) for k in KEYS))
