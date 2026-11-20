// Browser-local task identity. Normal release is not a global stop.
export class ChassisOutput {
  constructor(source,clock=()=>Date.now()/1000){
    this.source=source;this.clock=clock;this.generation=0;this.sequence=0;this.active=false;
  }
  begin(){if(this.active)return this.generation;this.active=true;this.sequence=0;return ++this.generation;}
  envelope(type,generation){return {type,source_id:this.source,task_id:'keyboard',generation,
    seq:++this.sequence,issued_at:this.clock(),ttl_s:1.0};}
  move(values,generation=this.generation,sdkPerInput=1){
    if(!this.active || generation!==this.generation)throw Error('stale keyboard task');
    if(!Number.isFinite(sdkPerInput)||sdkPerInput<=0)throw Error('invalid chassis input scale');
    return {...this.envelope('chassis',generation),x:values.x/sdkPerInput,y:values.y/sdkPerInput,omega:values.omega/sdkPerInput};
  }
  end(generation=this.generation){
    if(!this.active || generation!==this.generation)return null;
    this.active=false;return this.envelope('chassis.end_output',generation);
  }
}
