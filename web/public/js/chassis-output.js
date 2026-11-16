// Browser-local task identity. Normal release is not a global stop.
export class ChassisOutput {
  constructor(source,clock=()=>Date.now()/1000){
    this.source=source;this.clock=clock;this.generation=0;this.sequence=0;this.active=false;
  }
  begin(){if(this.active)return this.generation;this.active=true;this.sequence=0;return ++this.generation;}
  envelope(type,generation){return {type,source_id:this.source,task_id:'keyboard',generation,
    seq:++this.sequence,issued_at:this.clock(),ttl_s:.3};}
  move(values,generation=this.generation){
    if(!this.active || generation!==this.generation)throw Error('stale keyboard task');
    return {...this.envelope('chassis',generation),...values};
  }
  end(generation=this.generation){
    if(!this.active || generation!==this.generation)return null;
    this.active=false;return this.envelope('chassis.end_output',generation);
  }
}
