import type {Project, Step} from './model';
export type Frame={x:number;y:number;heading:number;time:number;label:string};
export function buildPreview(p:Project,steps:Step[]){
 const frames:Frame[]=[],warnings=new Set<string>();let mirror=false;
 const add=(x:number,y:number,heading:number,duration:number,label:string)=>frames.push({x,y,heading,time:(frames.at(-1)?.time||0)+duration,label});
 const visit=(nodes:Step[],stack:string[]=[])=>{for(const n of nodes){
  if(n.name==='getMirrorAllWaypointsCommand'){mirror=!mirror;continue;}
  const command=p.commands.find(c=>c.name===n.name);
  if(command){if(stack.includes(command.name)){warnings.add('Recursive command skipped');continue;}visit(command.steps,[...stack,command.name]);continue;}
  if(n.kind==='group'){
   if(n.name==='ConditionalCommand'){warnings.add('Conditional commands preview the first branch');visit(n.children.slice(0,1),stack);}
   else if(n.name==='RepeatCommand'){warnings.add('Repeated commands preview one cycle');visit(n.children,stack);}
   else if(n.name==='SequentialCommandGroup')visit(n.children,stack);
   else {warnings.add('Concurrent groups preview their drive branch; mechanism timing is approximate');const containsMotion=(s:Step):boolean=>s.args.some(a=>a.includes('positions.'))||s.children.some(containsMotion)||!!p.commands.find(c=>c.name===s.name);visit([n.children.find(containsMotion)||n.children[0]].filter(Boolean),stack);}continue;
  }
  const name=n.args.join(' ').match(/positions\.(\w+)/)?.[1],w=p.waypoints.find(w=>w.name===name);
  if(w){const headingArg=n.args.find(a=>a.includes('faceAngle('));const heading=headingArg?Number(headingArg.match(/faceAngle\(\s*([-\d.]+)/)?.[1]||w.heading):w.heading;const pose={x:w.x,y:mirror?8.07-w.y:w.y,heading:mirror?-heading:heading};const last=frames.at(-1);const isReset=n.name.includes('runOnce')||n.name==='InstantCommand';const speed=n.name.includes('WayPoint')?Math.max(.1,Number(n.args.at(-1))||2):2;let duration=last&&!isReset?Math.hypot(pose.x-last.x,pose.y-last.y)/speed:0;if(last&&Math.hypot(pose.x-last.x,pose.y-last.y)<.01)duration=n.name.includes('firing')?2:n.name.includes('aiming')?1:.15;
   const timeout=n.decorators.find(d=>d.name==='withTimeout');if(timeout&&Number.isFinite(Number(timeout.args[0])))duration=Math.min(duration,Math.max(0,Number(timeout.args[0])));
   add(pose.x,pose.y,pose.heading,duration,n.name+' → '+w.name);
  }else if(n.name==='WaitCommand'&&frames.length){const last=frames.at(-1)!;add(last.x,last.y,last.heading,Math.max(0,Number(n.args[0])||0),'Wait');}
  if(n.decorators.some(d=>['until','onlyIf','unless','repeatedly'].includes(d.name)))warnings.add('Runtime conditions and dynamic headings are not simulated');
 }};
 visit(steps);return {frames,duration:frames.at(-1)?.time||0,warnings:[...warnings]};
}
export function samplePreview(frames:Frame[],time:number):Frame|null{if(!frames.length)return null;const b=frames.find(f=>f.time>time);if(!b)return frames.at(-1)!;const index=frames.indexOf(b),a=frames[Math.max(0,index-1)];const fraction=b.time===a.time?1:Math.max(0,Math.min(1,(time-a.time)/(b.time-a.time)));const delta=((b.heading-a.heading+540)%360)-180;return{x:a.x+(b.x-a.x)*fraction,y:a.y+(b.y-a.y)*fraction,heading:a.heading+delta*fraction,time,label:b.label};}
