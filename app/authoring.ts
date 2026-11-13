import {methods,newCall,splitArgs,type Project,type Step} from './model';
export type Helper={name:string;params:{type:string;name:string}[]};
export const readable=(name:string)=>name.replace(/([a-z0-9])([A-Z])/g,'$1 $2').replace(/_/g,' ').replace(/^pos$/,'Waypoint').replace(/^error Margin$/i,'Arrival tolerance').replace(/^velocity$/i,'Speed').replace(/^angle$/i,'Heading supplier');
export function commandHelpers(source:string):Helper[]{return methods(source).filter(m=>/\bCommand\s+\w+\s*\(/.test(source.slice(m.start,m.bodyStart))).map(m=>({name:m.name,params:splitArgs(m.params).map(p=>{const match=/^(.*?)\s+(\w+)$/.exec(p.trim());return{type:match?.[1]||'',name:match?.[2]||p};})}));}
export function helperStep(helper:Helper,p:Project):Step{return newCall(helper.name,helper.params.map(param=>param.type==='positions'?'positions.'+(p.waypoints[0]?.name||''):param.type==='DoubleSupplier'?(methods(p.source).some(h=>h.name==='faceAngle')?'faceAngle(0)':'() -> 0.0'):param.name==='errorMargin'?'0.1':param.name==='velocity'?'2.0':/double|Double|int/.test(param.type)?'0.0':''));}
export function renameReferences(p:Project,oldName:string,newName:string,kind:'waypoints'|'commands'){
 const visit=(steps:Step[])=>steps.forEach(n=>{if(kind==='commands'&&n.name===oldName)n.name=newName;const rename=(arg:string)=>kind==='waypoints'?arg.replace(/positions\.(\w+)/g,(all,name)=>name===oldName?'positions.'+newName:all):arg;n.args=n.args.map(rename);n.decorators.forEach(d=>d.args=d.args.map(rename));visit(n.children);});
 p.commands.forEach(c=>visit(c.steps));p.routines.forEach(c=>visit(c.steps));
}
