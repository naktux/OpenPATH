import { exportJava, type Project } from './model';
self.onmessage = (event: MessageEvent<{id:number; project:Project}>) => {
 const {id,project}=event.data;
 try { self.postMessage({id,full:exportJava(project),errors:[]}); }
 catch(error){self.postMessage({id,full:'',errors:[String(error)]});}
};
