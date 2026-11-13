import {useLayoutEffect,useRef,type RefObject} from 'react';

/** Animate a reordered chain with transforms; layout and field dragging stay immediate. */
export function useReorderMotion(root:RefObject<HTMLElement|null>,order:string){
 const previous=useRef({order,positions:new Map<string,DOMRect>()});
 useLayoutEffect(()=>{
  const elements=[...(root.current?.querySelectorAll<HTMLElement>('[data-step-id]')||[])];
  const positions=new Map(elements.map(el=>[el.dataset.stepId!,el.getBoundingClientRect()]));
  if(previous.current.order!==order&&!matchMedia('(prefers-reduced-motion: reduce)').matches){
   for(const element of elements){const old=previous.current.positions.get(element.dataset.stepId!),next=positions.get(element.dataset.stepId!);if(!old||!next)continue;
    const x=old.left-next.left,y=old.top-next.top;
    if(Math.abs(x)+Math.abs(y)>1)element.animate([{transform:`translate(${x}px, ${y}px)`},{transform:'translate(0, 0)'}],{duration:180,easing:'cubic-bezier(.2,.8,.2,1)'});
   }
  }
  previous.current={order,positions};
 });
}
