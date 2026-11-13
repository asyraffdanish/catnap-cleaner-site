var tu=Object.defineProperty;var eu=(s,t,e)=>t in s?tu(s,t,{enumerable:!0,configurable:!0,writable:!0,value:e}):s[t]=e;var y=(s,t,e)=>eu(s,typeof t!="symbol"?t+"":t,e);(function(){const t=document.createElement("link").relList;if(t&&t.supports&&t.supports("modulepreload"))return;for(const n of document.querySelectorAll('link[rel="modulepreload"]'))i(n);new MutationObserver(n=>{for(const r of n)if(r.type==="childList")for(const a of r.addedNodes)a.tagName==="LINK"&&a.rel==="modulepreload"&&i(a)}).observe(document,{childList:!0,subtree:!0});function e(n){const r={};return n.integrity&&(r.integrity=n.integrity),n.referrerPolicy&&(r.referrerPolicy=n.referrerPolicy),n.crossOrigin==="use-credentials"?r.credentials="include":n.crossOrigin==="anonymous"?r.credentials="omit":r.credentials="same-origin",r}function i(n){if(n.ep)return;n.ep=!0;const r=e(n);fetch(n.href,r)}})();const zt=(s,t,e)=>Math.max(t,Math.min(e,s)),iu=(s,t,e)=>s+(t-s)*e,xe=(s,t,e,i)=>iu(s,t,1-Math.exp(-e*i)),Tt=(s,t)=>Math.hypot(s.x-t.x,s.z-t.z),Ci=s=>Math.atan2(Math.sin(s),Math.cos(s));function zo(s){let t=s>>>0;return()=>{t+=1831565813;let e=t;return e=Math.imul(e^e>>>15,e|1),e^=e+Math.imul(e^e>>>7,e|61),((e^e>>>14)>>>0)/4294967296}}const nu={criticalFrequency:0,fullAutoCriticalAvoidSuccessProbability:.1,dynamicForceFeedbackEnabled:!0},Jt={critical:{intervals:[[1/0,1/0],[28,42],[20,30],[14,22]],jumpDuration:.6,jumpHeight:.65,replanInterval:.25,straightCorridorLength:6,restDistance:5,approachSpeed:7,roamSpeed:1.15,fleeSpeed:9,radius:.36,approachTimeout:7,lookAhead:3.4,corridorHalfWidth:.82,onsetMinDistance:1.5,onsetMaxDistance:4.5,hazardHold:1.55,hazardDuration:2.1,restAfterImpact:8,collisionDisturbance:32,nearMissDisturbance:5,nearMissClearance:.85,autonomousReactionDelay:.1},emergency:{swipeVelocityThreshold:700,swipeDistanceThreshold:48,maximumGestureDuration:.22,windowSeconds:.14,evasiveImpulse:8,sweepTimeStep:.02,evasiveTailDuration:.08,evasiveDuration:.3,clearanceHoldDuration:1.35,interventionSuccessWindow:1.15,lateralAlignmentThreshold:.25},force:{detectionRange:3.5,minimumForce:0,maximumForce:1,proximityExponent:1.6,authorityMinimum:.3,authorityMaximum:1,authorityExponent:1,smoothingRate:5,pulseFrequencyMin:2,pulseFrequencyMax:7,pulseDurationMin:12,pulseDurationMax:45,imminentCollisionThreshold:.3,sampleStep:.12},camera:{normalZoom:1.3,shortLandscapeZoom:1.6,portraitZoom:.92,minimumZoomRatio:.78,portraitMinimumZoomRatio:.68,interpolationSpeed:2.8,visibilityPadding:36,hysteresisThreshold:.08,returnDelay:.65},gold:{minimumDistance:10.5,maximumDistance:13,initialExclusionRadius:6}};function qn(s={}){return{criticalFrequency:[0,1,2,3].includes(s.criticalFrequency)?s.criticalFrequency:0,fullAutoCriticalAvoidSuccessProbability:typeof s.fullAutoCriticalAvoidSuccessProbability=="number"&&Number.isFinite(s.fullAutoCriticalAvoidSuccessProbability)?zt(s.fullAutoCriticalAvoidSuccessProbability,0,1):.1,dynamicForceFeedbackEnabled:typeof s.dynamicForceFeedbackEnabled=="boolean"?s.dynamicForceFeedbackEnabled:!0}}const je=s=>({x:Math.cos(s),z:Math.sin(s)}),Yc=s=>Math.atan2(s.z,s.x);function Ti(s,t={x:1,z:0}){const e=Math.hypot(s.x,s.z);return e>1e-8?{x:s.x/e,z:s.z/e}:{...t}}const li=(s,t)=>zt(s.x*t.x+s.z*t.z,-1,1);function $r(s,t,e,i,n){return Ti({x:s.x*e+t.x*i,z:s.z*e+t.z*i},n)}function ml(s,t,e,i){const n=zt(Math.hypot(s.x,s.y),0,1);if(n<.001)return null;t.updateMatrixWorld();const r=t.matrixWorldInverse.elements,a=t.projectionMatrix.elements,o=r[0]*a[0]*e/2,c=r[8]*a[0]*e/2,l=-r[1]*a[5]*i/2,h=-r[9]*a[5]*i/2,u=o*h-c*l;return Math.abs(u)<1e-8?null:{direction:Ti({x:(h*s.x-c*s.y)/u,z:(o*s.y-l*s.x)/u}),magnitude:n}}function As(s,t,e,i,n=.85){const r=Ci(Yc(s)-t);return zt(Math.sign(r)*2*e*Math.sin(Math.min(Math.abs(r),Math.PI/2))/(Math.max(.55,n)*i),-1,1)}const We=3.15,on=4.8;class Di{constructor(t,e=!1){y(this,"loop");y(this,"points");y(this,"lengths",[]);y(this,"progress",0);y(this,"total",0);y(this,"index",0);this.loop=e,this.points=t.map(i=>({...i}));for(let i=0;i<t.length;i++)i&&(this.total+=Tt(t[i-1],t[i])),this.lengths.push(this.total)}indexAt(t){let e=0,i=this.lengths.length-2;for(;e<i;){const n=e+i>>1;this.lengths[n+1]<t?e=n+1:i=n}return Math.max(0,e)}get target(){return this.points[this.points.length-1]}get done(){return!this.loop&&this.total-this.progress<.22}poseAt(t){const e=this.loop&&this.total>0?(t%this.total+this.total)%this.total:zt(t,0,this.total),i=this.indexAt(e),n=this.points[i],r=this.points[i+1]??n,a=(e-this.lengths[i])/(Tt(n,r)||1);return{x:n.x+(r.x-n.x)*a,z:n.z+(r.z-n.z)*a,heading:Math.atan2(r.z-n.z,r.x-n.x)}}seek(t){this.progress=zt(t,0,this.total),this.index=this.indexAt(this.progress)}update(t,e){this.loop&&this.total-this.progress<.25&&Tt(t,this.target)<.65&&this.seek(0);let i=1/0,n=this.progress;for(let l=Math.max(0,this.index-3);l<this.points.length-1&&this.lengths[l]<this.progress+4;l++){const h=this.points[l],u=this.points[l+1],d=u.x-h.x,f=u.z-h.z,g=Tt(h,u),v=zt(((t.x-h.x)*d+(t.z-h.z)*f)/(g*g||1),0,1),m=this.lengths[l]+v*g;if(m<this.progress-.3)continue;const p=Math.hypot(t.x-h.x-v*d,t.z-h.z-v*f);p<i&&(i=p,n=m)}this.seek(Math.max(this.progress,n));const r=this.poseAt(this.progress+.85);if(!this.loop&&this.progress+.85>this.total){const l=this.progress+.85-this.total;r.x+=Math.cos(r.heading)*l,r.z+=Math.sin(r.heading)*l}const a=Tt(t,r),o=Ci(Math.atan2(r.z-t.z,r.x-t.x)-e),c=Ti({x:r.x-t.x,z:r.z-t.z},je(e));return{direction:c,steer:As(c,e,We,on,a),error:o,distance:a}}remaining(){return[this.poseAt(this.progress),...this.points.slice(this.index+1)]}}function gl(s,t,e,i=1.1,n=1){const r=[{...s}];let a=s.x,o=s.z,c=t;for(let l=0;l<i;l+=.045)c+=As(e,c,We,on)*n*on*.045,a+=Math.cos(c)*We*.045,o+=Math.sin(c)*We*.045,r.push({x:a,z:o});return r}const Yr=Math.PI*2,su=s=>(s%Yr+Yr)%Yr,Ys=(s,t,e)=>({x:s.x-t*e*Math.sin(s.heading),z:s.z+t*e*Math.cos(s.heading)}),us=(s,t,e,i)=>{const n=s.points[s.points.length-1];s.arc(t,i*su(i*(Math.atan2(e.z-t.z,e.x-t.x)-Math.atan2(n.z-t.z,n.x-t.x))))};function Bo(s,t,e=1.25,i=ti){const n=[];for(const r of[-1,1])for(const a of[-1,1]){const o=Ys(s,r,e),c=Ys(t,a,e),l=Tt(o,c),h=(r-a)*e/l;if(!l||Math.abs(h)>1)continue;const u=Math.atan2(c.z-o.z,c.x-o.x)+Math.asin(h),d={x:-Math.sin(u),z:Math.cos(u)},f={x:o.x-r*e*d.x,z:o.z-r*e*d.z},g={x:c.x-a*e*d.x,z:c.z-a*e*d.z},v=new Ds(s);us(v,o,f,r),v.line(g),us(v,c,t,a),n.push(v.points)}for(const r of[-1,1]){const a=Ys(s,r,e),o=Ys(t,r,e),c=Tt(a,o);if(!(!c||c>4*e))for(const l of[-1,1]){const h=Math.sqrt(4*e*e-c*c/4),u={x:(a.x+o.x)/2-l*(o.z-a.z)/c*h,z:(a.z+o.z)/2+l*(o.x-a.x)/c*h},d={x:(a.x+u.x)/2,z:(a.z+u.z)/2},f={x:(o.x+u.x)/2,z:(o.z+u.z)/2},g=new Ds(s);us(g,a,d,r),us(g,u,f,-r),us(g,o,t,r),n.push(g.points)}}return n.filter(r=>r.length>1&&r.every(a=>!i(a,Ee+.27))).sort((r,a)=>ai(r)-ai(a))[0]??[]}function ai(s){let t=0;for(let e=1;e<s.length;e++)t+=Tt(s[e-1],s[e]);return t}function qc(s,t,e,i=ti){return[e,Math.atan2(t.z-s.z,t.x-s.x),e+.6,e-.6,e+Math.PI/2,e-Math.PI/2].map(n=>Bo(s,{...t,heading:n},1.25,i)).filter(n=>n.length).sort((n,r)=>ai(n)-ai(r))[0]??[]}const Oa=()=>Ee+.32,Kc=.9,Es=1.25;class ru{constructor(){y(this,"items",[])}push(t){let e=this.items.length;for(this.items.push(t);e;){const i=e-1>>1;if(this.items[i].rank<=t.rank)break;this.items[e]=this.items[i],e=i}this.items[e]=t}pop(){const t=this.items[0],e=this.items.pop();if(this.items.length){let i=0;for(;i*2+1<this.items.length;){let n=i*2+1;if(n+1<this.items.length&&this.items[n+1].rank<this.items[n].rank&&n++,this.items[n].rank>=e.rank)break;this.items[i]=this.items[n],i=n}this.items[i]=e}return t}}function au(s,t){const e=[];for(let i=1;i<=9;i++){const n=Kc*i/9,r=s.heading+n*t;e.push(Math.abs(t)<1e-9?{x:s.x+Math.cos(r)*n,z:s.z+Math.sin(r)*n,heading:r}:{x:s.x+(Math.sin(r)-Math.sin(s.heading))/t,z:s.z-(Math.cos(r)-Math.cos(s.heading))/t,heading:Ci(r)})}return e}function vl(s,t,e,i){const n=Math.atan2(t.z-s.z,t.x-s.x);return(e===void 0?[n,n+Math.PI/2,n-Math.PI/2,n+Math.PI]:[e]).map(a=>Bo(s,{...t,heading:a},Es,i)).filter(a=>a.length&&a.every(o=>!i(o,Oa()))).sort((a,o)=>ai(a)-ai(o))[0]??[]}function Wn(s,t,e,i=ti){if(i(s,Ee+.05)||i(t,Oa()))return[];const n=vl(s,t,e,i);if(n.length)return n;const r=new ru,a=new Map;r.push({...s,cost:0,rank:Tt(s,t),parent:null,segment:[{...s}]});for(let o=0;r.items.length&&o<1e4;o++){const c=r.pop();if(o>0&&o%5===0&&Tt(c,t)<7){const l=vl(c,t,e,i);if(l.length){const h=[l.slice(1)];let u=c;for(;u;)h.unshift(u.segment),u=u.parent;return h.flat()}}for(const l of[0,-.5/Es,.5/Es,-1/Es,1/Es]){const h=au(c,l);if(h.some(g=>i(g,Oa())))continue;const u=h[h.length-1],d=c.cost+Kc*(1+.06*Math.abs(l)),f=`${Math.round(u.x/.35)},${Math.round(u.z/.35)},${Math.round(Ci(u.heading)/(Math.PI/18))}`;(a.get(f)??1/0)<=d||(a.set(f,d),r.push({...u,cost:d,rank:d+Tt(u,t)*1.15,parent:c,segment:h}))}}return[]}const _l=4.6,ou=5.8;function xl(s,t,e,i=ti,n=Math.random){for(let r=0;r<80;r++){const a=_l+n()*(ou-_l),o=s.heading+t*(n()<.5?-1:1)*(.32+n()*.4),c={x:s.x+Math.cos(o)*a,z:s.z+Math.sin(o)*a};if(i(c,.75))continue;const l=qc(s,c,s.heading,i);if(l.length&&ai(l)/We<e-2)return c}return null}const yl=Jt.gold.minimumDistance,lu=Jt.gold.maximumDistance,Zc=18,cu=1500;function Ml(s,t,e=()=>!0,i,n=ti,r=Math.random){for(let a=0;a<100;a++){const o=yl+r()*(lu-yl),c=s.heading+t*(r()*2-1)*Math.PI,l={x:s.x+Math.cos(c)*o,z:s.z+Math.sin(c)*o};if(n(l,.85)||i&&Math.hypot(l.x-i.x,l.z-i.z)<Jt.gold.initialExclusionRadius)continue;const h=Wn(s,l,void 0,n);if(h.length&&ai(h)/We<Zc-6)return l}return null}const dn=26,fn=18,Je=.5,Ee=.42,Kn=1.5,Sl=2.4,Ii=[{x:-5,z:-5.7,w:3.7,d:1.35,kind:"sofa"},{x:-4,z:-.9,w:2.2,d:1.3,kind:"coffee"},{x:5,z:-3.3,w:2.5,d:1.3,kind:"dining"},{x:3,z:-3.3,w:1.05,d:.65,kind:"chair"},{x:7,z:-3.3,w:1.05,d:.65,kind:"chair"},{x:-8.6,z:-5.7,w:2.7,d:.85,kind:"shelf"},{x:-7,z:6.3,w:3.6,d:1.2,kind:"desk"},{x:8.7,z:3.9,w:.9,d:.9,kind:"plant"},{x:-8,z:1.5,w:.8,d:.8,kind:"plant"},{x:8,z:-5.7,w:.8,d:.8,kind:"plant"},{x:-3,z:3.9,w:.9,d:.8,kind:"box"}];function ti(s,t=Ee+.1,e=Ii){return Math.abs(s.x)>dn/2-t||Math.abs(s.z)>fn/2-t?!0:e.some(i=>{const n=Math.max(Math.abs(s.x-i.x)-i.w/2,0),r=Math.max(Math.abs(s.z-i.z)-i.d/2,0);return n*n+r*r<t*t})}class hu{constructor(t=Ii){y(this,"obstacles");y(this,"lastCleaned",[]);y(this,"cols",dn/Je);y(this,"rows",fn/Je);y(this,"cells",[]);y(this,"total",0);y(this,"cleaned",0);y(this,"revision",0);y(this,"blocked",(t,e)=>ti(t,e,this.obstacles));this.obstacles=t;for(let a=0;a<this.rows;a++)for(let o=0;o<this.cols;o++){const c={x:-dn/2+Je/2+o*Je,z:-fn/2+Je/2+a*Je};this.cells.push({...c,clean:!1,eligible:!this.blocked(c,.12),nav:!this.blocked(c,.55)})}const e=[],i=new Set;for(let a=0;a<this.cells.length;a++){if(!this.cells[a].nav||i.has(a))continue;const o=[a];i.add(a);for(let c=0;c<o.length;c++){const l=o[c],h=l%this.cols,u=Math.floor(l/this.cols);for(const[d,f]of[[1,0],[-1,0],[0,1],[0,-1]]){const g=h+d,v=u+f;if(g<0||g>=this.cols||v<0||v>=this.rows)continue;const m=v*this.cols+g;this.cells[m].nav&&!i.has(m)&&(i.add(m),o.push(m))}}e.push(o)}e.sort((a,o)=>o.length-a.length);const n=new Set(e[0]);this.cells.forEach((a,o)=>a.nav=a.nav&&n.has(o));const r=this.cells.filter(a=>a.nav);for(const a of this.cells)a.eligible=a.eligible&&r.some(o=>Tt(a,o)<Kn-.23),a.eligible&&this.total++}index(t){return Math.max(0,Math.min(this.rows-1,Math.round((t.z+fn/2-Je/2)/Je)))*this.cols+Math.max(0,Math.min(this.cols-1,Math.round((t.x+dn/2-Je/2)/Je)))}nearestNav(t){let e=0,i=1/0;return this.cells.forEach((n,r)=>{const a=Tt(n,t);n.nav&&a<i&&(e=r,i=a)}),e}cleanAt(t,e=Kn){let i=0;this.lastCleaned=[];for(const n of this.cells)n.eligible&&!n.clean&&Tt(n,t)<=e&&(Tt(n,t)<=Kn||this.clearLine(t,n,.08))&&(n.clean=!0,i++,this.lastCleaned.push({x:n.x,z:n.z}));return this.cleaned+=i,i&&this.revision++,i}get progress(){return this.cleaned/this.total*100}nearestDirty(t){let e,i=1/0;for(const n of this.cells)if(n.eligible&&!n.clean){const r=Tt(n,t);r<i&&(i=r,e=n)}return e}path(t,e){const i=this.nearestNav(t),n=this.nearestNav(e);if(i===n)return[{...this.cells[n]}];const r=new Set([i]),a=new Map,o=new Map([[i,0]]),c=new Map([[i,Tt(this.cells[i],this.cells[n])]]);for(;r.size;){let l=-1,h=1/0;for(const f of r){const g=c.get(f)??1/0;g<h&&(l=f,h=g)}if(l===n){const f=[{...this.cells[l]}];for(;a.has(l);)l=a.get(l),f.unshift({...this.cells[l]});const g=[];let v=0;for(;v<f.length-1;){let m=f.length-1;for(;m>v+1&&!this.clearLine(f[v],f[m]);)m--;g.push(f[m]),v=m}return g}r.delete(l);const u=l%this.cols,d=Math.floor(l/this.cols);for(const[f,g]of[[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]]){const v=u+f,m=d+g;if(v<0||v>=this.cols||m<0||m>=this.rows)continue;const p=m*this.cols+v;if(!this.cells[p].nav||f&&g&&(!this.cells[d*this.cols+v].nav||!this.cells[m*this.cols+u].nav))continue;const w=(o.get(l)??1/0)+Math.hypot(f,g)*Je;w<(o.get(p)??1/0)&&(a.set(p,l),o.set(p,w),c.set(p,w+Tt(this.cells[p],this.cells[n])),r.add(p))}}return[]}clearLine(t,e,i=.5){const n=Math.ceil(Tt(t,e)/.12);for(let r=0;r<=n;r++){const a=n?r/n:0;if(this.blocked({x:t.x+(e.x-t.x)*a,z:t.z+(e.z-t.z)*a},i))return!1}return!0}}class Ds{constructor(t){y(this,"points",[]);this.points.push({...t})}line(t){const e=this.points[this.points.length-1],i=Math.ceil(Tt(e,t)/.16);for(let n=1;n<=i;n++)this.points.push({x:e.x+(t.x-e.x)*n/i,z:e.z+(t.z-e.z)*n/i});return this}arc(t,e){const i=this.points[this.points.length-1],n=Tt(i,t),r=Math.atan2(i.z-t.z,i.x-t.x),a=Math.ceil(Math.abs(e)*n/.12);for(let o=1;o<=a;o++)this.points.push({x:t.x+Math.cos(r+e*o/a)*n,z:t.z+Math.sin(r+e*o/a)*n});return this}}function uu(s=!1){const t=new Ds({x:10.8,z:s?-6.9:7.5});let e=t.points[0].z,i=1;for(let n=0;n<6;n++){const r=s?Sl:-Sl;t.arc({x:i*10.8,z:e+r/2},i*Math.sign(r)*Math.PI),e+=r,i*=-1,t.line({x:i*10.8,z:e})}return t.points}function du(){const s=new Ds({x:-10.8,z:8.2});s.line({x:11.1,z:8.2}).arc({x:11.1,z:7.1},-Math.PI/2).line({x:12.2,z:-7.1}).arc({x:11.1,z:-7.1},-Math.PI/2).line({x:-11.1,z:-8.2}).arc({x:-11.1,z:-7.1},-Math.PI/2).line({x:-12.2,z:7.1}).arc({x:-11.1,z:7.1},-Math.PI/2);const t=Math.acos(1-.7/2.6);s.arc({x:-11.1,z:6.9},-t);const e=s.points[s.points.length-1];s.arc({x:e.x+Math.sin(t)*1.3,z:e.z+Math.cos(t)*1.3},t).line({x:10.8,z:7.5});const i=uu(!1),n=new Ds(i[i.length-1]);return n.arc({x:10.8,z:-5.5},Math.PI/2).line({x:12.2,z:3.7}).arc({x:10.8,z:3.7},Math.PI/2).line({x:-10.8,z:5.1}).arc({x:-10.8,z:6.3},-Math.PI/2).line({x:-12,z:7}).arc({x:-10.8,z:7},-Math.PI/2),n.points[n.points.length-1]={...s.points[0]},[...s.points,...i.slice(1),...n.points.slice(1)]}class fu{constructor(t,e=Ii,i=!0){y(this,"seed");y(this,"obstacles");y(this,"showBlackCat");y(this,"grid");y(this,"route",du());y(this,"bonuses",[]);y(this,"start",{x:-10.8,z:8.2});this.seed=t,this.obstacles=e,this.showBlackCat=i,this.grid=new hu(e);const n=zo(t);for(const[r,a,o,c,l,h,u,d]of[["red",14,2.5,3,4,5,6,250],["gold",4,12,8,16,12,Zc,cu],["magnet",3,7,6,24,12,9,0]]){let f=o+n()*c;for(let g=0;g<a;g++)this.bonuses.push({x:0,z:0,id:this.bonuses.length,relativeSpawn:!0,kind:r,spawn:f,duration:u,value:r==="red"?[100,250,500][g%3]:d,collected:!1,expired:!1}),f+=l+n()*h}}}const bl=Ii.filter(s=>s.kind==="box"||s.kind==="plant"&&s.x===8),Jc=[{id:"tour",label:"Level 0 · Pairdriver tour",subtitle:"LEARN BY DOING",description:"A guided, hands-on introduction to shared steering.",obstacles:bl,blackCat:!1},{id:"practice",label:"Level 1 · Open room",subtitle:"ROOM TO EXPLORE",description:"A large, mostly empty floor. Two obstacles, no black-cat intervention.",obstacles:bl,blackCat:!1},{id:"level1",label:"Level 2 · The original home",subtitle:"FAMILIAR FLOOR",description:"The existing room, furniture, pickups and experiment settings.",obstacles:Ii,blackCat:!0}],Fa=s=>Jc.find(t=>t.id===s),Hi={heavyShare:.3,lightShare:.9,practiceInputSeconds:1,obstacleTakeover:.6,liveTimeScale:1,bonusDuration:600},pu={inputThreshold:.35},ka=[{id:"welcome",label:"WELCOME",title:"Hello, little cleaner.",text:"Clean the floor for points. Red bunnies give 250, gold gives 1,500.",task:"Swipe or tap Next.",picture:0,live:!1},{id:"controls",label:"HOW TO PLAY",title:"Drag toward your destination.",text:"Touch the floor, then drag in any direction. Blue is you, amber is automation, white is the shared result. Steering changes direction, not cruise speed.",task:"Try each function next. Keyboard: WASD or arrows.",picture:1,live:!1},{id:"switch",label:"01 · INTERACTION · COMPARE",title:"First, a normal takeover.",text:"In Full Automation, hold a drag to switch AD off and drive yourself.",task:"This switch is what we compare against.",picture:1,live:!1},{id:"takeover",label:"01 · INTERACTION · COMPARE",title:"Take over, then return to AD.",text:"Hold a drag until Manual appears. Steer, then press AD back on (or E).",task:"Take over, then turn AD back on.",picture:1,live:!0},{id:"shared",label:"01 · PAIRDRIVER INTERACTION",title:"Now steer, without a switch.",text:"In Collaborative, your drag and AD blend into one direction. No takeover needed.",task:"Next: try it yourself.",picture:1,live:!1},{id:"seamless",label:"01 · PAIRDRIVER INTERACTION",title:"Steer seamlessly.",text:"Drag any way. AD stays on and blends with you.",task:"Steer for one second.",picture:1,live:!0},{id:"heavy",label:"02 · ARBITRATION",title:"30 / 70 · Feel the weight.",text:"AD carries 70%. Steer away from the amber arrow and feel it pull back.",task:"Steer for one second.",picture:2,live:!0},{id:"light",label:"02 · ARBITRATION",title:"Change to 90 / 10 · Feel it lighten.",text:"Move the slider to 90 / 10, then steer. It feels much lighter.",task:"Set 90 / 10 and steer for one second.",picture:2,live:!0},{id:"weight-choice",label:"02 · ARBITRATION",title:"Choose your own balance.",text:"Pick any weight you like. It carries into the next exercises.",task:"Tap Next when it feels right.",picture:2,live:!0},{id:"obstacle",label:"02 · ARBITRATION · OBSTACLES",title:"Near furniture, AD leans in.",text:"Close to an obstacle, AD takes up to 90% and steers you clear.",task:"Next: drive at the pink furniture.",picture:2,live:!1},{id:"obstacle-test",label:"02 · ARBITRATION · OBSTACLES",title:"Drive at the highlighted furniture.",text:"Steer straight at the pink outline and keep holding. AD will take over.",task:"Hold until AD takes over.",picture:2,live:!0},{id:"red-intro",label:"03 · INCLUSION · NEARBY",title:"A red bunny appears.",text:"Steer toward it to ask for a short detour. It is worth 250.",task:"Next shows a red bunny nearby.",picture:3,live:!1},{id:"red",label:"03 · INCLUSION · NEARBY",title:"Take the red bunny.",text:"Keep aiming at it until you collect it.",task:"Collect the red bunny.",picture:3,live:!0},{id:"gold-intro",label:"03 · INCLUSION · FARTHER AWAY",title:"Now make your goal the plan.",text:"Aim at the far gold bunny until LOCKED, then let go. AD plans the route there.",task:"Next shows gold farther away.",picture:3,live:!1},{id:"gold",label:"03 · INCLUSION · FARTHER AWAY",title:"Aim, lock, then let go.",text:"Once LOCKED, you can still steer and the route follows. Press E to release it.",task:"Lock gold, collect it, and let AD return to cleaning.",picture:3,live:!0},{id:"done",label:"TOUR COMPLETE",title:"Two contributors. One plan.",text:"Interaction: steer without a switch. Arbitration: share the weight. Inclusion: your goal becomes the plan.",task:"A simple game analogy, not real steering force.",picture:0,live:!1}];function mu(s){let t=s.scenario.obstacles[0]??null,e=1/0;for(const i of s.scenario.obstacles){const n=Math.hypot(Math.max(0,Math.abs(s.position.x-i.x)-i.w/2),Math.max(0,Math.abs(s.position.z-i.z)-i.d/2));n<e&&(e=n,t=i)}return t}class gu{constructor(){y(this,"compact",!1);y(this,"index",0);y(this,"eventStart",0);y(this,"elapsedInput",0);y(this,"ready",!1);y(this,"tookOver",!1);y(this,"reactivated",!1);y(this,"goldLocked",!1)}get step(){return ka[this.index]}get timeScale(){return this.step.live?Hi.liveTimeScale:0}enter(t){this.compact=this.step.live,this.elapsedInput=0,this.ready=!this.step.live||this.step.id==="weight-choice";const e=this.step.id;for(const n of t.scenario.bonuses)n.kind!==e&&(n.expired=!0);t.mode==="collaborative"&&t.releaseRoute();const i=["seamless","heavy","light","obstacle-test","weight-choice"].includes(e);t.tourWeightOnly=i,i&&t.setControlMode("collaborative"),e==="seamless"&&t.setInterventionShare(Hi.lightShare),(e==="heavy"||e==="light")&&t.setInterventionShare(Hi.heavyShare),t.highlightedObstacle=e==="obstacle-test"?mu(t):null,e==="takeover"&&(this.eventStart=t.eventHistory.length,t.setControlMode("automation"),this.tookOver=!1,this.reactivated=!1),e==="red"&&(t.setControlMode("collaborative"),t.releaseRoute(),t.revealTourBonus("red")),e==="gold"&&(t.setControlMode("collaborative"),t.releaseRoute(),this.goldLocked=!1,t.revealTourBonus("gold")),t.emit("tour_step",this.index)}next(t){return!this.ready||this.index===ka.length-1?!1:(this.index++,this.enter(t),!0)}previous(t){return this.index===0?!1:(this.index--,this.enter(t),!0)}update(t,e,i){const n=this.step.id;if(n==="heavy"||n==="light"||n==="seamless"){const r=(e?.magnitude??0)>.65&&i.mode==="collaborative"&&(n==="heavy"?i.interventionShare<=.31:n==="light"?i.interventionShare>=.89:!0);this.elapsedInput=r?this.elapsedInput+t:0,this.elapsedInput>=Hi.practiceInputSeconds&&(this.ready=!0)}n==="takeover"&&(i.eventHistory.slice(this.eventStart).some(r=>r.type==="automation_takeover")&&(this.tookOver=!0),this.tookOver&&i.mode==="automation"&&i.eventHistory.slice(this.eventStart).some(r=>r.type==="automation_reactivated")&&(this.reactivated=!0),this.ready=this.tookOver&&this.reactivated),n==="obstacle-test"&&(e?.magnitude??0)>.5&&i.obstacleArbitration>=Hi.obstacleTakeover&&(this.ready=!0),(n==="red"||n==="gold")&&i.scenario.bonuses.find(r=>r.kind===n)?.spawn===1/0&&i.revealTourBonus(n),n==="red"&&i.scenario.bonuses.find(r=>r.kind==="red")?.collected&&(this.ready=!0),n==="gold"&&(i.inclusionLocked&&(this.goldLocked=!0),this.goldLocked&&i.scenario.bonuses.find(r=>r.kind==="gold")?.collected&&i.state==="normal"&&!i.inclusionLocked&&(this.ready=!0))}}class vu{constructor(t){y(this,"back");y(this,"guarded",!1);this.back=t,history.replaceState({catnap:!0},""),window.addEventListener("popstate",()=>{this.guarded&&(this.guarded=!1,this.back())})}sync(t){t&&!this.guarded&&(history.pushState({catnap:!0,guard:!0},""),this.guarded=!0)}atHomeBack(){this.guarded&&(this.guarded=!1,history.back())}}function _u(s){return{schemaVersion:11,level:s.level,initialMode:s.initialMode,trialId:s.critical.trialId,startedAt:s.startedAt,feedbackHardware:{vibrationAvailable:typeof navigator<"u"&&typeof navigator.vibrate=="function"},mode:s.mode,flow:s.flow,seed:s.seed,status:s.status,completed:s.metrics.completed,initialExperimentSettings:s.initialExperiment,experimentSettings:{...s.experiment},criticalEventsEnabled:s.experiment.criticalFrequency!==0,tuning:Jt,controlSettings:{interventionShare:s.interventionShare,inclusionDelay:s.inclusionDelay,takeoverHold:s.takeoverHold},metrics:{...s.metrics},criticalEvents:s.critical.records,events:s.eventHistory,samples:s.telemetry,bonuses:s.scenario.bonuses}}class xu{constructor(){y(this,"key","catnap-experiment-trials");y(this,"trials",[]);y(this,"persistent",!0);try{const t=JSON.parse(localStorage.getItem(this.key)??"[]");Array.isArray(t)&&(this.trials=t.slice(-6))}catch{this.persistent=!1}}save(t){if(t.time<=0)return;const e=JSON.parse(JSON.stringify(_u(t)));for(this.trials=this.trials.filter(i=>i.trialId!==e.trialId),this.trials.push(e),this.trials=this.trials.slice(-6);;)try{localStorage.setItem(this.key,JSON.stringify(this.trials)),this.persistent=!0;break}catch{if(this.persistent=!1,this.trials.length<=1)break;this.trials.shift()}}}const El={coffee:{height:.69,offset:.7},dining:{height:1.19,offset:.75},desk:{height:1.19,offset:0},chair:{height:.7,offset:0},box:{height:.575,offset:0},sofa:{height:.95,offset:.4}},Sn=Ii.filter(s=>s.kind in El).flatMap(s=>{const{height:t,offset:e}=El[s.kind],i={source:s.kind,surface:{x:s.x+e,z:s.z},height:t};return[{...i,x:s.x,z:s.z+s.d/2+.58},{...i,x:s.x+s.w/2+.58,z:s.z}]}).filter(s=>!ti(s,.4));class yu{constructor(t,e,i=Math.random,n=globalThis.crypto?.randomUUID?.()??`trial-${Date.now()}-${Math.random()}`){y(this,"settings");y(this,"grid");y(this,"random");y(this,"trialId");y(this,"phase","rest");y(this,"position");y(this,"heading",0);y(this,"height",0);y(this,"visualPosition");y(this,"records",[]);y(this,"current",null);y(this,"signals",[]);y(this,"nextEvent");y(this,"path",[]);y(this,"pathIndex",0);y(this,"phaseStart",0);y(this,"lastPlan",-1);y(this,"goal");y(this,"nextRoam",0);y(this,"hasEmerged",!1);y(this,"impulseRemaining",0);y(this,"headingRemaining",0);y(this,"motionHeading",null);y(this,"crossingTarget",null);y(this,"impulseDirection",{x:0,z:0});y(this,"autoTriggered",!1);y(this,"launch",{x:0,z:0});y(this,"launchHeight",0);this.settings=t,this.grid=e,this.random=i,this.trialId=n;const r=Math.floor(this.random()*Sn.length);this.goal=Sn[this.settings.criticalFrequency?r:Sn.length-1],this.position={x:this.goal.x,z:this.goal.z},this.visualPosition={...this.goal.surface},this.height=this.goal.height,this.nextEvent=this.interval()}get activeId(){return this.phase==="active"?this.current?.eventId??null:null}interval(){const[t,e]=Jt.critical.intervals[this.settings.criticalFrequency];return this.settings.criticalFrequency?t+this.random()*(e-t):1/0}configure(t,e){const i=t.criticalFrequency!==this.settings.criticalFrequency;this.settings={...t},i&&(t.criticalFrequency||(this.interrupt(e),this.phase="rest",this.position={x:this.goal.x,z:this.goal.z},this.height=0,this.impulseRemaining=0),this.nextEvent=e+this.interval())}pickRest(t){const e=Sn.filter(i=>Tt(i,t)>Jt.critical.restDistance);return(e.length?e:Sn)[Math.floor(this.random()*(e.length||Sn.length))]}move(t,e){const i=this.path[this.pathIndex];if(!i)return!0;const n=Tt(this.position,i),r=e*t;return this.heading=Math.atan2(i.z-this.position.z,i.x-this.position.x),n<=r?(this.position={x:i.x,z:i.z},this.pathIndex++):(this.position.x+=(i.x-this.position.x)/n*r,this.position.z+=(i.z-this.position.z)/n*r),this.pathIndex>=this.path.length}navigate(t){this.path=this.grid.path(this.position,t),this.pathIndex=0}begin(t){this.launch={...this.visualPosition},this.launchHeight=Tt(this.visualPosition,this.goal.surface)<.1?this.goal.height:0,this.phase="emerging",this.phaseStart=t.time,this.lastPlan=-1,this.hasEmerged=!0,this.current={eventId:`${this.trialId}-${this.records.length+1}`,mode:t.mode,eventTimestamp:t.time,hazardOnsetTimestamp:null,humanInterventionTimestamp:null,reactionTimeMs:null,playerIntervened:!1,automationAttemptedAvoidance:!1,autonomousCapableDraw:null,automationSucceeded:null,humanInterventionSucceeded:null,nearMiss:!1,collision:!1,disturbanceBefore:t.disturbance,disturbanceAfter:t.disturbance,playerAuthorityAtOnset:null,roombaAuthorityAtOnset:null,dynamicForceFeedbackEnabled:this.settings.dynamicForceFeedbackEnabled,minimumSeparation:null,outcome:"approaching",resolvedTimestamp:null,source:Tt(this.position,this.goal)<.8?this.goal.source:"floor_roam",randomDraw:null,configuredAutoSuccessProbability:this.settings.fullAutoCriticalAvoidSuccessProbability,automationAvoidanceTimestamp:null,humanEvasionApplied:!1},this.records.push(this.current),this.signals.push({type:"critical_event_start"})}activate(t){const e=this.current;this.phase="active",this.phaseStart=t.time,this.autoTriggered=!1;const i=(this.position.x-t.position.x)*Math.cos(t.heading)+(this.position.z-t.position.z)*Math.sin(t.heading);this.crossingTarget={x:t.position.x+Math.cos(t.heading)*i,z:t.position.z+Math.sin(t.heading)*i},e.dynamicForceFeedbackEnabled=this.settings.dynamicForceFeedbackEnabled,e.configuredAutoSuccessProbability=this.settings.fullAutoCriticalAvoidSuccessProbability,e.hazardOnsetTimestamp=t.time,e.disturbanceBefore=t.disturbance,e.playerAuthorityAtOnset=t.playerAuthority,e.roombaAuthorityAtOnset=1-t.playerAuthority,e.outcome="active",this.signals.push({type:"critical_hazard"}),t.mode==="automation"&&(e.automationAttemptedAvoidance=!0,e.randomDraw=this.random(),e.autonomousCapableDraw=e.randomDraw<this.settings.fullAutoCriticalAvoidSuccessProbability,this.signals.push({type:"autonomous_avoidance_attempt",value:e.autonomousCapableDraw?1:0}))}evade(t,e){const i=Jt.emergency;this.impulseDirection={x:-Math.sin(t.heading)*e,z:Math.cos(t.heading)*e},this.impulseRemaining=i.evasiveDuration,this.headingRemaining=i.clearanceHoldDuration,this.motionHeading=t.heading}safeSign(t){for(const e of[1,-1]){let i=!0;for(let n=Jt.emergency.sweepTimeStep;n<=Jt.emergency.clearanceHoldDuration;n+=Jt.emergency.sweepTimeStep){const r={x:t.position.x+Math.cos(t.heading)*t.speed*n-Math.sin(t.heading)*e*Jt.emergency.evasiveImpulse*Math.min(n,Jt.emergency.evasiveDuration),z:t.position.z+Math.sin(t.heading)*t.speed*n+Math.cos(t.heading)*e*Jt.emergency.evasiveImpulse*Math.min(n,Jt.emergency.evasiveDuration)};if(ti(r,Ee)){i=!1;break}}if(i)return e}return 0}emergency(t,e){const i=this.current,n=Jt.emergency;if(!this.activeId||!i||i.playerIntervened||i.hazardOnsetTimestamp===null)return!1;const r=t.x*-Math.sin(e.heading)+t.z*Math.cos(e.heading);return Math.abs(r)<n.lateralAlignmentThreshold?!1:(i.playerIntervened=!0,i.humanInterventionTimestamp=e.time,i.reactionTimeMs=Math.max(0,(e.time-i.hazardOnsetTimestamp)*1e3),i.humanEvasionApplied=e.time-i.hazardOnsetTimestamp<=n.interventionSuccessWindow,i.humanEvasionApplied&&this.evade(e,Math.sign(r)),this.signals.push({type:i.humanEvasionApplied?"emergency_swipe":"emergency_late",value:i.reactionTimeMs}),!0)}update(t,e){const i=Jt.critical;if(this.height=0,this.headingRemaining=Math.max(0,this.headingRemaining-t),this.headingRemaining||(this.motionHeading=null),this.settings.criticalFrequency===0)return this.visualPosition={...this.goal.surface},this.height=this.goal.height,{x:0,z:0};if(this.phase==="rest"&&e.time>=this.nextEvent?this.begin(e):this.phase==="rest"&&this.hasEmerged&&e.time>=this.nextRoam&&e.time<this.nextEvent-4&&(this.goal=this.pickRest(e.position),this.navigate(this.goal),this.phase="roam"),this.phase==="roam"&&(Tt(this.position,e.position)>2.5&&this.move(t,i.roamSpeed),e.time>=this.nextEvent?this.begin(e):this.pathIndex>=this.path.length&&(this.phase="rest",this.nextRoam=e.time+4+this.random()*4)),this.phase==="emerging"){if(this.height=Math.max(0,Math.sin(Math.min(1,(e.time-this.phaseStart)/i.jumpDuration)*Math.PI))*i.jumpHeight,e.time-this.lastPlan>i.replanInterval){this.lastPlan=e.time;const c={x:e.position.x+Math.cos(e.heading)*i.lookAhead,z:e.position.z+Math.sin(e.heading)*i.lookAhead};ti(c,.45)||this.navigate(c)}this.move(t,i.approachSpeed);const n=this.position.x-e.position.x,r=this.position.z-e.position.z,a=n*Math.cos(e.heading)+r*Math.sin(e.heading),o=Math.abs(-n*Math.sin(e.heading)+r*Math.cos(e.heading));e.time-this.phaseStart>=i.jumpDuration&&a>=i.onsetMinDistance&&a<=i.onsetMaxDistance&&o<i.corridorHalfWidth&&this.grid.clearLine(e.position,this.position,.1)&&this.grid.clearLine(e.position,{x:e.position.x+Math.cos(e.heading)*i.straightCorridorLength,z:e.position.z+Math.sin(e.heading)*i.straightCorridorLength},Ee)&&this.safeSign(e)!==0?this.activate(e):e.time-this.phaseStart>i.approachTimeout&&this.resolve(e,"irrelevant")}if(this.phase==="active"){const n=this.current,r=e.time-n.hazardOnsetTimestamp;if(n.autonomousCapableDraw&&!this.autoTriggered&&r>=i.autonomousReactionDelay){this.autoTriggered=!0;const a=this.safeSign(e);a&&(n.automationAvoidanceTimestamp=e.time,this.evade(e,a))}if(this.crossingTarget&&r<i.hazardHold){const a=Tt(this.position,this.crossingTarget),o=Math.min(a,i.approachSpeed*t);a>.001&&(this.position.x+=(this.crossingTarget.x-this.position.x)/a*o,this.position.z+=(this.crossingTarget.z-this.position.z)/a*o)}if(r>i.hazardHold){const a=-Math.sin(e.heading),o=Math.cos(e.heading),c={x:this.position.x+a*i.approachSpeed*t,z:this.position.z+o*i.approachSpeed*t};ti(c,i.radius)||(this.position=c)}r>i.hazardDuration&&this.resolve(e,"avoided")}if(this.phase==="flee"&&(this.move(t,i.fleeSpeed),this.pathIndex>=this.path.length&&(this.phase="rest",this.nextRoam=Math.max(e.time+2,this.phaseStart+i.restAfterImpact))),this.visualPosition={...this.position},this.phase==="rest")this.visualPosition={...this.goal.surface},this.height=this.goal.height;else if(this.phase==="emerging"&&e.time-this.phaseStart<i.jumpDuration){const n=zt((e.time-this.phaseStart)/i.jumpDuration,0,1);this.visualPosition={x:this.launch.x+(this.position.x-this.launch.x)*n,z:this.launch.z+(this.position.z-this.launch.z)*n},this.height+=(1-n)*this.launchHeight}if(this.impulseRemaining>0){const n=this.impulseRemaining;this.impulseRemaining=Math.max(0,n-t);const r=Math.min(1,n/Jt.emergency.evasiveTailDuration),a=Jt.emergency.evasiveImpulse*Math.min(t,n)*r;return{x:this.impulseDirection.x*a,z:this.impulseDirection.z*a}}return{x:0,z:0}}afterMovement(t,e){if(this.phase!=="active"||!this.current)return null;const i=t.position.x-e.x,n=t.position.z-e.z,r=zt(((this.position.x-e.x)*i+(this.position.z-e.z)*n)/(i*i+n*n||1),0,1),a=Tt(this.position,{x:e.x+i*r,z:e.z+n*r}),o=this.current;return o.minimumSeparation=Math.min(o.minimumSeparation??1/0,a),a<Ee+Jt.critical.radius?(this.resolve(t,"collision"),"collision"):null}resolve(t,e){const i=this.current;if(!i||i.resolvedTimestamp!==null)return;const n=Jt.critical;i.collision=e==="collision",i.nearMiss=!i.collision&&e!=="irrelevant"&&(i.minimumSeparation??1/0)<Ee+n.radius+n.nearMissClearance,i.automationSucceeded=i.automationAttemptedAvoidance?i.playerIntervened&&!i.collision?null:!!(i.autonomousCapableDraw&&i.automationAvoidanceTimestamp!==null&&!i.collision):null;const r=i.humanEvasionApplied&&i.automationAvoidanceTimestamp!==null;i.humanInterventionSucceeded=i.playerIntervened?r&&!i.collision?null:!i.collision&&i.humanEvasionApplied:null,i.outcome=i.collision?"collision":r?"combined_avoidance":i.automationSucceeded?"autonomous_success":i.humanInterventionSucceeded?"human_rescue":e,i.resolvedTimestamp=t.time,i.disturbanceAfter=t.disturbance+(i.collision?n.collisionDisturbance:i.nearMiss||i.outcome==="autonomous_success"||i.outcome==="human_rescue"||i.outcome==="combined_avoidance"?n.nearMissDisturbance:0),i.disturbanceAfter=Math.min(100,i.disturbanceAfter),this.signals.push({type:i.collision?"black_cat_collision":i.outcome==="irrelevant"?"critical_irrelevant":"critical_resolved",value:i.disturbanceAfter-t.disturbance}),this.goal=this.pickRest(t.position),this.navigate(this.goal),this.phase="flee",this.phaseStart=t.time,this.nextEvent=t.time+this.interval(),this.nextRoam=t.time+n.restAfterImpact}animateExit(t){this.phase==="flee"&&(this.move(t,Jt.critical.fleeSpeed),this.visualPosition={...this.position},this.height=0,this.pathIndex>=this.path.length&&(this.phase="rest",this.visualPosition={...this.goal.surface},this.height=this.goal.height))}interrupt(t){this.current&&this.current.resolvedTimestamp===null&&(this.current.outcome="interrupted",this.current.resolvedTimestamp=t),this.impulseRemaining=0,this.headingRemaining=0,this.motionHeading=null}drain(){const t=this.signals;return this.signals=[],t}}function Mu(s,t=Ii){let e=Math.min(dn/2-Math.abs(s.x),fn/2-Math.abs(s.z))-Ee;for(const i of t)e=Math.min(e,Math.hypot(Math.max(0,Math.abs(s.x-i.x)-i.w/2),Math.max(0,Math.abs(s.z-i.z)-i.d/2))-Ee);return Math.max(0,e)}function Su(s,t,e,i=ti){const n=Jt.force;let r=null;for(let c=n.sampleStep;c<=n.detectionRange;c+=n.sampleStep)if(i({x:s.x+Math.cos(t)*c,z:s.z+Math.sin(t)*c},Ee)){r=c;break}const a=r!==null?r/Math.max(.01,e):null,o=r===null?0:Math.pow(1-zt(r/n.detectionRange,0,1),n.proximityExponent);return{distance:r,ttc:a,risk:zt(Math.max(o,a!==null&&a<n.imminentCollisionThreshold?.95:0)*zt(e/We,0,1),0,1)}}function bu(s){const t=Jt.force;return t.authorityMinimum+(t.authorityMaximum-t.authorityMinimum)*Math.pow(1-zt(s,0,1),t.authorityExponent)}class Eu{constructor(){y(this,"force",0);y(this,"minimumClearance",1/0)}update(t,e,i,n,r,a,o=Ii,c=ti){if(this.minimumClearance=Math.min(this.minimumClearance,Mu(e,o)),!a)return this.force=0,null;const l=Jt.force,h=Su(e,i,n,c),u=h.risk?zt(h.risk*bu(r),l.minimumForce,l.maximumForce):0;return this.force=xe(this.force,u,l.smoothingRate,t),{obstacleDistance:h.distance,obstacleRisk:h.risk,generatedDynamicForce:this.force,minimumObstacleClearance:this.minimumClearance,timeToCollision:h.ttc}}}const Pi={range:3.5,near:1,maxSystemShare:.9,smoothingRate:8,sampleStep:.12,sideRayAngle:.3,sideRayWeight:1.25};function za(s,t,e,i=Pi.range){const{sampleStep:n}=Pi,r=Math.cos(t),a=Math.sin(t);for(let o=n;o<=i;o+=n)if(e({x:s.x+r*o,z:s.z+a*o},Ee))return o;return null}function wu(s,t,e){const{sideRayAngle:i,sideRayWeight:n}=Pi;let r=za(s,t,e);for(const a of[-i,i]){const o=za(s,t+a,e);if(o!==null){const c=o*n;(r===null||c<r)&&(r=c)}}return r!==null&&r<=Pi.range?r:null}function Tu(s){if(s===null)return 0;const{range:t,near:e,maxSystemShare:i}=Pi,n=zt((t-s)/(t-e),0,1);return i*n*n*(3-2*n)}function Au(s,t){return Math.min(s,1-zt(t,0,Pi.maxSystemShare))}const Ru=[0,.35,-.35,.7,-.7,1.05,-1.05,1.4,-1.4,1.75,-1.75];function Cu(s,t,e,i){let n=je(t),r=-1/0;for(const a of Ru){const o=je(t+a),l=(za(s,t+a,i)??Pi.range*1.5)+li(o,e)*.6-Math.abs(a)*.25;l>r&&(r=l,n=o)}return n}const Zn={idleSeconds:2.5,offRouteDistance:2.5,replanCooldown:1.5,lookAhead:3.6,sampleStep:.6,anchorStep:1.2};function qr(s,t,e){const i=new Set,n=Kn-.05;for(let r=0;r<=Zn.lookAhead;r+=Zn.sampleStep){const a=s.poseAt(t+r),o=e.index(a),c=o%e.cols,l=Math.floor(o/e.cols),h=Math.ceil(n/Je);for(let u=Math.max(0,l-h);u<=Math.min(e.rows-1,l+h);u++)for(let d=Math.max(0,c-h);d<=Math.min(e.cols-1,c+h);d++){const f=u*e.cols+d,g=e.cells[f];g.eligible&&!g.clean&&Tt(g,a)<=n&&i.add(f)}}return i.size}function Pu(s,t,e=[],i){const n=[];for(let h=0;h<t.points.length-1;h++){const u=t.points[h],d=t.points[h+1],f=d.x-u.x,g=d.z-u.z,v=Tt(u,d),m=zt(((s.x-u.x)*f+(s.z-u.z)*g)/(v*v||1),0,1),p=t.lengths[h]+m*v;n.push({s:p,distance:Tt(s,t.poseAt(p))})}if(i){n.length=0;for(let h=0;h<t.total;h+=Zn.anchorStep)qr(t,h,i)>0&&n.push({s:h,distance:Tt(s,t.poseAt(h))})}n.sort((h,u)=>h.distance-u.distance);const r=[];for(const h of n)if(r.every(u=>Math.abs(h.s-u)>2.8)&&r.push(h.s),r.length===7)break;const a=[];for(const h of r)for(const u of[1.2,2.8,4.5,0]){const d=(h+u)%t.total,f=t.poseAt(d),g=Tt(s,f);if(i&&qr(t,d,i)===0||e.some(p=>Tt(p,f)<2)||a.some(p=>Math.abs(p.s-d)<.5))continue;const v=Bo(s,f,1.25,i?.blocked),m=ai(v);v.length&&m<Math.max(11,g*2+4)&&a.push({s:d,point:f,path:v,score:m+g*.25})}if(!a.length)for(const h of r.slice(0,3)){const u=(h+2.8)%t.total,d=t.poseAt(u);if(i&&qr(t,u,i)===0||e.some(g=>Tt(g,d)<2))continue;const f=Wn(s,d,d.heading,i?.blocked);f.length&&a.push({s:u,point:d,path:f,score:ai(f)+Tt(s,d)*.25})}a.sort((h,u)=>h.score-u.score);const o=a[0];if(!o)return null;const c=ai(o.path),l=[...o.path];for(let h=.12;h<=3.6;h+=.12)l.push(t.poseAt(o.s+h));return{path:l,progress:o.s,mergeDistance:c,point:o.point}}function wl(s,t,e){let i=null,n=.65;for(let r=-.25;r<=3.8;r+=.1){const a=t.poseAt(e+r),o=Tt(s,a);o<n&&Math.abs(Ci(s.heading-a.heading))<.5&&(n=o,i=(e+r+t.total)%t.total)}return i}class Iu{constructor(){y(this,"timer",0);y(this,"direction",null)}reset(){this.timer=0,this.direction=null}update(t,e,i,n){return i>.35&&li(t,e)<Math.cos(.2)?(this.direction&&li(t,this.direction)<Math.cos(.45)&&this.reset(),this.direction||(this.direction={...t}),this.timer=zt(this.timer+n,0,.8)):(this.timer=Math.max(0,this.timer-n*2.5),this.timer===0&&(this.direction=null)),this.timer>=.8-1e-9}}const Du=[60,75,90];function jc(s){return Du.filter(t=>s>=t).length}class Lu{constructor(){y(this,"enabled",!0);y(this,"value",0);y(this,"peak",0);y(this,"smoothTime",0);y(this,"previousTurn",0);y(this,"lastSign",0);y(this,"lastReversal",-10);y(this,"roughness",0);y(this,"oscillation",0);y(this,"lateralAcceleration",0);y(this,"lean",0);y(this,"sway",0);y(this,"swayVelocity",0);y(this,"startle",0)}update(t,e,i,n,r){const a=typeof r=="boolean"?r?3.15:0:r,o=(e-this.previousTurn)/t,c=Math.abs(o);this.previousTurn=e,this.lateralAcceleration=e*a,this.lean=xe(this.lean,zt(this.lateralAcceleration*.019,-.25,.25),7,t),this.swayVelocity+=(-this.sway*48-this.swayVelocity*8-o*.13)*t,this.sway+=this.swayVelocity*t,this.sway=zt(this.sway,-.36,.36),this.startle=Math.max(0,this.startle-t*2.4);const l=Math.abs(e)>1.35?Math.sign(e):0;if(l&&this.lastSign&&l!==this.lastSign){const d=n-this.lastReversal<1.4;this.oscillation=d?this.oscillation+1:0,d&&this.add(1.8+Math.min(5,this.oscillation)),this.lastReversal=n}l&&(this.lastSign=l);const u=a>.2&&(Math.abs(e)>3.25||c>19)?(Math.max(0,Math.abs(e)-3.25)*2+Math.max(0,c-19)*.22)*t:0;u>0?(this.value+=u,this.smoothTime=0):this.smoothTime+=t,this.smoothTime>2.4&&(this.value-=t*(1.8+this.value*.035)),this.value=this.enabled?zt(this.value,0,100):0,this.peak=Math.max(this.peak,this.value),this.roughness+=(Math.max(0,c-19)*.035+Math.max(0,Math.abs(e)-3.25)*.22)*t}impact(t){this.add(8+t*3.4),this.startle=1,this.swayVelocity+=2.8}add(t){this.enabled&&(this.value=zt(this.value+t,0,100),this.peak=Math.max(this.value,this.peak),this.smoothTime=0)}get label(){return["DEEP SLEEP","LIGHT SLEEP","STIRRING","AWAKE","ANGRY","CAT WOKE UP"][Math.min(5,Math.floor(this.value/20))]}}const Ho=.9;function Dr(s){return typeof s=="number"&&Number.isFinite(s)?zt(s,.1,1):Ho}function Tr(s){return Math.min(1,Dr(s)+.25)}const Go=.3;function Ba(s){return typeof s=="number"&&Number.isFinite(s)?zt(s,.1,3):Go}const Vo=1;function Ha(s){return typeof s=="number"&&Number.isFinite(s)?zt(s,.5,5):Vo}const Nu=Math.PI*65/180,Uu=.8,Ou=.25,Li=["normal","player_deviation","conflict","conflict_cancelled"];class Tl{constructor(t,e,i={}){y(this,"experiment");y(this,"initialExperiment");y(this,"critical");y(this,"dynamicForce",new Eu);y(this,"forceSample",null);y(this,"startedAt",new Date().toISOString());y(this,"scenario");y(this,"mode");y(this,"initialMode");y(this,"level");y(this,"takeoverTimer",0);y(this,"automationDisengaged",!1);y(this,"tourWeightOnly",!1);y(this,"tourInteractionForce",0);y(this,"obstacleArbitration",0);y(this,"collisionRunUp",0);y(this,"highlightedObstacle",null);y(this,"freeTravel",0);y(this,"inclusionReplanAt",-10);y(this,"inclusionSteering",!1);y(this,"avoidDirection",null);y(this,"seed");y(this,"flow");y(this,"interventionShare",Ho);y(this,"inclusionDelay",Go);y(this,"takeoverHold",Vo);y(this,"inclusionTimer",0);y(this,"inclusionLocked",!1);y(this,"overrideTimer",0);y(this,"inclusionCandidate",null);y(this,"magnetRemaining",0);y(this,"magnetDust",[]);y(this,"lastMagnetSpawn",-100);y(this,"bonusRandom");y(this,"watchedManeuver",null);y(this,"watchedProgress",0);y(this,"progressTime",0);y(this,"lastRepair",-10);y(this,"overrideDirection",null);y(this,"overrideControl",!1);y(this,"bonusVisible",()=>!0);y(this,"position");y(this,"heading",0);y(this,"speed",We);y(this,"turnRate",0);y(this,"time",0);y(this,"status","playing");y(this,"steering",{driverIntent:0,systemIntent:0,finalSteering:0,systemAuthority:0,driverAuthority:1,intentTimer:0,inputMagnitude:0,playerDirection:je(0),systemDirection:je(0),sharedDirection:je(0)});y(this,"lastFloorClean",0);y(this,"lastCleaningPlan",-10);y(this,"authorityShare",.35);y(this,"routeSample",{direction:je(0),distance:.85,error:0,steer:0});y(this,"state","normal");y(this,"stateTime",0);y(this,"detector",new Iu);y(this,"cat",new Lu);y(this,"navigator");y(this,"routeIndex",0);y(this,"routeFollower");y(this,"plannedPath",[]);y(this,"trail",[]);y(this,"maneuver",null);y(this,"departure",0);y(this,"rejoin",0);y(this,"mergeDistance",0);y(this,"returnStarted",0);y(this,"returnRepairs",0);y(this,"returnRewards",!0);y(this,"failedMerges",[]);y(this,"events",[]);y(this,"telemetry",[]);y(this,"eventHistory",[]);y(this,"telemetryClock",0);y(this,"lastCollision",-10);y(this,"stuckTime",0);y(this,"activeInput",0);y(this,"collisionRecovery",0);y(this,"targetBonus",null);y(this,"candidateBonus",null);y(this,"returnPoint",null);y(this,"acceptedHeading",0);y(this,"maneuverCollision",0);y(this,"combo",0);y(this,"comboGrace",0);y(this,"spawned",new Set);y(this,"metrics",{score:0,cleaningProgress:0,cleaningTime:0,bonusesCollected:0,collisions:0,peakCatDisturbance:0,smoothness:100,playerInvolvement:0,intentsDetected:0,successfulYields:0,syncEvents:0,smoothReturns:0,completed:!1});this.bonusRandom=zo(e^1540483477),this.mode=t,this.initialMode=t,this.level=i.level??"level1",this.seed=e,this.flow=i.flow??"free_play",this.interventionShare=Dr(i.interventionShare),this.inclusionDelay=Ba(i.inclusionDelay),this.takeoverHold=Ha(i.takeoverHold);const n=Fa(this.level);this.scenario=new fu(e,n.obstacles,n.blackCat),this.level==="tour"&&(this.scenario.start={x:-7,z:2.7},this.scenario.bonuses=[{x:0,z:0,id:1e3,kind:"red",spawn:1/0,duration:Hi.bonusDuration,value:250,collected:!1,expired:!1},{x:0,z:0,id:1001,kind:"gold",spawn:1/0,duration:Hi.bonusDuration,value:1500,collected:!1,expired:!1}]),this.experiment=qn({...i.experiment,...n.blackCat?{}:{criticalFrequency:0}}),this.initialExperiment={...this.experiment},this.critical=new yu(this.experiment,this.scenario.grid,i.criticalRandom),this.position={...this.scenario.start},this.routeFollower=new Di(this.scenario.route,!0),this.navigator=this.routeFollower,this.level==="tour"&&(this.reanchorRoute(),this.cat.enabled=!1),this.steering.systemAuthority=t==="manual"?0:t==="automation"?1:.65,this.steering.driverAuthority=1-this.steering.systemAuthority,this.scenario.grid.cleanAt(this.position)}setControlMode(t){this.mode!==t&&(this.steering.systemAuthority=t==="manual"?0:t==="automation"?1:.65,this.steering.driverAuthority=1-this.steering.systemAuthority,this.inclusionLocked=!1,this.targetBonus=null,this.candidateBonus=null,this.maneuver=null,this.plannedPath=[],this.detector.reset(),this.overrideControl=!1,this.inclusionTimer=0,this.inclusionCandidate=null,this.overrideTimer=0,this.mode=t,this.takeoverTimer=0,this.transition("normal"),this.reanchorRoute())}reactivateAutomation(){!this.automationDisengaged||this.status!=="playing"||(this.setControlMode("automation"),this.automationDisengaged=!1,this.beginReturn(!0),this.emit("automation_reactivated"))}revealTourBonus(t){if(this.level!=="tour")return!1;const e=this.scenario.bonuses.find(r=>r.kind===t),i={...this.position,heading:this.heading},n=t==="gold"?Ml(i,1,void 0,void 0,this.scenario.grid.blocked,this.bonusRandom):xl(i,-1,10,this.scenario.grid.blocked,this.bonusRandom);return n?(Object.assign(e,n,{spawn:this.time,duration:Hi.bonusDuration,collected:!1,expired:!1}),this.spawned.add(e.id),this.emit("bonus_spawn",e.id),!0):!1}hazardContext(){return{time:this.time,position:this.position,heading:this.heading,speed:this.speed,mode:this.mode,disturbance:this.cat.value,playerAuthority:this.steering.driverAuthority}}setExperiment(t){this.experiment=qn({...this.experiment,...t,...this.scenario.showBlackCat?{}:{criticalFrequency:0}}),this.critical.configure(this.experiment,this.time),this.emit("experiment_settings_changed"),this.eventHistory[this.eventHistory.length-1].settings={...this.experiment}}emergencySwipe(t){return this.status==="playing"&&this.critical.emergency(t,this.hazardContext())}get hasAcceptedRoute(){return!Li.includes(this.state)||this.inclusionLocked}collectMagnet(){return this.status!=="playing"?!1:(this.magnetRemaining=10,this.emit("magnet_started"),!0)}reanchorRoute(){let t=1/0,e=0;const i=this.routeFollower;for(let n=0;n<i.points.length-1;n++){const r=i.points[n],a=i.points[n+1],o=a.x-r.x,c=a.z-r.z,l=Tt(r,a),h=zt(((this.position.x-r.x)*o+(this.position.z-r.z)*c)/(l*l||1),0,1),u=Math.hypot(this.position.x-r.x-o*h,this.position.z-r.z-c*h);u<t&&(t=u,e=i.lengths[n]+h*l)}i.seek(e),this.routeSample=i.update(this.position,this.heading),this.navigator=i}releaseRoute(){this.mode==="manual"||this.status!=="playing"||(this.inclusionLocked=!1,this.overrideTimer=0,this.overrideDirection=null,this.inclusionTimer=0,this.inclusionCandidate=null,this.targetBonus=null,this.candidateBonus=null,this.detector.reset(),this.maneuver=null,this.plannedPath=[],this.watchedManeuver=null,this.overrideControl=this.mode==="collaborative",this.lastRepair=this.time,this.reanchorRoute(),this.transition(this.mode==="automation"?"normal":"player_deviation"),this.emit("inclusion_overridden"))}checkNavigation(){if(!this.hasAcceptedRoute||this.state==="bonus_capture")return;if(this.targetBonus?.expired){this.emit("missed_bonus"),this.beginReturn();return}const t=this.maneuver;if(t!==this.watchedManeuver&&(this.watchedManeuver=t,this.watchedProgress=t?.progress??0,this.progressTime=this.time),t&&t.progress>this.watchedProgress+.08&&(this.watchedProgress=t.progress,this.progressTime=this.time),this.state==="return_to_path"&&(this.time-this.returnStarted>12||this.returnRepairs>=3)){this.returnPoint&&this.failedMerges.push({...this.returnPoint}),this.releaseRoute(),this.emit("route_unavailable");return}const e=this.state==="return_to_path"?t?.target:this.targetBonus;if(!(!t||t.done&&!!e&&Tt(this.position,e)>.75||!!t&&Tt(this.position,t.poseAt(t.progress))>1.7||this.time-this.progressTime>1.4||this.stuckTime>.3)||this.time-this.lastRepair<.75)return;this.lastRepair=this.time,this.state==="return_to_path"&&this.returnPoint&&this.failedMerges.push({...this.returnPoint});const n=this.state==="return_to_path"?(this.returnRepairs++,this.nearestReturn()):e?Wn({...this.position,heading:this.heading},e,void 0,this.scenario.grid.blocked):[];n.length?(this.plannedPath=n,this.maneuver=new Di(n),this.watchedManeuver=null,this.progressTime=this.time,this.emit("route_replanned")):this.state!=="inclusion"&&(this.releaseRoute(),this.emit("route_unavailable"))}setInterventionShare(t){const e=Dr(t);e!==this.interventionShare&&(this.interventionShare=e,this.emit("intervention_share_changed",e))}setTakeoverHold(t){this.takeoverHold=Ha(t)}setInclusionDelay(t){this.inclusionDelay=Ba(t),this.emit("inclusion_delay_changed",this.inclusionDelay)}emit(t,e){const i={type:t,time:this.time,value:e,x:this.position.x,z:this.position.z};this.events.push(i),this.eventHistory.push(i)}transition(t){this.state=t,this.stateTime=0,this.emit(t)}get activeBonuses(){return this.scenario.bonuses.filter(t=>this.time>=t.spawn&&this.time<t.spawn+t.duration&&!t.collected&&!t.expired)}get playerPath(){return gl(this.position,this.heading,this.steering.playerDirection,1.1,this.steering.inputMagnitude)}get activePath(){return gl(this.position,this.heading,this.steering.sharedDirection,1.5)}nextRoute(){this.routeFollower.seek(0),this.navigator=this.routeFollower}chooseBonus(t){let e=null,i=1/0;for(const n of this.activeBonuses){const r=Ti({x:n.x-this.position.x,z:n.z-this.position.z}),a=Tt(n,this.position),o=n.spawn+n.duration-this.time,c=li(t,r);if(c<Math.cos(.8)||a>(n.kind==="gold"?14:9)||o<.7)continue;const l=(1-c)*15+a*.15;l<i&&(e=n,i=l)}return e}acceptIntent(t){this.overrideControl=!1,this.overrideTimer=0,this.overrideDirection=null,this.metrics.intentsDetected++,this.metrics.score+=25,this.departure=this.routeFollower.progress,this.returnPoint=this.routeFollower.poseAt(this.departure),this.targetBonus=this.candidateBonus??this.chooseBonus(t),this.candidateBonus=null,this.acceptedHeading=Yc(t),this.maneuverCollision=this.metrics.collisions;const e=this.targetBonus??{x:this.position.x+t.x*5,z:this.position.z+t.z*5};this.plannedPath=this.targetBonus?.kind==="gold"||this.targetBonus?.kind==="red"?Wn({...this.position,heading:this.heading},e,void 0,this.scenario.grid.blocked):qc({...this.position,heading:this.heading},e,this.acceptedHeading,this.scenario.grid.blocked),this.maneuver=this.plannedPath.length?new Di(this.plannedPath):null,this.transition("intent_detected")}lockInclusion(){if(!this.targetBonus||this.targetBonus.kind!=="gold"||this.targetBonus.expired||this.targetBonus.collected)return;const t=Wn({...this.position,heading:this.heading},this.targetBonus,void 0,this.scenario.grid.blocked);if(!t.length){this.emit("inclusion_unreachable"),this.beginReturn();return}this.state==="intent_detected"&&(this.metrics.successfulYields++,this.metrics.score+=25,this.emit("yielding")),this.inclusionLocked=!0,this.overrideTimer=0,this.plannedPath=t,this.maneuver=new Di(t),["sync","target_bonus"].includes(this.state)||(this.metrics.syncEvents++,this.metrics.score+=100,this.emit("sync")),this.transition("inclusion")}updateInclusionPath(t){const e=t>.12,i=this.inclusionSteering&&!e;if(this.inclusionSteering=e,!i&&!(e&&this.time-this.inclusionReplanAt>=Ou))return;this.inclusionReplanAt=this.time;let n=Wn({...this.position,heading:this.heading},this.targetBonus,void 0,this.scenario.grid.blocked);n.length||(n=this.spliceInclusionPath()),n.length&&(this.plannedPath=n,this.maneuver=new Di(n),this.watchedManeuver=null,this.progressTime=this.time)}spliceInclusionPath(){const t=this.plannedPath;if(t.length<2)return[];let e=0;for(let n=1;n<t.length;n++)Tt(t[n],this.position)<Tt(t[e],this.position)&&(e=n);let i=-1;for(let n=e;n<t.length&&!(this.scenario.grid.clearLine(this.position,t[n],Ee)&&((i<0||Tt(t[n],this.position)<Tt(t[i],this.position))&&(i=n),Tt(t[n],this.position)>3));n++);return i<0?[]:[{...this.position},...t.slice(i)]}nearestReturn(){const t=new Di(this.scenario.route,!0),e=Pu({...this.position,heading:this.heading},t,this.failedMerges,this.scenario.grid);return e?(this.routeFollower=t,this.rejoin=e.progress,this.mergeDistance=e.mergeDistance,this.returnPoint=e.point,e.path):[]}beginReturn(t=!1){this.returnRewards=!t;const e=this.inclusionLocked||this.targetBonus?.kind==="gold";this.detector.reset(),this.targetBonus=null,this.inclusionTimer=0,this.inclusionCandidate=null,this.returnStarted=this.time,this.returnRepairs=0,this.failedMerges=[],this.transition("return_to_path");const i=this.nearestReturn();this.plannedPath=i,this.maneuver=i.length?new Di(i):null,this.inclusionLocked=e&&!!this.maneuver,this.navigator=this.maneuver??this.routeFollower,this.maneuver||(this.releaseRoute(),this.emit("route_unavailable"))}finishReturn(t){this.mode==="collaborative"&&this.returnRewards&&this.metrics.collisions===this.maneuverCollision&&(this.metrics.smoothReturns++,this.metrics.score+=100,this.combo++,this.emit("perfect_return")),this.failedMerges=[],this.routeFollower.seek(t),this.routeSample=this.routeFollower.update(this.position,this.heading),this.inclusionLocked=!1,this.overrideTimer=0,this.maneuver=null,this.plannedPath=[],this.navigator=this.routeFollower,this.lastFloorClean=this.time,this.transition("normal")}seekRemainingFloor(t){this.mode==="manual"||!Li.includes(this.state)||t>.12||this.critical.phase==="active"||this.time-this.lastCleaningPlan<Zn.replanCooldown||!(Tt(this.position,this.routeFollower.poseAt(this.routeFollower.progress))>Zn.offRouteDistance)&&this.time-this.lastFloorClean<Zn.idleSeconds||(this.lastCleaningPlan=this.time,this.maneuverCollision=this.metrics.collisions,this.beginReturn(!0),this.maneuver&&this.emit("cleaning_repath"))}automationNavigation(t){if(this.state!=="return_to_path")return this.routeSample;if(this.stateTime+=t,this.checkNavigation(),this.state!=="return_to_path"||!this.maneuver)return this.routeSample;const e=this.maneuver.update(this.position,this.heading);this.navigator=this.maneuver;const i=this.maneuver.progress>=this.mergeDistance-.45?wl({...this.position,heading:this.heading},this.routeFollower,this.rejoin):null;return i!==null?(this.finishReturn(i),this.routeSample):e}collaboration(t,e,i,n){if(this.tourWeightOnly)return this.authorityShare=xe(this.authorityShare,this.interventionShare,7,t),this.steering.intentTimer=0,this.state==="return_to_path"?this.automationNavigation(t):n;this.stateTime+=t,this.checkNavigation();let r=.35,a=n.direction,o=n.distance;if((Li.includes(this.state)&&i>.12||this.state==="intent_detected")&&(r=this.interventionShare,this.state==="normal"&&(this.authorityShare=r)),!this.inclusionLocked){const l=this.targetBonus?.kind==="gold"?this.targetBonus:this.chooseBonus(e);l?.kind==="gold"&&!l.collected&&!l.expired&&i>.35&&li(e,Ti({x:l.x-this.position.x,z:l.z-this.position.z}))>Math.cos(.65)?(this.inclusionCandidate!==l.id&&(this.inclusionCandidate=l.id,this.inclusionTimer=0),this.inclusionTimer+=t):(this.inclusionTimer=0,this.inclusionCandidate=null)}Li.includes(this.state)&&(this.candidateBonus&&(this.candidateBonus.expired||li(e,Ti({x:this.candidateBonus.x-this.position.x,z:this.candidateBonus.z-this.position.z}))<Math.cos(.9))&&(this.candidateBonus=null),i>.35&&!this.candidateBonus&&(this.candidateBonus=this.chooseBonus(e)),i<.15&&(this.candidateBonus=null),this.detector.update(e,n.direction,i,t)?this.acceptIntent(e):this.detector.timer>=.22&&this.state!=="conflict"?this.transition("conflict"):i>.12&&this.state==="normal"?this.transition("player_deviation"):this.detector.timer===0&&i<.12&&["conflict","player_deviation"].includes(this.state)?this.transition("conflict_cancelled"):this.state==="conflict_cancelled"&&this.stateTime>.45&&this.transition("normal")),Li.includes(this.state)&&this.inclusionTimer+1e-9>=Math.min(.8,this.inclusionDelay)&&this.inclusionCandidate!==null&&this.acceptIntent(e),this.state==="intent_detected"&&this.stateTime>=.22&&(this.transition("yielding"),this.metrics.successfulYields++,this.metrics.score+=25),!this.inclusionLocked&&this.targetBonus?.kind==="gold"&&this.inclusionTimer+1e-9>=this.inclusionDelay&&["intent_detected","yielding","sync","target_bonus"].includes(this.state)&&this.lockInclusion(),this.state==="inclusion"&&this.targetBonus&&!this.targetBonus.expired&&this.updateInclusionPath(i);const c=this.maneuver?.update(this.position,this.heading)??{...n,direction:je(this.acceptedHeading)};if(this.state==="intent_detected"&&this.targetBonus?.kind==="gold"&&(a=c.direction,o=c.distance),this.state==="yielding"){r=Tr(this.interventionShare);const l=zt(this.stateTime/.5,0,1);a=$r(n.direction,c.direction,1-l,l,this.steering.systemDirection),o=n.distance*(1-l)+c.distance*l,this.stateTime>.55&&(i<.25||li(e,c.direction)>Math.cos(.65))&&(this.metrics.syncEvents++,this.metrics.score+=100,this.combo++,this.comboGrace=this.time+2,this.transition("sync")),this.stateTime>1.5&&this.beginReturn()}if((this.state==="sync"||this.state==="target_bonus")&&(r=.35,a=c.direction,o=c.distance,this.state==="sync"&&this.stateTime>.65&&this.transition("target_bonus"),this.targetBonus?.expired?(this.time>this.comboGrace&&(this.combo=0),this.emit("missed_bonus"),this.beginReturn()):(!this.maneuver||this.maneuver.done)&&this.beginReturn()),this.state==="inclusion"&&(r=i>.12?this.interventionShare:.35,a=c.direction,o=c.distance,this.targetBonus?.expired&&(this.emit("missed_bonus"),this.beginReturn())),this.state==="bonus_capture"&&(r=.35,a=c.direction,o=c.distance,this.stateTime>.22&&this.beginReturn()),this.state==="return_to_path"){const l=this.maneuver?.update(this.position,this.heading)??n;r=.35,a=l.direction,o=l.distance;const h=this.maneuver&&this.maneuver.progress>=this.mergeDistance-.45?wl({...this.position,heading:this.heading},this.routeFollower,this.rejoin):null;h!==null&&(this.finishReturn(h),a=this.routeSample.direction,o=this.routeSample.distance)}if(this.hasAcceptedRoute&&this.state!=="inclusion"){const l=this.targetBonus?Ti({x:this.targetBonus.x-this.position.x,z:this.targetBonus.z-this.position.z}):a;i>(this.inclusionLocked?.65:.35)&&li(e,l)<Math.cos(Nu)?(this.overrideDirection&&li(e,this.overrideDirection)<Math.cos(.45)&&(this.overrideTimer=0,this.overrideDirection=null),this.overrideDirection??(this.overrideDirection={...e}),this.overrideTimer+=t):(this.overrideTimer=0,this.overrideDirection=null),this.overrideTimer+1e-9>=Math.min(Uu,this.inclusionDelay)&&(this.releaseRoute(),a=this.routeSample.direction,o=this.routeSample.distance)}if(this.overrideControl&&(r=Math.max(.65,this.interventionShare),i<.12&&this.state==="normal"&&(this.overrideControl=!1)),!this.hasAcceptedRoute&&i<.12&&n.distance>2&&this.time-this.lastRepair>1){this.lastRepair=this.time;const l=this.nearestReturn();if(l.length){this.plannedPath=l,this.maneuver=new Di(l),this.returnStarted=this.time,this.returnRepairs=0,this.transition("return_to_path");const h=this.maneuver.update(this.position,this.heading);a=h.direction,o=h.distance}}return this.navigator=this.maneuver&&!Li.includes(this.state)?this.maneuver:this.routeFollower,this.authorityShare=this.overrideControl?r:xe(this.authorityShare,r,7,t),this.steering.intentTimer=this.detector.timer,{direction:a,distance:o}}step(t,e=null,i=t){if(this.status!=="playing")return;this.events=[],this.time+=t,this.magnetDust=[];const n=this.magnetRemaining>0;this.magnetRemaining=Math.max(0,this.magnetRemaining-t),n&&this.magnetRemaining<1e-8&&(this.magnetRemaining=0,this.emit("magnet_ended")),this.mode==="automation"&&(this.takeoverTimer=(e?.magnitude??0)>=pu.inputThreshold?this.takeoverTimer+i:0,this.takeoverTimer+1e-9>=this.takeoverHold&&(this.setControlMode("manual"),this.automationDisengaged=!0,this.emit("automation_takeover")));const r=this.mode==="automation"?0:zt(e?.magnitude??0,0,1),a=r>.001&&e?Ti(e.direction,je(this.heading)):je(this.heading);r>.12&&(this.activeInput+=t);for(const T of this.scenario.bonuses){if(this.time>=T.spawn&&!this.spawned.has(T.id)){if(this.level!=="tour"&&T.relativeSpawn){if(T.kind==="magnet"&&this.time<this.lastMagnetSpawn+24){T.spawn=this.lastMagnetSpawn+24;continue}const S=T.kind==="gold"?Ml({...this.position,heading:this.heading},T.id%2?1:-1,this.bonusVisible,this.scenario.start,this.scenario.grid.blocked,this.bonusRandom):xl({...this.position,heading:this.heading},T.id%2?1:-1,T.duration,this.scenario.grid.blocked,this.bonusRandom);if(!S){T.spawn=this.time+.35;continue}T.x=S.x,T.z=S.z,T.spawn=this.time}this.spawned.add(T.id),T.kind==="magnet"&&(this.lastMagnetSpawn=this.time),this.emit("bonus_spawn",T.id)}this.time>=T.spawn+T.duration&&!T.collected&&!T.expired&&(T.expired=!0,this.emit("bonus_expired",T.id))}this.seekRemainingFloor(r),Li.includes(this.state)&&(this.routeSample=this.routeFollower.update(this.position,this.heading)),this.routeFollower.done&&Li.includes(this.state)&&(this.nextRoute(),this.routeSample=this.routeFollower.update(this.position,this.heading)),this.routeIndex=this.routeFollower.index;const o=this.mode==="collaborative"?this.collaboration(t,a,r,this.routeSample):this.mode==="automation"?this.automationNavigation(t):this.routeSample,c=this.mode==="collaborative"&&r>.12,l=this.scenario.grid.blocked,h=c?Tu(wu(this.position,this.heading,l)):0;if(this.obstacleArbitration=xe(this.obstacleArbitration,h,Pi.smoothingRate,t),c&&this.obstacleArbitration>.01){const T=Cu(this.position,this.heading,o.direction,l);this.avoidDirection=this.avoidDirection?Ti({x:this.avoidDirection.x+(T.x-this.avoidDirection.x)*(1-Math.exp(-8*t)),z:this.avoidDirection.z+(T.z-this.avoidDirection.z)*(1-Math.exp(-8*t))},T):T}else this.avoidDirection=null;this.steering.driverAuthority=this.mode==="manual"?1:this.mode==="automation"?0:Au(this.authorityShare*r,this.obstacleArbitration),this.steering.systemAuthority=1-this.steering.driverAuthority;const u=this.avoidDirection?zt(this.obstacleArbitration/Pi.maxSystemShare,0,1):0,d=this.mode==="manual"?je(this.heading):u>0&&this.avoidDirection?$r(o.direction,this.avoidDirection,1-u,u,o.direction):o.direction,f=$r(a,d,this.steering.driverAuthority,this.steering.systemAuthority,je(this.heading)),g=As(a,this.heading,We,on)*r,v=this.mode==="manual"?0:As(d,this.heading,We,on,o.distance),m=this.mode==="manual"?g:As(f,this.heading,We,on,r>.001?Math.min(.85,o.distance):o.distance);Object.assign(this.steering,{driverIntent:g,systemIntent:v,finalSteering:m,inputMagnitude:r,playerDirection:a,systemDirection:d,sharedDirection:f}),this.turnRate=xe(this.turnRate,m*on,15,t),this.heading=Ci(this.heading+this.turnRate*t),this.critical.motionHeading!==null&&(this.heading=this.critical.motionHeading,this.turnRate=0),this.collisionRecovery=Math.max(0,this.collisionRecovery-t),this.speed=this.collisionRecovery>0?We*.35:We;const p=this.critical.update(t,this.hazardContext()),w=this.scenario.grid.blocked,E={...this.position},_={x:E.x+Math.cos(this.heading)*this.speed*t+p.x,z:E.z+Math.sin(this.heading)*this.speed*t+p.z};let P=!1;if(w(_,Ee)){P=!0,this.stuckTime+=t;const T=this.speed,S={x:_.x,z:E.z},b={x:E.x,z:_.z};if(w(S,Ee)?w(b,Ee)||(this.position=b):this.position=S,this.time-this.lastCollision>.65){this.lastCollision=this.time,this.collisionRunUp=this.freeTravel,this.freeTravel=0,this.collisionRecovery=.16;const L={x:E.x-Math.cos(this.heading)*.11,z:E.z-Math.sin(this.heading)*.11};w(L,Ee)||(this.position=L),this.metrics.collisions++,this.metrics.score=Math.max(0,this.metrics.score-100),this.cat.impact(T),T>1.8&&this.time>this.comboGrace&&(this.combo=0),this.emit("collision",T)}this.stuckTime>1.2&&this.cat.add(t*4)}else this.freeTravel+=Tt(E,_),this.position=_,this.stuckTime=0;if(this.critical.afterMovement(this.hazardContext(),E)){this.collisionRunUp=this.freeTravel,this.freeTravel=0,this.metrics.collisions++,this.metrics.score=Math.max(0,this.metrics.score-100),this.collisionRecovery=.16;const T={x:this.position.x-Math.cos(this.heading)*.18,z:this.position.z-Math.sin(this.heading)*.18};w(T,Ee)||(this.position=T),this.cat.startle=1,this.cat.swayVelocity+=3,P=!0}for(const T of this.critical.drain())["black_cat_collision","critical_resolved"].includes(T.type)&&this.cat.add(Math.max(0,T.value??0)),T.type==="emergency_swipe"&&(this.cat.startle=.65,this.cat.swayVelocity+=1.5),this.emit(T.type,T.value);this.forceSample=this.dynamicForce.update(t,this.position,this.heading,this.speed,this.steering.driverAuthority,this.mode==="collaborative"&&this.experiment.dynamicForceFeedbackEnabled,this.scenario.obstacles,this.scenario.grid.blocked),this.tourInteractionForce=xe(this.tourInteractionForce,this.tourWeightOnly?r*(1-li(a,d))*.5*(1-this.interventionShare):0,7,t),(!this.trail.length||Tt(this.trail[this.trail.length-1],this.position)>.12)&&(this.trail.push({...this.position}),this.trail.length>30&&this.trail.shift());const I=this.scenario.grid.cleanAt(this.position,n?3.2:Kn);n&&(this.magnetDust=this.scenario.grid.lastCleaned.filter(T=>Tt(T,this.position)>Kn)),this.metrics.score+=I*5,I&&(this.lastFloorClean=this.time);for(const T of this.activeBonuses)(this.scenario.grid.clearLine(E,this.position,.1)&&Fu(T,E,this.position)<.86||n&&Tt(T,this.position)<3.2&&this.scenario.grid.clearLine(this.position,T,.1))&&(T.collected=!0,T.kind==="magnet"?(this.collectMagnet(),this.emit("magnet_collection",T.id)):(this.metrics.bonusesCollected++,this.metrics.score+=T.value,this.emit("bonus_collection",T.id)),this.mode==="collaborative"&&this.targetBonus?.id===T.id&&(T.kind!=="magnet"&&this.metrics.collisions===this.maneuverCollision&&(this.metrics.score+=150,this.combo++,this.emit("smooth_bonus")),this.transition("bonus_capture")));this.cat.update(t,this.turnRate,g,this.time,this.speed),this.metrics.cleaningProgress=this.scenario.grid.progress,this.metrics.cleaningTime=this.time,this.metrics.peakCatDisturbance=this.cat.peak,this.metrics.smoothness=zt(100-this.cat.roughness*2-this.metrics.collisions*2,0,100),this.metrics.playerInvolvement=this.activeInput/this.time*100,this.level!=="tour"&&this.cat.value>=100?(this.status="failed",this.speed=0,this.emit("cat_woke")):this.level!=="tour"&&this.scenario.grid.cleaned===this.scenario.grid.total&&(this.status="success",this.speed=0,this.metrics.completed=!0,this.metrics.score+=Math.round((100-this.cat.peak)*5+this.metrics.smoothness*2),this.emit("complete")),this.critical.current?.resolvedTimestamp===this.time&&(this.critical.current.disturbanceAfter=this.cat.value),this.status!=="playing"&&this.critical.interrupt(this.time),this.telemetryClock+=t,(this.telemetryClock>=.1||this.events.length||this.status!=="playing")&&(this.telemetryClock=0,this.telemetry.push({timestamp:this.time,mode:this.mode,dynamicForceFeedbackEnabled:this.experiment.dynamicForceFeedbackEnabled,forceFeedback:this.forceSample?{...this.forceSample}:null,blackCatPosition:{x:this.critical.position.x,z:this.critical.position.z},blackCatPhase:this.critical.phase,criticalEventId:this.critical.activeId,emergencyMovementOverride:this.critical.motionHeading!==null,position:{...this.position},driverInput:g,inputMagnitude:r,playerDirection:{...a},systemDirection:{...d},sharedDirection:{...f},systemSteering:v,finalSteering:m,systemAuthority:this.steering.systemAuthority,driverAuthority:this.steering.driverAuthority,interventionShare:this.interventionShare,inclusionDelay:this.inclusionDelay,inclusionTimer:this.inclusionTimer,inclusionLocked:this.inclusionLocked,magnetRemaining:this.magnetRemaining,intentTimer:this.steering.intentTimer,collaborationState:this.state,catDisturbance:this.cat.value,cleaningProgress:this.metrics.cleaningProgress,runStatus:this.status,collision:P,bonusSpawn:this.events.filter(T=>T.type==="bonus_spawn").map(T=>T.value),bonusCollection:this.events.filter(T=>T.type==="bonus_collection"||T.type==="magnet_collection").map(T=>T.value)}),this.telemetry.length>12e3&&this.telemetry.shift())}abandon(){this.status==="playing"&&(this.critical.interrupt(this.time),this.status="abandoned",this.speed=0,this.metrics.completed=!1,this.emit("abandoned"))}get currentTarget(){return this.targetBonus?this.targetBonus.kind==="gold"?"Gold dust bunny":`Dust bunny ${this.targetBonus.id+1}`:this.state==="return_to_path"?"Uncleaned floor · planned cleaning lane":"Floor sweep"}get nearbyObstacle(){return this.scenario.obstacles.reduce((t,e)=>Tt(t,this.position)<Tt(e,this.position)?t:e)}}function Fu(s,t,e){const i=e.x-t.x,n=e.z-t.z,r=i*i+n*n,a=r?zt(((s.x-t.x)*i+(s.z-t.z)*n)/r,0,1):0;return Math.hypot(s.x-t.x-a*i,s.z-t.z-a*n)}const Lr=["manual","automation","collaborative"];class ku{constructor(){y(this,"level","level1");y(this,"flow","free_play");y(this,"mode","collaborative");y(this,"seed",20260908);y(this,"index",0);y(this,"results",{})}begin(t,e,i="collaborative"){this.flow=t,this.seed=e,this.index=0,this.mode=t==="comparison"?"manual":i,this.results={}}accept(t){t.completed&&(this.results[this.mode]={...t})}next(){return this.flow!=="comparison"||!this.results[this.mode]?.completed||this.index>=2?!1:(this.mode=Lr[++this.index],!0)}get comparisonComplete(){return Lr.every(t=>this.results[t]?.completed)}}const Wo="180",zu=0,Al=1,Bu=2,Qc=1,th=2,wi=3,Xi=0,Oe=1,Ne=2,Vi=0,Jn=1,Rl=2,Cl=3,Pl=4,Hu=5,rn=100,Gu=101,Vu=102,Wu=103,Xu=104,$u=200,Yu=201,qu=202,Ku=203,Ga=204,Va=205,Zu=206,Ju=207,ju=208,Qu=209,td=210,ed=211,id=212,nd=213,sd=214,Wa=0,Xa=1,$a=2,ts=3,Ya=4,qa=5,Ka=6,Za=7,eh=0,rd=1,ad=2,Wi=0,od=1,ld=2,cd=3,ih=4,hd=5,ud=6,dd=7,nh=300,es=301,is=302,Ja=303,ja=304,Gr=306,Qa=1e3,ln=1001,to=1002,Xe=1003,fd=1004,qs=1005,ui=1006,Kr=1007,cn=1008,mi=1009,sh=1010,rh=1011,Ls=1012,Xo=1013,gn=1014,di=1015,Gs=1016,$o=1017,Yo=1018,Ns=1020,ah=35902,oh=35899,lh=1021,ch=1022,ri=1023,Us=1026,Os=1027,qo=1028,Ko=1029,hh=1030,Zo=1031,Jo=1033,Ar=33776,Rr=33777,Cr=33778,Pr=33779,eo=35840,io=35841,no=35842,so=35843,ro=36196,ao=37492,oo=37496,lo=37808,co=37809,ho=37810,uo=37811,fo=37812,po=37813,mo=37814,go=37815,vo=37816,_o=37817,xo=37818,yo=37819,Mo=37820,So=37821,bo=36492,Eo=36494,wo=36495,To=36283,Ao=36284,Ro=36285,Co=36286,pd=3200,md=3201,uh=0,gd=1,Gi="",Ve="srgb",ns="srgb-linear",Nr="linear",te="srgb",bn=7680,Il=519,vd=512,_d=513,xd=514,dh=515,yd=516,Md=517,Sd=518,bd=519,Po=35044,fh=35048,Dl="300 es",fi=2e3,Ur=2001;class os{addEventListener(t,e){this._listeners===void 0&&(this._listeners={});const i=this._listeners;i[t]===void 0&&(i[t]=[]),i[t].indexOf(e)===-1&&i[t].push(e)}hasEventListener(t,e){const i=this._listeners;return i===void 0?!1:i[t]!==void 0&&i[t].indexOf(e)!==-1}removeEventListener(t,e){const i=this._listeners;if(i===void 0)return;const n=i[t];if(n!==void 0){const r=n.indexOf(e);r!==-1&&n.splice(r,1)}}dispatchEvent(t){const e=this._listeners;if(e===void 0)return;const i=e[t.type];if(i!==void 0){t.target=this;const n=i.slice(0);for(let r=0,a=n.length;r<a;r++)n[r].call(this,t);t.target=null}}}const Ae=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"],Zr=Math.PI/180,Io=180/Math.PI;function Ai(){const s=Math.random()*4294967295|0,t=Math.random()*4294967295|0,e=Math.random()*4294967295|0,i=Math.random()*4294967295|0;return(Ae[s&255]+Ae[s>>8&255]+Ae[s>>16&255]+Ae[s>>24&255]+"-"+Ae[t&255]+Ae[t>>8&255]+"-"+Ae[t>>16&15|64]+Ae[t>>24&255]+"-"+Ae[e&63|128]+Ae[e>>8&255]+"-"+Ae[e>>16&255]+Ae[e>>24&255]+Ae[i&255]+Ae[i>>8&255]+Ae[i>>16&255]+Ae[i>>24&255]).toLowerCase()}function Wt(s,t,e){return Math.max(t,Math.min(e,s))}function Ed(s,t){return(s%t+t)%t}function Jr(s,t,e){return(1-e)*s+e*t}function hi(s,t){switch(t.constructor){case Float32Array:return s;case Uint32Array:return s/4294967295;case Uint16Array:return s/65535;case Uint8Array:return s/255;case Int32Array:return Math.max(s/2147483647,-1);case Int16Array:return Math.max(s/32767,-1);case Int8Array:return Math.max(s/127,-1);default:throw new Error("Invalid component type.")}}function ee(s,t){switch(t.constructor){case Float32Array:return s;case Uint32Array:return Math.round(s*4294967295);case Uint16Array:return Math.round(s*65535);case Uint8Array:return Math.round(s*255);case Int32Array:return Math.round(s*2147483647);case Int16Array:return Math.round(s*32767);case Int8Array:return Math.round(s*127);default:throw new Error("Invalid component type.")}}class ht{constructor(t=0,e=0){ht.prototype.isVector2=!0,this.x=t,this.y=e}get width(){return this.x}set width(t){this.x=t}get height(){return this.y}set height(t){this.y=t}set(t,e){return this.x=t,this.y=e,this}setScalar(t){return this.x=t,this.y=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y)}copy(t){return this.x=t.x,this.y=t.y,this}add(t){return this.x+=t.x,this.y+=t.y,this}addScalar(t){return this.x+=t,this.y+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this}subScalar(t){return this.x-=t,this.y-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this}multiply(t){return this.x*=t.x,this.y*=t.y,this}multiplyScalar(t){return this.x*=t,this.y*=t,this}divide(t){return this.x/=t.x,this.y/=t.y,this}divideScalar(t){return this.multiplyScalar(1/t)}applyMatrix3(t){const e=this.x,i=this.y,n=t.elements;return this.x=n[0]*e+n[3]*i+n[6],this.y=n[1]*e+n[4]*i+n[7],this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this}clamp(t,e){return this.x=Wt(this.x,t.x,e.x),this.y=Wt(this.y,t.y,e.y),this}clampScalar(t,e){return this.x=Wt(this.x,t,e),this.y=Wt(this.y,t,e),this}clampLength(t,e){const i=this.length();return this.divideScalar(i||1).multiplyScalar(Wt(i,t,e))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(t){return this.x*t.x+this.y*t.y}cross(t){return this.x*t.y-this.y*t.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const i=this.dot(t)/e;return Math.acos(Wt(i,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,i=this.y-t.y;return e*e+i*i}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this}lerpVectors(t,e,i){return this.x=t.x+(e.x-t.x)*i,this.y=t.y+(e.y-t.y)*i,this}equals(t){return t.x===this.x&&t.y===this.y}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this}rotateAround(t,e){const i=Math.cos(e),n=Math.sin(e),r=this.x-t.x,a=this.y-t.y;return this.x=r*i-a*n+t.x,this.y=r*n+a*i+t.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}}class Vs{constructor(t=0,e=0,i=0,n=1){this.isQuaternion=!0,this._x=t,this._y=e,this._z=i,this._w=n}static slerpFlat(t,e,i,n,r,a,o){let c=i[n+0],l=i[n+1],h=i[n+2],u=i[n+3];const d=r[a+0],f=r[a+1],g=r[a+2],v=r[a+3];if(o===0){t[e+0]=c,t[e+1]=l,t[e+2]=h,t[e+3]=u;return}if(o===1){t[e+0]=d,t[e+1]=f,t[e+2]=g,t[e+3]=v;return}if(u!==v||c!==d||l!==f||h!==g){let m=1-o;const p=c*d+l*f+h*g+u*v,w=p>=0?1:-1,E=1-p*p;if(E>Number.EPSILON){const P=Math.sqrt(E),R=Math.atan2(P,p*w);m=Math.sin(m*R)/P,o=Math.sin(o*R)/P}const _=o*w;if(c=c*m+d*_,l=l*m+f*_,h=h*m+g*_,u=u*m+v*_,m===1-o){const P=1/Math.sqrt(c*c+l*l+h*h+u*u);c*=P,l*=P,h*=P,u*=P}}t[e]=c,t[e+1]=l,t[e+2]=h,t[e+3]=u}static multiplyQuaternionsFlat(t,e,i,n,r,a){const o=i[n],c=i[n+1],l=i[n+2],h=i[n+3],u=r[a],d=r[a+1],f=r[a+2],g=r[a+3];return t[e]=o*g+h*u+c*f-l*d,t[e+1]=c*g+h*d+l*u-o*f,t[e+2]=l*g+h*f+o*d-c*u,t[e+3]=h*g-o*u-c*d-l*f,t}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get w(){return this._w}set w(t){this._w=t,this._onChangeCallback()}set(t,e,i,n){return this._x=t,this._y=e,this._z=i,this._w=n,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(t){return this._x=t.x,this._y=t.y,this._z=t.z,this._w=t.w,this._onChangeCallback(),this}setFromEuler(t,e=!0){const i=t._x,n=t._y,r=t._z,a=t._order,o=Math.cos,c=Math.sin,l=o(i/2),h=o(n/2),u=o(r/2),d=c(i/2),f=c(n/2),g=c(r/2);switch(a){case"XYZ":this._x=d*h*u+l*f*g,this._y=l*f*u-d*h*g,this._z=l*h*g+d*f*u,this._w=l*h*u-d*f*g;break;case"YXZ":this._x=d*h*u+l*f*g,this._y=l*f*u-d*h*g,this._z=l*h*g-d*f*u,this._w=l*h*u+d*f*g;break;case"ZXY":this._x=d*h*u-l*f*g,this._y=l*f*u+d*h*g,this._z=l*h*g+d*f*u,this._w=l*h*u-d*f*g;break;case"ZYX":this._x=d*h*u-l*f*g,this._y=l*f*u+d*h*g,this._z=l*h*g-d*f*u,this._w=l*h*u+d*f*g;break;case"YZX":this._x=d*h*u+l*f*g,this._y=l*f*u+d*h*g,this._z=l*h*g-d*f*u,this._w=l*h*u-d*f*g;break;case"XZY":this._x=d*h*u-l*f*g,this._y=l*f*u-d*h*g,this._z=l*h*g+d*f*u,this._w=l*h*u+d*f*g;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+a)}return e===!0&&this._onChangeCallback(),this}setFromAxisAngle(t,e){const i=e/2,n=Math.sin(i);return this._x=t.x*n,this._y=t.y*n,this._z=t.z*n,this._w=Math.cos(i),this._onChangeCallback(),this}setFromRotationMatrix(t){const e=t.elements,i=e[0],n=e[4],r=e[8],a=e[1],o=e[5],c=e[9],l=e[2],h=e[6],u=e[10],d=i+o+u;if(d>0){const f=.5/Math.sqrt(d+1);this._w=.25/f,this._x=(h-c)*f,this._y=(r-l)*f,this._z=(a-n)*f}else if(i>o&&i>u){const f=2*Math.sqrt(1+i-o-u);this._w=(h-c)/f,this._x=.25*f,this._y=(n+a)/f,this._z=(r+l)/f}else if(o>u){const f=2*Math.sqrt(1+o-i-u);this._w=(r-l)/f,this._x=(n+a)/f,this._y=.25*f,this._z=(c+h)/f}else{const f=2*Math.sqrt(1+u-i-o);this._w=(a-n)/f,this._x=(r+l)/f,this._y=(c+h)/f,this._z=.25*f}return this._onChangeCallback(),this}setFromUnitVectors(t,e){let i=t.dot(e)+1;return i<1e-8?(i=0,Math.abs(t.x)>Math.abs(t.z)?(this._x=-t.y,this._y=t.x,this._z=0,this._w=i):(this._x=0,this._y=-t.z,this._z=t.y,this._w=i)):(this._x=t.y*e.z-t.z*e.y,this._y=t.z*e.x-t.x*e.z,this._z=t.x*e.y-t.y*e.x,this._w=i),this.normalize()}angleTo(t){return 2*Math.acos(Math.abs(Wt(this.dot(t),-1,1)))}rotateTowards(t,e){const i=this.angleTo(t);if(i===0)return this;const n=Math.min(1,e/i);return this.slerp(t,n),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(t){return this._x*t._x+this._y*t._y+this._z*t._z+this._w*t._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let t=this.length();return t===0?(this._x=0,this._y=0,this._z=0,this._w=1):(t=1/t,this._x=this._x*t,this._y=this._y*t,this._z=this._z*t,this._w=this._w*t),this._onChangeCallback(),this}multiply(t){return this.multiplyQuaternions(this,t)}premultiply(t){return this.multiplyQuaternions(t,this)}multiplyQuaternions(t,e){const i=t._x,n=t._y,r=t._z,a=t._w,o=e._x,c=e._y,l=e._z,h=e._w;return this._x=i*h+a*o+n*l-r*c,this._y=n*h+a*c+r*o-i*l,this._z=r*h+a*l+i*c-n*o,this._w=a*h-i*o-n*c-r*l,this._onChangeCallback(),this}slerp(t,e){if(e===0)return this;if(e===1)return this.copy(t);const i=this._x,n=this._y,r=this._z,a=this._w;let o=a*t._w+i*t._x+n*t._y+r*t._z;if(o<0?(this._w=-t._w,this._x=-t._x,this._y=-t._y,this._z=-t._z,o=-o):this.copy(t),o>=1)return this._w=a,this._x=i,this._y=n,this._z=r,this;const c=1-o*o;if(c<=Number.EPSILON){const f=1-e;return this._w=f*a+e*this._w,this._x=f*i+e*this._x,this._y=f*n+e*this._y,this._z=f*r+e*this._z,this.normalize(),this}const l=Math.sqrt(c),h=Math.atan2(l,o),u=Math.sin((1-e)*h)/l,d=Math.sin(e*h)/l;return this._w=a*u+this._w*d,this._x=i*u+this._x*d,this._y=n*u+this._y*d,this._z=r*u+this._z*d,this._onChangeCallback(),this}slerpQuaternions(t,e,i){return this.copy(t).slerp(e,i)}random(){const t=2*Math.PI*Math.random(),e=2*Math.PI*Math.random(),i=Math.random(),n=Math.sqrt(1-i),r=Math.sqrt(i);return this.set(n*Math.sin(t),n*Math.cos(t),r*Math.sin(e),r*Math.cos(e))}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._w===this._w}fromArray(t,e=0){return this._x=t[e],this._y=t[e+1],this._z=t[e+2],this._w=t[e+3],this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._w,t}fromBufferAttribute(t,e){return this._x=t.getX(e),this._y=t.getY(e),this._z=t.getZ(e),this._w=t.getW(e),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}}class C{constructor(t=0,e=0,i=0){C.prototype.isVector3=!0,this.x=t,this.y=e,this.z=i}set(t,e,i){return i===void 0&&(i=this.z),this.x=t,this.y=e,this.z=i,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this}multiplyVectors(t,e){return this.x=t.x*e.x,this.y=t.y*e.y,this.z=t.z*e.z,this}applyEuler(t){return this.applyQuaternion(Ll.setFromEuler(t))}applyAxisAngle(t,e){return this.applyQuaternion(Ll.setFromAxisAngle(t,e))}applyMatrix3(t){const e=this.x,i=this.y,n=this.z,r=t.elements;return this.x=r[0]*e+r[3]*i+r[6]*n,this.y=r[1]*e+r[4]*i+r[7]*n,this.z=r[2]*e+r[5]*i+r[8]*n,this}applyNormalMatrix(t){return this.applyMatrix3(t).normalize()}applyMatrix4(t){const e=this.x,i=this.y,n=this.z,r=t.elements,a=1/(r[3]*e+r[7]*i+r[11]*n+r[15]);return this.x=(r[0]*e+r[4]*i+r[8]*n+r[12])*a,this.y=(r[1]*e+r[5]*i+r[9]*n+r[13])*a,this.z=(r[2]*e+r[6]*i+r[10]*n+r[14])*a,this}applyQuaternion(t){const e=this.x,i=this.y,n=this.z,r=t.x,a=t.y,o=t.z,c=t.w,l=2*(a*n-o*i),h=2*(o*e-r*n),u=2*(r*i-a*e);return this.x=e+c*l+a*u-o*h,this.y=i+c*h+o*l-r*u,this.z=n+c*u+r*h-a*l,this}project(t){return this.applyMatrix4(t.matrixWorldInverse).applyMatrix4(t.projectionMatrix)}unproject(t){return this.applyMatrix4(t.projectionMatrixInverse).applyMatrix4(t.matrixWorld)}transformDirection(t){const e=this.x,i=this.y,n=this.z,r=t.elements;return this.x=r[0]*e+r[4]*i+r[8]*n,this.y=r[1]*e+r[5]*i+r[9]*n,this.z=r[2]*e+r[6]*i+r[10]*n,this.normalize()}divide(t){return this.x/=t.x,this.y/=t.y,this.z/=t.z,this}divideScalar(t){return this.multiplyScalar(1/t)}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this}clamp(t,e){return this.x=Wt(this.x,t.x,e.x),this.y=Wt(this.y,t.y,e.y),this.z=Wt(this.z,t.z,e.z),this}clampScalar(t,e){return this.x=Wt(this.x,t,e),this.y=Wt(this.y,t,e),this.z=Wt(this.z,t,e),this}clampLength(t,e){const i=this.length();return this.divideScalar(i||1).multiplyScalar(Wt(i,t,e))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this}lerpVectors(t,e,i){return this.x=t.x+(e.x-t.x)*i,this.y=t.y+(e.y-t.y)*i,this.z=t.z+(e.z-t.z)*i,this}cross(t){return this.crossVectors(this,t)}crossVectors(t,e){const i=t.x,n=t.y,r=t.z,a=e.x,o=e.y,c=e.z;return this.x=n*c-r*o,this.y=r*a-i*c,this.z=i*o-n*a,this}projectOnVector(t){const e=t.lengthSq();if(e===0)return this.set(0,0,0);const i=t.dot(this)/e;return this.copy(t).multiplyScalar(i)}projectOnPlane(t){return jr.copy(this).projectOnVector(t),this.sub(jr)}reflect(t){return this.sub(jr.copy(t).multiplyScalar(2*this.dot(t)))}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const i=this.dot(t)/e;return Math.acos(Wt(i,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,i=this.y-t.y,n=this.z-t.z;return e*e+i*i+n*n}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)+Math.abs(this.z-t.z)}setFromSpherical(t){return this.setFromSphericalCoords(t.radius,t.phi,t.theta)}setFromSphericalCoords(t,e,i){const n=Math.sin(e)*t;return this.x=n*Math.sin(i),this.y=Math.cos(e)*t,this.z=n*Math.cos(i),this}setFromCylindrical(t){return this.setFromCylindricalCoords(t.radius,t.theta,t.y)}setFromCylindricalCoords(t,e,i){return this.x=t*Math.sin(e),this.y=i,this.z=t*Math.cos(e),this}setFromMatrixPosition(t){const e=t.elements;return this.x=e[12],this.y=e[13],this.z=e[14],this}setFromMatrixScale(t){const e=this.setFromMatrixColumn(t,0).length(),i=this.setFromMatrixColumn(t,1).length(),n=this.setFromMatrixColumn(t,2).length();return this.x=e,this.y=i,this.z=n,this}setFromMatrixColumn(t,e){return this.fromArray(t.elements,e*4)}setFromMatrix3Column(t,e){return this.fromArray(t.elements,e*3)}setFromEuler(t){return this.x=t._x,this.y=t._y,this.z=t._z,this}setFromColor(t){return this.x=t.r,this.y=t.g,this.z=t.b,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){const t=Math.random()*Math.PI*2,e=Math.random()*2-1,i=Math.sqrt(1-e*e);return this.x=i*Math.cos(t),this.y=e,this.z=i*Math.sin(t),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}}const jr=new C,Ll=new Vs;class Ot{constructor(t,e,i,n,r,a,o,c,l){Ot.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],t!==void 0&&this.set(t,e,i,n,r,a,o,c,l)}set(t,e,i,n,r,a,o,c,l){const h=this.elements;return h[0]=t,h[1]=n,h[2]=o,h[3]=e,h[4]=r,h[5]=c,h[6]=i,h[7]=a,h[8]=l,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(t){const e=this.elements,i=t.elements;return e[0]=i[0],e[1]=i[1],e[2]=i[2],e[3]=i[3],e[4]=i[4],e[5]=i[5],e[6]=i[6],e[7]=i[7],e[8]=i[8],this}extractBasis(t,e,i){return t.setFromMatrix3Column(this,0),e.setFromMatrix3Column(this,1),i.setFromMatrix3Column(this,2),this}setFromMatrix4(t){const e=t.elements;return this.set(e[0],e[4],e[8],e[1],e[5],e[9],e[2],e[6],e[10]),this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const i=t.elements,n=e.elements,r=this.elements,a=i[0],o=i[3],c=i[6],l=i[1],h=i[4],u=i[7],d=i[2],f=i[5],g=i[8],v=n[0],m=n[3],p=n[6],w=n[1],E=n[4],_=n[7],P=n[2],R=n[5],I=n[8];return r[0]=a*v+o*w+c*P,r[3]=a*m+o*E+c*R,r[6]=a*p+o*_+c*I,r[1]=l*v+h*w+u*P,r[4]=l*m+h*E+u*R,r[7]=l*p+h*_+u*I,r[2]=d*v+f*w+g*P,r[5]=d*m+f*E+g*R,r[8]=d*p+f*_+g*I,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[3]*=t,e[6]*=t,e[1]*=t,e[4]*=t,e[7]*=t,e[2]*=t,e[5]*=t,e[8]*=t,this}determinant(){const t=this.elements,e=t[0],i=t[1],n=t[2],r=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8];return e*a*h-e*o*l-i*r*h+i*o*c+n*r*l-n*a*c}invert(){const t=this.elements,e=t[0],i=t[1],n=t[2],r=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8],u=h*a-o*l,d=o*c-h*r,f=l*r-a*c,g=e*u+i*d+n*f;if(g===0)return this.set(0,0,0,0,0,0,0,0,0);const v=1/g;return t[0]=u*v,t[1]=(n*l-h*i)*v,t[2]=(o*i-n*a)*v,t[3]=d*v,t[4]=(h*e-n*c)*v,t[5]=(n*r-o*e)*v,t[6]=f*v,t[7]=(i*c-l*e)*v,t[8]=(a*e-i*r)*v,this}transpose(){let t;const e=this.elements;return t=e[1],e[1]=e[3],e[3]=t,t=e[2],e[2]=e[6],e[6]=t,t=e[5],e[5]=e[7],e[7]=t,this}getNormalMatrix(t){return this.setFromMatrix4(t).invert().transpose()}transposeIntoArray(t){const e=this.elements;return t[0]=e[0],t[1]=e[3],t[2]=e[6],t[3]=e[1],t[4]=e[4],t[5]=e[7],t[6]=e[2],t[7]=e[5],t[8]=e[8],this}setUvTransform(t,e,i,n,r,a,o){const c=Math.cos(r),l=Math.sin(r);return this.set(i*c,i*l,-i*(c*a+l*o)+a+t,-n*l,n*c,-n*(-l*a+c*o)+o+e,0,0,1),this}scale(t,e){return this.premultiply(Qr.makeScale(t,e)),this}rotate(t){return this.premultiply(Qr.makeRotation(-t)),this}translate(t,e){return this.premultiply(Qr.makeTranslation(t,e)),this}makeTranslation(t,e){return t.isVector2?this.set(1,0,t.x,0,1,t.y,0,0,1):this.set(1,0,t,0,1,e,0,0,1),this}makeRotation(t){const e=Math.cos(t),i=Math.sin(t);return this.set(e,-i,0,i,e,0,0,0,1),this}makeScale(t,e){return this.set(t,0,0,0,e,0,0,0,1),this}equals(t){const e=this.elements,i=t.elements;for(let n=0;n<9;n++)if(e[n]!==i[n])return!1;return!0}fromArray(t,e=0){for(let i=0;i<9;i++)this.elements[i]=t[i+e];return this}toArray(t=[],e=0){const i=this.elements;return t[e]=i[0],t[e+1]=i[1],t[e+2]=i[2],t[e+3]=i[3],t[e+4]=i[4],t[e+5]=i[5],t[e+6]=i[6],t[e+7]=i[7],t[e+8]=i[8],t}clone(){return new this.constructor().fromArray(this.elements)}}const Qr=new Ot;function ph(s){for(let t=s.length-1;t>=0;--t)if(s[t]>=65535)return!0;return!1}function Or(s){return document.createElementNS("http://www.w3.org/1999/xhtml",s)}function wd(){const s=Or("canvas");return s.style.display="block",s}const Nl={};function Fs(s){s in Nl||(Nl[s]=!0,console.warn(s))}function Td(s,t,e){return new Promise(function(i,n){function r(){switch(s.clientWaitSync(t,s.SYNC_FLUSH_COMMANDS_BIT,0)){case s.WAIT_FAILED:n();break;case s.TIMEOUT_EXPIRED:setTimeout(r,e);break;default:i()}}setTimeout(r,e)})}const Ul=new Ot().set(.4123908,.3575843,.1804808,.212639,.7151687,.0721923,.0193308,.1191948,.9505322),Ol=new Ot().set(3.2409699,-1.5373832,-.4986108,-.9692436,1.8759675,.0415551,.0556301,-.203977,1.0569715);function Ad(){const s={enabled:!0,workingColorSpace:ns,spaces:{},convert:function(n,r,a){return this.enabled===!1||r===a||!r||!a||(this.spaces[r].transfer===te&&(n.r=Ri(n.r),n.g=Ri(n.g),n.b=Ri(n.b)),this.spaces[r].primaries!==this.spaces[a].primaries&&(n.applyMatrix3(this.spaces[r].toXYZ),n.applyMatrix3(this.spaces[a].fromXYZ)),this.spaces[a].transfer===te&&(n.r=jn(n.r),n.g=jn(n.g),n.b=jn(n.b))),n},workingToColorSpace:function(n,r){return this.convert(n,this.workingColorSpace,r)},colorSpaceToWorking:function(n,r){return this.convert(n,r,this.workingColorSpace)},getPrimaries:function(n){return this.spaces[n].primaries},getTransfer:function(n){return n===Gi?Nr:this.spaces[n].transfer},getToneMappingMode:function(n){return this.spaces[n].outputColorSpaceConfig.toneMappingMode||"standard"},getLuminanceCoefficients:function(n,r=this.workingColorSpace){return n.fromArray(this.spaces[r].luminanceCoefficients)},define:function(n){Object.assign(this.spaces,n)},_getMatrix:function(n,r,a){return n.copy(this.spaces[r].toXYZ).multiply(this.spaces[a].fromXYZ)},_getDrawingBufferColorSpace:function(n){return this.spaces[n].outputColorSpaceConfig.drawingBufferColorSpace},_getUnpackColorSpace:function(n=this.workingColorSpace){return this.spaces[n].workingColorSpaceConfig.unpackColorSpace},fromWorkingColorSpace:function(n,r){return Fs("THREE.ColorManagement: .fromWorkingColorSpace() has been renamed to .workingToColorSpace()."),s.workingToColorSpace(n,r)},toWorkingColorSpace:function(n,r){return Fs("THREE.ColorManagement: .toWorkingColorSpace() has been renamed to .colorSpaceToWorking()."),s.colorSpaceToWorking(n,r)}},t=[.64,.33,.3,.6,.15,.06],e=[.2126,.7152,.0722],i=[.3127,.329];return s.define({[ns]:{primaries:t,whitePoint:i,transfer:Nr,toXYZ:Ul,fromXYZ:Ol,luminanceCoefficients:e,workingColorSpaceConfig:{unpackColorSpace:Ve},outputColorSpaceConfig:{drawingBufferColorSpace:Ve}},[Ve]:{primaries:t,whitePoint:i,transfer:te,toXYZ:Ul,fromXYZ:Ol,luminanceCoefficients:e,outputColorSpaceConfig:{drawingBufferColorSpace:Ve}}}),s}const qt=Ad();function Ri(s){return s<.04045?s*.0773993808:Math.pow(s*.9478672986+.0521327014,2.4)}function jn(s){return s<.0031308?s*12.92:1.055*Math.pow(s,.41666)-.055}let En;class Rd{static getDataURL(t,e="image/png"){if(/^data:/i.test(t.src)||typeof HTMLCanvasElement>"u")return t.src;let i;if(t instanceof HTMLCanvasElement)i=t;else{En===void 0&&(En=Or("canvas")),En.width=t.width,En.height=t.height;const n=En.getContext("2d");t instanceof ImageData?n.putImageData(t,0,0):n.drawImage(t,0,0,t.width,t.height),i=En}return i.toDataURL(e)}static sRGBToLinear(t){if(typeof HTMLImageElement<"u"&&t instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&t instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&t instanceof ImageBitmap){const e=Or("canvas");e.width=t.width,e.height=t.height;const i=e.getContext("2d");i.drawImage(t,0,0,t.width,t.height);const n=i.getImageData(0,0,t.width,t.height),r=n.data;for(let a=0;a<r.length;a++)r[a]=Ri(r[a]/255)*255;return i.putImageData(n,0,0),e}else if(t.data){const e=t.data.slice(0);for(let i=0;i<e.length;i++)e instanceof Uint8Array||e instanceof Uint8ClampedArray?e[i]=Math.floor(Ri(e[i]/255)*255):e[i]=Ri(e[i]);return{data:e,width:t.width,height:t.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),t}}let Cd=0;class jo{constructor(t=null){this.isSource=!0,Object.defineProperty(this,"id",{value:Cd++}),this.uuid=Ai(),this.data=t,this.dataReady=!0,this.version=0}getSize(t){const e=this.data;return typeof HTMLVideoElement<"u"&&e instanceof HTMLVideoElement?t.set(e.videoWidth,e.videoHeight,0):e instanceof VideoFrame?t.set(e.displayHeight,e.displayWidth,0):e!==null?t.set(e.width,e.height,e.depth||0):t.set(0,0,0),t}set needsUpdate(t){t===!0&&this.version++}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.images[this.uuid]!==void 0)return t.images[this.uuid];const i={uuid:this.uuid,url:""},n=this.data;if(n!==null){let r;if(Array.isArray(n)){r=[];for(let a=0,o=n.length;a<o;a++)n[a].isDataTexture?r.push(ta(n[a].image)):r.push(ta(n[a]))}else r=ta(n);i.url=r}return e||(t.images[this.uuid]=i),i}}function ta(s){return typeof HTMLImageElement<"u"&&s instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&s instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&s instanceof ImageBitmap?Rd.getDataURL(s):s.data?{data:Array.from(s.data),width:s.width,height:s.height,type:s.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}let Pd=0;const ea=new C;class Ce extends os{constructor(t=Ce.DEFAULT_IMAGE,e=Ce.DEFAULT_MAPPING,i=ln,n=ln,r=ui,a=cn,o=ri,c=mi,l=Ce.DEFAULT_ANISOTROPY,h=Gi){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:Pd++}),this.uuid=Ai(),this.name="",this.source=new jo(t),this.mipmaps=[],this.mapping=e,this.channel=0,this.wrapS=i,this.wrapT=n,this.magFilter=r,this.minFilter=a,this.anisotropy=l,this.format=o,this.internalFormat=null,this.type=c,this.offset=new ht(0,0),this.repeat=new ht(1,1),this.center=new ht(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new Ot,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,this.colorSpace=h,this.userData={},this.updateRanges=[],this.version=0,this.onUpdate=null,this.renderTarget=null,this.isRenderTargetTexture=!1,this.isArrayTexture=!!(t&&t.depth&&t.depth>1),this.pmremVersion=0}get width(){return this.source.getSize(ea).x}get height(){return this.source.getSize(ea).y}get depth(){return this.source.getSize(ea).z}get image(){return this.source.data}set image(t=null){this.source.data=t}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}addUpdateRange(t,e){this.updateRanges.push({start:t,count:e})}clearUpdateRanges(){this.updateRanges.length=0}clone(){return new this.constructor().copy(this)}copy(t){return this.name=t.name,this.source=t.source,this.mipmaps=t.mipmaps.slice(0),this.mapping=t.mapping,this.channel=t.channel,this.wrapS=t.wrapS,this.wrapT=t.wrapT,this.magFilter=t.magFilter,this.minFilter=t.minFilter,this.anisotropy=t.anisotropy,this.format=t.format,this.internalFormat=t.internalFormat,this.type=t.type,this.offset.copy(t.offset),this.repeat.copy(t.repeat),this.center.copy(t.center),this.rotation=t.rotation,this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrix.copy(t.matrix),this.generateMipmaps=t.generateMipmaps,this.premultiplyAlpha=t.premultiplyAlpha,this.flipY=t.flipY,this.unpackAlignment=t.unpackAlignment,this.colorSpace=t.colorSpace,this.renderTarget=t.renderTarget,this.isRenderTargetTexture=t.isRenderTargetTexture,this.isArrayTexture=t.isArrayTexture,this.userData=JSON.parse(JSON.stringify(t.userData)),this.needsUpdate=!0,this}setValues(t){for(const e in t){const i=t[e];if(i===void 0){console.warn(`THREE.Texture.setValues(): parameter '${e}' has value of undefined.`);continue}const n=this[e];if(n===void 0){console.warn(`THREE.Texture.setValues(): property '${e}' does not exist.`);continue}n&&i&&n.isVector2&&i.isVector2||n&&i&&n.isVector3&&i.isVector3||n&&i&&n.isMatrix3&&i.isMatrix3?n.copy(i):this[e]=i}}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.textures[this.uuid]!==void 0)return t.textures[this.uuid];const i={metadata:{version:4.7,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(t).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(i.userData=this.userData),e||(t.textures[this.uuid]=i),i}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(t){if(this.mapping!==nh)return t;if(t.applyMatrix3(this.matrix),t.x<0||t.x>1)switch(this.wrapS){case Qa:t.x=t.x-Math.floor(t.x);break;case ln:t.x=t.x<0?0:1;break;case to:Math.abs(Math.floor(t.x)%2)===1?t.x=Math.ceil(t.x)-t.x:t.x=t.x-Math.floor(t.x);break}if(t.y<0||t.y>1)switch(this.wrapT){case Qa:t.y=t.y-Math.floor(t.y);break;case ln:t.y=t.y<0?0:1;break;case to:Math.abs(Math.floor(t.y)%2)===1?t.y=Math.ceil(t.y)-t.y:t.y=t.y-Math.floor(t.y);break}return this.flipY&&(t.y=1-t.y),t}set needsUpdate(t){t===!0&&(this.version++,this.source.needsUpdate=!0)}set needsPMREMUpdate(t){t===!0&&this.pmremVersion++}}Ce.DEFAULT_IMAGE=null;Ce.DEFAULT_MAPPING=nh;Ce.DEFAULT_ANISOTROPY=1;class pe{constructor(t=0,e=0,i=0,n=1){pe.prototype.isVector4=!0,this.x=t,this.y=e,this.z=i,this.w=n}get width(){return this.z}set width(t){this.z=t}get height(){return this.w}set height(t){this.w=t}set(t,e,i,n){return this.x=t,this.y=e,this.z=i,this.w=n,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this.w=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setW(t){return this.w=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;case 3:this.w=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this.w=t.w!==void 0?t.w:1,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this.w+=t.w,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this.w+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this.w=t.w+e.w,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this.w+=t.w*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this.w-=t.w,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this.w-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this.w=t.w-e.w,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this.w*=t.w,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this.w*=t,this}applyMatrix4(t){const e=this.x,i=this.y,n=this.z,r=this.w,a=t.elements;return this.x=a[0]*e+a[4]*i+a[8]*n+a[12]*r,this.y=a[1]*e+a[5]*i+a[9]*n+a[13]*r,this.z=a[2]*e+a[6]*i+a[10]*n+a[14]*r,this.w=a[3]*e+a[7]*i+a[11]*n+a[15]*r,this}divide(t){return this.x/=t.x,this.y/=t.y,this.z/=t.z,this.w/=t.w,this}divideScalar(t){return this.multiplyScalar(1/t)}setAxisAngleFromQuaternion(t){this.w=2*Math.acos(t.w);const e=Math.sqrt(1-t.w*t.w);return e<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=t.x/e,this.y=t.y/e,this.z=t.z/e),this}setAxisAngleFromRotationMatrix(t){let e,i,n,r;const c=t.elements,l=c[0],h=c[4],u=c[8],d=c[1],f=c[5],g=c[9],v=c[2],m=c[6],p=c[10];if(Math.abs(h-d)<.01&&Math.abs(u-v)<.01&&Math.abs(g-m)<.01){if(Math.abs(h+d)<.1&&Math.abs(u+v)<.1&&Math.abs(g+m)<.1&&Math.abs(l+f+p-3)<.1)return this.set(1,0,0,0),this;e=Math.PI;const E=(l+1)/2,_=(f+1)/2,P=(p+1)/2,R=(h+d)/4,I=(u+v)/4,T=(g+m)/4;return E>_&&E>P?E<.01?(i=0,n=.707106781,r=.707106781):(i=Math.sqrt(E),n=R/i,r=I/i):_>P?_<.01?(i=.707106781,n=0,r=.707106781):(n=Math.sqrt(_),i=R/n,r=T/n):P<.01?(i=.707106781,n=.707106781,r=0):(r=Math.sqrt(P),i=I/r,n=T/r),this.set(i,n,r,e),this}let w=Math.sqrt((m-g)*(m-g)+(u-v)*(u-v)+(d-h)*(d-h));return Math.abs(w)<.001&&(w=1),this.x=(m-g)/w,this.y=(u-v)/w,this.z=(d-h)/w,this.w=Math.acos((l+f+p-1)/2),this}setFromMatrixPosition(t){const e=t.elements;return this.x=e[12],this.y=e[13],this.z=e[14],this.w=e[15],this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this.w=Math.min(this.w,t.w),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this.w=Math.max(this.w,t.w),this}clamp(t,e){return this.x=Wt(this.x,t.x,e.x),this.y=Wt(this.y,t.y,e.y),this.z=Wt(this.z,t.z,e.z),this.w=Wt(this.w,t.w,e.w),this}clampScalar(t,e){return this.x=Wt(this.x,t,e),this.y=Wt(this.y,t,e),this.z=Wt(this.z,t,e),this.w=Wt(this.w,t,e),this}clampLength(t,e){const i=this.length();return this.divideScalar(i||1).multiplyScalar(Wt(i,t,e))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z+this.w*t.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this.w+=(t.w-this.w)*e,this}lerpVectors(t,e,i){return this.x=t.x+(e.x-t.x)*i,this.y=t.y+(e.y-t.y)*i,this.z=t.z+(e.z-t.z)*i,this.w=t.w+(e.w-t.w)*i,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z&&t.w===this.w}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this.w=t[e+3],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t[e+3]=this.w,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this.w=t.getW(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}}class Id extends os{constructor(t=1,e=1,i={}){super(),i=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:ui,depthBuffer:!0,stencilBuffer:!1,resolveDepthBuffer:!0,resolveStencilBuffer:!0,depthTexture:null,samples:0,count:1,depth:1,multiview:!1},i),this.isRenderTarget=!0,this.width=t,this.height=e,this.depth=i.depth,this.scissor=new pe(0,0,t,e),this.scissorTest=!1,this.viewport=new pe(0,0,t,e);const n={width:t,height:e,depth:i.depth},r=new Ce(n);this.textures=[];const a=i.count;for(let o=0;o<a;o++)this.textures[o]=r.clone(),this.textures[o].isRenderTargetTexture=!0,this.textures[o].renderTarget=this;this._setTextureOptions(i),this.depthBuffer=i.depthBuffer,this.stencilBuffer=i.stencilBuffer,this.resolveDepthBuffer=i.resolveDepthBuffer,this.resolveStencilBuffer=i.resolveStencilBuffer,this._depthTexture=null,this.depthTexture=i.depthTexture,this.samples=i.samples,this.multiview=i.multiview}_setTextureOptions(t={}){const e={minFilter:ui,generateMipmaps:!1,flipY:!1,internalFormat:null};t.mapping!==void 0&&(e.mapping=t.mapping),t.wrapS!==void 0&&(e.wrapS=t.wrapS),t.wrapT!==void 0&&(e.wrapT=t.wrapT),t.wrapR!==void 0&&(e.wrapR=t.wrapR),t.magFilter!==void 0&&(e.magFilter=t.magFilter),t.minFilter!==void 0&&(e.minFilter=t.minFilter),t.format!==void 0&&(e.format=t.format),t.type!==void 0&&(e.type=t.type),t.anisotropy!==void 0&&(e.anisotropy=t.anisotropy),t.colorSpace!==void 0&&(e.colorSpace=t.colorSpace),t.flipY!==void 0&&(e.flipY=t.flipY),t.generateMipmaps!==void 0&&(e.generateMipmaps=t.generateMipmaps),t.internalFormat!==void 0&&(e.internalFormat=t.internalFormat);for(let i=0;i<this.textures.length;i++)this.textures[i].setValues(e)}get texture(){return this.textures[0]}set texture(t){this.textures[0]=t}set depthTexture(t){this._depthTexture!==null&&(this._depthTexture.renderTarget=null),t!==null&&(t.renderTarget=this),this._depthTexture=t}get depthTexture(){return this._depthTexture}setSize(t,e,i=1){if(this.width!==t||this.height!==e||this.depth!==i){this.width=t,this.height=e,this.depth=i;for(let n=0,r=this.textures.length;n<r;n++)this.textures[n].image.width=t,this.textures[n].image.height=e,this.textures[n].image.depth=i,this.textures[n].isArrayTexture=this.textures[n].image.depth>1;this.dispose()}this.viewport.set(0,0,t,e),this.scissor.set(0,0,t,e)}clone(){return new this.constructor().copy(this)}copy(t){this.width=t.width,this.height=t.height,this.depth=t.depth,this.scissor.copy(t.scissor),this.scissorTest=t.scissorTest,this.viewport.copy(t.viewport),this.textures.length=0;for(let e=0,i=t.textures.length;e<i;e++){this.textures[e]=t.textures[e].clone(),this.textures[e].isRenderTargetTexture=!0,this.textures[e].renderTarget=this;const n=Object.assign({},t.textures[e].image);this.textures[e].source=new jo(n)}return this.depthBuffer=t.depthBuffer,this.stencilBuffer=t.stencilBuffer,this.resolveDepthBuffer=t.resolveDepthBuffer,this.resolveStencilBuffer=t.resolveStencilBuffer,t.depthTexture!==null&&(this.depthTexture=t.depthTexture.clone()),this.samples=t.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}}class vn extends Id{constructor(t=1,e=1,i={}){super(t,e,i),this.isWebGLRenderTarget=!0}}class mh extends Ce{constructor(t=null,e=1,i=1,n=1){super(null),this.isDataArrayTexture=!0,this.image={data:t,width:e,height:i,depth:n},this.magFilter=Xe,this.minFilter=Xe,this.wrapR=ln,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1,this.layerUpdates=new Set}addLayerUpdate(t){this.layerUpdates.add(t)}clearLayerUpdates(){this.layerUpdates.clear()}}class Dd extends Ce{constructor(t=null,e=1,i=1,n=1){super(null),this.isData3DTexture=!0,this.image={data:t,width:e,height:i,depth:n},this.magFilter=Xe,this.minFilter=Xe,this.wrapR=ln,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class xn{constructor(t=new C(1/0,1/0,1/0),e=new C(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=t,this.max=e}set(t,e){return this.min.copy(t),this.max.copy(e),this}setFromArray(t){this.makeEmpty();for(let e=0,i=t.length;e<i;e+=3)this.expandByPoint(ei.fromArray(t,e));return this}setFromBufferAttribute(t){this.makeEmpty();for(let e=0,i=t.count;e<i;e++)this.expandByPoint(ei.fromBufferAttribute(t,e));return this}setFromPoints(t){this.makeEmpty();for(let e=0,i=t.length;e<i;e++)this.expandByPoint(t[e]);return this}setFromCenterAndSize(t,e){const i=ei.copy(e).multiplyScalar(.5);return this.min.copy(t).sub(i),this.max.copy(t).add(i),this}setFromObject(t,e=!1){return this.makeEmpty(),this.expandByObject(t,e)}clone(){return new this.constructor().copy(this)}copy(t){return this.min.copy(t.min),this.max.copy(t.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(t){return this.isEmpty()?t.set(0,0,0):t.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(t){return this.isEmpty()?t.set(0,0,0):t.subVectors(this.max,this.min)}expandByPoint(t){return this.min.min(t),this.max.max(t),this}expandByVector(t){return this.min.sub(t),this.max.add(t),this}expandByScalar(t){return this.min.addScalar(-t),this.max.addScalar(t),this}expandByObject(t,e=!1){t.updateWorldMatrix(!1,!1);const i=t.geometry;if(i!==void 0){const r=i.getAttribute("position");if(e===!0&&r!==void 0&&t.isInstancedMesh!==!0)for(let a=0,o=r.count;a<o;a++)t.isMesh===!0?t.getVertexPosition(a,ei):ei.fromBufferAttribute(r,a),ei.applyMatrix4(t.matrixWorld),this.expandByPoint(ei);else t.boundingBox!==void 0?(t.boundingBox===null&&t.computeBoundingBox(),Ks.copy(t.boundingBox)):(i.boundingBox===null&&i.computeBoundingBox(),Ks.copy(i.boundingBox)),Ks.applyMatrix4(t.matrixWorld),this.union(Ks)}const n=t.children;for(let r=0,a=n.length;r<a;r++)this.expandByObject(n[r],e);return this}containsPoint(t){return t.x>=this.min.x&&t.x<=this.max.x&&t.y>=this.min.y&&t.y<=this.max.y&&t.z>=this.min.z&&t.z<=this.max.z}containsBox(t){return this.min.x<=t.min.x&&t.max.x<=this.max.x&&this.min.y<=t.min.y&&t.max.y<=this.max.y&&this.min.z<=t.min.z&&t.max.z<=this.max.z}getParameter(t,e){return e.set((t.x-this.min.x)/(this.max.x-this.min.x),(t.y-this.min.y)/(this.max.y-this.min.y),(t.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(t){return t.max.x>=this.min.x&&t.min.x<=this.max.x&&t.max.y>=this.min.y&&t.min.y<=this.max.y&&t.max.z>=this.min.z&&t.min.z<=this.max.z}intersectsSphere(t){return this.clampPoint(t.center,ei),ei.distanceToSquared(t.center)<=t.radius*t.radius}intersectsPlane(t){let e,i;return t.normal.x>0?(e=t.normal.x*this.min.x,i=t.normal.x*this.max.x):(e=t.normal.x*this.max.x,i=t.normal.x*this.min.x),t.normal.y>0?(e+=t.normal.y*this.min.y,i+=t.normal.y*this.max.y):(e+=t.normal.y*this.max.y,i+=t.normal.y*this.min.y),t.normal.z>0?(e+=t.normal.z*this.min.z,i+=t.normal.z*this.max.z):(e+=t.normal.z*this.max.z,i+=t.normal.z*this.min.z),e<=-t.constant&&i>=-t.constant}intersectsTriangle(t){if(this.isEmpty())return!1;this.getCenter(ds),Zs.subVectors(this.max,ds),wn.subVectors(t.a,ds),Tn.subVectors(t.b,ds),An.subVectors(t.c,ds),Ni.subVectors(Tn,wn),Ui.subVectors(An,Tn),Zi.subVectors(wn,An);let e=[0,-Ni.z,Ni.y,0,-Ui.z,Ui.y,0,-Zi.z,Zi.y,Ni.z,0,-Ni.x,Ui.z,0,-Ui.x,Zi.z,0,-Zi.x,-Ni.y,Ni.x,0,-Ui.y,Ui.x,0,-Zi.y,Zi.x,0];return!ia(e,wn,Tn,An,Zs)||(e=[1,0,0,0,1,0,0,0,1],!ia(e,wn,Tn,An,Zs))?!1:(Js.crossVectors(Ni,Ui),e=[Js.x,Js.y,Js.z],ia(e,wn,Tn,An,Zs))}clampPoint(t,e){return e.copy(t).clamp(this.min,this.max)}distanceToPoint(t){return this.clampPoint(t,ei).distanceTo(t)}getBoundingSphere(t){return this.isEmpty()?t.makeEmpty():(this.getCenter(t.center),t.radius=this.getSize(ei).length()*.5),t}intersect(t){return this.min.max(t.min),this.max.min(t.max),this.isEmpty()&&this.makeEmpty(),this}union(t){return this.min.min(t.min),this.max.max(t.max),this}applyMatrix4(t){return this.isEmpty()?this:(xi[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(t),xi[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(t),xi[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(t),xi[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(t),xi[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(t),xi[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(t),xi[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(t),xi[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(t),this.setFromPoints(xi),this)}translate(t){return this.min.add(t),this.max.add(t),this}equals(t){return t.min.equals(this.min)&&t.max.equals(this.max)}toJSON(){return{min:this.min.toArray(),max:this.max.toArray()}}fromJSON(t){return this.min.fromArray(t.min),this.max.fromArray(t.max),this}}const xi=[new C,new C,new C,new C,new C,new C,new C,new C],ei=new C,Ks=new xn,wn=new C,Tn=new C,An=new C,Ni=new C,Ui=new C,Zi=new C,ds=new C,Zs=new C,Js=new C,Ji=new C;function ia(s,t,e,i,n){for(let r=0,a=s.length-3;r<=a;r+=3){Ji.fromArray(s,r);const o=n.x*Math.abs(Ji.x)+n.y*Math.abs(Ji.y)+n.z*Math.abs(Ji.z),c=t.dot(Ji),l=e.dot(Ji),h=i.dot(Ji);if(Math.max(-Math.max(c,l,h),Math.min(c,l,h))>o)return!1}return!0}const Ld=new xn,fs=new C,na=new C;class ls{constructor(t=new C,e=-1){this.isSphere=!0,this.center=t,this.radius=e}set(t,e){return this.center.copy(t),this.radius=e,this}setFromPoints(t,e){const i=this.center;e!==void 0?i.copy(e):Ld.setFromPoints(t).getCenter(i);let n=0;for(let r=0,a=t.length;r<a;r++)n=Math.max(n,i.distanceToSquared(t[r]));return this.radius=Math.sqrt(n),this}copy(t){return this.center.copy(t.center),this.radius=t.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(t){return t.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(t){return t.distanceTo(this.center)-this.radius}intersectsSphere(t){const e=this.radius+t.radius;return t.center.distanceToSquared(this.center)<=e*e}intersectsBox(t){return t.intersectsSphere(this)}intersectsPlane(t){return Math.abs(t.distanceToPoint(this.center))<=this.radius}clampPoint(t,e){const i=this.center.distanceToSquared(t);return e.copy(t),i>this.radius*this.radius&&(e.sub(this.center).normalize(),e.multiplyScalar(this.radius).add(this.center)),e}getBoundingBox(t){return this.isEmpty()?(t.makeEmpty(),t):(t.set(this.center,this.center),t.expandByScalar(this.radius),t)}applyMatrix4(t){return this.center.applyMatrix4(t),this.radius=this.radius*t.getMaxScaleOnAxis(),this}translate(t){return this.center.add(t),this}expandByPoint(t){if(this.isEmpty())return this.center.copy(t),this.radius=0,this;fs.subVectors(t,this.center);const e=fs.lengthSq();if(e>this.radius*this.radius){const i=Math.sqrt(e),n=(i-this.radius)*.5;this.center.addScaledVector(fs,n/i),this.radius+=n}return this}union(t){return t.isEmpty()?this:this.isEmpty()?(this.copy(t),this):(this.center.equals(t.center)===!0?this.radius=Math.max(this.radius,t.radius):(na.subVectors(t.center,this.center).setLength(t.radius),this.expandByPoint(fs.copy(t.center).add(na)),this.expandByPoint(fs.copy(t.center).sub(na))),this)}equals(t){return t.center.equals(this.center)&&t.radius===this.radius}clone(){return new this.constructor().copy(this)}toJSON(){return{radius:this.radius,center:this.center.toArray()}}fromJSON(t){return this.radius=t.radius,this.center.fromArray(t.center),this}}const yi=new C,sa=new C,js=new C,Oi=new C,ra=new C,Qs=new C,aa=new C;class gh{constructor(t=new C,e=new C(0,0,-1)){this.origin=t,this.direction=e}set(t,e){return this.origin.copy(t),this.direction.copy(e),this}copy(t){return this.origin.copy(t.origin),this.direction.copy(t.direction),this}at(t,e){return e.copy(this.origin).addScaledVector(this.direction,t)}lookAt(t){return this.direction.copy(t).sub(this.origin).normalize(),this}recast(t){return this.origin.copy(this.at(t,yi)),this}closestPointToPoint(t,e){e.subVectors(t,this.origin);const i=e.dot(this.direction);return i<0?e.copy(this.origin):e.copy(this.origin).addScaledVector(this.direction,i)}distanceToPoint(t){return Math.sqrt(this.distanceSqToPoint(t))}distanceSqToPoint(t){const e=yi.subVectors(t,this.origin).dot(this.direction);return e<0?this.origin.distanceToSquared(t):(yi.copy(this.origin).addScaledVector(this.direction,e),yi.distanceToSquared(t))}distanceSqToSegment(t,e,i,n){sa.copy(t).add(e).multiplyScalar(.5),js.copy(e).sub(t).normalize(),Oi.copy(this.origin).sub(sa);const r=t.distanceTo(e)*.5,a=-this.direction.dot(js),o=Oi.dot(this.direction),c=-Oi.dot(js),l=Oi.lengthSq(),h=Math.abs(1-a*a);let u,d,f,g;if(h>0)if(u=a*c-o,d=a*o-c,g=r*h,u>=0)if(d>=-g)if(d<=g){const v=1/h;u*=v,d*=v,f=u*(u+a*d+2*o)+d*(a*u+d+2*c)+l}else d=r,u=Math.max(0,-(a*d+o)),f=-u*u+d*(d+2*c)+l;else d=-r,u=Math.max(0,-(a*d+o)),f=-u*u+d*(d+2*c)+l;else d<=-g?(u=Math.max(0,-(-a*r+o)),d=u>0?-r:Math.min(Math.max(-r,-c),r),f=-u*u+d*(d+2*c)+l):d<=g?(u=0,d=Math.min(Math.max(-r,-c),r),f=d*(d+2*c)+l):(u=Math.max(0,-(a*r+o)),d=u>0?r:Math.min(Math.max(-r,-c),r),f=-u*u+d*(d+2*c)+l);else d=a>0?-r:r,u=Math.max(0,-(a*d+o)),f=-u*u+d*(d+2*c)+l;return i&&i.copy(this.origin).addScaledVector(this.direction,u),n&&n.copy(sa).addScaledVector(js,d),f}intersectSphere(t,e){yi.subVectors(t.center,this.origin);const i=yi.dot(this.direction),n=yi.dot(yi)-i*i,r=t.radius*t.radius;if(n>r)return null;const a=Math.sqrt(r-n),o=i-a,c=i+a;return c<0?null:o<0?this.at(c,e):this.at(o,e)}intersectsSphere(t){return t.radius<0?!1:this.distanceSqToPoint(t.center)<=t.radius*t.radius}distanceToPlane(t){const e=t.normal.dot(this.direction);if(e===0)return t.distanceToPoint(this.origin)===0?0:null;const i=-(this.origin.dot(t.normal)+t.constant)/e;return i>=0?i:null}intersectPlane(t,e){const i=this.distanceToPlane(t);return i===null?null:this.at(i,e)}intersectsPlane(t){const e=t.distanceToPoint(this.origin);return e===0||t.normal.dot(this.direction)*e<0}intersectBox(t,e){let i,n,r,a,o,c;const l=1/this.direction.x,h=1/this.direction.y,u=1/this.direction.z,d=this.origin;return l>=0?(i=(t.min.x-d.x)*l,n=(t.max.x-d.x)*l):(i=(t.max.x-d.x)*l,n=(t.min.x-d.x)*l),h>=0?(r=(t.min.y-d.y)*h,a=(t.max.y-d.y)*h):(r=(t.max.y-d.y)*h,a=(t.min.y-d.y)*h),i>a||r>n||((r>i||isNaN(i))&&(i=r),(a<n||isNaN(n))&&(n=a),u>=0?(o=(t.min.z-d.z)*u,c=(t.max.z-d.z)*u):(o=(t.max.z-d.z)*u,c=(t.min.z-d.z)*u),i>c||o>n)||((o>i||i!==i)&&(i=o),(c<n||n!==n)&&(n=c),n<0)?null:this.at(i>=0?i:n,e)}intersectsBox(t){return this.intersectBox(t,yi)!==null}intersectTriangle(t,e,i,n,r){ra.subVectors(e,t),Qs.subVectors(i,t),aa.crossVectors(ra,Qs);let a=this.direction.dot(aa),o;if(a>0){if(n)return null;o=1}else if(a<0)o=-1,a=-a;else return null;Oi.subVectors(this.origin,t);const c=o*this.direction.dot(Qs.crossVectors(Oi,Qs));if(c<0)return null;const l=o*this.direction.dot(ra.cross(Oi));if(l<0||c+l>a)return null;const h=-o*Oi.dot(aa);return h<0?null:this.at(h/a,r)}applyMatrix4(t){return this.origin.applyMatrix4(t),this.direction.transformDirection(t),this}equals(t){return t.origin.equals(this.origin)&&t.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}}class ie{constructor(t,e,i,n,r,a,o,c,l,h,u,d,f,g,v,m){ie.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],t!==void 0&&this.set(t,e,i,n,r,a,o,c,l,h,u,d,f,g,v,m)}set(t,e,i,n,r,a,o,c,l,h,u,d,f,g,v,m){const p=this.elements;return p[0]=t,p[4]=e,p[8]=i,p[12]=n,p[1]=r,p[5]=a,p[9]=o,p[13]=c,p[2]=l,p[6]=h,p[10]=u,p[14]=d,p[3]=f,p[7]=g,p[11]=v,p[15]=m,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new ie().fromArray(this.elements)}copy(t){const e=this.elements,i=t.elements;return e[0]=i[0],e[1]=i[1],e[2]=i[2],e[3]=i[3],e[4]=i[4],e[5]=i[5],e[6]=i[6],e[7]=i[7],e[8]=i[8],e[9]=i[9],e[10]=i[10],e[11]=i[11],e[12]=i[12],e[13]=i[13],e[14]=i[14],e[15]=i[15],this}copyPosition(t){const e=this.elements,i=t.elements;return e[12]=i[12],e[13]=i[13],e[14]=i[14],this}setFromMatrix3(t){const e=t.elements;return this.set(e[0],e[3],e[6],0,e[1],e[4],e[7],0,e[2],e[5],e[8],0,0,0,0,1),this}extractBasis(t,e,i){return t.setFromMatrixColumn(this,0),e.setFromMatrixColumn(this,1),i.setFromMatrixColumn(this,2),this}makeBasis(t,e,i){return this.set(t.x,e.x,i.x,0,t.y,e.y,i.y,0,t.z,e.z,i.z,0,0,0,0,1),this}extractRotation(t){const e=this.elements,i=t.elements,n=1/Rn.setFromMatrixColumn(t,0).length(),r=1/Rn.setFromMatrixColumn(t,1).length(),a=1/Rn.setFromMatrixColumn(t,2).length();return e[0]=i[0]*n,e[1]=i[1]*n,e[2]=i[2]*n,e[3]=0,e[4]=i[4]*r,e[5]=i[5]*r,e[6]=i[6]*r,e[7]=0,e[8]=i[8]*a,e[9]=i[9]*a,e[10]=i[10]*a,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromEuler(t){const e=this.elements,i=t.x,n=t.y,r=t.z,a=Math.cos(i),o=Math.sin(i),c=Math.cos(n),l=Math.sin(n),h=Math.cos(r),u=Math.sin(r);if(t.order==="XYZ"){const d=a*h,f=a*u,g=o*h,v=o*u;e[0]=c*h,e[4]=-c*u,e[8]=l,e[1]=f+g*l,e[5]=d-v*l,e[9]=-o*c,e[2]=v-d*l,e[6]=g+f*l,e[10]=a*c}else if(t.order==="YXZ"){const d=c*h,f=c*u,g=l*h,v=l*u;e[0]=d+v*o,e[4]=g*o-f,e[8]=a*l,e[1]=a*u,e[5]=a*h,e[9]=-o,e[2]=f*o-g,e[6]=v+d*o,e[10]=a*c}else if(t.order==="ZXY"){const d=c*h,f=c*u,g=l*h,v=l*u;e[0]=d-v*o,e[4]=-a*u,e[8]=g+f*o,e[1]=f+g*o,e[5]=a*h,e[9]=v-d*o,e[2]=-a*l,e[6]=o,e[10]=a*c}else if(t.order==="ZYX"){const d=a*h,f=a*u,g=o*h,v=o*u;e[0]=c*h,e[4]=g*l-f,e[8]=d*l+v,e[1]=c*u,e[5]=v*l+d,e[9]=f*l-g,e[2]=-l,e[6]=o*c,e[10]=a*c}else if(t.order==="YZX"){const d=a*c,f=a*l,g=o*c,v=o*l;e[0]=c*h,e[4]=v-d*u,e[8]=g*u+f,e[1]=u,e[5]=a*h,e[9]=-o*h,e[2]=-l*h,e[6]=f*u+g,e[10]=d-v*u}else if(t.order==="XZY"){const d=a*c,f=a*l,g=o*c,v=o*l;e[0]=c*h,e[4]=-u,e[8]=l*h,e[1]=d*u+v,e[5]=a*h,e[9]=f*u-g,e[2]=g*u-f,e[6]=o*h,e[10]=v*u+d}return e[3]=0,e[7]=0,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromQuaternion(t){return this.compose(Nd,t,Ud)}lookAt(t,e,i){const n=this.elements;return ze.subVectors(t,e),ze.lengthSq()===0&&(ze.z=1),ze.normalize(),Fi.crossVectors(i,ze),Fi.lengthSq()===0&&(Math.abs(i.z)===1?ze.x+=1e-4:ze.z+=1e-4,ze.normalize(),Fi.crossVectors(i,ze)),Fi.normalize(),tr.crossVectors(ze,Fi),n[0]=Fi.x,n[4]=tr.x,n[8]=ze.x,n[1]=Fi.y,n[5]=tr.y,n[9]=ze.y,n[2]=Fi.z,n[6]=tr.z,n[10]=ze.z,this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const i=t.elements,n=e.elements,r=this.elements,a=i[0],o=i[4],c=i[8],l=i[12],h=i[1],u=i[5],d=i[9],f=i[13],g=i[2],v=i[6],m=i[10],p=i[14],w=i[3],E=i[7],_=i[11],P=i[15],R=n[0],I=n[4],T=n[8],S=n[12],b=n[1],L=n[5],F=n[9],B=n[13],Y=n[2],W=n[6],$=n[10],J=n[14],G=n[3],at=n[7],ut=n[11],Et=n[15];return r[0]=a*R+o*b+c*Y+l*G,r[4]=a*I+o*L+c*W+l*at,r[8]=a*T+o*F+c*$+l*ut,r[12]=a*S+o*B+c*J+l*Et,r[1]=h*R+u*b+d*Y+f*G,r[5]=h*I+u*L+d*W+f*at,r[9]=h*T+u*F+d*$+f*ut,r[13]=h*S+u*B+d*J+f*Et,r[2]=g*R+v*b+m*Y+p*G,r[6]=g*I+v*L+m*W+p*at,r[10]=g*T+v*F+m*$+p*ut,r[14]=g*S+v*B+m*J+p*Et,r[3]=w*R+E*b+_*Y+P*G,r[7]=w*I+E*L+_*W+P*at,r[11]=w*T+E*F+_*$+P*ut,r[15]=w*S+E*B+_*J+P*Et,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[4]*=t,e[8]*=t,e[12]*=t,e[1]*=t,e[5]*=t,e[9]*=t,e[13]*=t,e[2]*=t,e[6]*=t,e[10]*=t,e[14]*=t,e[3]*=t,e[7]*=t,e[11]*=t,e[15]*=t,this}determinant(){const t=this.elements,e=t[0],i=t[4],n=t[8],r=t[12],a=t[1],o=t[5],c=t[9],l=t[13],h=t[2],u=t[6],d=t[10],f=t[14],g=t[3],v=t[7],m=t[11],p=t[15];return g*(+r*c*u-n*l*u-r*o*d+i*l*d+n*o*f-i*c*f)+v*(+e*c*f-e*l*d+r*a*d-n*a*f+n*l*h-r*c*h)+m*(+e*l*u-e*o*f-r*a*u+i*a*f+r*o*h-i*l*h)+p*(-n*o*h-e*c*u+e*o*d+n*a*u-i*a*d+i*c*h)}transpose(){const t=this.elements;let e;return e=t[1],t[1]=t[4],t[4]=e,e=t[2],t[2]=t[8],t[8]=e,e=t[6],t[6]=t[9],t[9]=e,e=t[3],t[3]=t[12],t[12]=e,e=t[7],t[7]=t[13],t[13]=e,e=t[11],t[11]=t[14],t[14]=e,this}setPosition(t,e,i){const n=this.elements;return t.isVector3?(n[12]=t.x,n[13]=t.y,n[14]=t.z):(n[12]=t,n[13]=e,n[14]=i),this}invert(){const t=this.elements,e=t[0],i=t[1],n=t[2],r=t[3],a=t[4],o=t[5],c=t[6],l=t[7],h=t[8],u=t[9],d=t[10],f=t[11],g=t[12],v=t[13],m=t[14],p=t[15],w=u*m*l-v*d*l+v*c*f-o*m*f-u*c*p+o*d*p,E=g*d*l-h*m*l-g*c*f+a*m*f+h*c*p-a*d*p,_=h*v*l-g*u*l+g*o*f-a*v*f-h*o*p+a*u*p,P=g*u*c-h*v*c-g*o*d+a*v*d+h*o*m-a*u*m,R=e*w+i*E+n*_+r*P;if(R===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);const I=1/R;return t[0]=w*I,t[1]=(v*d*r-u*m*r-v*n*f+i*m*f+u*n*p-i*d*p)*I,t[2]=(o*m*r-v*c*r+v*n*l-i*m*l-o*n*p+i*c*p)*I,t[3]=(u*c*r-o*d*r-u*n*l+i*d*l+o*n*f-i*c*f)*I,t[4]=E*I,t[5]=(h*m*r-g*d*r+g*n*f-e*m*f-h*n*p+e*d*p)*I,t[6]=(g*c*r-a*m*r-g*n*l+e*m*l+a*n*p-e*c*p)*I,t[7]=(a*d*r-h*c*r+h*n*l-e*d*l-a*n*f+e*c*f)*I,t[8]=_*I,t[9]=(g*u*r-h*v*r-g*i*f+e*v*f+h*i*p-e*u*p)*I,t[10]=(a*v*r-g*o*r+g*i*l-e*v*l-a*i*p+e*o*p)*I,t[11]=(h*o*r-a*u*r-h*i*l+e*u*l+a*i*f-e*o*f)*I,t[12]=P*I,t[13]=(h*v*n-g*u*n+g*i*d-e*v*d-h*i*m+e*u*m)*I,t[14]=(g*o*n-a*v*n-g*i*c+e*v*c+a*i*m-e*o*m)*I,t[15]=(a*u*n-h*o*n+h*i*c-e*u*c-a*i*d+e*o*d)*I,this}scale(t){const e=this.elements,i=t.x,n=t.y,r=t.z;return e[0]*=i,e[4]*=n,e[8]*=r,e[1]*=i,e[5]*=n,e[9]*=r,e[2]*=i,e[6]*=n,e[10]*=r,e[3]*=i,e[7]*=n,e[11]*=r,this}getMaxScaleOnAxis(){const t=this.elements,e=t[0]*t[0]+t[1]*t[1]+t[2]*t[2],i=t[4]*t[4]+t[5]*t[5]+t[6]*t[6],n=t[8]*t[8]+t[9]*t[9]+t[10]*t[10];return Math.sqrt(Math.max(e,i,n))}makeTranslation(t,e,i){return t.isVector3?this.set(1,0,0,t.x,0,1,0,t.y,0,0,1,t.z,0,0,0,1):this.set(1,0,0,t,0,1,0,e,0,0,1,i,0,0,0,1),this}makeRotationX(t){const e=Math.cos(t),i=Math.sin(t);return this.set(1,0,0,0,0,e,-i,0,0,i,e,0,0,0,0,1),this}makeRotationY(t){const e=Math.cos(t),i=Math.sin(t);return this.set(e,0,i,0,0,1,0,0,-i,0,e,0,0,0,0,1),this}makeRotationZ(t){const e=Math.cos(t),i=Math.sin(t);return this.set(e,-i,0,0,i,e,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(t,e){const i=Math.cos(e),n=Math.sin(e),r=1-i,a=t.x,o=t.y,c=t.z,l=r*a,h=r*o;return this.set(l*a+i,l*o-n*c,l*c+n*o,0,l*o+n*c,h*o+i,h*c-n*a,0,l*c-n*o,h*c+n*a,r*c*c+i,0,0,0,0,1),this}makeScale(t,e,i){return this.set(t,0,0,0,0,e,0,0,0,0,i,0,0,0,0,1),this}makeShear(t,e,i,n,r,a){return this.set(1,i,r,0,t,1,a,0,e,n,1,0,0,0,0,1),this}compose(t,e,i){const n=this.elements,r=e._x,a=e._y,o=e._z,c=e._w,l=r+r,h=a+a,u=o+o,d=r*l,f=r*h,g=r*u,v=a*h,m=a*u,p=o*u,w=c*l,E=c*h,_=c*u,P=i.x,R=i.y,I=i.z;return n[0]=(1-(v+p))*P,n[1]=(f+_)*P,n[2]=(g-E)*P,n[3]=0,n[4]=(f-_)*R,n[5]=(1-(d+p))*R,n[6]=(m+w)*R,n[7]=0,n[8]=(g+E)*I,n[9]=(m-w)*I,n[10]=(1-(d+v))*I,n[11]=0,n[12]=t.x,n[13]=t.y,n[14]=t.z,n[15]=1,this}decompose(t,e,i){const n=this.elements;let r=Rn.set(n[0],n[1],n[2]).length();const a=Rn.set(n[4],n[5],n[6]).length(),o=Rn.set(n[8],n[9],n[10]).length();this.determinant()<0&&(r=-r),t.x=n[12],t.y=n[13],t.z=n[14],ii.copy(this);const l=1/r,h=1/a,u=1/o;return ii.elements[0]*=l,ii.elements[1]*=l,ii.elements[2]*=l,ii.elements[4]*=h,ii.elements[5]*=h,ii.elements[6]*=h,ii.elements[8]*=u,ii.elements[9]*=u,ii.elements[10]*=u,e.setFromRotationMatrix(ii),i.x=r,i.y=a,i.z=o,this}makePerspective(t,e,i,n,r,a,o=fi,c=!1){const l=this.elements,h=2*r/(e-t),u=2*r/(i-n),d=(e+t)/(e-t),f=(i+n)/(i-n);let g,v;if(c)g=r/(a-r),v=a*r/(a-r);else if(o===fi)g=-(a+r)/(a-r),v=-2*a*r/(a-r);else if(o===Ur)g=-a/(a-r),v=-a*r/(a-r);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+o);return l[0]=h,l[4]=0,l[8]=d,l[12]=0,l[1]=0,l[5]=u,l[9]=f,l[13]=0,l[2]=0,l[6]=0,l[10]=g,l[14]=v,l[3]=0,l[7]=0,l[11]=-1,l[15]=0,this}makeOrthographic(t,e,i,n,r,a,o=fi,c=!1){const l=this.elements,h=2/(e-t),u=2/(i-n),d=-(e+t)/(e-t),f=-(i+n)/(i-n);let g,v;if(c)g=1/(a-r),v=a/(a-r);else if(o===fi)g=-2/(a-r),v=-(a+r)/(a-r);else if(o===Ur)g=-1/(a-r),v=-r/(a-r);else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+o);return l[0]=h,l[4]=0,l[8]=0,l[12]=d,l[1]=0,l[5]=u,l[9]=0,l[13]=f,l[2]=0,l[6]=0,l[10]=g,l[14]=v,l[3]=0,l[7]=0,l[11]=0,l[15]=1,this}equals(t){const e=this.elements,i=t.elements;for(let n=0;n<16;n++)if(e[n]!==i[n])return!1;return!0}fromArray(t,e=0){for(let i=0;i<16;i++)this.elements[i]=t[i+e];return this}toArray(t=[],e=0){const i=this.elements;return t[e]=i[0],t[e+1]=i[1],t[e+2]=i[2],t[e+3]=i[3],t[e+4]=i[4],t[e+5]=i[5],t[e+6]=i[6],t[e+7]=i[7],t[e+8]=i[8],t[e+9]=i[9],t[e+10]=i[10],t[e+11]=i[11],t[e+12]=i[12],t[e+13]=i[13],t[e+14]=i[14],t[e+15]=i[15],t}}const Rn=new C,ii=new ie,Nd=new C(0,0,0),Ud=new C(1,1,1),Fi=new C,tr=new C,ze=new C,Fl=new ie,kl=new Vs;class gi{constructor(t=0,e=0,i=0,n=gi.DEFAULT_ORDER){this.isEuler=!0,this._x=t,this._y=e,this._z=i,this._order=n}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get order(){return this._order}set order(t){this._order=t,this._onChangeCallback()}set(t,e,i,n=this._order){return this._x=t,this._y=e,this._z=i,this._order=n,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(t){return this._x=t._x,this._y=t._y,this._z=t._z,this._order=t._order,this._onChangeCallback(),this}setFromRotationMatrix(t,e=this._order,i=!0){const n=t.elements,r=n[0],a=n[4],o=n[8],c=n[1],l=n[5],h=n[9],u=n[2],d=n[6],f=n[10];switch(e){case"XYZ":this._y=Math.asin(Wt(o,-1,1)),Math.abs(o)<.9999999?(this._x=Math.atan2(-h,f),this._z=Math.atan2(-a,r)):(this._x=Math.atan2(d,l),this._z=0);break;case"YXZ":this._x=Math.asin(-Wt(h,-1,1)),Math.abs(h)<.9999999?(this._y=Math.atan2(o,f),this._z=Math.atan2(c,l)):(this._y=Math.atan2(-u,r),this._z=0);break;case"ZXY":this._x=Math.asin(Wt(d,-1,1)),Math.abs(d)<.9999999?(this._y=Math.atan2(-u,f),this._z=Math.atan2(-a,l)):(this._y=0,this._z=Math.atan2(c,r));break;case"ZYX":this._y=Math.asin(-Wt(u,-1,1)),Math.abs(u)<.9999999?(this._x=Math.atan2(d,f),this._z=Math.atan2(c,r)):(this._x=0,this._z=Math.atan2(-a,l));break;case"YZX":this._z=Math.asin(Wt(c,-1,1)),Math.abs(c)<.9999999?(this._x=Math.atan2(-h,l),this._y=Math.atan2(-u,r)):(this._x=0,this._y=Math.atan2(o,f));break;case"XZY":this._z=Math.asin(-Wt(a,-1,1)),Math.abs(a)<.9999999?(this._x=Math.atan2(d,l),this._y=Math.atan2(o,r)):(this._x=Math.atan2(-h,f),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+e)}return this._order=e,i===!0&&this._onChangeCallback(),this}setFromQuaternion(t,e,i){return Fl.makeRotationFromQuaternion(t),this.setFromRotationMatrix(Fl,e,i)}setFromVector3(t,e=this._order){return this.set(t.x,t.y,t.z,e)}reorder(t){return kl.setFromEuler(this),this.setFromQuaternion(kl,t)}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._order===this._order}fromArray(t){return this._x=t[0],this._y=t[1],this._z=t[2],t[3]!==void 0&&(this._order=t[3]),this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._order,t}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}}gi.DEFAULT_ORDER="XYZ";class vh{constructor(){this.mask=1}set(t){this.mask=(1<<t|0)>>>0}enable(t){this.mask|=1<<t|0}enableAll(){this.mask=-1}toggle(t){this.mask^=1<<t|0}disable(t){this.mask&=~(1<<t|0)}disableAll(){this.mask=0}test(t){return(this.mask&t.mask)!==0}isEnabled(t){return(this.mask&(1<<t|0))!==0}}let Od=0;const zl=new C,Cn=new Vs,Mi=new ie,er=new C,ps=new C,Fd=new C,kd=new Vs,Bl=new C(1,0,0),Hl=new C(0,1,0),Gl=new C(0,0,1),Vl={type:"added"},zd={type:"removed"},Pn={type:"childadded",child:null},oa={type:"childremoved",child:null};class _e extends os{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:Od++}),this.uuid=Ai(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=_e.DEFAULT_UP.clone();const t=new C,e=new gi,i=new Vs,n=new C(1,1,1);function r(){i.setFromEuler(e,!1)}function a(){e.setFromQuaternion(i,void 0,!1)}e._onChange(r),i._onChange(a),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:t},rotation:{configurable:!0,enumerable:!0,value:e},quaternion:{configurable:!0,enumerable:!0,value:i},scale:{configurable:!0,enumerable:!0,value:n},modelViewMatrix:{value:new ie},normalMatrix:{value:new Ot}}),this.matrix=new ie,this.matrixWorld=new ie,this.matrixAutoUpdate=_e.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=_e.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new vh,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.customDepthMaterial=void 0,this.customDistanceMaterial=void 0,this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(t){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(t),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(t){return this.quaternion.premultiply(t),this}setRotationFromAxisAngle(t,e){this.quaternion.setFromAxisAngle(t,e)}setRotationFromEuler(t){this.quaternion.setFromEuler(t,!0)}setRotationFromMatrix(t){this.quaternion.setFromRotationMatrix(t)}setRotationFromQuaternion(t){this.quaternion.copy(t)}rotateOnAxis(t,e){return Cn.setFromAxisAngle(t,e),this.quaternion.multiply(Cn),this}rotateOnWorldAxis(t,e){return Cn.setFromAxisAngle(t,e),this.quaternion.premultiply(Cn),this}rotateX(t){return this.rotateOnAxis(Bl,t)}rotateY(t){return this.rotateOnAxis(Hl,t)}rotateZ(t){return this.rotateOnAxis(Gl,t)}translateOnAxis(t,e){return zl.copy(t).applyQuaternion(this.quaternion),this.position.add(zl.multiplyScalar(e)),this}translateX(t){return this.translateOnAxis(Bl,t)}translateY(t){return this.translateOnAxis(Hl,t)}translateZ(t){return this.translateOnAxis(Gl,t)}localToWorld(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(this.matrixWorld)}worldToLocal(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(Mi.copy(this.matrixWorld).invert())}lookAt(t,e,i){t.isVector3?er.copy(t):er.set(t,e,i);const n=this.parent;this.updateWorldMatrix(!0,!1),ps.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?Mi.lookAt(ps,er,this.up):Mi.lookAt(er,ps,this.up),this.quaternion.setFromRotationMatrix(Mi),n&&(Mi.extractRotation(n.matrixWorld),Cn.setFromRotationMatrix(Mi),this.quaternion.premultiply(Cn.invert()))}add(t){if(arguments.length>1){for(let e=0;e<arguments.length;e++)this.add(arguments[e]);return this}return t===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",t),this):(t&&t.isObject3D?(t.removeFromParent(),t.parent=this,this.children.push(t),t.dispatchEvent(Vl),Pn.child=t,this.dispatchEvent(Pn),Pn.child=null):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",t),this)}remove(t){if(arguments.length>1){for(let i=0;i<arguments.length;i++)this.remove(arguments[i]);return this}const e=this.children.indexOf(t);return e!==-1&&(t.parent=null,this.children.splice(e,1),t.dispatchEvent(zd),oa.child=t,this.dispatchEvent(oa),oa.child=null),this}removeFromParent(){const t=this.parent;return t!==null&&t.remove(this),this}clear(){return this.remove(...this.children)}attach(t){return this.updateWorldMatrix(!0,!1),Mi.copy(this.matrixWorld).invert(),t.parent!==null&&(t.parent.updateWorldMatrix(!0,!1),Mi.multiply(t.parent.matrixWorld)),t.applyMatrix4(Mi),t.removeFromParent(),t.parent=this,this.children.push(t),t.updateWorldMatrix(!1,!0),t.dispatchEvent(Vl),Pn.child=t,this.dispatchEvent(Pn),Pn.child=null,this}getObjectById(t){return this.getObjectByProperty("id",t)}getObjectByName(t){return this.getObjectByProperty("name",t)}getObjectByProperty(t,e){if(this[t]===e)return this;for(let i=0,n=this.children.length;i<n;i++){const a=this.children[i].getObjectByProperty(t,e);if(a!==void 0)return a}}getObjectsByProperty(t,e,i=[]){this[t]===e&&i.push(this);const n=this.children;for(let r=0,a=n.length;r<a;r++)n[r].getObjectsByProperty(t,e,i);return i}getWorldPosition(t){return this.updateWorldMatrix(!0,!1),t.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(ps,t,Fd),t}getWorldScale(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(ps,kd,t),t}getWorldDirection(t){this.updateWorldMatrix(!0,!1);const e=this.matrixWorld.elements;return t.set(e[8],e[9],e[10]).normalize()}raycast(){}traverse(t){t(this);const e=this.children;for(let i=0,n=e.length;i<n;i++)e[i].traverse(t)}traverseVisible(t){if(this.visible===!1)return;t(this);const e=this.children;for(let i=0,n=e.length;i<n;i++)e[i].traverseVisible(t)}traverseAncestors(t){const e=this.parent;e!==null&&(t(e),e.traverseAncestors(t))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(t){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||t)&&(this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),this.matrixWorldNeedsUpdate=!1,t=!0);const e=this.children;for(let i=0,n=e.length;i<n;i++)e[i].updateMatrixWorld(t)}updateWorldMatrix(t,e){const i=this.parent;if(t===!0&&i!==null&&i.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),e===!0){const n=this.children;for(let r=0,a=n.length;r<a;r++)n[r].updateWorldMatrix(!1,!0)}}toJSON(t){const e=t===void 0||typeof t=="string",i={};e&&(t={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},i.metadata={version:4.7,type:"Object",generator:"Object3D.toJSON"});const n={};n.uuid=this.uuid,n.type=this.type,this.name!==""&&(n.name=this.name),this.castShadow===!0&&(n.castShadow=!0),this.receiveShadow===!0&&(n.receiveShadow=!0),this.visible===!1&&(n.visible=!1),this.frustumCulled===!1&&(n.frustumCulled=!1),this.renderOrder!==0&&(n.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(n.userData=this.userData),n.layers=this.layers.mask,n.matrix=this.matrix.toArray(),n.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(n.matrixAutoUpdate=!1),this.isInstancedMesh&&(n.type="InstancedMesh",n.count=this.count,n.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(n.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(n.type="BatchedMesh",n.perObjectFrustumCulled=this.perObjectFrustumCulled,n.sortObjects=this.sortObjects,n.drawRanges=this._drawRanges,n.reservedRanges=this._reservedRanges,n.geometryInfo=this._geometryInfo.map(o=>({...o,boundingBox:o.boundingBox?o.boundingBox.toJSON():void 0,boundingSphere:o.boundingSphere?o.boundingSphere.toJSON():void 0})),n.instanceInfo=this._instanceInfo.map(o=>({...o})),n.availableInstanceIds=this._availableInstanceIds.slice(),n.availableGeometryIds=this._availableGeometryIds.slice(),n.nextIndexStart=this._nextIndexStart,n.nextVertexStart=this._nextVertexStart,n.geometryCount=this._geometryCount,n.maxInstanceCount=this._maxInstanceCount,n.maxVertexCount=this._maxVertexCount,n.maxIndexCount=this._maxIndexCount,n.geometryInitialized=this._geometryInitialized,n.matricesTexture=this._matricesTexture.toJSON(t),n.indirectTexture=this._indirectTexture.toJSON(t),this._colorsTexture!==null&&(n.colorsTexture=this._colorsTexture.toJSON(t)),this.boundingSphere!==null&&(n.boundingSphere=this.boundingSphere.toJSON()),this.boundingBox!==null&&(n.boundingBox=this.boundingBox.toJSON()));function r(o,c){return o[c.uuid]===void 0&&(o[c.uuid]=c.toJSON(t)),c.uuid}if(this.isScene)this.background&&(this.background.isColor?n.background=this.background.toJSON():this.background.isTexture&&(n.background=this.background.toJSON(t).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(n.environment=this.environment.toJSON(t).uuid);else if(this.isMesh||this.isLine||this.isPoints){n.geometry=r(t.geometries,this.geometry);const o=this.geometry.parameters;if(o!==void 0&&o.shapes!==void 0){const c=o.shapes;if(Array.isArray(c))for(let l=0,h=c.length;l<h;l++){const u=c[l];r(t.shapes,u)}else r(t.shapes,c)}}if(this.isSkinnedMesh&&(n.bindMode=this.bindMode,n.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(r(t.skeletons,this.skeleton),n.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){const o=[];for(let c=0,l=this.material.length;c<l;c++)o.push(r(t.materials,this.material[c]));n.material=o}else n.material=r(t.materials,this.material);if(this.children.length>0){n.children=[];for(let o=0;o<this.children.length;o++)n.children.push(this.children[o].toJSON(t).object)}if(this.animations.length>0){n.animations=[];for(let o=0;o<this.animations.length;o++){const c=this.animations[o];n.animations.push(r(t.animations,c))}}if(e){const o=a(t.geometries),c=a(t.materials),l=a(t.textures),h=a(t.images),u=a(t.shapes),d=a(t.skeletons),f=a(t.animations),g=a(t.nodes);o.length>0&&(i.geometries=o),c.length>0&&(i.materials=c),l.length>0&&(i.textures=l),h.length>0&&(i.images=h),u.length>0&&(i.shapes=u),d.length>0&&(i.skeletons=d),f.length>0&&(i.animations=f),g.length>0&&(i.nodes=g)}return i.object=n,i;function a(o){const c=[];for(const l in o){const h=o[l];delete h.metadata,c.push(h)}return c}}clone(t){return new this.constructor().copy(this,t)}copy(t,e=!0){if(this.name=t.name,this.up.copy(t.up),this.position.copy(t.position),this.rotation.order=t.rotation.order,this.quaternion.copy(t.quaternion),this.scale.copy(t.scale),this.matrix.copy(t.matrix),this.matrixWorld.copy(t.matrixWorld),this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrixWorldAutoUpdate=t.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=t.matrixWorldNeedsUpdate,this.layers.mask=t.layers.mask,this.visible=t.visible,this.castShadow=t.castShadow,this.receiveShadow=t.receiveShadow,this.frustumCulled=t.frustumCulled,this.renderOrder=t.renderOrder,this.animations=t.animations.slice(),this.userData=JSON.parse(JSON.stringify(t.userData)),e===!0)for(let i=0;i<t.children.length;i++){const n=t.children[i];this.add(n.clone())}return this}}_e.DEFAULT_UP=new C(0,1,0);_e.DEFAULT_MATRIX_AUTO_UPDATE=!0;_e.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;const ni=new C,Si=new C,la=new C,bi=new C,In=new C,Dn=new C,Wl=new C,ca=new C,ha=new C,ua=new C,da=new pe,fa=new pe,pa=new pe;class Qe{constructor(t=new C,e=new C,i=new C){this.a=t,this.b=e,this.c=i}static getNormal(t,e,i,n){n.subVectors(i,e),ni.subVectors(t,e),n.cross(ni);const r=n.lengthSq();return r>0?n.multiplyScalar(1/Math.sqrt(r)):n.set(0,0,0)}static getBarycoord(t,e,i,n,r){ni.subVectors(n,e),Si.subVectors(i,e),la.subVectors(t,e);const a=ni.dot(ni),o=ni.dot(Si),c=ni.dot(la),l=Si.dot(Si),h=Si.dot(la),u=a*l-o*o;if(u===0)return r.set(0,0,0),null;const d=1/u,f=(l*c-o*h)*d,g=(a*h-o*c)*d;return r.set(1-f-g,g,f)}static containsPoint(t,e,i,n){return this.getBarycoord(t,e,i,n,bi)===null?!1:bi.x>=0&&bi.y>=0&&bi.x+bi.y<=1}static getInterpolation(t,e,i,n,r,a,o,c){return this.getBarycoord(t,e,i,n,bi)===null?(c.x=0,c.y=0,"z"in c&&(c.z=0),"w"in c&&(c.w=0),null):(c.setScalar(0),c.addScaledVector(r,bi.x),c.addScaledVector(a,bi.y),c.addScaledVector(o,bi.z),c)}static getInterpolatedAttribute(t,e,i,n,r,a){return da.setScalar(0),fa.setScalar(0),pa.setScalar(0),da.fromBufferAttribute(t,e),fa.fromBufferAttribute(t,i),pa.fromBufferAttribute(t,n),a.setScalar(0),a.addScaledVector(da,r.x),a.addScaledVector(fa,r.y),a.addScaledVector(pa,r.z),a}static isFrontFacing(t,e,i,n){return ni.subVectors(i,e),Si.subVectors(t,e),ni.cross(Si).dot(n)<0}set(t,e,i){return this.a.copy(t),this.b.copy(e),this.c.copy(i),this}setFromPointsAndIndices(t,e,i,n){return this.a.copy(t[e]),this.b.copy(t[i]),this.c.copy(t[n]),this}setFromAttributeAndIndices(t,e,i,n){return this.a.fromBufferAttribute(t,e),this.b.fromBufferAttribute(t,i),this.c.fromBufferAttribute(t,n),this}clone(){return new this.constructor().copy(this)}copy(t){return this.a.copy(t.a),this.b.copy(t.b),this.c.copy(t.c),this}getArea(){return ni.subVectors(this.c,this.b),Si.subVectors(this.a,this.b),ni.cross(Si).length()*.5}getMidpoint(t){return t.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(t){return Qe.getNormal(this.a,this.b,this.c,t)}getPlane(t){return t.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(t,e){return Qe.getBarycoord(t,this.a,this.b,this.c,e)}getInterpolation(t,e,i,n,r){return Qe.getInterpolation(t,this.a,this.b,this.c,e,i,n,r)}containsPoint(t){return Qe.containsPoint(t,this.a,this.b,this.c)}isFrontFacing(t){return Qe.isFrontFacing(this.a,this.b,this.c,t)}intersectsBox(t){return t.intersectsTriangle(this)}closestPointToPoint(t,e){const i=this.a,n=this.b,r=this.c;let a,o;In.subVectors(n,i),Dn.subVectors(r,i),ca.subVectors(t,i);const c=In.dot(ca),l=Dn.dot(ca);if(c<=0&&l<=0)return e.copy(i);ha.subVectors(t,n);const h=In.dot(ha),u=Dn.dot(ha);if(h>=0&&u<=h)return e.copy(n);const d=c*u-h*l;if(d<=0&&c>=0&&h<=0)return a=c/(c-h),e.copy(i).addScaledVector(In,a);ua.subVectors(t,r);const f=In.dot(ua),g=Dn.dot(ua);if(g>=0&&f<=g)return e.copy(r);const v=f*l-c*g;if(v<=0&&l>=0&&g<=0)return o=l/(l-g),e.copy(i).addScaledVector(Dn,o);const m=h*g-f*u;if(m<=0&&u-h>=0&&f-g>=0)return Wl.subVectors(r,n),o=(u-h)/(u-h+(f-g)),e.copy(n).addScaledVector(Wl,o);const p=1/(m+v+d);return a=v*p,o=d*p,e.copy(i).addScaledVector(In,a).addScaledVector(Dn,o)}equals(t){return t.a.equals(this.a)&&t.b.equals(this.b)&&t.c.equals(this.c)}}const _h={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},ki={h:0,s:0,l:0},ir={h:0,s:0,l:0};function ma(s,t,e){return e<0&&(e+=1),e>1&&(e-=1),e<1/6?s+(t-s)*6*e:e<1/2?t:e<2/3?s+(t-s)*6*(2/3-e):s}class Ht{constructor(t,e,i){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(t,e,i)}set(t,e,i){if(e===void 0&&i===void 0){const n=t;n&&n.isColor?this.copy(n):typeof n=="number"?this.setHex(n):typeof n=="string"&&this.setStyle(n)}else this.setRGB(t,e,i);return this}setScalar(t){return this.r=t,this.g=t,this.b=t,this}setHex(t,e=Ve){return t=Math.floor(t),this.r=(t>>16&255)/255,this.g=(t>>8&255)/255,this.b=(t&255)/255,qt.colorSpaceToWorking(this,e),this}setRGB(t,e,i,n=qt.workingColorSpace){return this.r=t,this.g=e,this.b=i,qt.colorSpaceToWorking(this,n),this}setHSL(t,e,i,n=qt.workingColorSpace){if(t=Ed(t,1),e=Wt(e,0,1),i=Wt(i,0,1),e===0)this.r=this.g=this.b=i;else{const r=i<=.5?i*(1+e):i+e-i*e,a=2*i-r;this.r=ma(a,r,t+1/3),this.g=ma(a,r,t),this.b=ma(a,r,t-1/3)}return qt.colorSpaceToWorking(this,n),this}setStyle(t,e=Ve){function i(r){r!==void 0&&parseFloat(r)<1&&console.warn("THREE.Color: Alpha component of "+t+" will be ignored.")}let n;if(n=/^(\w+)\(([^\)]*)\)/.exec(t)){let r;const a=n[1],o=n[2];switch(a){case"rgb":case"rgba":if(r=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return i(r[4]),this.setRGB(Math.min(255,parseInt(r[1],10))/255,Math.min(255,parseInt(r[2],10))/255,Math.min(255,parseInt(r[3],10))/255,e);if(r=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return i(r[4]),this.setRGB(Math.min(100,parseInt(r[1],10))/100,Math.min(100,parseInt(r[2],10))/100,Math.min(100,parseInt(r[3],10))/100,e);break;case"hsl":case"hsla":if(r=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return i(r[4]),this.setHSL(parseFloat(r[1])/360,parseFloat(r[2])/100,parseFloat(r[3])/100,e);break;default:console.warn("THREE.Color: Unknown color model "+t)}}else if(n=/^\#([A-Fa-f\d]+)$/.exec(t)){const r=n[1],a=r.length;if(a===3)return this.setRGB(parseInt(r.charAt(0),16)/15,parseInt(r.charAt(1),16)/15,parseInt(r.charAt(2),16)/15,e);if(a===6)return this.setHex(parseInt(r,16),e);console.warn("THREE.Color: Invalid hex color "+t)}else if(t&&t.length>0)return this.setColorName(t,e);return this}setColorName(t,e=Ve){const i=_h[t.toLowerCase()];return i!==void 0?this.setHex(i,e):console.warn("THREE.Color: Unknown color "+t),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(t){return this.r=t.r,this.g=t.g,this.b=t.b,this}copySRGBToLinear(t){return this.r=Ri(t.r),this.g=Ri(t.g),this.b=Ri(t.b),this}copyLinearToSRGB(t){return this.r=jn(t.r),this.g=jn(t.g),this.b=jn(t.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(t=Ve){return qt.workingToColorSpace(Re.copy(this),t),Math.round(Wt(Re.r*255,0,255))*65536+Math.round(Wt(Re.g*255,0,255))*256+Math.round(Wt(Re.b*255,0,255))}getHexString(t=Ve){return("000000"+this.getHex(t).toString(16)).slice(-6)}getHSL(t,e=qt.workingColorSpace){qt.workingToColorSpace(Re.copy(this),e);const i=Re.r,n=Re.g,r=Re.b,a=Math.max(i,n,r),o=Math.min(i,n,r);let c,l;const h=(o+a)/2;if(o===a)c=0,l=0;else{const u=a-o;switch(l=h<=.5?u/(a+o):u/(2-a-o),a){case i:c=(n-r)/u+(n<r?6:0);break;case n:c=(r-i)/u+2;break;case r:c=(i-n)/u+4;break}c/=6}return t.h=c,t.s=l,t.l=h,t}getRGB(t,e=qt.workingColorSpace){return qt.workingToColorSpace(Re.copy(this),e),t.r=Re.r,t.g=Re.g,t.b=Re.b,t}getStyle(t=Ve){qt.workingToColorSpace(Re.copy(this),t);const e=Re.r,i=Re.g,n=Re.b;return t!==Ve?`color(${t} ${e.toFixed(3)} ${i.toFixed(3)} ${n.toFixed(3)})`:`rgb(${Math.round(e*255)},${Math.round(i*255)},${Math.round(n*255)})`}offsetHSL(t,e,i){return this.getHSL(ki),this.setHSL(ki.h+t,ki.s+e,ki.l+i)}add(t){return this.r+=t.r,this.g+=t.g,this.b+=t.b,this}addColors(t,e){return this.r=t.r+e.r,this.g=t.g+e.g,this.b=t.b+e.b,this}addScalar(t){return this.r+=t,this.g+=t,this.b+=t,this}sub(t){return this.r=Math.max(0,this.r-t.r),this.g=Math.max(0,this.g-t.g),this.b=Math.max(0,this.b-t.b),this}multiply(t){return this.r*=t.r,this.g*=t.g,this.b*=t.b,this}multiplyScalar(t){return this.r*=t,this.g*=t,this.b*=t,this}lerp(t,e){return this.r+=(t.r-this.r)*e,this.g+=(t.g-this.g)*e,this.b+=(t.b-this.b)*e,this}lerpColors(t,e,i){return this.r=t.r+(e.r-t.r)*i,this.g=t.g+(e.g-t.g)*i,this.b=t.b+(e.b-t.b)*i,this}lerpHSL(t,e){this.getHSL(ki),t.getHSL(ir);const i=Jr(ki.h,ir.h,e),n=Jr(ki.s,ir.s,e),r=Jr(ki.l,ir.l,e);return this.setHSL(i,n,r),this}setFromVector3(t){return this.r=t.x,this.g=t.y,this.b=t.z,this}applyMatrix3(t){const e=this.r,i=this.g,n=this.b,r=t.elements;return this.r=r[0]*e+r[3]*i+r[6]*n,this.g=r[1]*e+r[4]*i+r[7]*n,this.b=r[2]*e+r[5]*i+r[8]*n,this}equals(t){return t.r===this.r&&t.g===this.g&&t.b===this.b}fromArray(t,e=0){return this.r=t[e],this.g=t[e+1],this.b=t[e+2],this}toArray(t=[],e=0){return t[e]=this.r,t[e+1]=this.g,t[e+2]=this.b,t}fromBufferAttribute(t,e){return this.r=t.getX(e),this.g=t.getY(e),this.b=t.getZ(e),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}}const Re=new Ht;Ht.NAMES=_h;let Bd=0;class yn extends os{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:Bd++}),this.uuid=Ai(),this.name="",this.type="Material",this.blending=Jn,this.side=Xi,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=Ga,this.blendDst=Va,this.blendEquation=rn,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new Ht(0,0,0),this.blendAlpha=0,this.depthFunc=ts,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=Il,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=bn,this.stencilZFail=bn,this.stencilZPass=bn,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.allowOverride=!0,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(t){this._alphaTest>0!=t>0&&this.version++,this._alphaTest=t}onBeforeRender(){}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(t){if(t!==void 0)for(const e in t){const i=t[e];if(i===void 0){console.warn(`THREE.Material: parameter '${e}' has value of undefined.`);continue}const n=this[e];if(n===void 0){console.warn(`THREE.Material: '${e}' is not a property of THREE.${this.type}.`);continue}n&&n.isColor?n.set(i):n&&n.isVector3&&i&&i.isVector3?n.copy(i):this[e]=i}}toJSON(t){const e=t===void 0||typeof t=="string";e&&(t={textures:{},images:{}});const i={metadata:{version:4.7,type:"Material",generator:"Material.toJSON"}};i.uuid=this.uuid,i.type=this.type,this.name!==""&&(i.name=this.name),this.color&&this.color.isColor&&(i.color=this.color.getHex()),this.roughness!==void 0&&(i.roughness=this.roughness),this.metalness!==void 0&&(i.metalness=this.metalness),this.sheen!==void 0&&(i.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(i.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(i.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(i.emissive=this.emissive.getHex()),this.emissiveIntensity!==void 0&&this.emissiveIntensity!==1&&(i.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(i.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(i.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(i.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(i.shininess=this.shininess),this.clearcoat!==void 0&&(i.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(i.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(i.clearcoatMap=this.clearcoatMap.toJSON(t).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(i.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(t).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(i.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(t).uuid,i.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.sheenColorMap&&this.sheenColorMap.isTexture&&(i.sheenColorMap=this.sheenColorMap.toJSON(t).uuid),this.sheenRoughnessMap&&this.sheenRoughnessMap.isTexture&&(i.sheenRoughnessMap=this.sheenRoughnessMap.toJSON(t).uuid),this.dispersion!==void 0&&(i.dispersion=this.dispersion),this.iridescence!==void 0&&(i.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(i.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(i.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(i.iridescenceMap=this.iridescenceMap.toJSON(t).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(i.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(t).uuid),this.anisotropy!==void 0&&(i.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(i.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(i.anisotropyMap=this.anisotropyMap.toJSON(t).uuid),this.map&&this.map.isTexture&&(i.map=this.map.toJSON(t).uuid),this.matcap&&this.matcap.isTexture&&(i.matcap=this.matcap.toJSON(t).uuid),this.alphaMap&&this.alphaMap.isTexture&&(i.alphaMap=this.alphaMap.toJSON(t).uuid),this.lightMap&&this.lightMap.isTexture&&(i.lightMap=this.lightMap.toJSON(t).uuid,i.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(i.aoMap=this.aoMap.toJSON(t).uuid,i.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(i.bumpMap=this.bumpMap.toJSON(t).uuid,i.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(i.normalMap=this.normalMap.toJSON(t).uuid,i.normalMapType=this.normalMapType,i.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(i.displacementMap=this.displacementMap.toJSON(t).uuid,i.displacementScale=this.displacementScale,i.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(i.roughnessMap=this.roughnessMap.toJSON(t).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(i.metalnessMap=this.metalnessMap.toJSON(t).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(i.emissiveMap=this.emissiveMap.toJSON(t).uuid),this.specularMap&&this.specularMap.isTexture&&(i.specularMap=this.specularMap.toJSON(t).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(i.specularIntensityMap=this.specularIntensityMap.toJSON(t).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(i.specularColorMap=this.specularColorMap.toJSON(t).uuid),this.envMap&&this.envMap.isTexture&&(i.envMap=this.envMap.toJSON(t).uuid,this.combine!==void 0&&(i.combine=this.combine)),this.envMapRotation!==void 0&&(i.envMapRotation=this.envMapRotation.toArray()),this.envMapIntensity!==void 0&&(i.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(i.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(i.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(i.gradientMap=this.gradientMap.toJSON(t).uuid),this.transmission!==void 0&&(i.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(i.transmissionMap=this.transmissionMap.toJSON(t).uuid),this.thickness!==void 0&&(i.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(i.thicknessMap=this.thicknessMap.toJSON(t).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(i.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(i.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(i.size=this.size),this.shadowSide!==null&&(i.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(i.sizeAttenuation=this.sizeAttenuation),this.blending!==Jn&&(i.blending=this.blending),this.side!==Xi&&(i.side=this.side),this.vertexColors===!0&&(i.vertexColors=!0),this.opacity<1&&(i.opacity=this.opacity),this.transparent===!0&&(i.transparent=!0),this.blendSrc!==Ga&&(i.blendSrc=this.blendSrc),this.blendDst!==Va&&(i.blendDst=this.blendDst),this.blendEquation!==rn&&(i.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(i.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(i.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(i.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(i.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(i.blendAlpha=this.blendAlpha),this.depthFunc!==ts&&(i.depthFunc=this.depthFunc),this.depthTest===!1&&(i.depthTest=this.depthTest),this.depthWrite===!1&&(i.depthWrite=this.depthWrite),this.colorWrite===!1&&(i.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(i.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==Il&&(i.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(i.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(i.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==bn&&(i.stencilFail=this.stencilFail),this.stencilZFail!==bn&&(i.stencilZFail=this.stencilZFail),this.stencilZPass!==bn&&(i.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(i.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(i.rotation=this.rotation),this.polygonOffset===!0&&(i.polygonOffset=!0),this.polygonOffsetFactor!==0&&(i.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(i.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(i.linewidth=this.linewidth),this.dashSize!==void 0&&(i.dashSize=this.dashSize),this.gapSize!==void 0&&(i.gapSize=this.gapSize),this.scale!==void 0&&(i.scale=this.scale),this.dithering===!0&&(i.dithering=!0),this.alphaTest>0&&(i.alphaTest=this.alphaTest),this.alphaHash===!0&&(i.alphaHash=!0),this.alphaToCoverage===!0&&(i.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(i.premultipliedAlpha=!0),this.forceSinglePass===!0&&(i.forceSinglePass=!0),this.wireframe===!0&&(i.wireframe=!0),this.wireframeLinewidth>1&&(i.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(i.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(i.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(i.flatShading=!0),this.visible===!1&&(i.visible=!1),this.toneMapped===!1&&(i.toneMapped=!1),this.fog===!1&&(i.fog=!1),Object.keys(this.userData).length>0&&(i.userData=this.userData);function n(r){const a=[];for(const o in r){const c=r[o];delete c.metadata,a.push(c)}return a}if(e){const r=n(t.textures),a=n(t.images);r.length>0&&(i.textures=r),a.length>0&&(i.images=a)}return i}clone(){return new this.constructor().copy(this)}copy(t){this.name=t.name,this.blending=t.blending,this.side=t.side,this.vertexColors=t.vertexColors,this.opacity=t.opacity,this.transparent=t.transparent,this.blendSrc=t.blendSrc,this.blendDst=t.blendDst,this.blendEquation=t.blendEquation,this.blendSrcAlpha=t.blendSrcAlpha,this.blendDstAlpha=t.blendDstAlpha,this.blendEquationAlpha=t.blendEquationAlpha,this.blendColor.copy(t.blendColor),this.blendAlpha=t.blendAlpha,this.depthFunc=t.depthFunc,this.depthTest=t.depthTest,this.depthWrite=t.depthWrite,this.stencilWriteMask=t.stencilWriteMask,this.stencilFunc=t.stencilFunc,this.stencilRef=t.stencilRef,this.stencilFuncMask=t.stencilFuncMask,this.stencilFail=t.stencilFail,this.stencilZFail=t.stencilZFail,this.stencilZPass=t.stencilZPass,this.stencilWrite=t.stencilWrite;const e=t.clippingPlanes;let i=null;if(e!==null){const n=e.length;i=new Array(n);for(let r=0;r!==n;++r)i[r]=e[r].clone()}return this.clippingPlanes=i,this.clipIntersection=t.clipIntersection,this.clipShadows=t.clipShadows,this.shadowSide=t.shadowSide,this.colorWrite=t.colorWrite,this.precision=t.precision,this.polygonOffset=t.polygonOffset,this.polygonOffsetFactor=t.polygonOffsetFactor,this.polygonOffsetUnits=t.polygonOffsetUnits,this.dithering=t.dithering,this.alphaTest=t.alphaTest,this.alphaHash=t.alphaHash,this.alphaToCoverage=t.alphaToCoverage,this.premultipliedAlpha=t.premultipliedAlpha,this.forceSinglePass=t.forceSinglePass,this.visible=t.visible,this.toneMapped=t.toneMapped,this.userData=JSON.parse(JSON.stringify(t.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(t){t===!0&&this.version++}}class Ge extends yn{constructor(t){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new Ht(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new gi,this.combine=eh,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapRotation.copy(t.envMapRotation),this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.fog=t.fog,this}}const ve=new C,nr=new ht;let Hd=0;class $e{constructor(t,e,i=!1){if(Array.isArray(t))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,Object.defineProperty(this,"id",{value:Hd++}),this.name="",this.array=t,this.itemSize=e,this.count=t!==void 0?t.length/e:0,this.normalized=i,this.usage=Po,this.updateRanges=[],this.gpuType=di,this.version=0}onUploadCallback(){}set needsUpdate(t){t===!0&&this.version++}setUsage(t){return this.usage=t,this}addUpdateRange(t,e){this.updateRanges.push({start:t,count:e})}clearUpdateRanges(){this.updateRanges.length=0}copy(t){return this.name=t.name,this.array=new t.array.constructor(t.array),this.itemSize=t.itemSize,this.count=t.count,this.normalized=t.normalized,this.usage=t.usage,this.gpuType=t.gpuType,this}copyAt(t,e,i){t*=this.itemSize,i*=e.itemSize;for(let n=0,r=this.itemSize;n<r;n++)this.array[t+n]=e.array[i+n];return this}copyArray(t){return this.array.set(t),this}applyMatrix3(t){if(this.itemSize===2)for(let e=0,i=this.count;e<i;e++)nr.fromBufferAttribute(this,e),nr.applyMatrix3(t),this.setXY(e,nr.x,nr.y);else if(this.itemSize===3)for(let e=0,i=this.count;e<i;e++)ve.fromBufferAttribute(this,e),ve.applyMatrix3(t),this.setXYZ(e,ve.x,ve.y,ve.z);return this}applyMatrix4(t){for(let e=0,i=this.count;e<i;e++)ve.fromBufferAttribute(this,e),ve.applyMatrix4(t),this.setXYZ(e,ve.x,ve.y,ve.z);return this}applyNormalMatrix(t){for(let e=0,i=this.count;e<i;e++)ve.fromBufferAttribute(this,e),ve.applyNormalMatrix(t),this.setXYZ(e,ve.x,ve.y,ve.z);return this}transformDirection(t){for(let e=0,i=this.count;e<i;e++)ve.fromBufferAttribute(this,e),ve.transformDirection(t),this.setXYZ(e,ve.x,ve.y,ve.z);return this}set(t,e=0){return this.array.set(t,e),this}getComponent(t,e){let i=this.array[t*this.itemSize+e];return this.normalized&&(i=hi(i,this.array)),i}setComponent(t,e,i){return this.normalized&&(i=ee(i,this.array)),this.array[t*this.itemSize+e]=i,this}getX(t){let e=this.array[t*this.itemSize];return this.normalized&&(e=hi(e,this.array)),e}setX(t,e){return this.normalized&&(e=ee(e,this.array)),this.array[t*this.itemSize]=e,this}getY(t){let e=this.array[t*this.itemSize+1];return this.normalized&&(e=hi(e,this.array)),e}setY(t,e){return this.normalized&&(e=ee(e,this.array)),this.array[t*this.itemSize+1]=e,this}getZ(t){let e=this.array[t*this.itemSize+2];return this.normalized&&(e=hi(e,this.array)),e}setZ(t,e){return this.normalized&&(e=ee(e,this.array)),this.array[t*this.itemSize+2]=e,this}getW(t){let e=this.array[t*this.itemSize+3];return this.normalized&&(e=hi(e,this.array)),e}setW(t,e){return this.normalized&&(e=ee(e,this.array)),this.array[t*this.itemSize+3]=e,this}setXY(t,e,i){return t*=this.itemSize,this.normalized&&(e=ee(e,this.array),i=ee(i,this.array)),this.array[t+0]=e,this.array[t+1]=i,this}setXYZ(t,e,i,n){return t*=this.itemSize,this.normalized&&(e=ee(e,this.array),i=ee(i,this.array),n=ee(n,this.array)),this.array[t+0]=e,this.array[t+1]=i,this.array[t+2]=n,this}setXYZW(t,e,i,n,r){return t*=this.itemSize,this.normalized&&(e=ee(e,this.array),i=ee(i,this.array),n=ee(n,this.array),r=ee(r,this.array)),this.array[t+0]=e,this.array[t+1]=i,this.array[t+2]=n,this.array[t+3]=r,this}onUpload(t){return this.onUploadCallback=t,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){const t={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(t.name=this.name),this.usage!==Po&&(t.usage=this.usage),t}}class xh extends $e{constructor(t,e,i){super(new Uint16Array(t),e,i)}}class yh extends $e{constructor(t,e,i){super(new Uint32Array(t),e,i)}}class jt extends $e{constructor(t,e,i){super(new Float32Array(t),e,i)}}let Gd=0;const Ke=new ie,ga=new _e,Ln=new C,Be=new xn,ms=new xn,be=new C;class we extends os{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:Gd++}),this.uuid=Ai(),this.name="",this.type="BufferGeometry",this.index=null,this.indirect=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(t){return Array.isArray(t)?this.index=new(ph(t)?yh:xh)(t,1):this.index=t,this}setIndirect(t){return this.indirect=t,this}getIndirect(){return this.indirect}getAttribute(t){return this.attributes[t]}setAttribute(t,e){return this.attributes[t]=e,this}deleteAttribute(t){return delete this.attributes[t],this}hasAttribute(t){return this.attributes[t]!==void 0}addGroup(t,e,i=0){this.groups.push({start:t,count:e,materialIndex:i})}clearGroups(){this.groups=[]}setDrawRange(t,e){this.drawRange.start=t,this.drawRange.count=e}applyMatrix4(t){const e=this.attributes.position;e!==void 0&&(e.applyMatrix4(t),e.needsUpdate=!0);const i=this.attributes.normal;if(i!==void 0){const r=new Ot().getNormalMatrix(t);i.applyNormalMatrix(r),i.needsUpdate=!0}const n=this.attributes.tangent;return n!==void 0&&(n.transformDirection(t),n.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(t){return Ke.makeRotationFromQuaternion(t),this.applyMatrix4(Ke),this}rotateX(t){return Ke.makeRotationX(t),this.applyMatrix4(Ke),this}rotateY(t){return Ke.makeRotationY(t),this.applyMatrix4(Ke),this}rotateZ(t){return Ke.makeRotationZ(t),this.applyMatrix4(Ke),this}translate(t,e,i){return Ke.makeTranslation(t,e,i),this.applyMatrix4(Ke),this}scale(t,e,i){return Ke.makeScale(t,e,i),this.applyMatrix4(Ke),this}lookAt(t){return ga.lookAt(t),ga.updateMatrix(),this.applyMatrix4(ga.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(Ln).negate(),this.translate(Ln.x,Ln.y,Ln.z),this}setFromPoints(t){const e=this.getAttribute("position");if(e===void 0){const i=[];for(let n=0,r=t.length;n<r;n++){const a=t[n];i.push(a.x,a.y,a.z||0)}this.setAttribute("position",new jt(i,3))}else{const i=Math.min(t.length,e.count);for(let n=0;n<i;n++){const r=t[n];e.setXYZ(n,r.x,r.y,r.z||0)}t.length>e.count&&console.warn("THREE.BufferGeometry: Buffer size too small for points data. Use .dispose() and create a new geometry."),e.needsUpdate=!0}return this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new xn);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box.",this),this.boundingBox.set(new C(-1/0,-1/0,-1/0),new C(1/0,1/0,1/0));return}if(t!==void 0){if(this.boundingBox.setFromBufferAttribute(t),e)for(let i=0,n=e.length;i<n;i++){const r=e[i];Be.setFromBufferAttribute(r),this.morphTargetsRelative?(be.addVectors(this.boundingBox.min,Be.min),this.boundingBox.expandByPoint(be),be.addVectors(this.boundingBox.max,Be.max),this.boundingBox.expandByPoint(be)):(this.boundingBox.expandByPoint(Be.min),this.boundingBox.expandByPoint(Be.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new ls);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere.",this),this.boundingSphere.set(new C,1/0);return}if(t){const i=this.boundingSphere.center;if(Be.setFromBufferAttribute(t),e)for(let r=0,a=e.length;r<a;r++){const o=e[r];ms.setFromBufferAttribute(o),this.morphTargetsRelative?(be.addVectors(Be.min,ms.min),Be.expandByPoint(be),be.addVectors(Be.max,ms.max),Be.expandByPoint(be)):(Be.expandByPoint(ms.min),Be.expandByPoint(ms.max))}Be.getCenter(i);let n=0;for(let r=0,a=t.count;r<a;r++)be.fromBufferAttribute(t,r),n=Math.max(n,i.distanceToSquared(be));if(e)for(let r=0,a=e.length;r<a;r++){const o=e[r],c=this.morphTargetsRelative;for(let l=0,h=o.count;l<h;l++)be.fromBufferAttribute(o,l),c&&(Ln.fromBufferAttribute(t,l),be.add(Ln)),n=Math.max(n,i.distanceToSquared(be))}this.boundingSphere.radius=Math.sqrt(n),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){const t=this.index,e=this.attributes;if(t===null||e.position===void 0||e.normal===void 0||e.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}const i=e.position,n=e.normal,r=e.uv;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new $e(new Float32Array(4*i.count),4));const a=this.getAttribute("tangent"),o=[],c=[];for(let T=0;T<i.count;T++)o[T]=new C,c[T]=new C;const l=new C,h=new C,u=new C,d=new ht,f=new ht,g=new ht,v=new C,m=new C;function p(T,S,b){l.fromBufferAttribute(i,T),h.fromBufferAttribute(i,S),u.fromBufferAttribute(i,b),d.fromBufferAttribute(r,T),f.fromBufferAttribute(r,S),g.fromBufferAttribute(r,b),h.sub(l),u.sub(l),f.sub(d),g.sub(d);const L=1/(f.x*g.y-g.x*f.y);isFinite(L)&&(v.copy(h).multiplyScalar(g.y).addScaledVector(u,-f.y).multiplyScalar(L),m.copy(u).multiplyScalar(f.x).addScaledVector(h,-g.x).multiplyScalar(L),o[T].add(v),o[S].add(v),o[b].add(v),c[T].add(m),c[S].add(m),c[b].add(m))}let w=this.groups;w.length===0&&(w=[{start:0,count:t.count}]);for(let T=0,S=w.length;T<S;++T){const b=w[T],L=b.start,F=b.count;for(let B=L,Y=L+F;B<Y;B+=3)p(t.getX(B+0),t.getX(B+1),t.getX(B+2))}const E=new C,_=new C,P=new C,R=new C;function I(T){P.fromBufferAttribute(n,T),R.copy(P);const S=o[T];E.copy(S),E.sub(P.multiplyScalar(P.dot(S))).normalize(),_.crossVectors(R,S);const L=_.dot(c[T])<0?-1:1;a.setXYZW(T,E.x,E.y,E.z,L)}for(let T=0,S=w.length;T<S;++T){const b=w[T],L=b.start,F=b.count;for(let B=L,Y=L+F;B<Y;B+=3)I(t.getX(B+0)),I(t.getX(B+1)),I(t.getX(B+2))}}computeVertexNormals(){const t=this.index,e=this.getAttribute("position");if(e!==void 0){let i=this.getAttribute("normal");if(i===void 0)i=new $e(new Float32Array(e.count*3),3),this.setAttribute("normal",i);else for(let d=0,f=i.count;d<f;d++)i.setXYZ(d,0,0,0);const n=new C,r=new C,a=new C,o=new C,c=new C,l=new C,h=new C,u=new C;if(t)for(let d=0,f=t.count;d<f;d+=3){const g=t.getX(d+0),v=t.getX(d+1),m=t.getX(d+2);n.fromBufferAttribute(e,g),r.fromBufferAttribute(e,v),a.fromBufferAttribute(e,m),h.subVectors(a,r),u.subVectors(n,r),h.cross(u),o.fromBufferAttribute(i,g),c.fromBufferAttribute(i,v),l.fromBufferAttribute(i,m),o.add(h),c.add(h),l.add(h),i.setXYZ(g,o.x,o.y,o.z),i.setXYZ(v,c.x,c.y,c.z),i.setXYZ(m,l.x,l.y,l.z)}else for(let d=0,f=e.count;d<f;d+=3)n.fromBufferAttribute(e,d+0),r.fromBufferAttribute(e,d+1),a.fromBufferAttribute(e,d+2),h.subVectors(a,r),u.subVectors(n,r),h.cross(u),i.setXYZ(d+0,h.x,h.y,h.z),i.setXYZ(d+1,h.x,h.y,h.z),i.setXYZ(d+2,h.x,h.y,h.z);this.normalizeNormals(),i.needsUpdate=!0}}normalizeNormals(){const t=this.attributes.normal;for(let e=0,i=t.count;e<i;e++)be.fromBufferAttribute(t,e),be.normalize(),t.setXYZ(e,be.x,be.y,be.z)}toNonIndexed(){function t(o,c){const l=o.array,h=o.itemSize,u=o.normalized,d=new l.constructor(c.length*h);let f=0,g=0;for(let v=0,m=c.length;v<m;v++){o.isInterleavedBufferAttribute?f=c[v]*o.data.stride+o.offset:f=c[v]*h;for(let p=0;p<h;p++)d[g++]=l[f++]}return new $e(d,h,u)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;const e=new we,i=this.index.array,n=this.attributes;for(const o in n){const c=n[o],l=t(c,i);e.setAttribute(o,l)}const r=this.morphAttributes;for(const o in r){const c=[],l=r[o];for(let h=0,u=l.length;h<u;h++){const d=l[h],f=t(d,i);c.push(f)}e.morphAttributes[o]=c}e.morphTargetsRelative=this.morphTargetsRelative;const a=this.groups;for(let o=0,c=a.length;o<c;o++){const l=a[o];e.addGroup(l.start,l.count,l.materialIndex)}return e}toJSON(){const t={metadata:{version:4.7,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(t.uuid=this.uuid,t.type=this.type,this.name!==""&&(t.name=this.name),Object.keys(this.userData).length>0&&(t.userData=this.userData),this.parameters!==void 0){const c=this.parameters;for(const l in c)c[l]!==void 0&&(t[l]=c[l]);return t}t.data={attributes:{}};const e=this.index;e!==null&&(t.data.index={type:e.array.constructor.name,array:Array.prototype.slice.call(e.array)});const i=this.attributes;for(const c in i){const l=i[c];t.data.attributes[c]=l.toJSON(t.data)}const n={};let r=!1;for(const c in this.morphAttributes){const l=this.morphAttributes[c],h=[];for(let u=0,d=l.length;u<d;u++){const f=l[u];h.push(f.toJSON(t.data))}h.length>0&&(n[c]=h,r=!0)}r&&(t.data.morphAttributes=n,t.data.morphTargetsRelative=this.morphTargetsRelative);const a=this.groups;a.length>0&&(t.data.groups=JSON.parse(JSON.stringify(a)));const o=this.boundingSphere;return o!==null&&(t.data.boundingSphere=o.toJSON()),t}clone(){return new this.constructor().copy(this)}copy(t){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;const e={};this.name=t.name;const i=t.index;i!==null&&this.setIndex(i.clone());const n=t.attributes;for(const l in n){const h=n[l];this.setAttribute(l,h.clone(e))}const r=t.morphAttributes;for(const l in r){const h=[],u=r[l];for(let d=0,f=u.length;d<f;d++)h.push(u[d].clone(e));this.morphAttributes[l]=h}this.morphTargetsRelative=t.morphTargetsRelative;const a=t.groups;for(let l=0,h=a.length;l<h;l++){const u=a[l];this.addGroup(u.start,u.count,u.materialIndex)}const o=t.boundingBox;o!==null&&(this.boundingBox=o.clone());const c=t.boundingSphere;return c!==null&&(this.boundingSphere=c.clone()),this.drawRange.start=t.drawRange.start,this.drawRange.count=t.drawRange.count,this.userData=t.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}}const Xl=new ie,ji=new gh,sr=new ls,$l=new C,rr=new C,ar=new C,or=new C,va=new C,lr=new C,Yl=new C,cr=new C;class Vt extends _e{constructor(t=new we,e=new Ge){super(),this.isMesh=!0,this.type="Mesh",this.geometry=t,this.material=e,this.morphTargetDictionary=void 0,this.morphTargetInfluences=void 0,this.count=1,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),t.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=t.morphTargetInfluences.slice()),t.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},t.morphTargetDictionary)),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}updateMorphTargets(){const e=this.geometry.morphAttributes,i=Object.keys(e);if(i.length>0){const n=e[i[0]];if(n!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,a=n.length;r<a;r++){const o=n[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=r}}}}getVertexPosition(t,e){const i=this.geometry,n=i.attributes.position,r=i.morphAttributes.position,a=i.morphTargetsRelative;e.fromBufferAttribute(n,t);const o=this.morphTargetInfluences;if(r&&o){lr.set(0,0,0);for(let c=0,l=r.length;c<l;c++){const h=o[c],u=r[c];h!==0&&(va.fromBufferAttribute(u,t),a?lr.addScaledVector(va,h):lr.addScaledVector(va.sub(e),h))}e.add(lr)}return e}raycast(t,e){const i=this.geometry,n=this.material,r=this.matrixWorld;n!==void 0&&(i.boundingSphere===null&&i.computeBoundingSphere(),sr.copy(i.boundingSphere),sr.applyMatrix4(r),ji.copy(t.ray).recast(t.near),!(sr.containsPoint(ji.origin)===!1&&(ji.intersectSphere(sr,$l)===null||ji.origin.distanceToSquared($l)>(t.far-t.near)**2))&&(Xl.copy(r).invert(),ji.copy(t.ray).applyMatrix4(Xl),!(i.boundingBox!==null&&ji.intersectsBox(i.boundingBox)===!1)&&this._computeIntersections(t,e,ji)))}_computeIntersections(t,e,i){let n;const r=this.geometry,a=this.material,o=r.index,c=r.attributes.position,l=r.attributes.uv,h=r.attributes.uv1,u=r.attributes.normal,d=r.groups,f=r.drawRange;if(o!==null)if(Array.isArray(a))for(let g=0,v=d.length;g<v;g++){const m=d[g],p=a[m.materialIndex],w=Math.max(m.start,f.start),E=Math.min(o.count,Math.min(m.start+m.count,f.start+f.count));for(let _=w,P=E;_<P;_+=3){const R=o.getX(_),I=o.getX(_+1),T=o.getX(_+2);n=hr(this,p,t,i,l,h,u,R,I,T),n&&(n.faceIndex=Math.floor(_/3),n.face.materialIndex=m.materialIndex,e.push(n))}}else{const g=Math.max(0,f.start),v=Math.min(o.count,f.start+f.count);for(let m=g,p=v;m<p;m+=3){const w=o.getX(m),E=o.getX(m+1),_=o.getX(m+2);n=hr(this,a,t,i,l,h,u,w,E,_),n&&(n.faceIndex=Math.floor(m/3),e.push(n))}}else if(c!==void 0)if(Array.isArray(a))for(let g=0,v=d.length;g<v;g++){const m=d[g],p=a[m.materialIndex],w=Math.max(m.start,f.start),E=Math.min(c.count,Math.min(m.start+m.count,f.start+f.count));for(let _=w,P=E;_<P;_+=3){const R=_,I=_+1,T=_+2;n=hr(this,p,t,i,l,h,u,R,I,T),n&&(n.faceIndex=Math.floor(_/3),n.face.materialIndex=m.materialIndex,e.push(n))}}else{const g=Math.max(0,f.start),v=Math.min(c.count,f.start+f.count);for(let m=g,p=v;m<p;m+=3){const w=m,E=m+1,_=m+2;n=hr(this,a,t,i,l,h,u,w,E,_),n&&(n.faceIndex=Math.floor(m/3),e.push(n))}}}}function Vd(s,t,e,i,n,r,a,o){let c;if(t.side===Oe?c=i.intersectTriangle(a,r,n,!0,o):c=i.intersectTriangle(n,r,a,t.side===Xi,o),c===null)return null;cr.copy(o),cr.applyMatrix4(s.matrixWorld);const l=e.ray.origin.distanceTo(cr);return l<e.near||l>e.far?null:{distance:l,point:cr.clone(),object:s}}function hr(s,t,e,i,n,r,a,o,c,l){s.getVertexPosition(o,rr),s.getVertexPosition(c,ar),s.getVertexPosition(l,or);const h=Vd(s,t,e,i,rr,ar,or,Yl);if(h){const u=new C;Qe.getBarycoord(Yl,rr,ar,or,u),n&&(h.uv=Qe.getInterpolatedAttribute(n,o,c,l,u,new ht)),r&&(h.uv1=Qe.getInterpolatedAttribute(r,o,c,l,u,new ht)),a&&(h.normal=Qe.getInterpolatedAttribute(a,o,c,l,u,new C),h.normal.dot(i.direction)>0&&h.normal.multiplyScalar(-1));const d={a:o,b:c,c:l,normal:new C,materialIndex:0};Qe.getNormal(rr,ar,or,d.normal),h.face=d,h.barycoord=u}return h}class $i extends we{constructor(t=1,e=1,i=1,n=1,r=1,a=1){super(),this.type="BoxGeometry",this.parameters={width:t,height:e,depth:i,widthSegments:n,heightSegments:r,depthSegments:a};const o=this;n=Math.floor(n),r=Math.floor(r),a=Math.floor(a);const c=[],l=[],h=[],u=[];let d=0,f=0;g("z","y","x",-1,-1,i,e,t,a,r,0),g("z","y","x",1,-1,i,e,-t,a,r,1),g("x","z","y",1,1,t,i,e,n,a,2),g("x","z","y",1,-1,t,i,-e,n,a,3),g("x","y","z",1,-1,t,e,i,n,r,4),g("x","y","z",-1,-1,t,e,-i,n,r,5),this.setIndex(c),this.setAttribute("position",new jt(l,3)),this.setAttribute("normal",new jt(h,3)),this.setAttribute("uv",new jt(u,2));function g(v,m,p,w,E,_,P,R,I,T,S){const b=_/I,L=P/T,F=_/2,B=P/2,Y=R/2,W=I+1,$=T+1;let J=0,G=0;const at=new C;for(let ut=0;ut<$;ut++){const Et=ut*L-B;for(let Gt=0;Gt<W;Gt++){const se=Gt*b-F;at[v]=se*w,at[m]=Et*E,at[p]=Y,l.push(at.x,at.y,at.z),at[v]=0,at[m]=0,at[p]=R>0?1:-1,h.push(at.x,at.y,at.z),u.push(Gt/I),u.push(1-ut/T),J+=1}}for(let ut=0;ut<T;ut++)for(let Et=0;Et<I;Et++){const Gt=d+Et+W*ut,se=d+Et+W*(ut+1),oe=d+(Et+1)+W*(ut+1),Kt=d+(Et+1)+W*ut;c.push(Gt,se,Kt),c.push(se,oe,Kt),G+=6}o.addGroup(f,G,S),f+=G,d+=J}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new $i(t.width,t.height,t.depth,t.widthSegments,t.heightSegments,t.depthSegments)}}function ss(s){const t={};for(const e in s){t[e]={};for(const i in s[e]){const n=s[e][i];n&&(n.isColor||n.isMatrix3||n.isMatrix4||n.isVector2||n.isVector3||n.isVector4||n.isTexture||n.isQuaternion)?n.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),t[e][i]=null):t[e][i]=n.clone():Array.isArray(n)?t[e][i]=n.slice():t[e][i]=n}}return t}function Le(s){const t={};for(let e=0;e<s.length;e++){const i=ss(s[e]);for(const n in i)t[n]=i[n]}return t}function Wd(s){const t=[];for(let e=0;e<s.length;e++)t.push(s[e].clone());return t}function Mh(s){const t=s.getRenderTarget();return t===null?s.outputColorSpace:t.isXRRenderTarget===!0?t.texture.colorSpace:qt.workingColorSpace}const Xd={clone:ss,merge:Le};var $d=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,Yd=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`;class Yi extends yn{constructor(t){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=$d,this.fragmentShader=Yd,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={clipCullDistance:!1,multiDraw:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,t!==void 0&&this.setValues(t)}copy(t){return super.copy(t),this.fragmentShader=t.fragmentShader,this.vertexShader=t.vertexShader,this.uniforms=ss(t.uniforms),this.uniformsGroups=Wd(t.uniformsGroups),this.defines=Object.assign({},t.defines),this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.fog=t.fog,this.lights=t.lights,this.clipping=t.clipping,this.extensions=Object.assign({},t.extensions),this.glslVersion=t.glslVersion,this}toJSON(t){const e=super.toJSON(t);e.glslVersion=this.glslVersion,e.uniforms={};for(const n in this.uniforms){const a=this.uniforms[n].value;a&&a.isTexture?e.uniforms[n]={type:"t",value:a.toJSON(t).uuid}:a&&a.isColor?e.uniforms[n]={type:"c",value:a.getHex()}:a&&a.isVector2?e.uniforms[n]={type:"v2",value:a.toArray()}:a&&a.isVector3?e.uniforms[n]={type:"v3",value:a.toArray()}:a&&a.isVector4?e.uniforms[n]={type:"v4",value:a.toArray()}:a&&a.isMatrix3?e.uniforms[n]={type:"m3",value:a.toArray()}:a&&a.isMatrix4?e.uniforms[n]={type:"m4",value:a.toArray()}:e.uniforms[n]={value:a}}Object.keys(this.defines).length>0&&(e.defines=this.defines),e.vertexShader=this.vertexShader,e.fragmentShader=this.fragmentShader,e.lights=this.lights,e.clipping=this.clipping;const i={};for(const n in this.extensions)this.extensions[n]===!0&&(i[n]=!0);return Object.keys(i).length>0&&(e.extensions=i),e}}class Sh extends _e{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new ie,this.projectionMatrix=new ie,this.projectionMatrixInverse=new ie,this.coordinateSystem=fi,this._reversedDepth=!1}get reversedDepth(){return this._reversedDepth}copy(t,e){return super.copy(t,e),this.matrixWorldInverse.copy(t.matrixWorldInverse),this.projectionMatrix.copy(t.projectionMatrix),this.projectionMatrixInverse.copy(t.projectionMatrixInverse),this.coordinateSystem=t.coordinateSystem,this}getWorldDirection(t){return super.getWorldDirection(t).negate()}updateMatrixWorld(t){super.updateMatrixWorld(t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(t,e){super.updateWorldMatrix(t,e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}}const zi=new C,ql=new ht,Kl=new ht;class si extends Sh{constructor(t=50,e=1,i=.1,n=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=t,this.zoom=1,this.near=i,this.far=n,this.focus=10,this.aspect=e,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.fov=t.fov,this.zoom=t.zoom,this.near=t.near,this.far=t.far,this.focus=t.focus,this.aspect=t.aspect,this.view=t.view===null?null:Object.assign({},t.view),this.filmGauge=t.filmGauge,this.filmOffset=t.filmOffset,this}setFocalLength(t){const e=.5*this.getFilmHeight()/t;this.fov=Io*2*Math.atan(e),this.updateProjectionMatrix()}getFocalLength(){const t=Math.tan(Zr*.5*this.fov);return .5*this.getFilmHeight()/t}getEffectiveFOV(){return Io*2*Math.atan(Math.tan(Zr*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}getViewBounds(t,e,i){zi.set(-1,-1,.5).applyMatrix4(this.projectionMatrixInverse),e.set(zi.x,zi.y).multiplyScalar(-t/zi.z),zi.set(1,1,.5).applyMatrix4(this.projectionMatrixInverse),i.set(zi.x,zi.y).multiplyScalar(-t/zi.z)}getViewSize(t,e){return this.getViewBounds(t,ql,Kl),e.subVectors(Kl,ql)}setViewOffset(t,e,i,n,r,a){this.aspect=t/e,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=i,this.view.offsetY=n,this.view.width=r,this.view.height=a,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=this.near;let e=t*Math.tan(Zr*.5*this.fov)/this.zoom,i=2*e,n=this.aspect*i,r=-.5*n;const a=this.view;if(this.view!==null&&this.view.enabled){const c=a.fullWidth,l=a.fullHeight;r+=a.offsetX*n/c,e-=a.offsetY*i/l,n*=a.width/c,i*=a.height/l}const o=this.filmOffset;o!==0&&(r+=t*o/this.getFilmWidth()),this.projectionMatrix.makePerspective(r,r+n,e,e-i,t,this.far,this.coordinateSystem,this.reversedDepth),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.fov=this.fov,e.object.zoom=this.zoom,e.object.near=this.near,e.object.far=this.far,e.object.focus=this.focus,e.object.aspect=this.aspect,this.view!==null&&(e.object.view=Object.assign({},this.view)),e.object.filmGauge=this.filmGauge,e.object.filmOffset=this.filmOffset,e}}const Nn=-90,Un=1;class qd extends _e{constructor(t,e,i){super(),this.type="CubeCamera",this.renderTarget=i,this.coordinateSystem=null,this.activeMipmapLevel=0;const n=new si(Nn,Un,t,e);n.layers=this.layers,this.add(n);const r=new si(Nn,Un,t,e);r.layers=this.layers,this.add(r);const a=new si(Nn,Un,t,e);a.layers=this.layers,this.add(a);const o=new si(Nn,Un,t,e);o.layers=this.layers,this.add(o);const c=new si(Nn,Un,t,e);c.layers=this.layers,this.add(c);const l=new si(Nn,Un,t,e);l.layers=this.layers,this.add(l)}updateCoordinateSystem(){const t=this.coordinateSystem,e=this.children.concat(),[i,n,r,a,o,c]=e;for(const l of e)this.remove(l);if(t===fi)i.up.set(0,1,0),i.lookAt(1,0,0),n.up.set(0,1,0),n.lookAt(-1,0,0),r.up.set(0,0,-1),r.lookAt(0,1,0),a.up.set(0,0,1),a.lookAt(0,-1,0),o.up.set(0,1,0),o.lookAt(0,0,1),c.up.set(0,1,0),c.lookAt(0,0,-1);else if(t===Ur)i.up.set(0,-1,0),i.lookAt(-1,0,0),n.up.set(0,-1,0),n.lookAt(1,0,0),r.up.set(0,0,1),r.lookAt(0,1,0),a.up.set(0,0,-1),a.lookAt(0,-1,0),o.up.set(0,-1,0),o.lookAt(0,0,1),c.up.set(0,-1,0),c.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+t);for(const l of e)this.add(l),l.updateMatrixWorld()}update(t,e){this.parent===null&&this.updateMatrixWorld();const{renderTarget:i,activeMipmapLevel:n}=this;this.coordinateSystem!==t.coordinateSystem&&(this.coordinateSystem=t.coordinateSystem,this.updateCoordinateSystem());const[r,a,o,c,l,h]=this.children,u=t.getRenderTarget(),d=t.getActiveCubeFace(),f=t.getActiveMipmapLevel(),g=t.xr.enabled;t.xr.enabled=!1;const v=i.texture.generateMipmaps;i.texture.generateMipmaps=!1,t.setRenderTarget(i,0,n),t.render(e,r),t.setRenderTarget(i,1,n),t.render(e,a),t.setRenderTarget(i,2,n),t.render(e,o),t.setRenderTarget(i,3,n),t.render(e,c),t.setRenderTarget(i,4,n),t.render(e,l),i.texture.generateMipmaps=v,t.setRenderTarget(i,5,n),t.render(e,h),t.setRenderTarget(u,d,f),t.xr.enabled=g,i.texture.needsPMREMUpdate=!0}}class bh extends Ce{constructor(t=[],e=es,i,n,r,a,o,c,l,h){super(t,e,i,n,r,a,o,c,l,h),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(t){this.image=t}}class Kd extends vn{constructor(t=1,e={}){super(t,t,e),this.isWebGLCubeRenderTarget=!0;const i={width:t,height:t,depth:1},n=[i,i,i,i,i,i];this.texture=new bh(n),this._setTextureOptions(e),this.texture.isRenderTargetTexture=!0}fromEquirectangularTexture(t,e){this.texture.type=e.type,this.texture.colorSpace=e.colorSpace,this.texture.generateMipmaps=e.generateMipmaps,this.texture.minFilter=e.minFilter,this.texture.magFilter=e.magFilter;const i={uniforms:{tEquirect:{value:null}},vertexShader:`

				varying vec3 vWorldDirection;

				vec3 transformDirection( in vec3 dir, in mat4 matrix ) {

					return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );

				}

				void main() {

					vWorldDirection = transformDirection( position, modelMatrix );

					#include <begin_vertex>
					#include <project_vertex>

				}
			`,fragmentShader:`

				uniform sampler2D tEquirect;

				varying vec3 vWorldDirection;

				#include <common>

				void main() {

					vec3 direction = normalize( vWorldDirection );

					vec2 sampleUV = equirectUv( direction );

					gl_FragColor = texture2D( tEquirect, sampleUV );

				}
			`},n=new $i(5,5,5),r=new Yi({name:"CubemapFromEquirect",uniforms:ss(i.uniforms),vertexShader:i.vertexShader,fragmentShader:i.fragmentShader,side:Oe,blending:Vi});r.uniforms.tEquirect.value=e;const a=new Vt(n,r),o=e.minFilter;return e.minFilter===cn&&(e.minFilter=ui),new qd(1,10,this).update(t,a),e.minFilter=o,a.geometry.dispose(),a.material.dispose(),this}clear(t,e=!0,i=!0,n=!0){const r=t.getRenderTarget();for(let a=0;a<6;a++)t.setRenderTarget(this,a),t.clear(e,i,n);t.setRenderTarget(r)}}class ge extends _e{constructor(){super(),this.isGroup=!0,this.type="Group"}}const Zd={type:"move"};class _a{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new ge,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new ge,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new C,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new C),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new ge,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new C,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new C),this._grip}dispatchEvent(t){return this._targetRay!==null&&this._targetRay.dispatchEvent(t),this._grip!==null&&this._grip.dispatchEvent(t),this._hand!==null&&this._hand.dispatchEvent(t),this}connect(t){if(t&&t.hand){const e=this._hand;if(e)for(const i of t.hand.values())this._getHandJoint(e,i)}return this.dispatchEvent({type:"connected",data:t}),this}disconnect(t){return this.dispatchEvent({type:"disconnected",data:t}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(t,e,i){let n=null,r=null,a=null;const o=this._targetRay,c=this._grip,l=this._hand;if(t&&e.session.visibilityState!=="visible-blurred"){if(l&&t.hand){a=!0;for(const v of t.hand.values()){const m=e.getJointPose(v,i),p=this._getHandJoint(l,v);m!==null&&(p.matrix.fromArray(m.transform.matrix),p.matrix.decompose(p.position,p.rotation,p.scale),p.matrixWorldNeedsUpdate=!0,p.jointRadius=m.radius),p.visible=m!==null}const h=l.joints["index-finger-tip"],u=l.joints["thumb-tip"],d=h.position.distanceTo(u.position),f=.02,g=.005;l.inputState.pinching&&d>f+g?(l.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:t.handedness,target:this})):!l.inputState.pinching&&d<=f-g&&(l.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:t.handedness,target:this}))}else c!==null&&t.gripSpace&&(r=e.getPose(t.gripSpace,i),r!==null&&(c.matrix.fromArray(r.transform.matrix),c.matrix.decompose(c.position,c.rotation,c.scale),c.matrixWorldNeedsUpdate=!0,r.linearVelocity?(c.hasLinearVelocity=!0,c.linearVelocity.copy(r.linearVelocity)):c.hasLinearVelocity=!1,r.angularVelocity?(c.hasAngularVelocity=!0,c.angularVelocity.copy(r.angularVelocity)):c.hasAngularVelocity=!1));o!==null&&(n=e.getPose(t.targetRaySpace,i),n===null&&r!==null&&(n=r),n!==null&&(o.matrix.fromArray(n.transform.matrix),o.matrix.decompose(o.position,o.rotation,o.scale),o.matrixWorldNeedsUpdate=!0,n.linearVelocity?(o.hasLinearVelocity=!0,o.linearVelocity.copy(n.linearVelocity)):o.hasLinearVelocity=!1,n.angularVelocity?(o.hasAngularVelocity=!0,o.angularVelocity.copy(n.angularVelocity)):o.hasAngularVelocity=!1,this.dispatchEvent(Zd)))}return o!==null&&(o.visible=n!==null),c!==null&&(c.visible=r!==null),l!==null&&(l.visible=a!==null),this}_getHandJoint(t,e){if(t.joints[e.jointName]===void 0){const i=new ge;i.matrixAutoUpdate=!1,i.visible=!1,t.joints[e.jointName]=i,t.add(i)}return t.joints[e.jointName]}}class Qo{constructor(t,e=1,i=1e3){this.isFog=!0,this.name="",this.color=new Ht(t),this.near=e,this.far=i}clone(){return new Qo(this.color,this.near,this.far)}toJSON(){return{type:"Fog",name:this.name,color:this.color.getHex(),near:this.near,far:this.far}}}class Jd extends _e{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.backgroundRotation=new gi,this.environmentIntensity=1,this.environmentRotation=new gi,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(t,e){return super.copy(t,e),t.background!==null&&(this.background=t.background.clone()),t.environment!==null&&(this.environment=t.environment.clone()),t.fog!==null&&(this.fog=t.fog.clone()),this.backgroundBlurriness=t.backgroundBlurriness,this.backgroundIntensity=t.backgroundIntensity,this.backgroundRotation.copy(t.backgroundRotation),this.environmentIntensity=t.environmentIntensity,this.environmentRotation.copy(t.environmentRotation),t.overrideMaterial!==null&&(this.overrideMaterial=t.overrideMaterial.clone()),this.matrixAutoUpdate=t.matrixAutoUpdate,this}toJSON(t){const e=super.toJSON(t);return this.fog!==null&&(e.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(e.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(e.object.backgroundIntensity=this.backgroundIntensity),e.object.backgroundRotation=this.backgroundRotation.toArray(),this.environmentIntensity!==1&&(e.object.environmentIntensity=this.environmentIntensity),e.object.environmentRotation=this.environmentRotation.toArray(),e}}class jd{constructor(t,e){this.isInterleavedBuffer=!0,this.array=t,this.stride=e,this.count=t!==void 0?t.length/e:0,this.usage=Po,this.updateRanges=[],this.version=0,this.uuid=Ai()}onUploadCallback(){}set needsUpdate(t){t===!0&&this.version++}setUsage(t){return this.usage=t,this}addUpdateRange(t,e){this.updateRanges.push({start:t,count:e})}clearUpdateRanges(){this.updateRanges.length=0}copy(t){return this.array=new t.array.constructor(t.array),this.count=t.count,this.stride=t.stride,this.usage=t.usage,this}copyAt(t,e,i){t*=this.stride,i*=e.stride;for(let n=0,r=this.stride;n<r;n++)this.array[t+n]=e.array[i+n];return this}set(t,e=0){return this.array.set(t,e),this}clone(t){t.arrayBuffers===void 0&&(t.arrayBuffers={}),this.array.buffer._uuid===void 0&&(this.array.buffer._uuid=Ai()),t.arrayBuffers[this.array.buffer._uuid]===void 0&&(t.arrayBuffers[this.array.buffer._uuid]=this.array.slice(0).buffer);const e=new this.array.constructor(t.arrayBuffers[this.array.buffer._uuid]),i=new this.constructor(e,this.stride);return i.setUsage(this.usage),i}onUpload(t){return this.onUploadCallback=t,this}toJSON(t){return t.arrayBuffers===void 0&&(t.arrayBuffers={}),this.array.buffer._uuid===void 0&&(this.array.buffer._uuid=Ai()),t.arrayBuffers[this.array.buffer._uuid]===void 0&&(t.arrayBuffers[this.array.buffer._uuid]=Array.from(new Uint32Array(this.array.buffer))),{uuid:this.uuid,buffer:this.array.buffer._uuid,type:this.array.constructor.name,stride:this.stride}}}const Ie=new C;class Fr{constructor(t,e,i,n=!1){this.isInterleavedBufferAttribute=!0,this.name="",this.data=t,this.itemSize=e,this.offset=i,this.normalized=n}get count(){return this.data.count}get array(){return this.data.array}set needsUpdate(t){this.data.needsUpdate=t}applyMatrix4(t){for(let e=0,i=this.data.count;e<i;e++)Ie.fromBufferAttribute(this,e),Ie.applyMatrix4(t),this.setXYZ(e,Ie.x,Ie.y,Ie.z);return this}applyNormalMatrix(t){for(let e=0,i=this.count;e<i;e++)Ie.fromBufferAttribute(this,e),Ie.applyNormalMatrix(t),this.setXYZ(e,Ie.x,Ie.y,Ie.z);return this}transformDirection(t){for(let e=0,i=this.count;e<i;e++)Ie.fromBufferAttribute(this,e),Ie.transformDirection(t),this.setXYZ(e,Ie.x,Ie.y,Ie.z);return this}getComponent(t,e){let i=this.array[t*this.data.stride+this.offset+e];return this.normalized&&(i=hi(i,this.array)),i}setComponent(t,e,i){return this.normalized&&(i=ee(i,this.array)),this.data.array[t*this.data.stride+this.offset+e]=i,this}setX(t,e){return this.normalized&&(e=ee(e,this.array)),this.data.array[t*this.data.stride+this.offset]=e,this}setY(t,e){return this.normalized&&(e=ee(e,this.array)),this.data.array[t*this.data.stride+this.offset+1]=e,this}setZ(t,e){return this.normalized&&(e=ee(e,this.array)),this.data.array[t*this.data.stride+this.offset+2]=e,this}setW(t,e){return this.normalized&&(e=ee(e,this.array)),this.data.array[t*this.data.stride+this.offset+3]=e,this}getX(t){let e=this.data.array[t*this.data.stride+this.offset];return this.normalized&&(e=hi(e,this.array)),e}getY(t){let e=this.data.array[t*this.data.stride+this.offset+1];return this.normalized&&(e=hi(e,this.array)),e}getZ(t){let e=this.data.array[t*this.data.stride+this.offset+2];return this.normalized&&(e=hi(e,this.array)),e}getW(t){let e=this.data.array[t*this.data.stride+this.offset+3];return this.normalized&&(e=hi(e,this.array)),e}setXY(t,e,i){return t=t*this.data.stride+this.offset,this.normalized&&(e=ee(e,this.array),i=ee(i,this.array)),this.data.array[t+0]=e,this.data.array[t+1]=i,this}setXYZ(t,e,i,n){return t=t*this.data.stride+this.offset,this.normalized&&(e=ee(e,this.array),i=ee(i,this.array),n=ee(n,this.array)),this.data.array[t+0]=e,this.data.array[t+1]=i,this.data.array[t+2]=n,this}setXYZW(t,e,i,n,r){return t=t*this.data.stride+this.offset,this.normalized&&(e=ee(e,this.array),i=ee(i,this.array),n=ee(n,this.array),r=ee(r,this.array)),this.data.array[t+0]=e,this.data.array[t+1]=i,this.data.array[t+2]=n,this.data.array[t+3]=r,this}clone(t){if(t===void 0){console.log("THREE.InterleavedBufferAttribute.clone(): Cloning an interleaved buffer attribute will de-interleave buffer data.");const e=[];for(let i=0;i<this.count;i++){const n=i*this.data.stride+this.offset;for(let r=0;r<this.itemSize;r++)e.push(this.data.array[n+r])}return new $e(new this.array.constructor(e),this.itemSize,this.normalized)}else return t.interleavedBuffers===void 0&&(t.interleavedBuffers={}),t.interleavedBuffers[this.data.uuid]===void 0&&(t.interleavedBuffers[this.data.uuid]=this.data.clone(t)),new Fr(t.interleavedBuffers[this.data.uuid],this.itemSize,this.offset,this.normalized)}toJSON(t){if(t===void 0){console.log("THREE.InterleavedBufferAttribute.toJSON(): Serializing an interleaved buffer attribute will de-interleave buffer data.");const e=[];for(let i=0;i<this.count;i++){const n=i*this.data.stride+this.offset;for(let r=0;r<this.itemSize;r++)e.push(this.data.array[n+r])}return{itemSize:this.itemSize,type:this.array.constructor.name,array:e,normalized:this.normalized}}else return t.interleavedBuffers===void 0&&(t.interleavedBuffers={}),t.interleavedBuffers[this.data.uuid]===void 0&&(t.interleavedBuffers[this.data.uuid]=this.data.toJSON(t)),{isInterleavedBufferAttribute:!0,itemSize:this.itemSize,data:this.data.uuid,offset:this.offset,normalized:this.normalized}}}class Do extends yn{constructor(t){super(),this.isSpriteMaterial=!0,this.type="SpriteMaterial",this.color=new Ht(16777215),this.map=null,this.alphaMap=null,this.rotation=0,this.sizeAttenuation=!0,this.transparent=!0,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.alphaMap=t.alphaMap,this.rotation=t.rotation,this.sizeAttenuation=t.sizeAttenuation,this.fog=t.fog,this}}let On;const gs=new C,Fn=new C,kn=new C,zn=new ht,vs=new ht,Eh=new ie,ur=new C,_s=new C,dr=new C,Zl=new ht,xa=new ht,Jl=new ht;class jl extends _e{constructor(t=new Do){if(super(),this.isSprite=!0,this.type="Sprite",On===void 0){On=new we;const e=new Float32Array([-.5,-.5,0,0,0,.5,-.5,0,1,0,.5,.5,0,1,1,-.5,.5,0,0,1]),i=new jd(e,5);On.setIndex([0,1,2,0,2,3]),On.setAttribute("position",new Fr(i,3,0,!1)),On.setAttribute("uv",new Fr(i,2,3,!1))}this.geometry=On,this.material=t,this.center=new ht(.5,.5),this.count=1}raycast(t,e){t.camera===null&&console.error('THREE.Sprite: "Raycaster.camera" needs to be set in order to raycast against sprites.'),Fn.setFromMatrixScale(this.matrixWorld),Eh.copy(t.camera.matrixWorld),this.modelViewMatrix.multiplyMatrices(t.camera.matrixWorldInverse,this.matrixWorld),kn.setFromMatrixPosition(this.modelViewMatrix),t.camera.isPerspectiveCamera&&this.material.sizeAttenuation===!1&&Fn.multiplyScalar(-kn.z);const i=this.material.rotation;let n,r;i!==0&&(r=Math.cos(i),n=Math.sin(i));const a=this.center;fr(ur.set(-.5,-.5,0),kn,a,Fn,n,r),fr(_s.set(.5,-.5,0),kn,a,Fn,n,r),fr(dr.set(.5,.5,0),kn,a,Fn,n,r),Zl.set(0,0),xa.set(1,0),Jl.set(1,1);let o=t.ray.intersectTriangle(ur,_s,dr,!1,gs);if(o===null&&(fr(_s.set(-.5,.5,0),kn,a,Fn,n,r),xa.set(0,1),o=t.ray.intersectTriangle(ur,dr,_s,!1,gs),o===null))return;const c=t.ray.origin.distanceTo(gs);c<t.near||c>t.far||e.push({distance:c,point:gs.clone(),uv:Qe.getInterpolation(gs,ur,_s,dr,Zl,xa,Jl,new ht),face:null,object:this})}copy(t,e){return super.copy(t,e),t.center!==void 0&&this.center.copy(t.center),this.material=t.material,this}}function fr(s,t,e,i,n,r){zn.subVectors(s,e).addScalar(.5).multiply(i),n!==void 0?(vs.x=r*zn.x-n*zn.y,vs.y=n*zn.x+r*zn.y):vs.copy(zn),s.copy(t),s.x+=vs.x,s.y+=vs.y,s.applyMatrix4(Eh)}class Qd extends Ce{constructor(t=null,e=1,i=1,n,r,a,o,c,l=Xe,h=Xe,u,d){super(null,a,o,c,l,h,n,r,u,d),this.isDataTexture=!0,this.image={data:t,width:e,height:i},this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class Ql extends $e{constructor(t,e,i,n=1){super(t,e,i),this.isInstancedBufferAttribute=!0,this.meshPerAttribute=n}copy(t){return super.copy(t),this.meshPerAttribute=t.meshPerAttribute,this}toJSON(){const t=super.toJSON();return t.meshPerAttribute=this.meshPerAttribute,t.isInstancedBufferAttribute=!0,t}}const Bn=new ie,tc=new ie,pr=[],ec=new xn,tf=new ie,xs=new Vt,ys=new ls;class kr extends Vt{constructor(t,e,i){super(t,e),this.isInstancedMesh=!0,this.instanceMatrix=new Ql(new Float32Array(i*16),16),this.instanceColor=null,this.morphTexture=null,this.count=i,this.boundingBox=null,this.boundingSphere=null;for(let n=0;n<i;n++)this.setMatrixAt(n,tf)}computeBoundingBox(){const t=this.geometry,e=this.count;this.boundingBox===null&&(this.boundingBox=new xn),t.boundingBox===null&&t.computeBoundingBox(),this.boundingBox.makeEmpty();for(let i=0;i<e;i++)this.getMatrixAt(i,Bn),ec.copy(t.boundingBox).applyMatrix4(Bn),this.boundingBox.union(ec)}computeBoundingSphere(){const t=this.geometry,e=this.count;this.boundingSphere===null&&(this.boundingSphere=new ls),t.boundingSphere===null&&t.computeBoundingSphere(),this.boundingSphere.makeEmpty();for(let i=0;i<e;i++)this.getMatrixAt(i,Bn),ys.copy(t.boundingSphere).applyMatrix4(Bn),this.boundingSphere.union(ys)}copy(t,e){return super.copy(t,e),this.instanceMatrix.copy(t.instanceMatrix),t.morphTexture!==null&&(this.morphTexture=t.morphTexture.clone()),t.instanceColor!==null&&(this.instanceColor=t.instanceColor.clone()),this.count=t.count,t.boundingBox!==null&&(this.boundingBox=t.boundingBox.clone()),t.boundingSphere!==null&&(this.boundingSphere=t.boundingSphere.clone()),this}getColorAt(t,e){e.fromArray(this.instanceColor.array,t*3)}getMatrixAt(t,e){e.fromArray(this.instanceMatrix.array,t*16)}getMorphAt(t,e){const i=e.morphTargetInfluences,n=this.morphTexture.source.data.data,r=i.length+1,a=t*r+1;for(let o=0;o<i.length;o++)i[o]=n[a+o]}raycast(t,e){const i=this.matrixWorld,n=this.count;if(xs.geometry=this.geometry,xs.material=this.material,xs.material!==void 0&&(this.boundingSphere===null&&this.computeBoundingSphere(),ys.copy(this.boundingSphere),ys.applyMatrix4(i),t.ray.intersectsSphere(ys)!==!1))for(let r=0;r<n;r++){this.getMatrixAt(r,Bn),tc.multiplyMatrices(i,Bn),xs.matrixWorld=tc,xs.raycast(t,pr);for(let a=0,o=pr.length;a<o;a++){const c=pr[a];c.instanceId=r,c.object=this,e.push(c)}pr.length=0}}setColorAt(t,e){this.instanceColor===null&&(this.instanceColor=new Ql(new Float32Array(this.instanceMatrix.count*3).fill(1),3)),e.toArray(this.instanceColor.array,t*3)}setMatrixAt(t,e){e.toArray(this.instanceMatrix.array,t*16)}setMorphAt(t,e){const i=e.morphTargetInfluences,n=i.length+1;this.morphTexture===null&&(this.morphTexture=new Qd(new Float32Array(n*this.count),n,this.count,qo,di));const r=this.morphTexture.source.data.data;let a=0;for(let l=0;l<i.length;l++)a+=i[l];const o=this.geometry.morphTargetsRelative?1:1-a,c=n*t;r[c]=o,r.set(i,c+1)}updateMorphTargets(){}dispose(){this.dispatchEvent({type:"dispose"}),this.morphTexture!==null&&(this.morphTexture.dispose(),this.morphTexture=null)}}const ya=new C,ef=new C,nf=new Ot;class nn{constructor(t=new C(1,0,0),e=0){this.isPlane=!0,this.normal=t,this.constant=e}set(t,e){return this.normal.copy(t),this.constant=e,this}setComponents(t,e,i,n){return this.normal.set(t,e,i),this.constant=n,this}setFromNormalAndCoplanarPoint(t,e){return this.normal.copy(t),this.constant=-e.dot(this.normal),this}setFromCoplanarPoints(t,e,i){const n=ya.subVectors(i,e).cross(ef.subVectors(t,e)).normalize();return this.setFromNormalAndCoplanarPoint(n,t),this}copy(t){return this.normal.copy(t.normal),this.constant=t.constant,this}normalize(){const t=1/this.normal.length();return this.normal.multiplyScalar(t),this.constant*=t,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(t){return this.normal.dot(t)+this.constant}distanceToSphere(t){return this.distanceToPoint(t.center)-t.radius}projectPoint(t,e){return e.copy(t).addScaledVector(this.normal,-this.distanceToPoint(t))}intersectLine(t,e){const i=t.delta(ya),n=this.normal.dot(i);if(n===0)return this.distanceToPoint(t.start)===0?e.copy(t.start):null;const r=-(t.start.dot(this.normal)+this.constant)/n;return r<0||r>1?null:e.copy(t.start).addScaledVector(i,r)}intersectsLine(t){const e=this.distanceToPoint(t.start),i=this.distanceToPoint(t.end);return e<0&&i>0||i<0&&e>0}intersectsBox(t){return t.intersectsPlane(this)}intersectsSphere(t){return t.intersectsPlane(this)}coplanarPoint(t){return t.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(t,e){const i=e||nf.getNormalMatrix(t),n=this.coplanarPoint(ya).applyMatrix4(t),r=this.normal.applyMatrix3(i).normalize();return this.constant=-n.dot(r),this}translate(t){return this.constant-=t.dot(this.normal),this}equals(t){return t.normal.equals(this.normal)&&t.constant===this.constant}clone(){return new this.constructor().copy(this)}}const Qi=new ls,sf=new ht(.5,.5),mr=new C;class tl{constructor(t=new nn,e=new nn,i=new nn,n=new nn,r=new nn,a=new nn){this.planes=[t,e,i,n,r,a]}set(t,e,i,n,r,a){const o=this.planes;return o[0].copy(t),o[1].copy(e),o[2].copy(i),o[3].copy(n),o[4].copy(r),o[5].copy(a),this}copy(t){const e=this.planes;for(let i=0;i<6;i++)e[i].copy(t.planes[i]);return this}setFromProjectionMatrix(t,e=fi,i=!1){const n=this.planes,r=t.elements,a=r[0],o=r[1],c=r[2],l=r[3],h=r[4],u=r[5],d=r[6],f=r[7],g=r[8],v=r[9],m=r[10],p=r[11],w=r[12],E=r[13],_=r[14],P=r[15];if(n[0].setComponents(l-a,f-h,p-g,P-w).normalize(),n[1].setComponents(l+a,f+h,p+g,P+w).normalize(),n[2].setComponents(l+o,f+u,p+v,P+E).normalize(),n[3].setComponents(l-o,f-u,p-v,P-E).normalize(),i)n[4].setComponents(c,d,m,_).normalize(),n[5].setComponents(l-c,f-d,p-m,P-_).normalize();else if(n[4].setComponents(l-c,f-d,p-m,P-_).normalize(),e===fi)n[5].setComponents(l+c,f+d,p+m,P+_).normalize();else if(e===Ur)n[5].setComponents(c,d,m,_).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+e);return this}intersectsObject(t){if(t.boundingSphere!==void 0)t.boundingSphere===null&&t.computeBoundingSphere(),Qi.copy(t.boundingSphere).applyMatrix4(t.matrixWorld);else{const e=t.geometry;e.boundingSphere===null&&e.computeBoundingSphere(),Qi.copy(e.boundingSphere).applyMatrix4(t.matrixWorld)}return this.intersectsSphere(Qi)}intersectsSprite(t){Qi.center.set(0,0,0);const e=sf.distanceTo(t.center);return Qi.radius=.7071067811865476+e,Qi.applyMatrix4(t.matrixWorld),this.intersectsSphere(Qi)}intersectsSphere(t){const e=this.planes,i=t.center,n=-t.radius;for(let r=0;r<6;r++)if(e[r].distanceToPoint(i)<n)return!1;return!0}intersectsBox(t){const e=this.planes;for(let i=0;i<6;i++){const n=e[i];if(mr.x=n.normal.x>0?t.max.x:t.min.x,mr.y=n.normal.y>0?t.max.y:t.min.y,mr.z=n.normal.z>0?t.max.z:t.min.z,n.distanceToPoint(mr)<0)return!1}return!0}containsPoint(t){const e=this.planes;for(let i=0;i<6;i++)if(e[i].distanceToPoint(t)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}}class wh extends yn{constructor(t){super(),this.isLineBasicMaterial=!0,this.type="LineBasicMaterial",this.color=new Ht(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.linewidth=t.linewidth,this.linecap=t.linecap,this.linejoin=t.linejoin,this.fog=t.fog,this}}const zr=new C,Br=new C,ic=new ie,Ms=new gh,gr=new ls,Ma=new C,nc=new C;class Th extends _e{constructor(t=new we,e=new wh){super(),this.isLine=!0,this.type="Line",this.geometry=t,this.material=e,this.morphTargetDictionary=void 0,this.morphTargetInfluences=void 0,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,i=[0];for(let n=1,r=e.count;n<r;n++)zr.fromBufferAttribute(e,n-1),Br.fromBufferAttribute(e,n),i[n]=i[n-1],i[n]+=zr.distanceTo(Br);t.setAttribute("lineDistance",new jt(i,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(t,e){const i=this.geometry,n=this.matrixWorld,r=t.params.Line.threshold,a=i.drawRange;if(i.boundingSphere===null&&i.computeBoundingSphere(),gr.copy(i.boundingSphere),gr.applyMatrix4(n),gr.radius+=r,t.ray.intersectsSphere(gr)===!1)return;ic.copy(n).invert(),Ms.copy(t.ray).applyMatrix4(ic);const o=r/((this.scale.x+this.scale.y+this.scale.z)/3),c=o*o,l=this.isLineSegments?2:1,h=i.index,d=i.attributes.position;if(h!==null){const f=Math.max(0,a.start),g=Math.min(h.count,a.start+a.count);for(let v=f,m=g-1;v<m;v+=l){const p=h.getX(v),w=h.getX(v+1),E=vr(this,t,Ms,c,p,w,v);E&&e.push(E)}if(this.isLineLoop){const v=h.getX(g-1),m=h.getX(f),p=vr(this,t,Ms,c,v,m,g-1);p&&e.push(p)}}else{const f=Math.max(0,a.start),g=Math.min(d.count,a.start+a.count);for(let v=f,m=g-1;v<m;v+=l){const p=vr(this,t,Ms,c,v,v+1,v);p&&e.push(p)}if(this.isLineLoop){const v=vr(this,t,Ms,c,g-1,f,g-1);v&&e.push(v)}}}updateMorphTargets(){const e=this.geometry.morphAttributes,i=Object.keys(e);if(i.length>0){const n=e[i[0]];if(n!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,a=n.length;r<a;r++){const o=n[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=r}}}}}function vr(s,t,e,i,n,r,a){const o=s.geometry.attributes.position;if(zr.fromBufferAttribute(o,n),Br.fromBufferAttribute(o,r),e.distanceSqToSegment(zr,Br,Ma,nc)>i)return;Ma.applyMatrix4(s.matrixWorld);const l=t.ray.origin.distanceTo(Ma);if(!(l<t.near||l>t.far))return{distance:l,point:nc.clone().applyMatrix4(s.matrixWorld),index:a,face:null,faceIndex:null,barycoord:null,object:s}}class sc extends Ce{constructor(t,e,i,n,r,a,o,c,l){super(t,e,i,n,r,a,o,c,l),this.isCanvasTexture=!0,this.needsUpdate=!0}}class Ah extends Ce{constructor(t,e,i=gn,n,r,a,o=Xe,c=Xe,l,h=Us,u=1){if(h!==Us&&h!==Os)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");const d={width:t,height:e,depth:u};super(d,n,r,a,o,c,h,i,l),this.isDepthTexture=!0,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(t){return super.copy(t),this.source=new jo(Object.assign({},t.image)),this.compareFunction=t.compareFunction,this}toJSON(t){const e=super.toJSON(t);return this.compareFunction!==null&&(e.compareFunction=this.compareFunction),e}}class Rh extends Ce{constructor(t=null){super(),this.sourceTexture=t,this.isExternalTexture=!0}copy(t){return super.copy(t),this.sourceTexture=t.sourceTexture,this}}class ks extends we{constructor(t=1,e=32,i=0,n=Math.PI*2){super(),this.type="CircleGeometry",this.parameters={radius:t,segments:e,thetaStart:i,thetaLength:n},e=Math.max(3,e);const r=[],a=[],o=[],c=[],l=new C,h=new ht;a.push(0,0,0),o.push(0,0,1),c.push(.5,.5);for(let u=0,d=3;u<=e;u++,d+=3){const f=i+u/e*n;l.x=t*Math.cos(f),l.y=t*Math.sin(f),a.push(l.x,l.y,l.z),o.push(0,0,1),h.x=(a[d]/t+1)/2,h.y=(a[d+1]/t+1)/2,c.push(h.x,h.y)}for(let u=1;u<=e;u++)r.push(u,u+1,0);this.setIndex(r),this.setAttribute("position",new jt(a,3)),this.setAttribute("normal",new jt(o,3)),this.setAttribute("uv",new jt(c,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new ks(t.radius,t.segments,t.thetaStart,t.thetaLength)}}class Ws extends we{constructor(t=1,e=1,i=1,n=32,r=1,a=!1,o=0,c=Math.PI*2){super(),this.type="CylinderGeometry",this.parameters={radiusTop:t,radiusBottom:e,height:i,radialSegments:n,heightSegments:r,openEnded:a,thetaStart:o,thetaLength:c};const l=this;n=Math.floor(n),r=Math.floor(r);const h=[],u=[],d=[],f=[];let g=0;const v=[],m=i/2;let p=0;w(),a===!1&&(t>0&&E(!0),e>0&&E(!1)),this.setIndex(h),this.setAttribute("position",new jt(u,3)),this.setAttribute("normal",new jt(d,3)),this.setAttribute("uv",new jt(f,2));function w(){const _=new C,P=new C;let R=0;const I=(e-t)/i;for(let T=0;T<=r;T++){const S=[],b=T/r,L=b*(e-t)+t;for(let F=0;F<=n;F++){const B=F/n,Y=B*c+o,W=Math.sin(Y),$=Math.cos(Y);P.x=L*W,P.y=-b*i+m,P.z=L*$,u.push(P.x,P.y,P.z),_.set(W,I,$).normalize(),d.push(_.x,_.y,_.z),f.push(B,1-b),S.push(g++)}v.push(S)}for(let T=0;T<n;T++)for(let S=0;S<r;S++){const b=v[S][T],L=v[S+1][T],F=v[S+1][T+1],B=v[S][T+1];(t>0||S!==0)&&(h.push(b,L,B),R+=3),(e>0||S!==r-1)&&(h.push(L,F,B),R+=3)}l.addGroup(p,R,0),p+=R}function E(_){const P=g,R=new ht,I=new C;let T=0;const S=_===!0?t:e,b=_===!0?1:-1;for(let F=1;F<=n;F++)u.push(0,m*b,0),d.push(0,b,0),f.push(.5,.5),g++;const L=g;for(let F=0;F<=n;F++){const Y=F/n*c+o,W=Math.cos(Y),$=Math.sin(Y);I.x=S*$,I.y=m*b,I.z=S*W,u.push(I.x,I.y,I.z),d.push(0,b,0),R.x=W*.5+.5,R.y=$*.5*b+.5,f.push(R.x,R.y),g++}for(let F=0;F<n;F++){const B=P+F,Y=L+F;_===!0?h.push(Y,Y+1,B):h.push(Y+1,Y,B),T+=3}l.addGroup(p,T,_===!0?1:2),p+=T}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Ws(t.radiusTop,t.radiusBottom,t.height,t.radialSegments,t.heightSegments,t.openEnded,t.thetaStart,t.thetaLength)}}class pn extends Ws{constructor(t=1,e=1,i=32,n=1,r=!1,a=0,o=Math.PI*2){super(0,t,e,i,n,r,a,o),this.type="ConeGeometry",this.parameters={radius:t,height:e,radialSegments:i,heightSegments:n,openEnded:r,thetaStart:a,thetaLength:o}}static fromJSON(t){return new pn(t.radius,t.height,t.radialSegments,t.heightSegments,t.openEnded,t.thetaStart,t.thetaLength)}}class vi{constructor(){this.type="Curve",this.arcLengthDivisions=200,this.needsUpdate=!1,this.cacheArcLengths=null}getPoint(){console.warn("THREE.Curve: .getPoint() not implemented.")}getPointAt(t,e){const i=this.getUtoTmapping(t);return this.getPoint(i,e)}getPoints(t=5){const e=[];for(let i=0;i<=t;i++)e.push(this.getPoint(i/t));return e}getSpacedPoints(t=5){const e=[];for(let i=0;i<=t;i++)e.push(this.getPointAt(i/t));return e}getLength(){const t=this.getLengths();return t[t.length-1]}getLengths(t=this.arcLengthDivisions){if(this.cacheArcLengths&&this.cacheArcLengths.length===t+1&&!this.needsUpdate)return this.cacheArcLengths;this.needsUpdate=!1;const e=[];let i,n=this.getPoint(0),r=0;e.push(0);for(let a=1;a<=t;a++)i=this.getPoint(a/t),r+=i.distanceTo(n),e.push(r),n=i;return this.cacheArcLengths=e,e}updateArcLengths(){this.needsUpdate=!0,this.getLengths()}getUtoTmapping(t,e=null){const i=this.getLengths();let n=0;const r=i.length;let a;e?a=e:a=t*i[r-1];let o=0,c=r-1,l;for(;o<=c;)if(n=Math.floor(o+(c-o)/2),l=i[n]-a,l<0)o=n+1;else if(l>0)c=n-1;else{c=n;break}if(n=c,i[n]===a)return n/(r-1);const h=i[n],d=i[n+1]-h,f=(a-h)/d;return(n+f)/(r-1)}getTangent(t,e){let n=t-1e-4,r=t+1e-4;n<0&&(n=0),r>1&&(r=1);const a=this.getPoint(n),o=this.getPoint(r),c=e||(a.isVector2?new ht:new C);return c.copy(o).sub(a).normalize(),c}getTangentAt(t,e){const i=this.getUtoTmapping(t);return this.getTangent(i,e)}computeFrenetFrames(t,e=!1){const i=new C,n=[],r=[],a=[],o=new C,c=new ie;for(let f=0;f<=t;f++){const g=f/t;n[f]=this.getTangentAt(g,new C)}r[0]=new C,a[0]=new C;let l=Number.MAX_VALUE;const h=Math.abs(n[0].x),u=Math.abs(n[0].y),d=Math.abs(n[0].z);h<=l&&(l=h,i.set(1,0,0)),u<=l&&(l=u,i.set(0,1,0)),d<=l&&i.set(0,0,1),o.crossVectors(n[0],i).normalize(),r[0].crossVectors(n[0],o),a[0].crossVectors(n[0],r[0]);for(let f=1;f<=t;f++){if(r[f]=r[f-1].clone(),a[f]=a[f-1].clone(),o.crossVectors(n[f-1],n[f]),o.length()>Number.EPSILON){o.normalize();const g=Math.acos(Wt(n[f-1].dot(n[f]),-1,1));r[f].applyMatrix4(c.makeRotationAxis(o,g))}a[f].crossVectors(n[f],r[f])}if(e===!0){let f=Math.acos(Wt(r[0].dot(r[t]),-1,1));f/=t,n[0].dot(o.crossVectors(r[0],r[t]))>0&&(f=-f);for(let g=1;g<=t;g++)r[g].applyMatrix4(c.makeRotationAxis(n[g],f*g)),a[g].crossVectors(n[g],r[g])}return{tangents:n,normals:r,binormals:a}}clone(){return new this.constructor().copy(this)}copy(t){return this.arcLengthDivisions=t.arcLengthDivisions,this}toJSON(){const t={metadata:{version:4.7,type:"Curve",generator:"Curve.toJSON"}};return t.arcLengthDivisions=this.arcLengthDivisions,t.type=this.type,t}fromJSON(t){return this.arcLengthDivisions=t.arcLengthDivisions,this}}class el extends vi{constructor(t=0,e=0,i=1,n=1,r=0,a=Math.PI*2,o=!1,c=0){super(),this.isEllipseCurve=!0,this.type="EllipseCurve",this.aX=t,this.aY=e,this.xRadius=i,this.yRadius=n,this.aStartAngle=r,this.aEndAngle=a,this.aClockwise=o,this.aRotation=c}getPoint(t,e=new ht){const i=e,n=Math.PI*2;let r=this.aEndAngle-this.aStartAngle;const a=Math.abs(r)<Number.EPSILON;for(;r<0;)r+=n;for(;r>n;)r-=n;r<Number.EPSILON&&(a?r=0:r=n),this.aClockwise===!0&&!a&&(r===n?r=-n:r=r-n);const o=this.aStartAngle+t*r;let c=this.aX+this.xRadius*Math.cos(o),l=this.aY+this.yRadius*Math.sin(o);if(this.aRotation!==0){const h=Math.cos(this.aRotation),u=Math.sin(this.aRotation),d=c-this.aX,f=l-this.aY;c=d*h-f*u+this.aX,l=d*u+f*h+this.aY}return i.set(c,l)}copy(t){return super.copy(t),this.aX=t.aX,this.aY=t.aY,this.xRadius=t.xRadius,this.yRadius=t.yRadius,this.aStartAngle=t.aStartAngle,this.aEndAngle=t.aEndAngle,this.aClockwise=t.aClockwise,this.aRotation=t.aRotation,this}toJSON(){const t=super.toJSON();return t.aX=this.aX,t.aY=this.aY,t.xRadius=this.xRadius,t.yRadius=this.yRadius,t.aStartAngle=this.aStartAngle,t.aEndAngle=this.aEndAngle,t.aClockwise=this.aClockwise,t.aRotation=this.aRotation,t}fromJSON(t){return super.fromJSON(t),this.aX=t.aX,this.aY=t.aY,this.xRadius=t.xRadius,this.yRadius=t.yRadius,this.aStartAngle=t.aStartAngle,this.aEndAngle=t.aEndAngle,this.aClockwise=t.aClockwise,this.aRotation=t.aRotation,this}}class rf extends el{constructor(t,e,i,n,r,a){super(t,e,i,i,n,r,a),this.isArcCurve=!0,this.type="ArcCurve"}}function il(){let s=0,t=0,e=0,i=0;function n(r,a,o,c){s=r,t=o,e=-3*r+3*a-2*o-c,i=2*r-2*a+o+c}return{initCatmullRom:function(r,a,o,c,l){n(a,o,l*(o-r),l*(c-a))},initNonuniformCatmullRom:function(r,a,o,c,l,h,u){let d=(a-r)/l-(o-r)/(l+h)+(o-a)/h,f=(o-a)/h-(c-a)/(h+u)+(c-o)/u;d*=h,f*=h,n(a,o,d,f)},calc:function(r){const a=r*r,o=a*r;return s+t*r+e*a+i*o}}}const _r=new C,Sa=new il,ba=new il,Ea=new il;class Ch extends vi{constructor(t=[],e=!1,i="centripetal",n=.5){super(),this.isCatmullRomCurve3=!0,this.type="CatmullRomCurve3",this.points=t,this.closed=e,this.curveType=i,this.tension=n}getPoint(t,e=new C){const i=e,n=this.points,r=n.length,a=(r-(this.closed?0:1))*t;let o=Math.floor(a),c=a-o;this.closed?o+=o>0?0:(Math.floor(Math.abs(o)/r)+1)*r:c===0&&o===r-1&&(o=r-2,c=1);let l,h;this.closed||o>0?l=n[(o-1)%r]:(_r.subVectors(n[0],n[1]).add(n[0]),l=_r);const u=n[o%r],d=n[(o+1)%r];if(this.closed||o+2<r?h=n[(o+2)%r]:(_r.subVectors(n[r-1],n[r-2]).add(n[r-1]),h=_r),this.curveType==="centripetal"||this.curveType==="chordal"){const f=this.curveType==="chordal"?.5:.25;let g=Math.pow(l.distanceToSquared(u),f),v=Math.pow(u.distanceToSquared(d),f),m=Math.pow(d.distanceToSquared(h),f);v<1e-4&&(v=1),g<1e-4&&(g=v),m<1e-4&&(m=v),Sa.initNonuniformCatmullRom(l.x,u.x,d.x,h.x,g,v,m),ba.initNonuniformCatmullRom(l.y,u.y,d.y,h.y,g,v,m),Ea.initNonuniformCatmullRom(l.z,u.z,d.z,h.z,g,v,m)}else this.curveType==="catmullrom"&&(Sa.initCatmullRom(l.x,u.x,d.x,h.x,this.tension),ba.initCatmullRom(l.y,u.y,d.y,h.y,this.tension),Ea.initCatmullRom(l.z,u.z,d.z,h.z,this.tension));return i.set(Sa.calc(c),ba.calc(c),Ea.calc(c)),i}copy(t){super.copy(t),this.points=[];for(let e=0,i=t.points.length;e<i;e++){const n=t.points[e];this.points.push(n.clone())}return this.closed=t.closed,this.curveType=t.curveType,this.tension=t.tension,this}toJSON(){const t=super.toJSON();t.points=[];for(let e=0,i=this.points.length;e<i;e++){const n=this.points[e];t.points.push(n.toArray())}return t.closed=this.closed,t.curveType=this.curveType,t.tension=this.tension,t}fromJSON(t){super.fromJSON(t),this.points=[];for(let e=0,i=t.points.length;e<i;e++){const n=t.points[e];this.points.push(new C().fromArray(n))}return this.closed=t.closed,this.curveType=t.curveType,this.tension=t.tension,this}}function rc(s,t,e,i,n){const r=(i-t)*.5,a=(n-e)*.5,o=s*s,c=s*o;return(2*e-2*i+r+a)*c+(-3*e+3*i-2*r-a)*o+r*s+e}function af(s,t){const e=1-s;return e*e*t}function of(s,t){return 2*(1-s)*s*t}function lf(s,t){return s*s*t}function Rs(s,t,e,i){return af(s,t)+of(s,e)+lf(s,i)}function cf(s,t){const e=1-s;return e*e*e*t}function hf(s,t){const e=1-s;return 3*e*e*s*t}function uf(s,t){return 3*(1-s)*s*s*t}function df(s,t){return s*s*s*t}function Cs(s,t,e,i,n){return cf(s,t)+hf(s,e)+uf(s,i)+df(s,n)}class Ph extends vi{constructor(t=new ht,e=new ht,i=new ht,n=new ht){super(),this.isCubicBezierCurve=!0,this.type="CubicBezierCurve",this.v0=t,this.v1=e,this.v2=i,this.v3=n}getPoint(t,e=new ht){const i=e,n=this.v0,r=this.v1,a=this.v2,o=this.v3;return i.set(Cs(t,n.x,r.x,a.x,o.x),Cs(t,n.y,r.y,a.y,o.y)),i}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this.v3.copy(t.v3),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t.v3=this.v3.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this.v3.fromArray(t.v3),this}}class ff extends vi{constructor(t=new C,e=new C,i=new C,n=new C){super(),this.isCubicBezierCurve3=!0,this.type="CubicBezierCurve3",this.v0=t,this.v1=e,this.v2=i,this.v3=n}getPoint(t,e=new C){const i=e,n=this.v0,r=this.v1,a=this.v2,o=this.v3;return i.set(Cs(t,n.x,r.x,a.x,o.x),Cs(t,n.y,r.y,a.y,o.y),Cs(t,n.z,r.z,a.z,o.z)),i}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this.v3.copy(t.v3),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t.v3=this.v3.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this.v3.fromArray(t.v3),this}}class Ih extends vi{constructor(t=new ht,e=new ht){super(),this.isLineCurve=!0,this.type="LineCurve",this.v1=t,this.v2=e}getPoint(t,e=new ht){const i=e;return t===1?i.copy(this.v2):(i.copy(this.v2).sub(this.v1),i.multiplyScalar(t).add(this.v1)),i}getPointAt(t,e){return this.getPoint(t,e)}getTangent(t,e=new ht){return e.subVectors(this.v2,this.v1).normalize()}getTangentAt(t,e){return this.getTangent(t,e)}copy(t){return super.copy(t),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class pf extends vi{constructor(t=new C,e=new C){super(),this.isLineCurve3=!0,this.type="LineCurve3",this.v1=t,this.v2=e}getPoint(t,e=new C){const i=e;return t===1?i.copy(this.v2):(i.copy(this.v2).sub(this.v1),i.multiplyScalar(t).add(this.v1)),i}getPointAt(t,e){return this.getPoint(t,e)}getTangent(t,e=new C){return e.subVectors(this.v2,this.v1).normalize()}getTangentAt(t,e){return this.getTangent(t,e)}copy(t){return super.copy(t),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class Dh extends vi{constructor(t=new ht,e=new ht,i=new ht){super(),this.isQuadraticBezierCurve=!0,this.type="QuadraticBezierCurve",this.v0=t,this.v1=e,this.v2=i}getPoint(t,e=new ht){const i=e,n=this.v0,r=this.v1,a=this.v2;return i.set(Rs(t,n.x,r.x,a.x),Rs(t,n.y,r.y,a.y)),i}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class Lh extends vi{constructor(t=new C,e=new C,i=new C){super(),this.isQuadraticBezierCurve3=!0,this.type="QuadraticBezierCurve3",this.v0=t,this.v1=e,this.v2=i}getPoint(t,e=new C){const i=e,n=this.v0,r=this.v1,a=this.v2;return i.set(Rs(t,n.x,r.x,a.x),Rs(t,n.y,r.y,a.y),Rs(t,n.z,r.z,a.z)),i}copy(t){return super.copy(t),this.v0.copy(t.v0),this.v1.copy(t.v1),this.v2.copy(t.v2),this}toJSON(){const t=super.toJSON();return t.v0=this.v0.toArray(),t.v1=this.v1.toArray(),t.v2=this.v2.toArray(),t}fromJSON(t){return super.fromJSON(t),this.v0.fromArray(t.v0),this.v1.fromArray(t.v1),this.v2.fromArray(t.v2),this}}class Nh extends vi{constructor(t=[]){super(),this.isSplineCurve=!0,this.type="SplineCurve",this.points=t}getPoint(t,e=new ht){const i=e,n=this.points,r=(n.length-1)*t,a=Math.floor(r),o=r-a,c=n[a===0?a:a-1],l=n[a],h=n[a>n.length-2?n.length-1:a+1],u=n[a>n.length-3?n.length-1:a+2];return i.set(rc(o,c.x,l.x,h.x,u.x),rc(o,c.y,l.y,h.y,u.y)),i}copy(t){super.copy(t),this.points=[];for(let e=0,i=t.points.length;e<i;e++){const n=t.points[e];this.points.push(n.clone())}return this}toJSON(){const t=super.toJSON();t.points=[];for(let e=0,i=this.points.length;e<i;e++){const n=this.points[e];t.points.push(n.toArray())}return t}fromJSON(t){super.fromJSON(t),this.points=[];for(let e=0,i=t.points.length;e<i;e++){const n=t.points[e];this.points.push(new ht().fromArray(n))}return this}}var Lo=Object.freeze({__proto__:null,ArcCurve:rf,CatmullRomCurve3:Ch,CubicBezierCurve:Ph,CubicBezierCurve3:ff,EllipseCurve:el,LineCurve:Ih,LineCurve3:pf,QuadraticBezierCurve:Dh,QuadraticBezierCurve3:Lh,SplineCurve:Nh});class mf extends vi{constructor(){super(),this.type="CurvePath",this.curves=[],this.autoClose=!1}add(t){this.curves.push(t)}closePath(){const t=this.curves[0].getPoint(0),e=this.curves[this.curves.length-1].getPoint(1);if(!t.equals(e)){const i=t.isVector2===!0?"LineCurve":"LineCurve3";this.curves.push(new Lo[i](e,t))}return this}getPoint(t,e){const i=t*this.getLength(),n=this.getCurveLengths();let r=0;for(;r<n.length;){if(n[r]>=i){const a=n[r]-i,o=this.curves[r],c=o.getLength(),l=c===0?0:1-a/c;return o.getPointAt(l,e)}r++}return null}getLength(){const t=this.getCurveLengths();return t[t.length-1]}updateArcLengths(){this.needsUpdate=!0,this.cacheLengths=null,this.getCurveLengths()}getCurveLengths(){if(this.cacheLengths&&this.cacheLengths.length===this.curves.length)return this.cacheLengths;const t=[];let e=0;for(let i=0,n=this.curves.length;i<n;i++)e+=this.curves[i].getLength(),t.push(e);return this.cacheLengths=t,t}getSpacedPoints(t=40){const e=[];for(let i=0;i<=t;i++)e.push(this.getPoint(i/t));return this.autoClose&&e.push(e[0]),e}getPoints(t=12){const e=[];let i;for(let n=0,r=this.curves;n<r.length;n++){const a=r[n],o=a.isEllipseCurve?t*2:a.isLineCurve||a.isLineCurve3?1:a.isSplineCurve?t*a.points.length:t,c=a.getPoints(o);for(let l=0;l<c.length;l++){const h=c[l];i&&i.equals(h)||(e.push(h),i=h)}}return this.autoClose&&e.length>1&&!e[e.length-1].equals(e[0])&&e.push(e[0]),e}copy(t){super.copy(t),this.curves=[];for(let e=0,i=t.curves.length;e<i;e++){const n=t.curves[e];this.curves.push(n.clone())}return this.autoClose=t.autoClose,this}toJSON(){const t=super.toJSON();t.autoClose=this.autoClose,t.curves=[];for(let e=0,i=this.curves.length;e<i;e++){const n=this.curves[e];t.curves.push(n.toJSON())}return t}fromJSON(t){super.fromJSON(t),this.autoClose=t.autoClose,this.curves=[];for(let e=0,i=t.curves.length;e<i;e++){const n=t.curves[e];this.curves.push(new Lo[n.type]().fromJSON(n))}return this}}class ac extends mf{constructor(t){super(),this.type="Path",this.currentPoint=new ht,t&&this.setFromPoints(t)}setFromPoints(t){this.moveTo(t[0].x,t[0].y);for(let e=1,i=t.length;e<i;e++)this.lineTo(t[e].x,t[e].y);return this}moveTo(t,e){return this.currentPoint.set(t,e),this}lineTo(t,e){const i=new Ih(this.currentPoint.clone(),new ht(t,e));return this.curves.push(i),this.currentPoint.set(t,e),this}quadraticCurveTo(t,e,i,n){const r=new Dh(this.currentPoint.clone(),new ht(t,e),new ht(i,n));return this.curves.push(r),this.currentPoint.set(i,n),this}bezierCurveTo(t,e,i,n,r,a){const o=new Ph(this.currentPoint.clone(),new ht(t,e),new ht(i,n),new ht(r,a));return this.curves.push(o),this.currentPoint.set(r,a),this}splineThru(t){const e=[this.currentPoint.clone()].concat(t),i=new Nh(e);return this.curves.push(i),this.currentPoint.copy(t[t.length-1]),this}arc(t,e,i,n,r,a){const o=this.currentPoint.x,c=this.currentPoint.y;return this.absarc(t+o,e+c,i,n,r,a),this}absarc(t,e,i,n,r,a){return this.absellipse(t,e,i,i,n,r,a),this}ellipse(t,e,i,n,r,a,o,c){const l=this.currentPoint.x,h=this.currentPoint.y;return this.absellipse(t+l,e+h,i,n,r,a,o,c),this}absellipse(t,e,i,n,r,a,o,c){const l=new el(t,e,i,n,r,a,o,c);if(this.curves.length>0){const u=l.getPoint(0);u.equals(this.currentPoint)||this.lineTo(u.x,u.y)}this.curves.push(l);const h=l.getPoint(1);return this.currentPoint.copy(h),this}copy(t){return super.copy(t),this.currentPoint.copy(t.currentPoint),this}toJSON(){const t=super.toJSON();return t.currentPoint=this.currentPoint.toArray(),t}fromJSON(t){return super.fromJSON(t),this.currentPoint.fromArray(t.currentPoint),this}}class No extends ac{constructor(t){super(t),this.uuid=Ai(),this.type="Shape",this.holes=[]}getPointsHoles(t){const e=[];for(let i=0,n=this.holes.length;i<n;i++)e[i]=this.holes[i].getPoints(t);return e}extractPoints(t){return{shape:this.getPoints(t),holes:this.getPointsHoles(t)}}copy(t){super.copy(t),this.holes=[];for(let e=0,i=t.holes.length;e<i;e++){const n=t.holes[e];this.holes.push(n.clone())}return this}toJSON(){const t=super.toJSON();t.uuid=this.uuid,t.holes=[];for(let e=0,i=this.holes.length;e<i;e++){const n=this.holes[e];t.holes.push(n.toJSON())}return t}fromJSON(t){super.fromJSON(t),this.uuid=t.uuid,this.holes=[];for(let e=0,i=t.holes.length;e<i;e++){const n=t.holes[e];this.holes.push(new ac().fromJSON(n))}return this}}function gf(s,t,e=2){const i=t&&t.length,n=i?t[0]*e:s.length;let r=Uh(s,0,n,e,!0);const a=[];if(!r||r.next===r.prev)return a;let o,c,l;if(i&&(r=Mf(s,t,r,e)),s.length>80*e){o=1/0,c=1/0;let h=-1/0,u=-1/0;for(let d=e;d<n;d+=e){const f=s[d],g=s[d+1];f<o&&(o=f),g<c&&(c=g),f>h&&(h=f),g>u&&(u=g)}l=Math.max(h-o,u-c),l=l!==0?32767/l:0}return zs(r,a,e,o,c,l,0),a}function Uh(s,t,e,i,n){let r;if(n===Df(s,t,e,i)>0)for(let a=t;a<e;a+=i)r=oc(a/i|0,s[a],s[a+1],r);else for(let a=e-i;a>=t;a-=i)r=oc(a/i|0,s[a],s[a+1],r);return r&&rs(r,r.next)&&(Hs(r),r=r.next),r}function _n(s,t){if(!s)return s;t||(t=s);let e=s,i;do if(i=!1,!e.steiner&&(rs(e,e.next)||de(e.prev,e,e.next)===0)){if(Hs(e),e=t=e.prev,e===e.next)break;i=!0}else e=e.next;while(i||e!==t);return t}function zs(s,t,e,i,n,r,a){if(!s)return;!a&&r&&Tf(s,i,n,r);let o=s;for(;s.prev!==s.next;){const c=s.prev,l=s.next;if(r?_f(s,i,n,r):vf(s)){t.push(c.i,s.i,l.i),Hs(s),s=l.next,o=l.next;continue}if(s=l,s===o){a?a===1?(s=xf(_n(s),t),zs(s,t,e,i,n,r,2)):a===2&&yf(s,t,e,i,n,r):zs(_n(s),t,e,i,n,r,1);break}}}function vf(s){const t=s.prev,e=s,i=s.next;if(de(t,e,i)>=0)return!1;const n=t.x,r=e.x,a=i.x,o=t.y,c=e.y,l=i.y,h=Math.min(n,r,a),u=Math.min(o,c,l),d=Math.max(n,r,a),f=Math.max(o,c,l);let g=i.next;for(;g!==t;){if(g.x>=h&&g.x<=d&&g.y>=u&&g.y<=f&&ws(n,o,r,c,a,l,g.x,g.y)&&de(g.prev,g,g.next)>=0)return!1;g=g.next}return!0}function _f(s,t,e,i){const n=s.prev,r=s,a=s.next;if(de(n,r,a)>=0)return!1;const o=n.x,c=r.x,l=a.x,h=n.y,u=r.y,d=a.y,f=Math.min(o,c,l),g=Math.min(h,u,d),v=Math.max(o,c,l),m=Math.max(h,u,d),p=Uo(f,g,t,e,i),w=Uo(v,m,t,e,i);let E=s.prevZ,_=s.nextZ;for(;E&&E.z>=p&&_&&_.z<=w;){if(E.x>=f&&E.x<=v&&E.y>=g&&E.y<=m&&E!==n&&E!==a&&ws(o,h,c,u,l,d,E.x,E.y)&&de(E.prev,E,E.next)>=0||(E=E.prevZ,_.x>=f&&_.x<=v&&_.y>=g&&_.y<=m&&_!==n&&_!==a&&ws(o,h,c,u,l,d,_.x,_.y)&&de(_.prev,_,_.next)>=0))return!1;_=_.nextZ}for(;E&&E.z>=p;){if(E.x>=f&&E.x<=v&&E.y>=g&&E.y<=m&&E!==n&&E!==a&&ws(o,h,c,u,l,d,E.x,E.y)&&de(E.prev,E,E.next)>=0)return!1;E=E.prevZ}for(;_&&_.z<=w;){if(_.x>=f&&_.x<=v&&_.y>=g&&_.y<=m&&_!==n&&_!==a&&ws(o,h,c,u,l,d,_.x,_.y)&&de(_.prev,_,_.next)>=0)return!1;_=_.nextZ}return!0}function xf(s,t){let e=s;do{const i=e.prev,n=e.next.next;!rs(i,n)&&Fh(i,e,e.next,n)&&Bs(i,n)&&Bs(n,i)&&(t.push(i.i,e.i,n.i),Hs(e),Hs(e.next),e=s=n),e=e.next}while(e!==s);return _n(e)}function yf(s,t,e,i,n,r){let a=s;do{let o=a.next.next;for(;o!==a.prev;){if(a.i!==o.i&&Cf(a,o)){let c=kh(a,o);a=_n(a,a.next),c=_n(c,c.next),zs(a,t,e,i,n,r,0),zs(c,t,e,i,n,r,0);return}o=o.next}a=a.next}while(a!==s)}function Mf(s,t,e,i){const n=[];for(let r=0,a=t.length;r<a;r++){const o=t[r]*i,c=r<a-1?t[r+1]*i:s.length,l=Uh(s,o,c,i,!1);l===l.next&&(l.steiner=!0),n.push(Rf(l))}n.sort(Sf);for(let r=0;r<n.length;r++)e=bf(n[r],e);return e}function Sf(s,t){let e=s.x-t.x;if(e===0&&(e=s.y-t.y,e===0)){const i=(s.next.y-s.y)/(s.next.x-s.x),n=(t.next.y-t.y)/(t.next.x-t.x);e=i-n}return e}function bf(s,t){const e=Ef(s,t);if(!e)return t;const i=kh(e,s);return _n(i,i.next),_n(e,e.next)}function Ef(s,t){let e=t;const i=s.x,n=s.y;let r=-1/0,a;if(rs(s,e))return e;do{if(rs(s,e.next))return e.next;if(n<=e.y&&n>=e.next.y&&e.next.y!==e.y){const u=e.x+(n-e.y)*(e.next.x-e.x)/(e.next.y-e.y);if(u<=i&&u>r&&(r=u,a=e.x<e.next.x?e:e.next,u===i))return a}e=e.next}while(e!==t);if(!a)return null;const o=a,c=a.x,l=a.y;let h=1/0;e=a;do{if(i>=e.x&&e.x>=c&&i!==e.x&&Oh(n<l?i:r,n,c,l,n<l?r:i,n,e.x,e.y)){const u=Math.abs(n-e.y)/(i-e.x);Bs(e,s)&&(u<h||u===h&&(e.x>a.x||e.x===a.x&&wf(a,e)))&&(a=e,h=u)}e=e.next}while(e!==o);return a}function wf(s,t){return de(s.prev,s,t.prev)<0&&de(t.next,s,s.next)<0}function Tf(s,t,e,i){let n=s;do n.z===0&&(n.z=Uo(n.x,n.y,t,e,i)),n.prevZ=n.prev,n.nextZ=n.next,n=n.next;while(n!==s);n.prevZ.nextZ=null,n.prevZ=null,Af(n)}function Af(s){let t,e=1;do{let i=s,n;s=null;let r=null;for(t=0;i;){t++;let a=i,o=0;for(let l=0;l<e&&(o++,a=a.nextZ,!!a);l++);let c=e;for(;o>0||c>0&&a;)o!==0&&(c===0||!a||i.z<=a.z)?(n=i,i=i.nextZ,o--):(n=a,a=a.nextZ,c--),r?r.nextZ=n:s=n,n.prevZ=r,r=n;i=a}r.nextZ=null,e*=2}while(t>1);return s}function Uo(s,t,e,i,n){return s=(s-e)*n|0,t=(t-i)*n|0,s=(s|s<<8)&16711935,s=(s|s<<4)&252645135,s=(s|s<<2)&858993459,s=(s|s<<1)&1431655765,t=(t|t<<8)&16711935,t=(t|t<<4)&252645135,t=(t|t<<2)&858993459,t=(t|t<<1)&1431655765,s|t<<1}function Rf(s){let t=s,e=s;do(t.x<e.x||t.x===e.x&&t.y<e.y)&&(e=t),t=t.next;while(t!==s);return e}function Oh(s,t,e,i,n,r,a,o){return(n-a)*(t-o)>=(s-a)*(r-o)&&(s-a)*(i-o)>=(e-a)*(t-o)&&(e-a)*(r-o)>=(n-a)*(i-o)}function ws(s,t,e,i,n,r,a,o){return!(s===a&&t===o)&&Oh(s,t,e,i,n,r,a,o)}function Cf(s,t){return s.next.i!==t.i&&s.prev.i!==t.i&&!Pf(s,t)&&(Bs(s,t)&&Bs(t,s)&&If(s,t)&&(de(s.prev,s,t.prev)||de(s,t.prev,t))||rs(s,t)&&de(s.prev,s,s.next)>0&&de(t.prev,t,t.next)>0)}function de(s,t,e){return(t.y-s.y)*(e.x-t.x)-(t.x-s.x)*(e.y-t.y)}function rs(s,t){return s.x===t.x&&s.y===t.y}function Fh(s,t,e,i){const n=yr(de(s,t,e)),r=yr(de(s,t,i)),a=yr(de(e,i,s)),o=yr(de(e,i,t));return!!(n!==r&&a!==o||n===0&&xr(s,e,t)||r===0&&xr(s,i,t)||a===0&&xr(e,s,i)||o===0&&xr(e,t,i))}function xr(s,t,e){return t.x<=Math.max(s.x,e.x)&&t.x>=Math.min(s.x,e.x)&&t.y<=Math.max(s.y,e.y)&&t.y>=Math.min(s.y,e.y)}function yr(s){return s>0?1:s<0?-1:0}function Pf(s,t){let e=s;do{if(e.i!==s.i&&e.next.i!==s.i&&e.i!==t.i&&e.next.i!==t.i&&Fh(e,e.next,s,t))return!0;e=e.next}while(e!==s);return!1}function Bs(s,t){return de(s.prev,s,s.next)<0?de(s,t,s.next)>=0&&de(s,s.prev,t)>=0:de(s,t,s.prev)<0||de(s,s.next,t)<0}function If(s,t){let e=s,i=!1;const n=(s.x+t.x)/2,r=(s.y+t.y)/2;do e.y>r!=e.next.y>r&&e.next.y!==e.y&&n<(e.next.x-e.x)*(r-e.y)/(e.next.y-e.y)+e.x&&(i=!i),e=e.next;while(e!==s);return i}function kh(s,t){const e=Oo(s.i,s.x,s.y),i=Oo(t.i,t.x,t.y),n=s.next,r=t.prev;return s.next=t,t.prev=s,e.next=n,n.prev=e,i.next=e,e.prev=i,r.next=i,i.prev=r,i}function oc(s,t,e,i){const n=Oo(s,t,e);return i?(n.next=i.next,n.prev=i,i.next.prev=n,i.next=n):(n.prev=n,n.next=n),n}function Hs(s){s.next.prev=s.prev,s.prev.next=s.next,s.prevZ&&(s.prevZ.nextZ=s.nextZ),s.nextZ&&(s.nextZ.prevZ=s.prevZ)}function Oo(s,t,e){return{i:s,x:t,y:e,prev:null,next:null,z:0,prevZ:null,nextZ:null,steiner:!1}}function Df(s,t,e,i){let n=0;for(let r=t,a=e-i;r<e;r+=i)n+=(s[a]-s[r])*(s[r+1]+s[a+1]),a=r;return n}class Lf{static triangulate(t,e,i=2){return gf(t,e,i)}}class Ps{static area(t){const e=t.length;let i=0;for(let n=e-1,r=0;r<e;n=r++)i+=t[n].x*t[r].y-t[r].x*t[n].y;return i*.5}static isClockWise(t){return Ps.area(t)<0}static triangulateShape(t,e){const i=[],n=[],r=[];lc(t),cc(i,t);let a=t.length;e.forEach(lc);for(let c=0;c<e.length;c++)n.push(a),a+=e[c].length,cc(i,e[c]);const o=Lf.triangulate(i,n);for(let c=0;c<o.length;c+=3)r.push(o.slice(c,c+3));return r}}function lc(s){const t=s.length;t>2&&s[t-1].equals(s[0])&&s.pop()}function cc(s,t){for(let e=0;e<t.length;e++)s.push(t[e].x),s.push(t[e].y)}class as extends we{constructor(t=1,e=1,i=1,n=1){super(),this.type="PlaneGeometry",this.parameters={width:t,height:e,widthSegments:i,heightSegments:n};const r=t/2,a=e/2,o=Math.floor(i),c=Math.floor(n),l=o+1,h=c+1,u=t/o,d=e/c,f=[],g=[],v=[],m=[];for(let p=0;p<h;p++){const w=p*d-a;for(let E=0;E<l;E++){const _=E*u-r;g.push(_,-w,0),v.push(0,0,1),m.push(E/o),m.push(1-p/c)}}for(let p=0;p<c;p++)for(let w=0;w<o;w++){const E=w+l*p,_=w+l*(p+1),P=w+1+l*(p+1),R=w+1+l*p;f.push(E,_,R),f.push(_,P,R)}this.setIndex(f),this.setAttribute("position",new jt(g,3)),this.setAttribute("normal",new jt(v,3)),this.setAttribute("uv",new jt(m,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new as(t.width,t.height,t.widthSegments,t.heightSegments)}}class mn extends we{constructor(t=.5,e=1,i=32,n=1,r=0,a=Math.PI*2){super(),this.type="RingGeometry",this.parameters={innerRadius:t,outerRadius:e,thetaSegments:i,phiSegments:n,thetaStart:r,thetaLength:a},i=Math.max(3,i),n=Math.max(1,n);const o=[],c=[],l=[],h=[];let u=t;const d=(e-t)/n,f=new C,g=new ht;for(let v=0;v<=n;v++){for(let m=0;m<=i;m++){const p=r+m/i*a;f.x=u*Math.cos(p),f.y=u*Math.sin(p),c.push(f.x,f.y,f.z),l.push(0,0,1),g.x=(f.x/e+1)/2,g.y=(f.y/e+1)/2,h.push(g.x,g.y)}u+=d}for(let v=0;v<n;v++){const m=v*(i+1);for(let p=0;p<i;p++){const w=p+m,E=w,_=w+i+1,P=w+i+2,R=w+1;o.push(E,_,R),o.push(_,P,R)}}this.setIndex(o),this.setAttribute("position",new jt(c,3)),this.setAttribute("normal",new jt(l,3)),this.setAttribute("uv",new jt(h,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new mn(t.innerRadius,t.outerRadius,t.thetaSegments,t.phiSegments,t.thetaStart,t.thetaLength)}}class Hr extends we{constructor(t=new No([new ht(0,.5),new ht(-.5,-.5),new ht(.5,-.5)]),e=12){super(),this.type="ShapeGeometry",this.parameters={shapes:t,curveSegments:e};const i=[],n=[],r=[],a=[];let o=0,c=0;if(Array.isArray(t)===!1)l(t);else for(let h=0;h<t.length;h++)l(t[h]),this.addGroup(o,c,h),o+=c,c=0;this.setIndex(i),this.setAttribute("position",new jt(n,3)),this.setAttribute("normal",new jt(r,3)),this.setAttribute("uv",new jt(a,2));function l(h){const u=n.length/3,d=h.extractPoints(e);let f=d.shape;const g=d.holes;Ps.isClockWise(f)===!1&&(f=f.reverse());for(let m=0,p=g.length;m<p;m++){const w=g[m];Ps.isClockWise(w)===!0&&(g[m]=w.reverse())}const v=Ps.triangulateShape(f,g);for(let m=0,p=g.length;m<p;m++){const w=g[m];f=f.concat(w)}for(let m=0,p=f.length;m<p;m++){const w=f[m];n.push(w.x,w.y,0),r.push(0,0,1),a.push(w.x,w.y)}for(let m=0,p=v.length;m<p;m++){const w=v[m],E=w[0]+u,_=w[1]+u,P=w[2]+u;i.push(E,_,P),c+=3}}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}toJSON(){const t=super.toJSON(),e=this.parameters.shapes;return Nf(e,t)}static fromJSON(t,e){const i=[];for(let n=0,r=t.shapes.length;n<r;n++){const a=e[t.shapes[n]];i.push(a)}return new Hr(i,t.curveSegments)}}function Nf(s,t){if(t.shapes=[],Array.isArray(s))for(let e=0,i=s.length;e<i;e++){const n=s[e];t.shapes.push(n.uuid)}else t.shapes.push(s.uuid);return t}class hn extends we{constructor(t=1,e=32,i=16,n=0,r=Math.PI*2,a=0,o=Math.PI){super(),this.type="SphereGeometry",this.parameters={radius:t,widthSegments:e,heightSegments:i,phiStart:n,phiLength:r,thetaStart:a,thetaLength:o},e=Math.max(3,Math.floor(e)),i=Math.max(2,Math.floor(i));const c=Math.min(a+o,Math.PI);let l=0;const h=[],u=new C,d=new C,f=[],g=[],v=[],m=[];for(let p=0;p<=i;p++){const w=[],E=p/i;let _=0;p===0&&a===0?_=.5/e:p===i&&c===Math.PI&&(_=-.5/e);for(let P=0;P<=e;P++){const R=P/e;u.x=-t*Math.cos(n+R*r)*Math.sin(a+E*o),u.y=t*Math.cos(a+E*o),u.z=t*Math.sin(n+R*r)*Math.sin(a+E*o),g.push(u.x,u.y,u.z),d.copy(u).normalize(),v.push(d.x,d.y,d.z),m.push(R+_,1-E),w.push(l++)}h.push(w)}for(let p=0;p<i;p++)for(let w=0;w<e;w++){const E=h[p][w+1],_=h[p][w],P=h[p+1][w],R=h[p+1][w+1];(p!==0||a>0)&&f.push(E,_,R),(p!==i-1||c<Math.PI)&&f.push(_,P,R)}this.setIndex(f),this.setAttribute("position",new jt(g,3)),this.setAttribute("normal",new jt(v,3)),this.setAttribute("uv",new jt(m,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new hn(t.radius,t.widthSegments,t.heightSegments,t.phiStart,t.phiLength,t.thetaStart,t.thetaLength)}}class pi extends we{constructor(t=1,e=.4,i=12,n=48,r=Math.PI*2){super(),this.type="TorusGeometry",this.parameters={radius:t,tube:e,radialSegments:i,tubularSegments:n,arc:r},i=Math.floor(i),n=Math.floor(n);const a=[],o=[],c=[],l=[],h=new C,u=new C,d=new C;for(let f=0;f<=i;f++)for(let g=0;g<=n;g++){const v=g/n*r,m=f/i*Math.PI*2;u.x=(t+e*Math.cos(m))*Math.cos(v),u.y=(t+e*Math.cos(m))*Math.sin(v),u.z=e*Math.sin(m),o.push(u.x,u.y,u.z),h.x=t*Math.cos(v),h.y=t*Math.sin(v),d.subVectors(u,h).normalize(),c.push(d.x,d.y,d.z),l.push(g/n),l.push(f/i)}for(let f=1;f<=i;f++)for(let g=1;g<=n;g++){const v=(n+1)*f+g-1,m=(n+1)*(f-1)+g-1,p=(n+1)*(f-1)+g,w=(n+1)*f+g;a.push(v,m,w),a.push(m,p,w)}this.setIndex(a),this.setAttribute("position",new jt(o,3)),this.setAttribute("normal",new jt(c,3)),this.setAttribute("uv",new jt(l,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new pi(t.radius,t.tube,t.radialSegments,t.tubularSegments,t.arc)}}class nl extends we{constructor(t=new Lh(new C(-1,-1,0),new C(-1,1,0),new C(1,1,0)),e=64,i=1,n=8,r=!1){super(),this.type="TubeGeometry",this.parameters={path:t,tubularSegments:e,radius:i,radialSegments:n,closed:r};const a=t.computeFrenetFrames(e,r);this.tangents=a.tangents,this.normals=a.normals,this.binormals=a.binormals;const o=new C,c=new C,l=new ht;let h=new C;const u=[],d=[],f=[],g=[];v(),this.setIndex(g),this.setAttribute("position",new jt(u,3)),this.setAttribute("normal",new jt(d,3)),this.setAttribute("uv",new jt(f,2));function v(){for(let E=0;E<e;E++)m(E);m(r===!1?e:0),w(),p()}function m(E){h=t.getPointAt(E/e,h);const _=a.normals[E],P=a.binormals[E];for(let R=0;R<=n;R++){const I=R/n*Math.PI*2,T=Math.sin(I),S=-Math.cos(I);c.x=S*_.x+T*P.x,c.y=S*_.y+T*P.y,c.z=S*_.z+T*P.z,c.normalize(),d.push(c.x,c.y,c.z),o.x=h.x+i*c.x,o.y=h.y+i*c.y,o.z=h.z+i*c.z,u.push(o.x,o.y,o.z)}}function p(){for(let E=1;E<=e;E++)for(let _=1;_<=n;_++){const P=(n+1)*(E-1)+(_-1),R=(n+1)*E+(_-1),I=(n+1)*E+_,T=(n+1)*(E-1)+_;g.push(P,R,T),g.push(R,I,T)}}function w(){for(let E=0;E<=e;E++)for(let _=0;_<=n;_++)l.x=E/e,l.y=_/n,f.push(l.x,l.y)}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}toJSON(){const t=super.toJSON();return t.path=this.parameters.path.toJSON(),t}static fromJSON(t){return new nl(new Lo[t.path.type]().fromJSON(t.path),t.tubularSegments,t.radius,t.radialSegments,t.closed)}}class Vn extends yn{constructor(t){super(),this.isMeshStandardMaterial=!0,this.type="MeshStandardMaterial",this.defines={STANDARD:""},this.color=new Ht(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new Ht(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=uh,this.normalScale=new ht(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new gi,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.defines={STANDARD:""},this.color.copy(t.color),this.roughness=t.roughness,this.metalness=t.metalness,this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.roughnessMap=t.roughnessMap,this.metalnessMap=t.metalnessMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapRotation.copy(t.envMapRotation),this.envMapIntensity=t.envMapIntensity,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}class Uf extends yn{constructor(t){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=pd,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(t)}copy(t){return super.copy(t),this.depthPacking=t.depthPacking,this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this}}class Of extends yn{constructor(t){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(t)}copy(t){return super.copy(t),this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this}}class zh extends _e{constructor(t,e=1){super(),this.isLight=!0,this.type="Light",this.color=new Ht(t),this.intensity=e}dispose(){}copy(t,e){return super.copy(t,e),this.color.copy(t.color),this.intensity=t.intensity,this}toJSON(t){const e=super.toJSON(t);return e.object.color=this.color.getHex(),e.object.intensity=this.intensity,this.groundColor!==void 0&&(e.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(e.object.distance=this.distance),this.angle!==void 0&&(e.object.angle=this.angle),this.decay!==void 0&&(e.object.decay=this.decay),this.penumbra!==void 0&&(e.object.penumbra=this.penumbra),this.shadow!==void 0&&(e.object.shadow=this.shadow.toJSON()),this.target!==void 0&&(e.object.target=this.target.uuid),e}}class Ff extends zh{constructor(t,e,i){super(t,i),this.isHemisphereLight=!0,this.type="HemisphereLight",this.position.copy(_e.DEFAULT_UP),this.updateMatrix(),this.groundColor=new Ht(e)}copy(t,e){return super.copy(t,e),this.groundColor.copy(t.groundColor),this}}const wa=new ie,hc=new C,uc=new C;class kf{constructor(t){this.camera=t,this.intensity=1,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new ht(512,512),this.mapType=mi,this.map=null,this.mapPass=null,this.matrix=new ie,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new tl,this._frameExtents=new ht(1,1),this._viewportCount=1,this._viewports=[new pe(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(t){const e=this.camera,i=this.matrix;hc.setFromMatrixPosition(t.matrixWorld),e.position.copy(hc),uc.setFromMatrixPosition(t.target.matrixWorld),e.lookAt(uc),e.updateMatrixWorld(),wa.multiplyMatrices(e.projectionMatrix,e.matrixWorldInverse),this._frustum.setFromProjectionMatrix(wa,e.coordinateSystem,e.reversedDepth),e.reversedDepth?i.set(.5,0,0,.5,0,.5,0,.5,0,0,1,0,0,0,0,1):i.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),i.multiply(wa)}getViewport(t){return this._viewports[t]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(t){return this.camera=t.camera.clone(),this.intensity=t.intensity,this.bias=t.bias,this.radius=t.radius,this.autoUpdate=t.autoUpdate,this.needsUpdate=t.needsUpdate,this.normalBias=t.normalBias,this.blurSamples=t.blurSamples,this.mapSize.copy(t.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){const t={};return this.intensity!==1&&(t.intensity=this.intensity),this.bias!==0&&(t.bias=this.bias),this.normalBias!==0&&(t.normalBias=this.normalBias),this.radius!==1&&(t.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(t.mapSize=this.mapSize.toArray()),t.camera=this.camera.toJSON(!1).object,delete t.camera.matrix,t}}class sl extends Sh{constructor(t=-1,e=1,i=1,n=-1,r=.1,a=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=t,this.right=e,this.top=i,this.bottom=n,this.near=r,this.far=a,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.left=t.left,this.right=t.right,this.top=t.top,this.bottom=t.bottom,this.near=t.near,this.far=t.far,this.zoom=t.zoom,this.view=t.view===null?null:Object.assign({},t.view),this}setViewOffset(t,e,i,n,r,a){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=i,this.view.offsetY=n,this.view.width=r,this.view.height=a,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=(this.right-this.left)/(2*this.zoom),e=(this.top-this.bottom)/(2*this.zoom),i=(this.right+this.left)/2,n=(this.top+this.bottom)/2;let r=i-t,a=i+t,o=n+e,c=n-e;if(this.view!==null&&this.view.enabled){const l=(this.right-this.left)/this.view.fullWidth/this.zoom,h=(this.top-this.bottom)/this.view.fullHeight/this.zoom;r+=l*this.view.offsetX,a=r+l*this.view.width,o-=h*this.view.offsetY,c=o-h*this.view.height}this.projectionMatrix.makeOrthographic(r,a,o,c,this.near,this.far,this.coordinateSystem,this.reversedDepth),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.zoom=this.zoom,e.object.left=this.left,e.object.right=this.right,e.object.top=this.top,e.object.bottom=this.bottom,e.object.near=this.near,e.object.far=this.far,this.view!==null&&(e.object.view=Object.assign({},this.view)),e}}class zf extends kf{constructor(){super(new sl(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}}class dc extends zh{constructor(t,e){super(t,e),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(_e.DEFAULT_UP),this.updateMatrix(),this.target=new _e,this.shadow=new zf}dispose(){this.shadow.dispose()}copy(t){return super.copy(t),this.target=t.target.clone(),this.shadow=t.shadow.clone(),this}}class Bf extends si{constructor(t=[]){super(),this.isArrayCamera=!0,this.isMultiViewCamera=!1,this.cameras=t}}function fc(s,t,e,i){const n=Hf(i);switch(e){case lh:return s*t;case qo:return s*t/n.components*n.byteLength;case Ko:return s*t/n.components*n.byteLength;case hh:return s*t*2/n.components*n.byteLength;case Zo:return s*t*2/n.components*n.byteLength;case ch:return s*t*3/n.components*n.byteLength;case ri:return s*t*4/n.components*n.byteLength;case Jo:return s*t*4/n.components*n.byteLength;case Ar:case Rr:return Math.floor((s+3)/4)*Math.floor((t+3)/4)*8;case Cr:case Pr:return Math.floor((s+3)/4)*Math.floor((t+3)/4)*16;case io:case so:return Math.max(s,16)*Math.max(t,8)/4;case eo:case no:return Math.max(s,8)*Math.max(t,8)/2;case ro:case ao:return Math.floor((s+3)/4)*Math.floor((t+3)/4)*8;case oo:return Math.floor((s+3)/4)*Math.floor((t+3)/4)*16;case lo:return Math.floor((s+3)/4)*Math.floor((t+3)/4)*16;case co:return Math.floor((s+4)/5)*Math.floor((t+3)/4)*16;case ho:return Math.floor((s+4)/5)*Math.floor((t+4)/5)*16;case uo:return Math.floor((s+5)/6)*Math.floor((t+4)/5)*16;case fo:return Math.floor((s+5)/6)*Math.floor((t+5)/6)*16;case po:return Math.floor((s+7)/8)*Math.floor((t+4)/5)*16;case mo:return Math.floor((s+7)/8)*Math.floor((t+5)/6)*16;case go:return Math.floor((s+7)/8)*Math.floor((t+7)/8)*16;case vo:return Math.floor((s+9)/10)*Math.floor((t+4)/5)*16;case _o:return Math.floor((s+9)/10)*Math.floor((t+5)/6)*16;case xo:return Math.floor((s+9)/10)*Math.floor((t+7)/8)*16;case yo:return Math.floor((s+9)/10)*Math.floor((t+9)/10)*16;case Mo:return Math.floor((s+11)/12)*Math.floor((t+9)/10)*16;case So:return Math.floor((s+11)/12)*Math.floor((t+11)/12)*16;case bo:case Eo:case wo:return Math.ceil(s/4)*Math.ceil(t/4)*16;case To:case Ao:return Math.ceil(s/4)*Math.ceil(t/4)*8;case Ro:case Co:return Math.ceil(s/4)*Math.ceil(t/4)*16}throw new Error(`Unable to determine texture byte length for ${e} format.`)}function Hf(s){switch(s){case mi:case sh:return{byteLength:1,components:1};case Ls:case rh:case Gs:return{byteLength:2,components:1};case $o:case Yo:return{byteLength:2,components:4};case gn:case Xo:case di:return{byteLength:4,components:1};case ah:case oh:return{byteLength:4,components:3}}throw new Error(`Unknown texture type ${s}.`)}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:Wo}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__=Wo);function Bh(){let s=null,t=!1,e=null,i=null;function n(r,a){e(r,a),i=s.requestAnimationFrame(n)}return{start:function(){t!==!0&&e!==null&&(i=s.requestAnimationFrame(n),t=!0)},stop:function(){s.cancelAnimationFrame(i),t=!1},setAnimationLoop:function(r){e=r},setContext:function(r){s=r}}}function Gf(s){const t=new WeakMap;function e(o,c){const l=o.array,h=o.usage,u=l.byteLength,d=s.createBuffer();s.bindBuffer(c,d),s.bufferData(c,l,h),o.onUploadCallback();let f;if(l instanceof Float32Array)f=s.FLOAT;else if(typeof Float16Array<"u"&&l instanceof Float16Array)f=s.HALF_FLOAT;else if(l instanceof Uint16Array)o.isFloat16BufferAttribute?f=s.HALF_FLOAT:f=s.UNSIGNED_SHORT;else if(l instanceof Int16Array)f=s.SHORT;else if(l instanceof Uint32Array)f=s.UNSIGNED_INT;else if(l instanceof Int32Array)f=s.INT;else if(l instanceof Int8Array)f=s.BYTE;else if(l instanceof Uint8Array)f=s.UNSIGNED_BYTE;else if(l instanceof Uint8ClampedArray)f=s.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+l);return{buffer:d,type:f,bytesPerElement:l.BYTES_PER_ELEMENT,version:o.version,size:u}}function i(o,c,l){const h=c.array,u=c.updateRanges;if(s.bindBuffer(l,o),u.length===0)s.bufferSubData(l,0,h);else{u.sort((f,g)=>f.start-g.start);let d=0;for(let f=1;f<u.length;f++){const g=u[d],v=u[f];v.start<=g.start+g.count+1?g.count=Math.max(g.count,v.start+v.count-g.start):(++d,u[d]=v)}u.length=d+1;for(let f=0,g=u.length;f<g;f++){const v=u[f];s.bufferSubData(l,v.start*h.BYTES_PER_ELEMENT,h,v.start,v.count)}c.clearUpdateRanges()}c.onUploadCallback()}function n(o){return o.isInterleavedBufferAttribute&&(o=o.data),t.get(o)}function r(o){o.isInterleavedBufferAttribute&&(o=o.data);const c=t.get(o);c&&(s.deleteBuffer(c.buffer),t.delete(o))}function a(o,c){if(o.isInterleavedBufferAttribute&&(o=o.data),o.isGLBufferAttribute){const h=t.get(o);(!h||h.version<o.version)&&t.set(o,{buffer:o.buffer,type:o.type,bytesPerElement:o.elementSize,version:o.version});return}const l=t.get(o);if(l===void 0)t.set(o,e(o,c));else if(l.version<o.version){if(l.size!==o.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");i(l.buffer,o,c),l.version=o.version}}return{get:n,remove:r,update:a}}var Vf=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,Wf=`#ifdef USE_ALPHAHASH
	const float ALPHA_HASH_SCALE = 0.05;
	float hash2D( vec2 value ) {
		return fract( 1.0e4 * sin( 17.0 * value.x + 0.1 * value.y ) * ( 0.1 + abs( sin( 13.0 * value.y + value.x ) ) ) );
	}
	float hash3D( vec3 value ) {
		return hash2D( vec2( hash2D( value.xy ), value.z ) );
	}
	float getAlphaHashThreshold( vec3 position ) {
		float maxDeriv = max(
			length( dFdx( position.xyz ) ),
			length( dFdy( position.xyz ) )
		);
		float pixScale = 1.0 / ( ALPHA_HASH_SCALE * maxDeriv );
		vec2 pixScales = vec2(
			exp2( floor( log2( pixScale ) ) ),
			exp2( ceil( log2( pixScale ) ) )
		);
		vec2 alpha = vec2(
			hash3D( floor( pixScales.x * position.xyz ) ),
			hash3D( floor( pixScales.y * position.xyz ) )
		);
		float lerpFactor = fract( log2( pixScale ) );
		float x = ( 1.0 - lerpFactor ) * alpha.x + lerpFactor * alpha.y;
		float a = min( lerpFactor, 1.0 - lerpFactor );
		vec3 cases = vec3(
			x * x / ( 2.0 * a * ( 1.0 - a ) ),
			( x - 0.5 * a ) / ( 1.0 - a ),
			1.0 - ( ( 1.0 - x ) * ( 1.0 - x ) / ( 2.0 * a * ( 1.0 - a ) ) )
		);
		float threshold = ( x < ( 1.0 - a ) )
			? ( ( x < a ) ? cases.x : cases.y )
			: cases.z;
		return clamp( threshold , 1.0e-6, 1.0 );
	}
#endif`,Xf=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,$f=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,Yf=`#ifdef USE_ALPHATEST
	#ifdef ALPHA_TO_COVERAGE
	diffuseColor.a = smoothstep( alphaTest, alphaTest + fwidth( diffuseColor.a ), diffuseColor.a );
	if ( diffuseColor.a == 0.0 ) discard;
	#else
	if ( diffuseColor.a < alphaTest ) discard;
	#endif
#endif`,qf=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,Kf=`#ifdef USE_AOMAP
	float ambientOcclusion = ( texture2D( aoMap, vAoMapUv ).r - 1.0 ) * aoMapIntensity + 1.0;
	reflectedLight.indirectDiffuse *= ambientOcclusion;
	#if defined( USE_CLEARCOAT ) 
		clearcoatSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_SHEEN ) 
		sheenSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD )
		float dotNV = saturate( dot( geometryNormal, geometryViewDir ) );
		reflectedLight.indirectSpecular *= computeSpecularOcclusion( dotNV, ambientOcclusion, material.roughness );
	#endif
#endif`,Zf=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,Jf=`#ifdef USE_BATCHING
	#if ! defined( GL_ANGLE_multi_draw )
	#define gl_DrawID _gl_DrawID
	uniform int _gl_DrawID;
	#endif
	uniform highp sampler2D batchingTexture;
	uniform highp usampler2D batchingIdTexture;
	mat4 getBatchingMatrix( const in float i ) {
		int size = textureSize( batchingTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( batchingTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( batchingTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( batchingTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( batchingTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
	float getIndirectIndex( const in int i ) {
		int size = textureSize( batchingIdTexture, 0 ).x;
		int x = i % size;
		int y = i / size;
		return float( texelFetch( batchingIdTexture, ivec2( x, y ), 0 ).r );
	}
#endif
#ifdef USE_BATCHING_COLOR
	uniform sampler2D batchingColorTexture;
	vec3 getBatchingColor( const in float i ) {
		int size = textureSize( batchingColorTexture, 0 ).x;
		int j = int( i );
		int x = j % size;
		int y = j / size;
		return texelFetch( batchingColorTexture, ivec2( x, y ), 0 ).rgb;
	}
#endif`,jf=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( getIndirectIndex( gl_DrawID ) );
#endif`,Qf=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,tp=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,ep=`float G_BlinnPhong_Implicit( ) {
	return 0.25;
}
float D_BlinnPhong( const in float shininess, const in float dotNH ) {
	return RECIPROCAL_PI * ( shininess * 0.5 + 1.0 ) * pow( dotNH, shininess );
}
vec3 BRDF_BlinnPhong( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in vec3 specularColor, const in float shininess ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( specularColor, 1.0, dotVH );
	float G = G_BlinnPhong_Implicit( );
	float D = D_BlinnPhong( shininess, dotNH );
	return F * ( G * D );
} // validated`,ip=`#ifdef USE_IRIDESCENCE
	const mat3 XYZ_TO_REC709 = mat3(
		 3.2404542, -0.9692660,  0.0556434,
		-1.5371385,  1.8760108, -0.2040259,
		-0.4985314,  0.0415560,  1.0572252
	);
	vec3 Fresnel0ToIor( vec3 fresnel0 ) {
		vec3 sqrtF0 = sqrt( fresnel0 );
		return ( vec3( 1.0 ) + sqrtF0 ) / ( vec3( 1.0 ) - sqrtF0 );
	}
	vec3 IorToFresnel0( vec3 transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - vec3( incidentIor ) ) / ( transmittedIor + vec3( incidentIor ) ) );
	}
	float IorToFresnel0( float transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - incidentIor ) / ( transmittedIor + incidentIor ));
	}
	vec3 evalSensitivity( float OPD, vec3 shift ) {
		float phase = 2.0 * PI * OPD * 1.0e-9;
		vec3 val = vec3( 5.4856e-13, 4.4201e-13, 5.2481e-13 );
		vec3 pos = vec3( 1.6810e+06, 1.7953e+06, 2.2084e+06 );
		vec3 var = vec3( 4.3278e+09, 9.3046e+09, 6.6121e+09 );
		vec3 xyz = val * sqrt( 2.0 * PI * var ) * cos( pos * phase + shift ) * exp( - pow2( phase ) * var );
		xyz.x += 9.7470e-14 * sqrt( 2.0 * PI * 4.5282e+09 ) * cos( 2.2399e+06 * phase + shift[ 0 ] ) * exp( - 4.5282e+09 * pow2( phase ) );
		xyz /= 1.0685e-7;
		vec3 rgb = XYZ_TO_REC709 * xyz;
		return rgb;
	}
	vec3 evalIridescence( float outsideIOR, float eta2, float cosTheta1, float thinFilmThickness, vec3 baseF0 ) {
		vec3 I;
		float iridescenceIOR = mix( outsideIOR, eta2, smoothstep( 0.0, 0.03, thinFilmThickness ) );
		float sinTheta2Sq = pow2( outsideIOR / iridescenceIOR ) * ( 1.0 - pow2( cosTheta1 ) );
		float cosTheta2Sq = 1.0 - sinTheta2Sq;
		if ( cosTheta2Sq < 0.0 ) {
			return vec3( 1.0 );
		}
		float cosTheta2 = sqrt( cosTheta2Sq );
		float R0 = IorToFresnel0( iridescenceIOR, outsideIOR );
		float R12 = F_Schlick( R0, 1.0, cosTheta1 );
		float T121 = 1.0 - R12;
		float phi12 = 0.0;
		if ( iridescenceIOR < outsideIOR ) phi12 = PI;
		float phi21 = PI - phi12;
		vec3 baseIOR = Fresnel0ToIor( clamp( baseF0, 0.0, 0.9999 ) );		vec3 R1 = IorToFresnel0( baseIOR, iridescenceIOR );
		vec3 R23 = F_Schlick( R1, 1.0, cosTheta2 );
		vec3 phi23 = vec3( 0.0 );
		if ( baseIOR[ 0 ] < iridescenceIOR ) phi23[ 0 ] = PI;
		if ( baseIOR[ 1 ] < iridescenceIOR ) phi23[ 1 ] = PI;
		if ( baseIOR[ 2 ] < iridescenceIOR ) phi23[ 2 ] = PI;
		float OPD = 2.0 * iridescenceIOR * thinFilmThickness * cosTheta2;
		vec3 phi = vec3( phi21 ) + phi23;
		vec3 R123 = clamp( R12 * R23, 1e-5, 0.9999 );
		vec3 r123 = sqrt( R123 );
		vec3 Rs = pow2( T121 ) * R23 / ( vec3( 1.0 ) - R123 );
		vec3 C0 = R12 + Rs;
		I = C0;
		vec3 Cm = Rs - T121;
		for ( int m = 1; m <= 2; ++ m ) {
			Cm *= r123;
			vec3 Sm = 2.0 * evalSensitivity( float( m ) * OPD, float( m ) * phi );
			I += Cm * Sm;
		}
		return max( I, vec3( 0.0 ) );
	}
#endif`,np=`#ifdef USE_BUMPMAP
	uniform sampler2D bumpMap;
	uniform float bumpScale;
	vec2 dHdxy_fwd() {
		vec2 dSTdx = dFdx( vBumpMapUv );
		vec2 dSTdy = dFdy( vBumpMapUv );
		float Hll = bumpScale * texture2D( bumpMap, vBumpMapUv ).x;
		float dBx = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdx ).x - Hll;
		float dBy = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdy ).x - Hll;
		return vec2( dBx, dBy );
	}
	vec3 perturbNormalArb( vec3 surf_pos, vec3 surf_norm, vec2 dHdxy, float faceDirection ) {
		vec3 vSigmaX = normalize( dFdx( surf_pos.xyz ) );
		vec3 vSigmaY = normalize( dFdy( surf_pos.xyz ) );
		vec3 vN = surf_norm;
		vec3 R1 = cross( vSigmaY, vN );
		vec3 R2 = cross( vN, vSigmaX );
		float fDet = dot( vSigmaX, R1 ) * faceDirection;
		vec3 vGrad = sign( fDet ) * ( dHdxy.x * R1 + dHdxy.y * R2 );
		return normalize( abs( fDet ) * surf_norm - vGrad );
	}
#endif`,sp=`#if NUM_CLIPPING_PLANES > 0
	vec4 plane;
	#ifdef ALPHA_TO_COVERAGE
		float distanceToPlane, distanceGradient;
		float clipOpacity = 1.0;
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
			distanceGradient = fwidth( distanceToPlane ) / 2.0;
			clipOpacity *= smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			if ( clipOpacity == 0.0 ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			float unionClipOpacity = 1.0;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
				distanceGradient = fwidth( distanceToPlane ) / 2.0;
				unionClipOpacity *= 1.0 - smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			}
			#pragma unroll_loop_end
			clipOpacity *= 1.0 - unionClipOpacity;
		#endif
		diffuseColor.a *= clipOpacity;
		if ( diffuseColor.a == 0.0 ) discard;
	#else
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			if ( dot( vClipPosition, plane.xyz ) > plane.w ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			bool clipped = true;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				clipped = ( dot( vClipPosition, plane.xyz ) > plane.w ) && clipped;
			}
			#pragma unroll_loop_end
			if ( clipped ) discard;
		#endif
	#endif
#endif`,rp=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,ap=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,op=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,lp=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,cp=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,hp=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	varying vec3 vColor;
#endif`,up=`#if defined( USE_COLOR_ALPHA )
	vColor = vec4( 1.0 );
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	vColor = vec3( 1.0 );
#endif
#ifdef USE_COLOR
	vColor *= color;
#endif
#ifdef USE_INSTANCING_COLOR
	vColor.xyz *= instanceColor.xyz;
#endif
#ifdef USE_BATCHING_COLOR
	vec3 batchingColor = getBatchingColor( getIndirectIndex( gl_DrawID ) );
	vColor.xyz *= batchingColor.xyz;
#endif`,dp=`#define PI 3.141592653589793
#define PI2 6.283185307179586
#define PI_HALF 1.5707963267948966
#define RECIPROCAL_PI 0.3183098861837907
#define RECIPROCAL_PI2 0.15915494309189535
#define EPSILON 1e-6
#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
#define whiteComplement( a ) ( 1.0 - saturate( a ) )
float pow2( const in float x ) { return x*x; }
vec3 pow2( const in vec3 x ) { return x*x; }
float pow3( const in float x ) { return x*x*x; }
float pow4( const in float x ) { float x2 = x*x; return x2*x2; }
float max3( const in vec3 v ) { return max( max( v.x, v.y ), v.z ); }
float average( const in vec3 v ) { return dot( v, vec3( 0.3333333 ) ); }
highp float rand( const in vec2 uv ) {
	const highp float a = 12.9898, b = 78.233, c = 43758.5453;
	highp float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	return fract( sin( sn ) * c );
}
#ifdef HIGH_PRECISION
	float precisionSafeLength( vec3 v ) { return length( v ); }
#else
	float precisionSafeLength( vec3 v ) {
		float maxComponent = max3( abs( v ) );
		return length( v / maxComponent ) * maxComponent;
	}
#endif
struct IncidentLight {
	vec3 color;
	vec3 direction;
	bool visible;
};
struct ReflectedLight {
	vec3 directDiffuse;
	vec3 directSpecular;
	vec3 indirectDiffuse;
	vec3 indirectSpecular;
};
#ifdef USE_ALPHAHASH
	varying vec3 vPosition;
#endif
vec3 transformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );
}
vec3 inverseTransformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( vec4( dir, 0.0 ) * matrix ).xyz );
}
mat3 transposeMat3( const in mat3 m ) {
	mat3 tmp;
	tmp[ 0 ] = vec3( m[ 0 ].x, m[ 1 ].x, m[ 2 ].x );
	tmp[ 1 ] = vec3( m[ 0 ].y, m[ 1 ].y, m[ 2 ].y );
	tmp[ 2 ] = vec3( m[ 0 ].z, m[ 1 ].z, m[ 2 ].z );
	return tmp;
}
bool isPerspectiveMatrix( mat4 m ) {
	return m[ 2 ][ 3 ] == - 1.0;
}
vec2 equirectUv( in vec3 dir ) {
	float u = atan( dir.z, dir.x ) * RECIPROCAL_PI2 + 0.5;
	float v = asin( clamp( dir.y, - 1.0, 1.0 ) ) * RECIPROCAL_PI + 0.5;
	return vec2( u, v );
}
vec3 BRDF_Lambert( const in vec3 diffuseColor ) {
	return RECIPROCAL_PI * diffuseColor;
}
vec3 F_Schlick( const in vec3 f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
}
float F_Schlick( const in float f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
} // validated`,fp=`#ifdef ENVMAP_TYPE_CUBE_UV
	#define cubeUV_minMipLevel 4.0
	#define cubeUV_minTileSize 16.0
	float getFace( vec3 direction ) {
		vec3 absDirection = abs( direction );
		float face = - 1.0;
		if ( absDirection.x > absDirection.z ) {
			if ( absDirection.x > absDirection.y )
				face = direction.x > 0.0 ? 0.0 : 3.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		} else {
			if ( absDirection.z > absDirection.y )
				face = direction.z > 0.0 ? 2.0 : 5.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		}
		return face;
	}
	vec2 getUV( vec3 direction, float face ) {
		vec2 uv;
		if ( face == 0.0 ) {
			uv = vec2( direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 1.0 ) {
			uv = vec2( - direction.x, - direction.z ) / abs( direction.y );
		} else if ( face == 2.0 ) {
			uv = vec2( - direction.x, direction.y ) / abs( direction.z );
		} else if ( face == 3.0 ) {
			uv = vec2( - direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 4.0 ) {
			uv = vec2( - direction.x, direction.z ) / abs( direction.y );
		} else {
			uv = vec2( direction.x, direction.y ) / abs( direction.z );
		}
		return 0.5 * ( uv + 1.0 );
	}
	vec3 bilinearCubeUV( sampler2D envMap, vec3 direction, float mipInt ) {
		float face = getFace( direction );
		float filterInt = max( cubeUV_minMipLevel - mipInt, 0.0 );
		mipInt = max( mipInt, cubeUV_minMipLevel );
		float faceSize = exp2( mipInt );
		highp vec2 uv = getUV( direction, face ) * ( faceSize - 2.0 ) + 1.0;
		if ( face > 2.0 ) {
			uv.y += faceSize;
			face -= 3.0;
		}
		uv.x += face * faceSize;
		uv.x += filterInt * 3.0 * cubeUV_minTileSize;
		uv.y += 4.0 * ( exp2( CUBEUV_MAX_MIP ) - faceSize );
		uv.x *= CUBEUV_TEXEL_WIDTH;
		uv.y *= CUBEUV_TEXEL_HEIGHT;
		#ifdef texture2DGradEXT
			return texture2DGradEXT( envMap, uv, vec2( 0.0 ), vec2( 0.0 ) ).rgb;
		#else
			return texture2D( envMap, uv ).rgb;
		#endif
	}
	#define cubeUV_r0 1.0
	#define cubeUV_m0 - 2.0
	#define cubeUV_r1 0.8
	#define cubeUV_m1 - 1.0
	#define cubeUV_r4 0.4
	#define cubeUV_m4 2.0
	#define cubeUV_r5 0.305
	#define cubeUV_m5 3.0
	#define cubeUV_r6 0.21
	#define cubeUV_m6 4.0
	float roughnessToMip( float roughness ) {
		float mip = 0.0;
		if ( roughness >= cubeUV_r1 ) {
			mip = ( cubeUV_r0 - roughness ) * ( cubeUV_m1 - cubeUV_m0 ) / ( cubeUV_r0 - cubeUV_r1 ) + cubeUV_m0;
		} else if ( roughness >= cubeUV_r4 ) {
			mip = ( cubeUV_r1 - roughness ) * ( cubeUV_m4 - cubeUV_m1 ) / ( cubeUV_r1 - cubeUV_r4 ) + cubeUV_m1;
		} else if ( roughness >= cubeUV_r5 ) {
			mip = ( cubeUV_r4 - roughness ) * ( cubeUV_m5 - cubeUV_m4 ) / ( cubeUV_r4 - cubeUV_r5 ) + cubeUV_m4;
		} else if ( roughness >= cubeUV_r6 ) {
			mip = ( cubeUV_r5 - roughness ) * ( cubeUV_m6 - cubeUV_m5 ) / ( cubeUV_r5 - cubeUV_r6 ) + cubeUV_m5;
		} else {
			mip = - 2.0 * log2( 1.16 * roughness );		}
		return mip;
	}
	vec4 textureCubeUV( sampler2D envMap, vec3 sampleDir, float roughness ) {
		float mip = clamp( roughnessToMip( roughness ), cubeUV_m0, CUBEUV_MAX_MIP );
		float mipF = fract( mip );
		float mipInt = floor( mip );
		vec3 color0 = bilinearCubeUV( envMap, sampleDir, mipInt );
		if ( mipF == 0.0 ) {
			return vec4( color0, 1.0 );
		} else {
			vec3 color1 = bilinearCubeUV( envMap, sampleDir, mipInt + 1.0 );
			return vec4( mix( color0, color1, mipF ), 1.0 );
		}
	}
#endif`,pp=`vec3 transformedNormal = objectNormal;
#ifdef USE_TANGENT
	vec3 transformedTangent = objectTangent;
#endif
#ifdef USE_BATCHING
	mat3 bm = mat3( batchingMatrix );
	transformedNormal /= vec3( dot( bm[ 0 ], bm[ 0 ] ), dot( bm[ 1 ], bm[ 1 ] ), dot( bm[ 2 ], bm[ 2 ] ) );
	transformedNormal = bm * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = bm * transformedTangent;
	#endif
#endif
#ifdef USE_INSTANCING
	mat3 im = mat3( instanceMatrix );
	transformedNormal /= vec3( dot( im[ 0 ], im[ 0 ] ), dot( im[ 1 ], im[ 1 ] ), dot( im[ 2 ], im[ 2 ] ) );
	transformedNormal = im * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = im * transformedTangent;
	#endif
#endif
transformedNormal = normalMatrix * transformedNormal;
#ifdef FLIP_SIDED
	transformedNormal = - transformedNormal;
#endif
#ifdef USE_TANGENT
	transformedTangent = ( modelViewMatrix * vec4( transformedTangent, 0.0 ) ).xyz;
	#ifdef FLIP_SIDED
		transformedTangent = - transformedTangent;
	#endif
#endif`,mp=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,gp=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,vp=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	#ifdef DECODE_VIDEO_TEXTURE_EMISSIVE
		emissiveColor = sRGBTransferEOTF( emissiveColor );
	#endif
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,_p=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,xp="gl_FragColor = linearToOutputTexel( gl_FragColor );",yp=`vec4 LinearTransferOETF( in vec4 value ) {
	return value;
}
vec4 sRGBTransferEOTF( in vec4 value ) {
	return vec4( mix( pow( value.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), value.rgb * 0.0773993808, vec3( lessThanEqual( value.rgb, vec3( 0.04045 ) ) ) ), value.a );
}
vec4 sRGBTransferOETF( in vec4 value ) {
	return vec4( mix( pow( value.rgb, vec3( 0.41666 ) ) * 1.055 - vec3( 0.055 ), value.rgb * 12.92, vec3( lessThanEqual( value.rgb, vec3( 0.0031308 ) ) ) ), value.a );
}`,Mp=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vec3 cameraToFrag;
		if ( isOrthographic ) {
			cameraToFrag = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToFrag = normalize( vWorldPosition - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vec3 reflectVec = reflect( cameraToFrag, worldNormal );
		#else
			vec3 reflectVec = refract( cameraToFrag, worldNormal, refractionRatio );
		#endif
	#else
		vec3 reflectVec = vReflect;
	#endif
	#ifdef ENVMAP_TYPE_CUBE
		vec4 envColor = textureCube( envMap, envMapRotation * vec3( flipEnvMap * reflectVec.x, reflectVec.yz ) );
	#else
		vec4 envColor = vec4( 0.0 );
	#endif
	#ifdef ENVMAP_BLENDING_MULTIPLY
		outgoingLight = mix( outgoingLight, outgoingLight * envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_MIX )
		outgoingLight = mix( outgoingLight, envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_ADD )
		outgoingLight += envColor.xyz * specularStrength * reflectivity;
	#endif
#endif`,Sp=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	uniform mat3 envMapRotation;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,bp=`#ifdef USE_ENVMAP
	uniform float reflectivity;
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		varying vec3 vWorldPosition;
		uniform float refractionRatio;
	#else
		varying vec3 vReflect;
	#endif
#endif`,Ep=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,wp=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vWorldPosition = worldPosition.xyz;
	#else
		vec3 cameraToVertex;
		if ( isOrthographic ) {
			cameraToVertex = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToVertex = normalize( worldPosition.xyz - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vReflect = reflect( cameraToVertex, worldNormal );
		#else
			vReflect = refract( cameraToVertex, worldNormal, refractionRatio );
		#endif
	#endif
#endif`,Tp=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,Ap=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,Rp=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,Cp=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,Pp=`#ifdef USE_GRADIENTMAP
	uniform sampler2D gradientMap;
#endif
vec3 getGradientIrradiance( vec3 normal, vec3 lightDirection ) {
	float dotNL = dot( normal, lightDirection );
	vec2 coord = vec2( dotNL * 0.5 + 0.5, 0.0 );
	#ifdef USE_GRADIENTMAP
		return vec3( texture2D( gradientMap, coord ).r );
	#else
		vec2 fw = fwidth( coord ) * 0.5;
		return mix( vec3( 0.7 ), vec3( 1.0 ), smoothstep( 0.7 - fw.x, 0.7 + fw.x, coord.x ) );
	#endif
}`,Ip=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,Dp=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,Lp=`varying vec3 vViewPosition;
struct LambertMaterial {
	vec3 diffuseColor;
	float specularStrength;
};
void RE_Direct_Lambert( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Lambert( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Lambert
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,Np=`uniform bool receiveShadow;
uniform vec3 ambientLightColor;
#if defined( USE_LIGHT_PROBES )
	uniform vec3 lightProbe[ 9 ];
#endif
vec3 shGetIrradianceAt( in vec3 normal, in vec3 shCoefficients[ 9 ] ) {
	float x = normal.x, y = normal.y, z = normal.z;
	vec3 result = shCoefficients[ 0 ] * 0.886227;
	result += shCoefficients[ 1 ] * 2.0 * 0.511664 * y;
	result += shCoefficients[ 2 ] * 2.0 * 0.511664 * z;
	result += shCoefficients[ 3 ] * 2.0 * 0.511664 * x;
	result += shCoefficients[ 4 ] * 2.0 * 0.429043 * x * y;
	result += shCoefficients[ 5 ] * 2.0 * 0.429043 * y * z;
	result += shCoefficients[ 6 ] * ( 0.743125 * z * z - 0.247708 );
	result += shCoefficients[ 7 ] * 2.0 * 0.429043 * x * z;
	result += shCoefficients[ 8 ] * 0.429043 * ( x * x - y * y );
	return result;
}
vec3 getLightProbeIrradiance( const in vec3 lightProbe[ 9 ], const in vec3 normal ) {
	vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
	vec3 irradiance = shGetIrradianceAt( worldNormal, lightProbe );
	return irradiance;
}
vec3 getAmbientLightIrradiance( const in vec3 ambientLightColor ) {
	vec3 irradiance = ambientLightColor;
	return irradiance;
}
float getDistanceAttenuation( const in float lightDistance, const in float cutoffDistance, const in float decayExponent ) {
	float distanceFalloff = 1.0 / max( pow( lightDistance, decayExponent ), 0.01 );
	if ( cutoffDistance > 0.0 ) {
		distanceFalloff *= pow2( saturate( 1.0 - pow4( lightDistance / cutoffDistance ) ) );
	}
	return distanceFalloff;
}
float getSpotAttenuation( const in float coneCosine, const in float penumbraCosine, const in float angleCosine ) {
	return smoothstep( coneCosine, penumbraCosine, angleCosine );
}
#if NUM_DIR_LIGHTS > 0
	struct DirectionalLight {
		vec3 direction;
		vec3 color;
	};
	uniform DirectionalLight directionalLights[ NUM_DIR_LIGHTS ];
	void getDirectionalLightInfo( const in DirectionalLight directionalLight, out IncidentLight light ) {
		light.color = directionalLight.color;
		light.direction = directionalLight.direction;
		light.visible = true;
	}
#endif
#if NUM_POINT_LIGHTS > 0
	struct PointLight {
		vec3 position;
		vec3 color;
		float distance;
		float decay;
	};
	uniform PointLight pointLights[ NUM_POINT_LIGHTS ];
	void getPointLightInfo( const in PointLight pointLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = pointLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float lightDistance = length( lVector );
		light.color = pointLight.color;
		light.color *= getDistanceAttenuation( lightDistance, pointLight.distance, pointLight.decay );
		light.visible = ( light.color != vec3( 0.0 ) );
	}
#endif
#if NUM_SPOT_LIGHTS > 0
	struct SpotLight {
		vec3 position;
		vec3 direction;
		vec3 color;
		float distance;
		float decay;
		float coneCos;
		float penumbraCos;
	};
	uniform SpotLight spotLights[ NUM_SPOT_LIGHTS ];
	void getSpotLightInfo( const in SpotLight spotLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = spotLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float angleCos = dot( light.direction, spotLight.direction );
		float spotAttenuation = getSpotAttenuation( spotLight.coneCos, spotLight.penumbraCos, angleCos );
		if ( spotAttenuation > 0.0 ) {
			float lightDistance = length( lVector );
			light.color = spotLight.color * spotAttenuation;
			light.color *= getDistanceAttenuation( lightDistance, spotLight.distance, spotLight.decay );
			light.visible = ( light.color != vec3( 0.0 ) );
		} else {
			light.color = vec3( 0.0 );
			light.visible = false;
		}
	}
#endif
#if NUM_RECT_AREA_LIGHTS > 0
	struct RectAreaLight {
		vec3 color;
		vec3 position;
		vec3 halfWidth;
		vec3 halfHeight;
	};
	uniform sampler2D ltc_1;	uniform sampler2D ltc_2;
	uniform RectAreaLight rectAreaLights[ NUM_RECT_AREA_LIGHTS ];
#endif
#if NUM_HEMI_LIGHTS > 0
	struct HemisphereLight {
		vec3 direction;
		vec3 skyColor;
		vec3 groundColor;
	};
	uniform HemisphereLight hemisphereLights[ NUM_HEMI_LIGHTS ];
	vec3 getHemisphereLightIrradiance( const in HemisphereLight hemiLight, const in vec3 normal ) {
		float dotNL = dot( normal, hemiLight.direction );
		float hemiDiffuseWeight = 0.5 * dotNL + 0.5;
		vec3 irradiance = mix( hemiLight.groundColor, hemiLight.skyColor, hemiDiffuseWeight );
		return irradiance;
	}
#endif`,Up=`#ifdef USE_ENVMAP
	vec3 getIBLIrradiance( const in vec3 normal ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * worldNormal, 1.0 );
			return PI * envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	vec3 getIBLRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 reflectVec = reflect( - viewDir, normal );
			reflectVec = normalize( mix( reflectVec, normal, roughness * roughness) );
			reflectVec = inverseTransformDirection( reflectVec, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * reflectVec, roughness );
			return envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	#ifdef USE_ANISOTROPY
		vec3 getIBLAnisotropyRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness, const in vec3 bitangent, const in float anisotropy ) {
			#ifdef ENVMAP_TYPE_CUBE_UV
				vec3 bentNormal = cross( bitangent, viewDir );
				bentNormal = normalize( cross( bentNormal, bitangent ) );
				bentNormal = normalize( mix( bentNormal, normal, pow2( pow2( 1.0 - anisotropy * ( 1.0 - roughness ) ) ) ) );
				return getIBLRadiance( viewDir, bentNormal, roughness );
			#else
				return vec3( 0.0 );
			#endif
		}
	#endif
#endif`,Op=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,Fp=`varying vec3 vViewPosition;
struct ToonMaterial {
	vec3 diffuseColor;
};
void RE_Direct_Toon( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	vec3 irradiance = getGradientIrradiance( geometryNormal, directLight.direction ) * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Toon( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Toon
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,kp=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,zp=`varying vec3 vViewPosition;
struct BlinnPhongMaterial {
	vec3 diffuseColor;
	vec3 specularColor;
	float specularShininess;
	float specularStrength;
};
void RE_Direct_BlinnPhong( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
	reflectedLight.directSpecular += irradiance * BRDF_BlinnPhong( directLight.direction, geometryViewDir, geometryNormal, material.specularColor, material.specularShininess ) * material.specularStrength;
}
void RE_IndirectDiffuse_BlinnPhong( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_BlinnPhong
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,Bp=`PhysicalMaterial material;
material.diffuseColor = diffuseColor.rgb * ( 1.0 - metalnessFactor );
vec3 dxy = max( abs( dFdx( nonPerturbedNormal ) ), abs( dFdy( nonPerturbedNormal ) ) );
float geometryRoughness = max( max( dxy.x, dxy.y ), dxy.z );
material.roughness = max( roughnessFactor, 0.0525 );material.roughness += geometryRoughness;
material.roughness = min( material.roughness, 1.0 );
#ifdef IOR
	material.ior = ior;
	#ifdef USE_SPECULAR
		float specularIntensityFactor = specularIntensity;
		vec3 specularColorFactor = specularColor;
		#ifdef USE_SPECULAR_COLORMAP
			specularColorFactor *= texture2D( specularColorMap, vSpecularColorMapUv ).rgb;
		#endif
		#ifdef USE_SPECULAR_INTENSITYMAP
			specularIntensityFactor *= texture2D( specularIntensityMap, vSpecularIntensityMapUv ).a;
		#endif
		material.specularF90 = mix( specularIntensityFactor, 1.0, metalnessFactor );
	#else
		float specularIntensityFactor = 1.0;
		vec3 specularColorFactor = vec3( 1.0 );
		material.specularF90 = 1.0;
	#endif
	material.specularColor = mix( min( pow2( ( material.ior - 1.0 ) / ( material.ior + 1.0 ) ) * specularColorFactor, vec3( 1.0 ) ) * specularIntensityFactor, diffuseColor.rgb, metalnessFactor );
#else
	material.specularColor = mix( vec3( 0.04 ), diffuseColor.rgb, metalnessFactor );
	material.specularF90 = 1.0;
#endif
#ifdef USE_CLEARCOAT
	material.clearcoat = clearcoat;
	material.clearcoatRoughness = clearcoatRoughness;
	material.clearcoatF0 = vec3( 0.04 );
	material.clearcoatF90 = 1.0;
	#ifdef USE_CLEARCOATMAP
		material.clearcoat *= texture2D( clearcoatMap, vClearcoatMapUv ).x;
	#endif
	#ifdef USE_CLEARCOAT_ROUGHNESSMAP
		material.clearcoatRoughness *= texture2D( clearcoatRoughnessMap, vClearcoatRoughnessMapUv ).y;
	#endif
	material.clearcoat = saturate( material.clearcoat );	material.clearcoatRoughness = max( material.clearcoatRoughness, 0.0525 );
	material.clearcoatRoughness += geometryRoughness;
	material.clearcoatRoughness = min( material.clearcoatRoughness, 1.0 );
#endif
#ifdef USE_DISPERSION
	material.dispersion = dispersion;
#endif
#ifdef USE_IRIDESCENCE
	material.iridescence = iridescence;
	material.iridescenceIOR = iridescenceIOR;
	#ifdef USE_IRIDESCENCEMAP
		material.iridescence *= texture2D( iridescenceMap, vIridescenceMapUv ).r;
	#endif
	#ifdef USE_IRIDESCENCE_THICKNESSMAP
		material.iridescenceThickness = (iridescenceThicknessMaximum - iridescenceThicknessMinimum) * texture2D( iridescenceThicknessMap, vIridescenceThicknessMapUv ).g + iridescenceThicknessMinimum;
	#else
		material.iridescenceThickness = iridescenceThicknessMaximum;
	#endif
#endif
#ifdef USE_SHEEN
	material.sheenColor = sheenColor;
	#ifdef USE_SHEEN_COLORMAP
		material.sheenColor *= texture2D( sheenColorMap, vSheenColorMapUv ).rgb;
	#endif
	material.sheenRoughness = clamp( sheenRoughness, 0.07, 1.0 );
	#ifdef USE_SHEEN_ROUGHNESSMAP
		material.sheenRoughness *= texture2D( sheenRoughnessMap, vSheenRoughnessMapUv ).a;
	#endif
#endif
#ifdef USE_ANISOTROPY
	#ifdef USE_ANISOTROPYMAP
		mat2 anisotropyMat = mat2( anisotropyVector.x, anisotropyVector.y, - anisotropyVector.y, anisotropyVector.x );
		vec3 anisotropyPolar = texture2D( anisotropyMap, vAnisotropyMapUv ).rgb;
		vec2 anisotropyV = anisotropyMat * normalize( 2.0 * anisotropyPolar.rg - vec2( 1.0 ) ) * anisotropyPolar.b;
	#else
		vec2 anisotropyV = anisotropyVector;
	#endif
	material.anisotropy = length( anisotropyV );
	if( material.anisotropy == 0.0 ) {
		anisotropyV = vec2( 1.0, 0.0 );
	} else {
		anisotropyV /= material.anisotropy;
		material.anisotropy = saturate( material.anisotropy );
	}
	material.alphaT = mix( pow2( material.roughness ), 1.0, pow2( material.anisotropy ) );
	material.anisotropyT = tbn[ 0 ] * anisotropyV.x + tbn[ 1 ] * anisotropyV.y;
	material.anisotropyB = tbn[ 1 ] * anisotropyV.x - tbn[ 0 ] * anisotropyV.y;
#endif`,Hp=`struct PhysicalMaterial {
	vec3 diffuseColor;
	float roughness;
	vec3 specularColor;
	float specularF90;
	float dispersion;
	#ifdef USE_CLEARCOAT
		float clearcoat;
		float clearcoatRoughness;
		vec3 clearcoatF0;
		float clearcoatF90;
	#endif
	#ifdef USE_IRIDESCENCE
		float iridescence;
		float iridescenceIOR;
		float iridescenceThickness;
		vec3 iridescenceFresnel;
		vec3 iridescenceF0;
	#endif
	#ifdef USE_SHEEN
		vec3 sheenColor;
		float sheenRoughness;
	#endif
	#ifdef IOR
		float ior;
	#endif
	#ifdef USE_TRANSMISSION
		float transmission;
		float transmissionAlpha;
		float thickness;
		float attenuationDistance;
		vec3 attenuationColor;
	#endif
	#ifdef USE_ANISOTROPY
		float anisotropy;
		float alphaT;
		vec3 anisotropyT;
		vec3 anisotropyB;
	#endif
};
vec3 clearcoatSpecularDirect = vec3( 0.0 );
vec3 clearcoatSpecularIndirect = vec3( 0.0 );
vec3 sheenSpecularDirect = vec3( 0.0 );
vec3 sheenSpecularIndirect = vec3(0.0 );
vec3 Schlick_to_F0( const in vec3 f, const in float f90, const in float dotVH ) {
    float x = clamp( 1.0 - dotVH, 0.0, 1.0 );
    float x2 = x * x;
    float x5 = clamp( x * x2 * x2, 0.0, 0.9999 );
    return ( f - vec3( f90 ) * x5 ) / ( 1.0 - x5 );
}
float V_GGX_SmithCorrelated( const in float alpha, const in float dotNL, const in float dotNV ) {
	float a2 = pow2( alpha );
	float gv = dotNL * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNV ) );
	float gl = dotNV * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNL ) );
	return 0.5 / max( gv + gl, EPSILON );
}
float D_GGX( const in float alpha, const in float dotNH ) {
	float a2 = pow2( alpha );
	float denom = pow2( dotNH ) * ( a2 - 1.0 ) + 1.0;
	return RECIPROCAL_PI * a2 / pow2( denom );
}
#ifdef USE_ANISOTROPY
	float V_GGX_SmithCorrelated_Anisotropic( const in float alphaT, const in float alphaB, const in float dotTV, const in float dotBV, const in float dotTL, const in float dotBL, const in float dotNV, const in float dotNL ) {
		float gv = dotNL * length( vec3( alphaT * dotTV, alphaB * dotBV, dotNV ) );
		float gl = dotNV * length( vec3( alphaT * dotTL, alphaB * dotBL, dotNL ) );
		float v = 0.5 / ( gv + gl );
		return saturate(v);
	}
	float D_GGX_Anisotropic( const in float alphaT, const in float alphaB, const in float dotNH, const in float dotTH, const in float dotBH ) {
		float a2 = alphaT * alphaB;
		highp vec3 v = vec3( alphaB * dotTH, alphaT * dotBH, a2 * dotNH );
		highp float v2 = dot( v, v );
		float w2 = a2 / v2;
		return RECIPROCAL_PI * a2 * pow2 ( w2 );
	}
#endif
#ifdef USE_CLEARCOAT
	vec3 BRDF_GGX_Clearcoat( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material) {
		vec3 f0 = material.clearcoatF0;
		float f90 = material.clearcoatF90;
		float roughness = material.clearcoatRoughness;
		float alpha = pow2( roughness );
		vec3 halfDir = normalize( lightDir + viewDir );
		float dotNL = saturate( dot( normal, lightDir ) );
		float dotNV = saturate( dot( normal, viewDir ) );
		float dotNH = saturate( dot( normal, halfDir ) );
		float dotVH = saturate( dot( viewDir, halfDir ) );
		vec3 F = F_Schlick( f0, f90, dotVH );
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
		return F * ( V * D );
	}
#endif
vec3 BRDF_GGX( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material ) {
	vec3 f0 = material.specularColor;
	float f90 = material.specularF90;
	float roughness = material.roughness;
	float alpha = pow2( roughness );
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( f0, f90, dotVH );
	#ifdef USE_IRIDESCENCE
		F = mix( F, material.iridescenceFresnel, material.iridescence );
	#endif
	#ifdef USE_ANISOTROPY
		float dotTL = dot( material.anisotropyT, lightDir );
		float dotTV = dot( material.anisotropyT, viewDir );
		float dotTH = dot( material.anisotropyT, halfDir );
		float dotBL = dot( material.anisotropyB, lightDir );
		float dotBV = dot( material.anisotropyB, viewDir );
		float dotBH = dot( material.anisotropyB, halfDir );
		float V = V_GGX_SmithCorrelated_Anisotropic( material.alphaT, alpha, dotTV, dotBV, dotTL, dotBL, dotNV, dotNL );
		float D = D_GGX_Anisotropic( material.alphaT, alpha, dotNH, dotTH, dotBH );
	#else
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
	#endif
	return F * ( V * D );
}
vec2 LTC_Uv( const in vec3 N, const in vec3 V, const in float roughness ) {
	const float LUT_SIZE = 64.0;
	const float LUT_SCALE = ( LUT_SIZE - 1.0 ) / LUT_SIZE;
	const float LUT_BIAS = 0.5 / LUT_SIZE;
	float dotNV = saturate( dot( N, V ) );
	vec2 uv = vec2( roughness, sqrt( 1.0 - dotNV ) );
	uv = uv * LUT_SCALE + LUT_BIAS;
	return uv;
}
float LTC_ClippedSphereFormFactor( const in vec3 f ) {
	float l = length( f );
	return max( ( l * l + f.z ) / ( l + 1.0 ), 0.0 );
}
vec3 LTC_EdgeVectorFormFactor( const in vec3 v1, const in vec3 v2 ) {
	float x = dot( v1, v2 );
	float y = abs( x );
	float a = 0.8543985 + ( 0.4965155 + 0.0145206 * y ) * y;
	float b = 3.4175940 + ( 4.1616724 + y ) * y;
	float v = a / b;
	float theta_sintheta = ( x > 0.0 ) ? v : 0.5 * inversesqrt( max( 1.0 - x * x, 1e-7 ) ) - v;
	return cross( v1, v2 ) * theta_sintheta;
}
vec3 LTC_Evaluate( const in vec3 N, const in vec3 V, const in vec3 P, const in mat3 mInv, const in vec3 rectCoords[ 4 ] ) {
	vec3 v1 = rectCoords[ 1 ] - rectCoords[ 0 ];
	vec3 v2 = rectCoords[ 3 ] - rectCoords[ 0 ];
	vec3 lightNormal = cross( v1, v2 );
	if( dot( lightNormal, P - rectCoords[ 0 ] ) < 0.0 ) return vec3( 0.0 );
	vec3 T1, T2;
	T1 = normalize( V - N * dot( V, N ) );
	T2 = - cross( N, T1 );
	mat3 mat = mInv * transposeMat3( mat3( T1, T2, N ) );
	vec3 coords[ 4 ];
	coords[ 0 ] = mat * ( rectCoords[ 0 ] - P );
	coords[ 1 ] = mat * ( rectCoords[ 1 ] - P );
	coords[ 2 ] = mat * ( rectCoords[ 2 ] - P );
	coords[ 3 ] = mat * ( rectCoords[ 3 ] - P );
	coords[ 0 ] = normalize( coords[ 0 ] );
	coords[ 1 ] = normalize( coords[ 1 ] );
	coords[ 2 ] = normalize( coords[ 2 ] );
	coords[ 3 ] = normalize( coords[ 3 ] );
	vec3 vectorFormFactor = vec3( 0.0 );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 0 ], coords[ 1 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 1 ], coords[ 2 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 2 ], coords[ 3 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 3 ], coords[ 0 ] );
	float result = LTC_ClippedSphereFormFactor( vectorFormFactor );
	return vec3( result );
}
#if defined( USE_SHEEN )
float D_Charlie( float roughness, float dotNH ) {
	float alpha = pow2( roughness );
	float invAlpha = 1.0 / alpha;
	float cos2h = dotNH * dotNH;
	float sin2h = max( 1.0 - cos2h, 0.0078125 );
	return ( 2.0 + invAlpha ) * pow( sin2h, invAlpha * 0.5 ) / ( 2.0 * PI );
}
float V_Neubelt( float dotNV, float dotNL ) {
	return saturate( 1.0 / ( 4.0 * ( dotNL + dotNV - dotNL * dotNV ) ) );
}
vec3 BRDF_Sheen( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, vec3 sheenColor, const in float sheenRoughness ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float D = D_Charlie( sheenRoughness, dotNH );
	float V = V_Neubelt( dotNV, dotNL );
	return sheenColor * ( D * V );
}
#endif
float IBLSheenBRDF( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	float r2 = roughness * roughness;
	float a = roughness < 0.25 ? -339.2 * r2 + 161.4 * roughness - 25.9 : -8.48 * r2 + 14.3 * roughness - 9.95;
	float b = roughness < 0.25 ? 44.0 * r2 - 23.7 * roughness + 3.26 : 1.97 * r2 - 3.27 * roughness + 0.72;
	float DG = exp( a * dotNV + b ) + ( roughness < 0.25 ? 0.0 : 0.1 * ( roughness - 0.25 ) );
	return saturate( DG * RECIPROCAL_PI );
}
vec2 DFGApprox( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	const vec4 c0 = vec4( - 1, - 0.0275, - 0.572, 0.022 );
	const vec4 c1 = vec4( 1, 0.0425, 1.04, - 0.04 );
	vec4 r = roughness * c0 + c1;
	float a004 = min( r.x * r.x, exp2( - 9.28 * dotNV ) ) * r.x + r.y;
	vec2 fab = vec2( - 1.04, 1.04 ) * a004 + r.zw;
	return fab;
}
vec3 EnvironmentBRDF( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness ) {
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	return specularColor * fab.x + specularF90 * fab.y;
}
#ifdef USE_IRIDESCENCE
void computeMultiscatteringIridescence( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float iridescence, const in vec3 iridescenceF0, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#else
void computeMultiscattering( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#endif
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	#ifdef USE_IRIDESCENCE
		vec3 Fr = mix( specularColor, iridescenceF0, iridescence );
	#else
		vec3 Fr = specularColor;
	#endif
	vec3 FssEss = Fr * fab.x + specularF90 * fab.y;
	float Ess = fab.x + fab.y;
	float Ems = 1.0 - Ess;
	vec3 Favg = Fr + ( 1.0 - Fr ) * 0.047619;	vec3 Fms = FssEss * Favg / ( 1.0 - Ems * Favg );
	singleScatter += FssEss;
	multiScatter += Fms * Ems;
}
#if NUM_RECT_AREA_LIGHTS > 0
	void RE_Direct_RectArea_Physical( const in RectAreaLight rectAreaLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
		vec3 normal = geometryNormal;
		vec3 viewDir = geometryViewDir;
		vec3 position = geometryPosition;
		vec3 lightPos = rectAreaLight.position;
		vec3 halfWidth = rectAreaLight.halfWidth;
		vec3 halfHeight = rectAreaLight.halfHeight;
		vec3 lightColor = rectAreaLight.color;
		float roughness = material.roughness;
		vec3 rectCoords[ 4 ];
		rectCoords[ 0 ] = lightPos + halfWidth - halfHeight;		rectCoords[ 1 ] = lightPos - halfWidth - halfHeight;
		rectCoords[ 2 ] = lightPos - halfWidth + halfHeight;
		rectCoords[ 3 ] = lightPos + halfWidth + halfHeight;
		vec2 uv = LTC_Uv( normal, viewDir, roughness );
		vec4 t1 = texture2D( ltc_1, uv );
		vec4 t2 = texture2D( ltc_2, uv );
		mat3 mInv = mat3(
			vec3( t1.x, 0, t1.y ),
			vec3(    0, 1,    0 ),
			vec3( t1.z, 0, t1.w )
		);
		vec3 fresnel = ( material.specularColor * t2.x + ( vec3( 1.0 ) - material.specularColor ) * t2.y );
		reflectedLight.directSpecular += lightColor * fresnel * LTC_Evaluate( normal, viewDir, position, mInv, rectCoords );
		reflectedLight.directDiffuse += lightColor * material.diffuseColor * LTC_Evaluate( normal, viewDir, position, mat3( 1.0 ), rectCoords );
	}
#endif
void RE_Direct_Physical( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	#ifdef USE_CLEARCOAT
		float dotNLcc = saturate( dot( geometryClearcoatNormal, directLight.direction ) );
		vec3 ccIrradiance = dotNLcc * directLight.color;
		clearcoatSpecularDirect += ccIrradiance * BRDF_GGX_Clearcoat( directLight.direction, geometryViewDir, geometryClearcoatNormal, material );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularDirect += irradiance * BRDF_Sheen( directLight.direction, geometryViewDir, geometryNormal, material.sheenColor, material.sheenRoughness );
	#endif
	reflectedLight.directSpecular += irradiance * BRDF_GGX( directLight.direction, geometryViewDir, geometryNormal, material );
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Physical( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectSpecular_Physical( const in vec3 radiance, const in vec3 irradiance, const in vec3 clearcoatRadiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight) {
	#ifdef USE_CLEARCOAT
		clearcoatSpecularIndirect += clearcoatRadiance * EnvironmentBRDF( geometryClearcoatNormal, geometryViewDir, material.clearcoatF0, material.clearcoatF90, material.clearcoatRoughness );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularIndirect += irradiance * material.sheenColor * IBLSheenBRDF( geometryNormal, geometryViewDir, material.sheenRoughness );
	#endif
	vec3 singleScattering = vec3( 0.0 );
	vec3 multiScattering = vec3( 0.0 );
	vec3 cosineWeightedIrradiance = irradiance * RECIPROCAL_PI;
	#ifdef USE_IRIDESCENCE
		computeMultiscatteringIridescence( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.iridescence, material.iridescenceFresnel, material.roughness, singleScattering, multiScattering );
	#else
		computeMultiscattering( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.roughness, singleScattering, multiScattering );
	#endif
	vec3 totalScattering = singleScattering + multiScattering;
	vec3 diffuse = material.diffuseColor * ( 1.0 - max( max( totalScattering.r, totalScattering.g ), totalScattering.b ) );
	reflectedLight.indirectSpecular += radiance * singleScattering;
	reflectedLight.indirectSpecular += multiScattering * cosineWeightedIrradiance;
	reflectedLight.indirectDiffuse += diffuse * cosineWeightedIrradiance;
}
#define RE_Direct				RE_Direct_Physical
#define RE_Direct_RectArea		RE_Direct_RectArea_Physical
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Physical
#define RE_IndirectSpecular		RE_IndirectSpecular_Physical
float computeSpecularOcclusion( const in float dotNV, const in float ambientOcclusion, const in float roughness ) {
	return saturate( pow( dotNV + ambientOcclusion, exp2( - 16.0 * roughness - 1.0 ) ) - 1.0 + ambientOcclusion );
}`,Gp=`
vec3 geometryPosition = - vViewPosition;
vec3 geometryNormal = normal;
vec3 geometryViewDir = ( isOrthographic ) ? vec3( 0, 0, 1 ) : normalize( vViewPosition );
vec3 geometryClearcoatNormal = vec3( 0.0 );
#ifdef USE_CLEARCOAT
	geometryClearcoatNormal = clearcoatNormal;
#endif
#ifdef USE_IRIDESCENCE
	float dotNVi = saturate( dot( normal, geometryViewDir ) );
	if ( material.iridescenceThickness == 0.0 ) {
		material.iridescence = 0.0;
	} else {
		material.iridescence = saturate( material.iridescence );
	}
	if ( material.iridescence > 0.0 ) {
		material.iridescenceFresnel = evalIridescence( 1.0, material.iridescenceIOR, dotNVi, material.iridescenceThickness, material.specularColor );
		material.iridescenceF0 = Schlick_to_F0( material.iridescenceFresnel, 1.0, dotNVi );
	}
#endif
IncidentLight directLight;
#if ( NUM_POINT_LIGHTS > 0 ) && defined( RE_Direct )
	PointLight pointLight;
	#if defined( USE_SHADOWMAP ) && NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHTS; i ++ ) {
		pointLight = pointLights[ i ];
		getPointLightInfo( pointLight, geometryPosition, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_POINT_LIGHT_SHADOWS )
		pointLightShadow = pointLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getPointShadow( pointShadowMap[ i ], pointLightShadow.shadowMapSize, pointLightShadow.shadowIntensity, pointLightShadow.shadowBias, pointLightShadow.shadowRadius, vPointShadowCoord[ i ], pointLightShadow.shadowCameraNear, pointLightShadow.shadowCameraFar ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_SPOT_LIGHTS > 0 ) && defined( RE_Direct )
	SpotLight spotLight;
	vec4 spotColor;
	vec3 spotLightCoord;
	bool inSpotLightMap;
	#if defined( USE_SHADOWMAP ) && NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHTS; i ++ ) {
		spotLight = spotLights[ i ];
		getSpotLightInfo( spotLight, geometryPosition, directLight );
		#if ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#define SPOT_LIGHT_MAP_INDEX UNROLLED_LOOP_INDEX
		#elif ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		#define SPOT_LIGHT_MAP_INDEX NUM_SPOT_LIGHT_MAPS
		#else
		#define SPOT_LIGHT_MAP_INDEX ( UNROLLED_LOOP_INDEX - NUM_SPOT_LIGHT_SHADOWS + NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#endif
		#if ( SPOT_LIGHT_MAP_INDEX < NUM_SPOT_LIGHT_MAPS )
			spotLightCoord = vSpotLightCoord[ i ].xyz / vSpotLightCoord[ i ].w;
			inSpotLightMap = all( lessThan( abs( spotLightCoord * 2. - 1. ), vec3( 1.0 ) ) );
			spotColor = texture2D( spotLightMap[ SPOT_LIGHT_MAP_INDEX ], spotLightCoord.xy );
			directLight.color = inSpotLightMap ? directLight.color * spotColor.rgb : directLight.color;
		#endif
		#undef SPOT_LIGHT_MAP_INDEX
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		spotLightShadow = spotLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( spotShadowMap[ i ], spotLightShadow.shadowMapSize, spotLightShadow.shadowIntensity, spotLightShadow.shadowBias, spotLightShadow.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_DIR_LIGHTS > 0 ) && defined( RE_Direct )
	DirectionalLight directionalLight;
	#if defined( USE_SHADOWMAP ) && NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHTS; i ++ ) {
		directionalLight = directionalLights[ i ];
		getDirectionalLightInfo( directionalLight, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_DIR_LIGHT_SHADOWS )
		directionalLightShadow = directionalLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( directionalShadowMap[ i ], directionalLightShadow.shadowMapSize, directionalLightShadow.shadowIntensity, directionalLightShadow.shadowBias, directionalLightShadow.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_RECT_AREA_LIGHTS > 0 ) && defined( RE_Direct_RectArea )
	RectAreaLight rectAreaLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_RECT_AREA_LIGHTS; i ++ ) {
		rectAreaLight = rectAreaLights[ i ];
		RE_Direct_RectArea( rectAreaLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if defined( RE_IndirectDiffuse )
	vec3 iblIrradiance = vec3( 0.0 );
	vec3 irradiance = getAmbientLightIrradiance( ambientLightColor );
	#if defined( USE_LIGHT_PROBES )
		irradiance += getLightProbeIrradiance( lightProbe, geometryNormal );
	#endif
	#if ( NUM_HEMI_LIGHTS > 0 )
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_HEMI_LIGHTS; i ++ ) {
			irradiance += getHemisphereLightIrradiance( hemisphereLights[ i ], geometryNormal );
		}
		#pragma unroll_loop_end
	#endif
#endif
#if defined( RE_IndirectSpecular )
	vec3 radiance = vec3( 0.0 );
	vec3 clearcoatRadiance = vec3( 0.0 );
#endif`,Vp=`#if defined( RE_IndirectDiffuse )
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
		irradiance += lightMapIrradiance;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD ) && defined( ENVMAP_TYPE_CUBE_UV )
		iblIrradiance += getIBLIrradiance( geometryNormal );
	#endif
#endif
#if defined( USE_ENVMAP ) && defined( RE_IndirectSpecular )
	#ifdef USE_ANISOTROPY
		radiance += getIBLAnisotropyRadiance( geometryViewDir, geometryNormal, material.roughness, material.anisotropyB, material.anisotropy );
	#else
		radiance += getIBLRadiance( geometryViewDir, geometryNormal, material.roughness );
	#endif
	#ifdef USE_CLEARCOAT
		clearcoatRadiance += getIBLRadiance( geometryViewDir, geometryClearcoatNormal, material.clearcoatRoughness );
	#endif
#endif`,Wp=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,Xp=`#if defined( USE_LOGARITHMIC_DEPTH_BUFFER )
	gl_FragDepth = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,$p=`#if defined( USE_LOGARITHMIC_DEPTH_BUFFER )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,Yp=`#ifdef USE_LOGARITHMIC_DEPTH_BUFFER
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,qp=`#ifdef USE_LOGARITHMIC_DEPTH_BUFFER
	vFragDepth = 1.0 + gl_Position.w;
	vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
#endif`,Kp=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = sRGBTransferEOTF( sampledDiffuseColor );
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,Zp=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,Jp=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
	#if defined( USE_POINTS_UV )
		vec2 uv = vUv;
	#else
		vec2 uv = ( uvTransform * vec3( gl_PointCoord.x, 1.0 - gl_PointCoord.y, 1 ) ).xy;
	#endif
#endif
#ifdef USE_MAP
	diffuseColor *= texture2D( map, uv );
#endif
#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, uv ).g;
#endif`,jp=`#if defined( USE_POINTS_UV )
	varying vec2 vUv;
#else
	#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
		uniform mat3 uvTransform;
	#endif
#endif
#ifdef USE_MAP
	uniform sampler2D map;
#endif
#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,Qp=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,tm=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,em=`#ifdef USE_INSTANCING_MORPH
	float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	float morphTargetBaseInfluence = texelFetch( morphTexture, ivec2( 0, gl_InstanceID ), 0 ).r;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		morphTargetInfluences[i] =  texelFetch( morphTexture, ivec2( i + 1, gl_InstanceID ), 0 ).r;
	}
#endif`,im=`#if defined( USE_MORPHCOLORS )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,nm=`#ifdef USE_MORPHNORMALS
	objectNormal *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) objectNormal += getMorph( gl_VertexID, i, 1 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,sm=`#ifdef USE_MORPHTARGETS
	#ifndef USE_INSTANCING_MORPH
		uniform float morphTargetBaseInfluence;
		uniform float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	#endif
	uniform sampler2DArray morphTargetsTexture;
	uniform ivec2 morphTargetsTextureSize;
	vec4 getMorph( const in int vertexIndex, const in int morphTargetIndex, const in int offset ) {
		int texelIndex = vertexIndex * MORPHTARGETS_TEXTURE_STRIDE + offset;
		int y = texelIndex / morphTargetsTextureSize.x;
		int x = texelIndex - y * morphTargetsTextureSize.x;
		ivec3 morphUV = ivec3( x, y, morphTargetIndex );
		return texelFetch( morphTargetsTexture, morphUV, 0 );
	}
#endif`,rm=`#ifdef USE_MORPHTARGETS
	transformed *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) transformed += getMorph( gl_VertexID, i, 0 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,am=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
#ifdef FLAT_SHADED
	vec3 fdx = dFdx( vViewPosition );
	vec3 fdy = dFdy( vViewPosition );
	vec3 normal = normalize( cross( fdx, fdy ) );
#else
	vec3 normal = normalize( vNormal );
	#ifdef DOUBLE_SIDED
		normal *= faceDirection;
	#endif
#endif
#if defined( USE_NORMALMAP_TANGENTSPACE ) || defined( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY )
	#ifdef USE_TANGENT
		mat3 tbn = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn = getTangentFrame( - vViewPosition, normal,
		#if defined( USE_NORMALMAP )
			vNormalMapUv
		#elif defined( USE_CLEARCOAT_NORMALMAP )
			vClearcoatNormalMapUv
		#else
			vUv
		#endif
		);
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn[0] *= faceDirection;
		tbn[1] *= faceDirection;
	#endif
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	#ifdef USE_TANGENT
		mat3 tbn2 = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn2 = getTangentFrame( - vViewPosition, normal, vClearcoatNormalMapUv );
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn2[0] *= faceDirection;
		tbn2[1] *= faceDirection;
	#endif
#endif
vec3 nonPerturbedNormal = normal;`,om=`#ifdef USE_NORMALMAP_OBJECTSPACE
	normal = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	#ifdef FLIP_SIDED
		normal = - normal;
	#endif
	#ifdef DOUBLE_SIDED
		normal = normal * faceDirection;
	#endif
	normal = normalize( normalMatrix * normal );
#elif defined( USE_NORMALMAP_TANGENTSPACE )
	vec3 mapN = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	mapN.xy *= normalScale;
	normal = normalize( tbn * mapN );
#elif defined( USE_BUMPMAP )
	normal = perturbNormalArb( - vViewPosition, normal, dHdxy_fwd(), faceDirection );
#endif`,lm=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,cm=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,hm=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,um=`#ifdef USE_NORMALMAP
	uniform sampler2D normalMap;
	uniform vec2 normalScale;
#endif
#ifdef USE_NORMALMAP_OBJECTSPACE
	uniform mat3 normalMatrix;
#endif
#if ! defined ( USE_TANGENT ) && ( defined ( USE_NORMALMAP_TANGENTSPACE ) || defined ( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY ) )
	mat3 getTangentFrame( vec3 eye_pos, vec3 surf_norm, vec2 uv ) {
		vec3 q0 = dFdx( eye_pos.xyz );
		vec3 q1 = dFdy( eye_pos.xyz );
		vec2 st0 = dFdx( uv.st );
		vec2 st1 = dFdy( uv.st );
		vec3 N = surf_norm;
		vec3 q1perp = cross( q1, N );
		vec3 q0perp = cross( N, q0 );
		vec3 T = q1perp * st0.x + q0perp * st1.x;
		vec3 B = q1perp * st0.y + q0perp * st1.y;
		float det = max( dot( T, T ), dot( B, B ) );
		float scale = ( det == 0.0 ) ? 0.0 : inversesqrt( det );
		return mat3( T * scale, B * scale, N );
	}
#endif`,dm=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,fm=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,pm=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,mm=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,gm=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,vm=`vec3 packNormalToRGB( const in vec3 normal ) {
	return normalize( normal ) * 0.5 + 0.5;
}
vec3 unpackRGBToNormal( const in vec3 rgb ) {
	return 2.0 * rgb.xyz - 1.0;
}
const float PackUpscale = 256. / 255.;const float UnpackDownscale = 255. / 256.;const float ShiftRight8 = 1. / 256.;
const float Inv255 = 1. / 255.;
const vec4 PackFactors = vec4( 1.0, 256.0, 256.0 * 256.0, 256.0 * 256.0 * 256.0 );
const vec2 UnpackFactors2 = vec2( UnpackDownscale, 1.0 / PackFactors.g );
const vec3 UnpackFactors3 = vec3( UnpackDownscale / PackFactors.rg, 1.0 / PackFactors.b );
const vec4 UnpackFactors4 = vec4( UnpackDownscale / PackFactors.rgb, 1.0 / PackFactors.a );
vec4 packDepthToRGBA( const in float v ) {
	if( v <= 0.0 )
		return vec4( 0., 0., 0., 0. );
	if( v >= 1.0 )
		return vec4( 1., 1., 1., 1. );
	float vuf;
	float af = modf( v * PackFactors.a, vuf );
	float bf = modf( vuf * ShiftRight8, vuf );
	float gf = modf( vuf * ShiftRight8, vuf );
	return vec4( vuf * Inv255, gf * PackUpscale, bf * PackUpscale, af );
}
vec3 packDepthToRGB( const in float v ) {
	if( v <= 0.0 )
		return vec3( 0., 0., 0. );
	if( v >= 1.0 )
		return vec3( 1., 1., 1. );
	float vuf;
	float bf = modf( v * PackFactors.b, vuf );
	float gf = modf( vuf * ShiftRight8, vuf );
	return vec3( vuf * Inv255, gf * PackUpscale, bf );
}
vec2 packDepthToRG( const in float v ) {
	if( v <= 0.0 )
		return vec2( 0., 0. );
	if( v >= 1.0 )
		return vec2( 1., 1. );
	float vuf;
	float gf = modf( v * 256., vuf );
	return vec2( vuf * Inv255, gf );
}
float unpackRGBAToDepth( const in vec4 v ) {
	return dot( v, UnpackFactors4 );
}
float unpackRGBToDepth( const in vec3 v ) {
	return dot( v, UnpackFactors3 );
}
float unpackRGToDepth( const in vec2 v ) {
	return v.r * UnpackFactors2.r + v.g * UnpackFactors2.g;
}
vec4 pack2HalfToRGBA( const in vec2 v ) {
	vec4 r = vec4( v.x, fract( v.x * 255.0 ), v.y, fract( v.y * 255.0 ) );
	return vec4( r.x - r.y / 255.0, r.y, r.z - r.w / 255.0, r.w );
}
vec2 unpackRGBATo2Half( const in vec4 v ) {
	return vec2( v.x + ( v.y / 255.0 ), v.z + ( v.w / 255.0 ) );
}
float viewZToOrthographicDepth( const in float viewZ, const in float near, const in float far ) {
	return ( viewZ + near ) / ( near - far );
}
float orthographicDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return depth * ( near - far ) - near;
}
float viewZToPerspectiveDepth( const in float viewZ, const in float near, const in float far ) {
	return ( ( near + viewZ ) * far ) / ( ( far - near ) * viewZ );
}
float perspectiveDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return ( near * far ) / ( ( far - near ) * depth - far );
}`,_m=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,xm=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,ym=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,Mm=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,Sm=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,bm=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,Em=`#if NUM_SPOT_LIGHT_COORDS > 0
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#if NUM_SPOT_LIGHT_MAPS > 0
	uniform sampler2D spotLightMap[ NUM_SPOT_LIGHT_MAPS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform sampler2D directionalShadowMap[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		uniform sampler2D spotShadowMap[ NUM_SPOT_LIGHT_SHADOWS ];
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform sampler2D pointShadowMap[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
	float texture2DCompare( sampler2D depths, vec2 uv, float compare ) {
		float depth = unpackRGBAToDepth( texture2D( depths, uv ) );
		#ifdef USE_REVERSED_DEPTH_BUFFER
			return step( depth, compare );
		#else
			return step( compare, depth );
		#endif
	}
	vec2 texture2DDistribution( sampler2D shadow, vec2 uv ) {
		return unpackRGBATo2Half( texture2D( shadow, uv ) );
	}
	float VSMShadow( sampler2D shadow, vec2 uv, float compare ) {
		float occlusion = 1.0;
		vec2 distribution = texture2DDistribution( shadow, uv );
		#ifdef USE_REVERSED_DEPTH_BUFFER
			float hard_shadow = step( distribution.x, compare );
		#else
			float hard_shadow = step( compare, distribution.x );
		#endif
		if ( hard_shadow != 1.0 ) {
			float distance = compare - distribution.x;
			float variance = max( 0.00000, distribution.y * distribution.y );
			float softness_probability = variance / (variance + distance * distance );			softness_probability = clamp( ( softness_probability - 0.3 ) / ( 0.95 - 0.3 ), 0.0, 1.0 );			occlusion = clamp( max( hard_shadow, softness_probability ), 0.0, 1.0 );
		}
		return occlusion;
	}
	float getShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord ) {
		float shadow = 1.0;
		shadowCoord.xyz /= shadowCoord.w;
		shadowCoord.z += shadowBias;
		bool inFrustum = shadowCoord.x >= 0.0 && shadowCoord.x <= 1.0 && shadowCoord.y >= 0.0 && shadowCoord.y <= 1.0;
		bool frustumTest = inFrustum && shadowCoord.z <= 1.0;
		if ( frustumTest ) {
		#if defined( SHADOWMAP_TYPE_PCF )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx0 = - texelSize.x * shadowRadius;
			float dy0 = - texelSize.y * shadowRadius;
			float dx1 = + texelSize.x * shadowRadius;
			float dy1 = + texelSize.y * shadowRadius;
			float dx2 = dx0 / 2.0;
			float dy2 = dy0 / 2.0;
			float dx3 = dx1 / 2.0;
			float dy3 = dy1 / 2.0;
			shadow = (
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy1 ), shadowCoord.z )
			) * ( 1.0 / 17.0 );
		#elif defined( SHADOWMAP_TYPE_PCF_SOFT )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx = texelSize.x;
			float dy = texelSize.y;
			vec2 uv = shadowCoord.xy;
			vec2 f = fract( uv * shadowMapSize + 0.5 );
			uv -= f * texelSize;
			shadow = (
				texture2DCompare( shadowMap, uv, shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( dx, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( 0.0, dy ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + texelSize, shadowCoord.z ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, 0.0 ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 0.0 ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, dy ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( 0.0, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 0.0, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( texture2DCompare( shadowMap, uv + vec2( dx, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( dx, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( mix( texture2DCompare( shadowMap, uv + vec2( -dx, -dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, -dy ), shadowCoord.z ),
						  f.x ),
					 mix( texture2DCompare( shadowMap, uv + vec2( -dx, 2.0 * dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 2.0 * dy ), shadowCoord.z ),
						  f.x ),
					 f.y )
			) * ( 1.0 / 9.0 );
		#elif defined( SHADOWMAP_TYPE_VSM )
			shadow = VSMShadow( shadowMap, shadowCoord.xy, shadowCoord.z );
		#else
			shadow = texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z );
		#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
	vec2 cubeToUV( vec3 v, float texelSizeY ) {
		vec3 absV = abs( v );
		float scaleToCube = 1.0 / max( absV.x, max( absV.y, absV.z ) );
		absV *= scaleToCube;
		v *= scaleToCube * ( 1.0 - 2.0 * texelSizeY );
		vec2 planar = v.xy;
		float almostATexel = 1.5 * texelSizeY;
		float almostOne = 1.0 - almostATexel;
		if ( absV.z >= almostOne ) {
			if ( v.z > 0.0 )
				planar.x = 4.0 - v.x;
		} else if ( absV.x >= almostOne ) {
			float signX = sign( v.x );
			planar.x = v.z * signX + 2.0 * signX;
		} else if ( absV.y >= almostOne ) {
			float signY = sign( v.y );
			planar.x = v.x + 2.0 * signY + 2.0;
			planar.y = v.z * signY - 2.0;
		}
		return vec2( 0.125, 0.25 ) * planar + vec2( 0.375, 0.75 );
	}
	float getPointShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord, float shadowCameraNear, float shadowCameraFar ) {
		float shadow = 1.0;
		vec3 lightToPosition = shadowCoord.xyz;
		
		float lightToPositionLength = length( lightToPosition );
		if ( lightToPositionLength - shadowCameraFar <= 0.0 && lightToPositionLength - shadowCameraNear >= 0.0 ) {
			float dp = ( lightToPositionLength - shadowCameraNear ) / ( shadowCameraFar - shadowCameraNear );			dp += shadowBias;
			vec3 bd3D = normalize( lightToPosition );
			vec2 texelSize = vec2( 1.0 ) / ( shadowMapSize * vec2( 4.0, 2.0 ) );
			#if defined( SHADOWMAP_TYPE_PCF ) || defined( SHADOWMAP_TYPE_PCF_SOFT ) || defined( SHADOWMAP_TYPE_VSM )
				vec2 offset = vec2( - 1, 1 ) * shadowRadius * texelSize.y;
				shadow = (
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxx, texelSize.y ), dp )
				) * ( 1.0 / 9.0 );
			#else
				shadow = texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp );
			#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
#endif`,wm=`#if NUM_SPOT_LIGHT_COORDS > 0
	uniform mat4 spotLightMatrix[ NUM_SPOT_LIGHT_COORDS ];
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform mat4 directionalShadowMatrix[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform mat4 pointShadowMatrix[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
#endif`,Tm=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
	vec3 shadowWorldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
	vec4 shadowWorldPosition;
#endif
#if defined( USE_SHADOWMAP )
	#if NUM_DIR_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * directionalLightShadows[ i ].shadowNormalBias, 0 );
			vDirectionalShadowCoord[ i ] = directionalShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * pointLightShadows[ i ].shadowNormalBias, 0 );
			vPointShadowCoord[ i ] = pointShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
#endif
#if NUM_SPOT_LIGHT_COORDS > 0
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_COORDS; i ++ ) {
		shadowWorldPosition = worldPosition;
		#if ( defined( USE_SHADOWMAP ) && UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
			shadowWorldPosition.xyz += shadowWorldNormal * spotLightShadows[ i ].shadowNormalBias;
		#endif
		vSpotLightCoord[ i ] = spotLightMatrix[ i ] * shadowWorldPosition;
	}
	#pragma unroll_loop_end
#endif`,Am=`float getShadowMask() {
	float shadow = 1.0;
	#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
		directionalLight = directionalLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( directionalShadowMap[ i ], directionalLight.shadowMapSize, directionalLight.shadowIntensity, directionalLight.shadowBias, directionalLight.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_SHADOWS; i ++ ) {
		spotLight = spotLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( spotShadowMap[ i ], spotLight.shadowMapSize, spotLight.shadowIntensity, spotLight.shadowBias, spotLight.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
		pointLight = pointLightShadows[ i ];
		shadow *= receiveShadow ? getPointShadow( pointShadowMap[ i ], pointLight.shadowMapSize, pointLight.shadowIntensity, pointLight.shadowBias, pointLight.shadowRadius, vPointShadowCoord[ i ], pointLight.shadowCameraNear, pointLight.shadowCameraFar ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#endif
	return shadow;
}`,Rm=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,Cm=`#ifdef USE_SKINNING
	uniform mat4 bindMatrix;
	uniform mat4 bindMatrixInverse;
	uniform highp sampler2D boneTexture;
	mat4 getBoneMatrix( const in float i ) {
		int size = textureSize( boneTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( boneTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( boneTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( boneTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( boneTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
#endif`,Pm=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,Im=`#ifdef USE_SKINNING
	mat4 skinMatrix = mat4( 0.0 );
	skinMatrix += skinWeight.x * boneMatX;
	skinMatrix += skinWeight.y * boneMatY;
	skinMatrix += skinWeight.z * boneMatZ;
	skinMatrix += skinWeight.w * boneMatW;
	skinMatrix = bindMatrixInverse * skinMatrix * bindMatrix;
	objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;
	#ifdef USE_TANGENT
		objectTangent = vec4( skinMatrix * vec4( objectTangent, 0.0 ) ).xyz;
	#endif
#endif`,Dm=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,Lm=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,Nm=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,Um=`#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
uniform float toneMappingExposure;
vec3 LinearToneMapping( vec3 color ) {
	return saturate( toneMappingExposure * color );
}
vec3 ReinhardToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	return saturate( color / ( vec3( 1.0 ) + color ) );
}
vec3 CineonToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	color = max( vec3( 0.0 ), color - 0.004 );
	return pow( ( color * ( 6.2 * color + 0.5 ) ) / ( color * ( 6.2 * color + 1.7 ) + 0.06 ), vec3( 2.2 ) );
}
vec3 RRTAndODTFit( vec3 v ) {
	vec3 a = v * ( v + 0.0245786 ) - 0.000090537;
	vec3 b = v * ( 0.983729 * v + 0.4329510 ) + 0.238081;
	return a / b;
}
vec3 ACESFilmicToneMapping( vec3 color ) {
	const mat3 ACESInputMat = mat3(
		vec3( 0.59719, 0.07600, 0.02840 ),		vec3( 0.35458, 0.90834, 0.13383 ),
		vec3( 0.04823, 0.01566, 0.83777 )
	);
	const mat3 ACESOutputMat = mat3(
		vec3(  1.60475, -0.10208, -0.00327 ),		vec3( -0.53108,  1.10813, -0.07276 ),
		vec3( -0.07367, -0.00605,  1.07602 )
	);
	color *= toneMappingExposure / 0.6;
	color = ACESInputMat * color;
	color = RRTAndODTFit( color );
	color = ACESOutputMat * color;
	return saturate( color );
}
const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(
	vec3( 1.6605, - 0.1246, - 0.0182 ),
	vec3( - 0.5876, 1.1329, - 0.1006 ),
	vec3( - 0.0728, - 0.0083, 1.1187 )
);
const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(
	vec3( 0.6274, 0.0691, 0.0164 ),
	vec3( 0.3293, 0.9195, 0.0880 ),
	vec3( 0.0433, 0.0113, 0.8956 )
);
vec3 agxDefaultContrastApprox( vec3 x ) {
	vec3 x2 = x * x;
	vec3 x4 = x2 * x2;
	return + 15.5 * x4 * x2
		- 40.14 * x4 * x
		+ 31.96 * x4
		- 6.868 * x2 * x
		+ 0.4298 * x2
		+ 0.1191 * x
		- 0.00232;
}
vec3 AgXToneMapping( vec3 color ) {
	const mat3 AgXInsetMatrix = mat3(
		vec3( 0.856627153315983, 0.137318972929847, 0.11189821299995 ),
		vec3( 0.0951212405381588, 0.761241990602591, 0.0767994186031903 ),
		vec3( 0.0482516061458583, 0.101439036467562, 0.811302368396859 )
	);
	const mat3 AgXOutsetMatrix = mat3(
		vec3( 1.1271005818144368, - 0.1413297634984383, - 0.14132976349843826 ),
		vec3( - 0.11060664309660323, 1.157823702216272, - 0.11060664309660294 ),
		vec3( - 0.016493938717834573, - 0.016493938717834257, 1.2519364065950405 )
	);
	const float AgxMinEv = - 12.47393;	const float AgxMaxEv = 4.026069;
	color *= toneMappingExposure;
	color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
	color = AgXInsetMatrix * color;
	color = max( color, 1e-10 );	color = log2( color );
	color = ( color - AgxMinEv ) / ( AgxMaxEv - AgxMinEv );
	color = clamp( color, 0.0, 1.0 );
	color = agxDefaultContrastApprox( color );
	color = AgXOutsetMatrix * color;
	color = pow( max( vec3( 0.0 ), color ), vec3( 2.2 ) );
	color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
	color = clamp( color, 0.0, 1.0 );
	return color;
}
vec3 NeutralToneMapping( vec3 color ) {
	const float StartCompression = 0.8 - 0.04;
	const float Desaturation = 0.15;
	color *= toneMappingExposure;
	float x = min( color.r, min( color.g, color.b ) );
	float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
	color -= offset;
	float peak = max( color.r, max( color.g, color.b ) );
	if ( peak < StartCompression ) return color;
	float d = 1. - StartCompression;
	float newPeak = 1. - d * d / ( peak + d - StartCompression );
	color *= newPeak / peak;
	float g = 1. - 1. / ( Desaturation * ( peak - newPeak ) + 1. );
	return mix( color, vec3( newPeak ), g );
}
vec3 CustomToneMapping( vec3 color ) { return color; }`,Om=`#ifdef USE_TRANSMISSION
	material.transmission = transmission;
	material.transmissionAlpha = 1.0;
	material.thickness = thickness;
	material.attenuationDistance = attenuationDistance;
	material.attenuationColor = attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		material.transmission *= texture2D( transmissionMap, vTransmissionMapUv ).r;
	#endif
	#ifdef USE_THICKNESSMAP
		material.thickness *= texture2D( thicknessMap, vThicknessMapUv ).g;
	#endif
	vec3 pos = vWorldPosition;
	vec3 v = normalize( cameraPosition - pos );
	vec3 n = inverseTransformDirection( normal, viewMatrix );
	vec4 transmitted = getIBLVolumeRefraction(
		n, v, material.roughness, material.diffuseColor, material.specularColor, material.specularF90,
		pos, modelMatrix, viewMatrix, projectionMatrix, material.dispersion, material.ior, material.thickness,
		material.attenuationColor, material.attenuationDistance );
	material.transmissionAlpha = mix( material.transmissionAlpha, transmitted.a, material.transmission );
	totalDiffuse = mix( totalDiffuse, transmitted.rgb, material.transmission );
#endif`,Fm=`#ifdef USE_TRANSMISSION
	uniform float transmission;
	uniform float thickness;
	uniform float attenuationDistance;
	uniform vec3 attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		uniform sampler2D transmissionMap;
	#endif
	#ifdef USE_THICKNESSMAP
		uniform sampler2D thicknessMap;
	#endif
	uniform vec2 transmissionSamplerSize;
	uniform sampler2D transmissionSamplerMap;
	uniform mat4 modelMatrix;
	uniform mat4 projectionMatrix;
	varying vec3 vWorldPosition;
	float w0( float a ) {
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - a + 3.0 ) - 3.0 ) + 1.0 );
	}
	float w1( float a ) {
		return ( 1.0 / 6.0 ) * ( a *  a * ( 3.0 * a - 6.0 ) + 4.0 );
	}
	float w2( float a ){
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - 3.0 * a + 3.0 ) + 3.0 ) + 1.0 );
	}
	float w3( float a ) {
		return ( 1.0 / 6.0 ) * ( a * a * a );
	}
	float g0( float a ) {
		return w0( a ) + w1( a );
	}
	float g1( float a ) {
		return w2( a ) + w3( a );
	}
	float h0( float a ) {
		return - 1.0 + w1( a ) / ( w0( a ) + w1( a ) );
	}
	float h1( float a ) {
		return 1.0 + w3( a ) / ( w2( a ) + w3( a ) );
	}
	vec4 bicubic( sampler2D tex, vec2 uv, vec4 texelSize, float lod ) {
		uv = uv * texelSize.zw + 0.5;
		vec2 iuv = floor( uv );
		vec2 fuv = fract( uv );
		float g0x = g0( fuv.x );
		float g1x = g1( fuv.x );
		float h0x = h0( fuv.x );
		float h1x = h1( fuv.x );
		float h0y = h0( fuv.y );
		float h1y = h1( fuv.y );
		vec2 p0 = ( vec2( iuv.x + h0x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p1 = ( vec2( iuv.x + h1x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p2 = ( vec2( iuv.x + h0x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		vec2 p3 = ( vec2( iuv.x + h1x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		return g0( fuv.y ) * ( g0x * textureLod( tex, p0, lod ) + g1x * textureLod( tex, p1, lod ) ) +
			g1( fuv.y ) * ( g0x * textureLod( tex, p2, lod ) + g1x * textureLod( tex, p3, lod ) );
	}
	vec4 textureBicubic( sampler2D sampler, vec2 uv, float lod ) {
		vec2 fLodSize = vec2( textureSize( sampler, int( lod ) ) );
		vec2 cLodSize = vec2( textureSize( sampler, int( lod + 1.0 ) ) );
		vec2 fLodSizeInv = 1.0 / fLodSize;
		vec2 cLodSizeInv = 1.0 / cLodSize;
		vec4 fSample = bicubic( sampler, uv, vec4( fLodSizeInv, fLodSize ), floor( lod ) );
		vec4 cSample = bicubic( sampler, uv, vec4( cLodSizeInv, cLodSize ), ceil( lod ) );
		return mix( fSample, cSample, fract( lod ) );
	}
	vec3 getVolumeTransmissionRay( const in vec3 n, const in vec3 v, const in float thickness, const in float ior, const in mat4 modelMatrix ) {
		vec3 refractionVector = refract( - v, normalize( n ), 1.0 / ior );
		vec3 modelScale;
		modelScale.x = length( vec3( modelMatrix[ 0 ].xyz ) );
		modelScale.y = length( vec3( modelMatrix[ 1 ].xyz ) );
		modelScale.z = length( vec3( modelMatrix[ 2 ].xyz ) );
		return normalize( refractionVector ) * thickness * modelScale;
	}
	float applyIorToRoughness( const in float roughness, const in float ior ) {
		return roughness * clamp( ior * 2.0 - 2.0, 0.0, 1.0 );
	}
	vec4 getTransmissionSample( const in vec2 fragCoord, const in float roughness, const in float ior ) {
		float lod = log2( transmissionSamplerSize.x ) * applyIorToRoughness( roughness, ior );
		return textureBicubic( transmissionSamplerMap, fragCoord.xy, lod );
	}
	vec3 volumeAttenuation( const in float transmissionDistance, const in vec3 attenuationColor, const in float attenuationDistance ) {
		if ( isinf( attenuationDistance ) ) {
			return vec3( 1.0 );
		} else {
			vec3 attenuationCoefficient = -log( attenuationColor ) / attenuationDistance;
			vec3 transmittance = exp( - attenuationCoefficient * transmissionDistance );			return transmittance;
		}
	}
	vec4 getIBLVolumeRefraction( const in vec3 n, const in vec3 v, const in float roughness, const in vec3 diffuseColor,
		const in vec3 specularColor, const in float specularF90, const in vec3 position, const in mat4 modelMatrix,
		const in mat4 viewMatrix, const in mat4 projMatrix, const in float dispersion, const in float ior, const in float thickness,
		const in vec3 attenuationColor, const in float attenuationDistance ) {
		vec4 transmittedLight;
		vec3 transmittance;
		#ifdef USE_DISPERSION
			float halfSpread = ( ior - 1.0 ) * 0.025 * dispersion;
			vec3 iors = vec3( ior - halfSpread, ior, ior + halfSpread );
			for ( int i = 0; i < 3; i ++ ) {
				vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, iors[ i ], modelMatrix );
				vec3 refractedRayExit = position + transmissionRay;
				vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
				vec2 refractionCoords = ndcPos.xy / ndcPos.w;
				refractionCoords += 1.0;
				refractionCoords /= 2.0;
				vec4 transmissionSample = getTransmissionSample( refractionCoords, roughness, iors[ i ] );
				transmittedLight[ i ] = transmissionSample[ i ];
				transmittedLight.a += transmissionSample.a;
				transmittance[ i ] = diffuseColor[ i ] * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance )[ i ];
			}
			transmittedLight.a /= 3.0;
		#else
			vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, ior, modelMatrix );
			vec3 refractedRayExit = position + transmissionRay;
			vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
			vec2 refractionCoords = ndcPos.xy / ndcPos.w;
			refractionCoords += 1.0;
			refractionCoords /= 2.0;
			transmittedLight = getTransmissionSample( refractionCoords, roughness, ior );
			transmittance = diffuseColor * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance );
		#endif
		vec3 attenuatedColor = transmittance * transmittedLight.rgb;
		vec3 F = EnvironmentBRDF( n, v, specularColor, specularF90, roughness );
		float transmittanceFactor = ( transmittance.r + transmittance.g + transmittance.b ) / 3.0;
		return vec4( ( 1.0 - F ) * attenuatedColor, 1.0 - ( 1.0 - transmittedLight.a ) * transmittanceFactor );
	}
#endif`,km=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_SPECULARMAP
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,zm=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	uniform mat3 mapTransform;
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	uniform mat3 alphaMapTransform;
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	uniform mat3 lightMapTransform;
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	uniform mat3 aoMapTransform;
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	uniform mat3 bumpMapTransform;
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	uniform mat3 normalMapTransform;
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_DISPLACEMENTMAP
	uniform mat3 displacementMapTransform;
	varying vec2 vDisplacementMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	uniform mat3 emissiveMapTransform;
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	uniform mat3 metalnessMapTransform;
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	uniform mat3 roughnessMapTransform;
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	uniform mat3 anisotropyMapTransform;
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	uniform mat3 clearcoatMapTransform;
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform mat3 clearcoatNormalMapTransform;
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform mat3 clearcoatRoughnessMapTransform;
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	uniform mat3 sheenColorMapTransform;
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	uniform mat3 sheenRoughnessMapTransform;
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	uniform mat3 iridescenceMapTransform;
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform mat3 iridescenceThicknessMapTransform;
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SPECULARMAP
	uniform mat3 specularMapTransform;
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	uniform mat3 specularColorMapTransform;
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	uniform mat3 specularIntensityMapTransform;
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,Bm=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	vUv = vec3( uv, 1 ).xy;
#endif
#ifdef USE_MAP
	vMapUv = ( mapTransform * vec3( MAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ALPHAMAP
	vAlphaMapUv = ( alphaMapTransform * vec3( ALPHAMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_LIGHTMAP
	vLightMapUv = ( lightMapTransform * vec3( LIGHTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_AOMAP
	vAoMapUv = ( aoMapTransform * vec3( AOMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_BUMPMAP
	vBumpMapUv = ( bumpMapTransform * vec3( BUMPMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_NORMALMAP
	vNormalMapUv = ( normalMapTransform * vec3( NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_DISPLACEMENTMAP
	vDisplacementMapUv = ( displacementMapTransform * vec3( DISPLACEMENTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_EMISSIVEMAP
	vEmissiveMapUv = ( emissiveMapTransform * vec3( EMISSIVEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_METALNESSMAP
	vMetalnessMapUv = ( metalnessMapTransform * vec3( METALNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ROUGHNESSMAP
	vRoughnessMapUv = ( roughnessMapTransform * vec3( ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ANISOTROPYMAP
	vAnisotropyMapUv = ( anisotropyMapTransform * vec3( ANISOTROPYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOATMAP
	vClearcoatMapUv = ( clearcoatMapTransform * vec3( CLEARCOATMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	vClearcoatNormalMapUv = ( clearcoatNormalMapTransform * vec3( CLEARCOAT_NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	vClearcoatRoughnessMapUv = ( clearcoatRoughnessMapTransform * vec3( CLEARCOAT_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCEMAP
	vIridescenceMapUv = ( iridescenceMapTransform * vec3( IRIDESCENCEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	vIridescenceThicknessMapUv = ( iridescenceThicknessMapTransform * vec3( IRIDESCENCE_THICKNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_COLORMAP
	vSheenColorMapUv = ( sheenColorMapTransform * vec3( SHEEN_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	vSheenRoughnessMapUv = ( sheenRoughnessMapTransform * vec3( SHEEN_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULARMAP
	vSpecularMapUv = ( specularMapTransform * vec3( SPECULARMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_COLORMAP
	vSpecularColorMapUv = ( specularColorMapTransform * vec3( SPECULAR_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	vSpecularIntensityMapUv = ( specularIntensityMapTransform * vec3( SPECULAR_INTENSITYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_TRANSMISSIONMAP
	vTransmissionMapUv = ( transmissionMapTransform * vec3( TRANSMISSIONMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_THICKNESSMAP
	vThicknessMapUv = ( thicknessMapTransform * vec3( THICKNESSMAP_UV, 1 ) ).xy;
#endif`,Hm=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`;const Gm=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,Vm=`uniform sampler2D t2D;
uniform float backgroundIntensity;
varying vec2 vUv;
void main() {
	vec4 texColor = texture2D( t2D, vUv );
	#ifdef DECODE_VIDEO_TEXTURE
		texColor = vec4( mix( pow( texColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), texColor.rgb * 0.0773993808, vec3( lessThanEqual( texColor.rgb, vec3( 0.04045 ) ) ) ), texColor.w );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,Wm=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,Xm=`#ifdef ENVMAP_TYPE_CUBE
	uniform samplerCube envMap;
#elif defined( ENVMAP_TYPE_CUBE_UV )
	uniform sampler2D envMap;
#endif
uniform float flipEnvMap;
uniform float backgroundBlurriness;
uniform float backgroundIntensity;
uniform mat3 backgroundRotation;
varying vec3 vWorldDirection;
#include <cube_uv_reflection_fragment>
void main() {
	#ifdef ENVMAP_TYPE_CUBE
		vec4 texColor = textureCube( envMap, backgroundRotation * vec3( flipEnvMap * vWorldDirection.x, vWorldDirection.yz ) );
	#elif defined( ENVMAP_TYPE_CUBE_UV )
		vec4 texColor = textureCubeUV( envMap, backgroundRotation * vWorldDirection, backgroundBlurriness );
	#else
		vec4 texColor = vec4( 0.0, 0.0, 0.0, 1.0 );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,$m=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,Ym=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,qm=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
varying vec2 vHighPrecisionZW;
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vHighPrecisionZW = gl_Position.zw;
}`,Km=`#if DEPTH_PACKING == 3200
	uniform float opacity;
#endif
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
varying vec2 vHighPrecisionZW;
void main() {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#if DEPTH_PACKING == 3200
		diffuseColor.a = opacity;
	#endif
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <logdepthbuf_fragment>
	#ifdef USE_REVERSED_DEPTH_BUFFER
		float fragCoordZ = vHighPrecisionZW[ 0 ] / vHighPrecisionZW[ 1 ];
	#else
		float fragCoordZ = 0.5 * vHighPrecisionZW[ 0 ] / vHighPrecisionZW[ 1 ] + 0.5;
	#endif
	#if DEPTH_PACKING == 3200
		gl_FragColor = vec4( vec3( 1.0 - fragCoordZ ), opacity );
	#elif DEPTH_PACKING == 3201
		gl_FragColor = packDepthToRGBA( fragCoordZ );
	#elif DEPTH_PACKING == 3202
		gl_FragColor = vec4( packDepthToRGB( fragCoordZ ), 1.0 );
	#elif DEPTH_PACKING == 3203
		gl_FragColor = vec4( packDepthToRG( fragCoordZ ), 0.0, 1.0 );
	#endif
}`,Zm=`#define DISTANCE
varying vec3 vWorldPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <worldpos_vertex>
	#include <clipping_planes_vertex>
	vWorldPosition = worldPosition.xyz;
}`,Jm=`#define DISTANCE
uniform vec3 referencePosition;
uniform float nearDistance;
uniform float farDistance;
varying vec3 vWorldPosition;
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <clipping_planes_pars_fragment>
void main () {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	float dist = length( vWorldPosition - referencePosition );
	dist = ( dist - nearDistance ) / ( farDistance - nearDistance );
	dist = saturate( dist );
	gl_FragColor = packDepthToRGBA( dist );
}`,jm=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,Qm=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,tg=`uniform float scale;
attribute float lineDistance;
varying float vLineDistance;
#include <common>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	vLineDistance = scale * lineDistance;
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,eg=`uniform vec3 diffuse;
uniform float opacity;
uniform float dashSize;
uniform float totalSize;
varying float vLineDistance;
#include <common>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	if ( mod( vLineDistance, totalSize ) > dashSize ) {
		discard;
	}
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,ig=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#if defined ( USE_ENVMAP ) || defined ( USE_SKINNING )
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinbase_vertex>
		#include <skinnormal_vertex>
		#include <defaultnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <fog_vertex>
}`,ng=`uniform vec3 diffuse;
uniform float opacity;
#ifndef FLAT_SHADED
	varying vec3 vNormal;
#endif
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		reflectedLight.indirectDiffuse += lightMapTexel.rgb * lightMapIntensity * RECIPROCAL_PI;
	#else
		reflectedLight.indirectDiffuse += vec3( 1.0 );
	#endif
	#include <aomap_fragment>
	reflectedLight.indirectDiffuse *= diffuseColor.rgb;
	vec3 outgoingLight = reflectedLight.indirectDiffuse;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,sg=`#define LAMBERT
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,rg=`#define LAMBERT
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_lambert_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_lambert_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,ag=`#define MATCAP
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <displacementmap_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
	vViewPosition = - mvPosition.xyz;
}`,og=`#define MATCAP
uniform vec3 diffuse;
uniform float opacity;
uniform sampler2D matcap;
varying vec3 vViewPosition;
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	vec3 viewDir = normalize( vViewPosition );
	vec3 x = normalize( vec3( viewDir.z, 0.0, - viewDir.x ) );
	vec3 y = cross( viewDir, x );
	vec2 uv = vec2( dot( x, normal ), dot( y, normal ) ) * 0.495 + 0.5;
	#ifdef USE_MATCAP
		vec4 matcapColor = texture2D( matcap, uv );
	#else
		vec4 matcapColor = vec4( vec3( mix( 0.2, 0.8, uv.y ) ), 1.0 );
	#endif
	vec3 outgoingLight = diffuseColor.rgb * matcapColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,lg=`#define NORMAL
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	vViewPosition = - mvPosition.xyz;
#endif
}`,cg=`#define NORMAL
uniform float opacity;
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <packing>
#include <uv_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( 0.0, 0.0, 0.0, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	gl_FragColor = vec4( packNormalToRGB( normal ), diffuseColor.a );
	#ifdef OPAQUE
		gl_FragColor.a = 1.0;
	#endif
}`,hg=`#define PHONG
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,ug=`#define PHONG
uniform vec3 diffuse;
uniform vec3 emissive;
uniform vec3 specular;
uniform float shininess;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_phong_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_phong_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + reflectedLight.directSpecular + reflectedLight.indirectSpecular + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,dg=`#define STANDARD
varying vec3 vViewPosition;
#ifdef USE_TRANSMISSION
	varying vec3 vWorldPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
#ifdef USE_TRANSMISSION
	vWorldPosition = worldPosition.xyz;
#endif
}`,fg=`#define STANDARD
#ifdef PHYSICAL
	#define IOR
	#define USE_SPECULAR
#endif
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float roughness;
uniform float metalness;
uniform float opacity;
#ifdef IOR
	uniform float ior;
#endif
#ifdef USE_SPECULAR
	uniform float specularIntensity;
	uniform vec3 specularColor;
	#ifdef USE_SPECULAR_COLORMAP
		uniform sampler2D specularColorMap;
	#endif
	#ifdef USE_SPECULAR_INTENSITYMAP
		uniform sampler2D specularIntensityMap;
	#endif
#endif
#ifdef USE_CLEARCOAT
	uniform float clearcoat;
	uniform float clearcoatRoughness;
#endif
#ifdef USE_DISPERSION
	uniform float dispersion;
#endif
#ifdef USE_IRIDESCENCE
	uniform float iridescence;
	uniform float iridescenceIOR;
	uniform float iridescenceThicknessMinimum;
	uniform float iridescenceThicknessMaximum;
#endif
#ifdef USE_SHEEN
	uniform vec3 sheenColor;
	uniform float sheenRoughness;
	#ifdef USE_SHEEN_COLORMAP
		uniform sampler2D sheenColorMap;
	#endif
	#ifdef USE_SHEEN_ROUGHNESSMAP
		uniform sampler2D sheenRoughnessMap;
	#endif
#endif
#ifdef USE_ANISOTROPY
	uniform vec2 anisotropyVector;
	#ifdef USE_ANISOTROPYMAP
		uniform sampler2D anisotropyMap;
	#endif
#endif
varying vec3 vViewPosition;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <iridescence_fragment>
#include <cube_uv_reflection_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_physical_pars_fragment>
#include <fog_pars_fragment>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_physical_pars_fragment>
#include <transmission_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <clearcoat_pars_fragment>
#include <iridescence_pars_fragment>
#include <roughnessmap_pars_fragment>
#include <metalnessmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <roughnessmap_fragment>
	#include <metalnessmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <clearcoat_normal_fragment_begin>
	#include <clearcoat_normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_physical_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 totalDiffuse = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse;
	vec3 totalSpecular = reflectedLight.directSpecular + reflectedLight.indirectSpecular;
	#include <transmission_fragment>
	vec3 outgoingLight = totalDiffuse + totalSpecular + totalEmissiveRadiance;
	#ifdef USE_SHEEN
		float sheenEnergyComp = 1.0 - 0.157 * max3( material.sheenColor );
		outgoingLight = outgoingLight * sheenEnergyComp + sheenSpecularDirect + sheenSpecularIndirect;
	#endif
	#ifdef USE_CLEARCOAT
		float dotNVcc = saturate( dot( geometryClearcoatNormal, geometryViewDir ) );
		vec3 Fcc = F_Schlick( material.clearcoatF0, material.clearcoatF90, dotNVcc );
		outgoingLight = outgoingLight * ( 1.0 - material.clearcoat * Fcc ) + ( clearcoatSpecularDirect + clearcoatSpecularIndirect ) * material.clearcoat;
	#endif
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,pg=`#define TOON
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,mg=`#define TOON
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <gradientmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_toon_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_toon_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,gg=`uniform float size;
uniform float scale;
#include <common>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
#ifdef USE_POINTS_UV
	varying vec2 vUv;
	uniform mat3 uvTransform;
#endif
void main() {
	#ifdef USE_POINTS_UV
		vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	#endif
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	gl_PointSize = size;
	#ifdef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) gl_PointSize *= ( scale / - mvPosition.z );
	#endif
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <fog_vertex>
}`,vg=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <color_pars_fragment>
#include <map_particle_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_particle_fragment>
	#include <color_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,_g=`#include <common>
#include <batching_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <shadowmap_pars_vertex>
void main() {
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,xg=`uniform vec3 color;
uniform float opacity;
#include <common>
#include <packing>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <logdepthbuf_pars_fragment>
#include <shadowmap_pars_fragment>
#include <shadowmask_pars_fragment>
void main() {
	#include <logdepthbuf_fragment>
	gl_FragColor = vec4( color, opacity * ( 1.0 - getShadowMask() ) );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,yg=`uniform float rotation;
uniform vec2 center;
#include <common>
#include <uv_pars_vertex>
#include <fog_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	vec4 mvPosition = modelViewMatrix[ 3 ];
	vec2 scale = vec2( length( modelMatrix[ 0 ].xyz ), length( modelMatrix[ 1 ].xyz ) );
	#ifndef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) scale *= - mvPosition.z;
	#endif
	vec2 alignedPosition = ( position.xy - ( center - vec2( 0.5 ) ) ) * scale;
	vec2 rotatedPosition;
	rotatedPosition.x = cos( rotation ) * alignedPosition.x - sin( rotation ) * alignedPosition.y;
	rotatedPosition.y = sin( rotation ) * alignedPosition.x + cos( rotation ) * alignedPosition.y;
	mvPosition.xy += rotatedPosition;
	gl_Position = projectionMatrix * mvPosition;
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,Mg=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,kt={alphahash_fragment:Vf,alphahash_pars_fragment:Wf,alphamap_fragment:Xf,alphamap_pars_fragment:$f,alphatest_fragment:Yf,alphatest_pars_fragment:qf,aomap_fragment:Kf,aomap_pars_fragment:Zf,batching_pars_vertex:Jf,batching_vertex:jf,begin_vertex:Qf,beginnormal_vertex:tp,bsdfs:ep,iridescence_fragment:ip,bumpmap_pars_fragment:np,clipping_planes_fragment:sp,clipping_planes_pars_fragment:rp,clipping_planes_pars_vertex:ap,clipping_planes_vertex:op,color_fragment:lp,color_pars_fragment:cp,color_pars_vertex:hp,color_vertex:up,common:dp,cube_uv_reflection_fragment:fp,defaultnormal_vertex:pp,displacementmap_pars_vertex:mp,displacementmap_vertex:gp,emissivemap_fragment:vp,emissivemap_pars_fragment:_p,colorspace_fragment:xp,colorspace_pars_fragment:yp,envmap_fragment:Mp,envmap_common_pars_fragment:Sp,envmap_pars_fragment:bp,envmap_pars_vertex:Ep,envmap_physical_pars_fragment:Up,envmap_vertex:wp,fog_vertex:Tp,fog_pars_vertex:Ap,fog_fragment:Rp,fog_pars_fragment:Cp,gradientmap_pars_fragment:Pp,lightmap_pars_fragment:Ip,lights_lambert_fragment:Dp,lights_lambert_pars_fragment:Lp,lights_pars_begin:Np,lights_toon_fragment:Op,lights_toon_pars_fragment:Fp,lights_phong_fragment:kp,lights_phong_pars_fragment:zp,lights_physical_fragment:Bp,lights_physical_pars_fragment:Hp,lights_fragment_begin:Gp,lights_fragment_maps:Vp,lights_fragment_end:Wp,logdepthbuf_fragment:Xp,logdepthbuf_pars_fragment:$p,logdepthbuf_pars_vertex:Yp,logdepthbuf_vertex:qp,map_fragment:Kp,map_pars_fragment:Zp,map_particle_fragment:Jp,map_particle_pars_fragment:jp,metalnessmap_fragment:Qp,metalnessmap_pars_fragment:tm,morphinstance_vertex:em,morphcolor_vertex:im,morphnormal_vertex:nm,morphtarget_pars_vertex:sm,morphtarget_vertex:rm,normal_fragment_begin:am,normal_fragment_maps:om,normal_pars_fragment:lm,normal_pars_vertex:cm,normal_vertex:hm,normalmap_pars_fragment:um,clearcoat_normal_fragment_begin:dm,clearcoat_normal_fragment_maps:fm,clearcoat_pars_fragment:pm,iridescence_pars_fragment:mm,opaque_fragment:gm,packing:vm,premultiplied_alpha_fragment:_m,project_vertex:xm,dithering_fragment:ym,dithering_pars_fragment:Mm,roughnessmap_fragment:Sm,roughnessmap_pars_fragment:bm,shadowmap_pars_fragment:Em,shadowmap_pars_vertex:wm,shadowmap_vertex:Tm,shadowmask_pars_fragment:Am,skinbase_vertex:Rm,skinning_pars_vertex:Cm,skinning_vertex:Pm,skinnormal_vertex:Im,specularmap_fragment:Dm,specularmap_pars_fragment:Lm,tonemapping_fragment:Nm,tonemapping_pars_fragment:Um,transmission_fragment:Om,transmission_pars_fragment:Fm,uv_pars_fragment:km,uv_pars_vertex:zm,uv_vertex:Bm,worldpos_vertex:Hm,background_vert:Gm,background_frag:Vm,backgroundCube_vert:Wm,backgroundCube_frag:Xm,cube_vert:$m,cube_frag:Ym,depth_vert:qm,depth_frag:Km,distanceRGBA_vert:Zm,distanceRGBA_frag:Jm,equirect_vert:jm,equirect_frag:Qm,linedashed_vert:tg,linedashed_frag:eg,meshbasic_vert:ig,meshbasic_frag:ng,meshlambert_vert:sg,meshlambert_frag:rg,meshmatcap_vert:ag,meshmatcap_frag:og,meshnormal_vert:lg,meshnormal_frag:cg,meshphong_vert:hg,meshphong_frag:ug,meshphysical_vert:dg,meshphysical_frag:fg,meshtoon_vert:pg,meshtoon_frag:mg,points_vert:gg,points_frag:vg,shadow_vert:_g,shadow_frag:xg,sprite_vert:yg,sprite_frag:Mg},rt={common:{diffuse:{value:new Ht(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new Ot},alphaMap:{value:null},alphaMapTransform:{value:new Ot},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new Ot}},envmap:{envMap:{value:null},envMapRotation:{value:new Ot},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new Ot}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new Ot}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new Ot},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new Ot},normalScale:{value:new ht(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new Ot},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new Ot}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new Ot}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new Ot}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new Ht(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new Ht(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new Ot},alphaTest:{value:0},uvTransform:{value:new Ot}},sprite:{diffuse:{value:new Ht(16777215)},opacity:{value:1},center:{value:new ht(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new Ot},alphaMap:{value:null},alphaMapTransform:{value:new Ot},alphaTest:{value:0}}},ci={basic:{uniforms:Le([rt.common,rt.specularmap,rt.envmap,rt.aomap,rt.lightmap,rt.fog]),vertexShader:kt.meshbasic_vert,fragmentShader:kt.meshbasic_frag},lambert:{uniforms:Le([rt.common,rt.specularmap,rt.envmap,rt.aomap,rt.lightmap,rt.emissivemap,rt.bumpmap,rt.normalmap,rt.displacementmap,rt.fog,rt.lights,{emissive:{value:new Ht(0)}}]),vertexShader:kt.meshlambert_vert,fragmentShader:kt.meshlambert_frag},phong:{uniforms:Le([rt.common,rt.specularmap,rt.envmap,rt.aomap,rt.lightmap,rt.emissivemap,rt.bumpmap,rt.normalmap,rt.displacementmap,rt.fog,rt.lights,{emissive:{value:new Ht(0)},specular:{value:new Ht(1118481)},shininess:{value:30}}]),vertexShader:kt.meshphong_vert,fragmentShader:kt.meshphong_frag},standard:{uniforms:Le([rt.common,rt.envmap,rt.aomap,rt.lightmap,rt.emissivemap,rt.bumpmap,rt.normalmap,rt.displacementmap,rt.roughnessmap,rt.metalnessmap,rt.fog,rt.lights,{emissive:{value:new Ht(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:kt.meshphysical_vert,fragmentShader:kt.meshphysical_frag},toon:{uniforms:Le([rt.common,rt.aomap,rt.lightmap,rt.emissivemap,rt.bumpmap,rt.normalmap,rt.displacementmap,rt.gradientmap,rt.fog,rt.lights,{emissive:{value:new Ht(0)}}]),vertexShader:kt.meshtoon_vert,fragmentShader:kt.meshtoon_frag},matcap:{uniforms:Le([rt.common,rt.bumpmap,rt.normalmap,rt.displacementmap,rt.fog,{matcap:{value:null}}]),vertexShader:kt.meshmatcap_vert,fragmentShader:kt.meshmatcap_frag},points:{uniforms:Le([rt.points,rt.fog]),vertexShader:kt.points_vert,fragmentShader:kt.points_frag},dashed:{uniforms:Le([rt.common,rt.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:kt.linedashed_vert,fragmentShader:kt.linedashed_frag},depth:{uniforms:Le([rt.common,rt.displacementmap]),vertexShader:kt.depth_vert,fragmentShader:kt.depth_frag},normal:{uniforms:Le([rt.common,rt.bumpmap,rt.normalmap,rt.displacementmap,{opacity:{value:1}}]),vertexShader:kt.meshnormal_vert,fragmentShader:kt.meshnormal_frag},sprite:{uniforms:Le([rt.sprite,rt.fog]),vertexShader:kt.sprite_vert,fragmentShader:kt.sprite_frag},background:{uniforms:{uvTransform:{value:new Ot},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:kt.background_vert,fragmentShader:kt.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1},backgroundRotation:{value:new Ot}},vertexShader:kt.backgroundCube_vert,fragmentShader:kt.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:kt.cube_vert,fragmentShader:kt.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:kt.equirect_vert,fragmentShader:kt.equirect_frag},distanceRGBA:{uniforms:Le([rt.common,rt.displacementmap,{referencePosition:{value:new C},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:kt.distanceRGBA_vert,fragmentShader:kt.distanceRGBA_frag},shadow:{uniforms:Le([rt.lights,rt.fog,{color:{value:new Ht(0)},opacity:{value:1}}]),vertexShader:kt.shadow_vert,fragmentShader:kt.shadow_frag}};ci.physical={uniforms:Le([ci.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new Ot},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new Ot},clearcoatNormalScale:{value:new ht(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new Ot},dispersion:{value:0},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new Ot},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new Ot},sheen:{value:0},sheenColor:{value:new Ht(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new Ot},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new Ot},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new Ot},transmissionSamplerSize:{value:new ht},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new Ot},attenuationDistance:{value:0},attenuationColor:{value:new Ht(0)},specularColor:{value:new Ht(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new Ot},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new Ot},anisotropyVector:{value:new ht},anisotropyMap:{value:null},anisotropyMapTransform:{value:new Ot}}]),vertexShader:kt.meshphysical_vert,fragmentShader:kt.meshphysical_frag};const Mr={r:0,b:0,g:0},tn=new gi,Sg=new ie;function bg(s,t,e,i,n,r,a){const o=new Ht(0);let c=r===!0?0:1,l,h,u=null,d=0,f=null;function g(E){let _=E.isScene===!0?E.background:null;return _&&_.isTexture&&(_=(E.backgroundBlurriness>0?e:t).get(_)),_}function v(E){let _=!1;const P=g(E);P===null?p(o,c):P&&P.isColor&&(p(P,1),_=!0);const R=s.xr.getEnvironmentBlendMode();R==="additive"?i.buffers.color.setClear(0,0,0,1,a):R==="alpha-blend"&&i.buffers.color.setClear(0,0,0,0,a),(s.autoClear||_)&&(i.buffers.depth.setTest(!0),i.buffers.depth.setMask(!0),i.buffers.color.setMask(!0),s.clear(s.autoClearColor,s.autoClearDepth,s.autoClearStencil))}function m(E,_){const P=g(_);P&&(P.isCubeTexture||P.mapping===Gr)?(h===void 0&&(h=new Vt(new $i(1,1,1),new Yi({name:"BackgroundCubeMaterial",uniforms:ss(ci.backgroundCube.uniforms),vertexShader:ci.backgroundCube.vertexShader,fragmentShader:ci.backgroundCube.fragmentShader,side:Oe,depthTest:!1,depthWrite:!1,fog:!1,allowOverride:!1})),h.geometry.deleteAttribute("normal"),h.geometry.deleteAttribute("uv"),h.onBeforeRender=function(R,I,T){this.matrixWorld.copyPosition(T.matrixWorld)},Object.defineProperty(h.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),n.update(h)),tn.copy(_.backgroundRotation),tn.x*=-1,tn.y*=-1,tn.z*=-1,P.isCubeTexture&&P.isRenderTargetTexture===!1&&(tn.y*=-1,tn.z*=-1),h.material.uniforms.envMap.value=P,h.material.uniforms.flipEnvMap.value=P.isCubeTexture&&P.isRenderTargetTexture===!1?-1:1,h.material.uniforms.backgroundBlurriness.value=_.backgroundBlurriness,h.material.uniforms.backgroundIntensity.value=_.backgroundIntensity,h.material.uniforms.backgroundRotation.value.setFromMatrix4(Sg.makeRotationFromEuler(tn)),h.material.toneMapped=qt.getTransfer(P.colorSpace)!==te,(u!==P||d!==P.version||f!==s.toneMapping)&&(h.material.needsUpdate=!0,u=P,d=P.version,f=s.toneMapping),h.layers.enableAll(),E.unshift(h,h.geometry,h.material,0,0,null)):P&&P.isTexture&&(l===void 0&&(l=new Vt(new as(2,2),new Yi({name:"BackgroundMaterial",uniforms:ss(ci.background.uniforms),vertexShader:ci.background.vertexShader,fragmentShader:ci.background.fragmentShader,side:Xi,depthTest:!1,depthWrite:!1,fog:!1,allowOverride:!1})),l.geometry.deleteAttribute("normal"),Object.defineProperty(l.material,"map",{get:function(){return this.uniforms.t2D.value}}),n.update(l)),l.material.uniforms.t2D.value=P,l.material.uniforms.backgroundIntensity.value=_.backgroundIntensity,l.material.toneMapped=qt.getTransfer(P.colorSpace)!==te,P.matrixAutoUpdate===!0&&P.updateMatrix(),l.material.uniforms.uvTransform.value.copy(P.matrix),(u!==P||d!==P.version||f!==s.toneMapping)&&(l.material.needsUpdate=!0,u=P,d=P.version,f=s.toneMapping),l.layers.enableAll(),E.unshift(l,l.geometry,l.material,0,0,null))}function p(E,_){E.getRGB(Mr,Mh(s)),i.buffers.color.setClear(Mr.r,Mr.g,Mr.b,_,a)}function w(){h!==void 0&&(h.geometry.dispose(),h.material.dispose(),h=void 0),l!==void 0&&(l.geometry.dispose(),l.material.dispose(),l=void 0)}return{getClearColor:function(){return o},setClearColor:function(E,_=1){o.set(E),c=_,p(o,c)},getClearAlpha:function(){return c},setClearAlpha:function(E){c=E,p(o,c)},render:v,addToRenderList:m,dispose:w}}function Eg(s,t){const e=s.getParameter(s.MAX_VERTEX_ATTRIBS),i={},n=d(null);let r=n,a=!1;function o(b,L,F,B,Y){let W=!1;const $=u(B,F,L);r!==$&&(r=$,l(r.object)),W=f(b,B,F,Y),W&&g(b,B,F,Y),Y!==null&&t.update(Y,s.ELEMENT_ARRAY_BUFFER),(W||a)&&(a=!1,_(b,L,F,B),Y!==null&&s.bindBuffer(s.ELEMENT_ARRAY_BUFFER,t.get(Y).buffer))}function c(){return s.createVertexArray()}function l(b){return s.bindVertexArray(b)}function h(b){return s.deleteVertexArray(b)}function u(b,L,F){const B=F.wireframe===!0;let Y=i[b.id];Y===void 0&&(Y={},i[b.id]=Y);let W=Y[L.id];W===void 0&&(W={},Y[L.id]=W);let $=W[B];return $===void 0&&($=d(c()),W[B]=$),$}function d(b){const L=[],F=[],B=[];for(let Y=0;Y<e;Y++)L[Y]=0,F[Y]=0,B[Y]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:L,enabledAttributes:F,attributeDivisors:B,object:b,attributes:{},index:null}}function f(b,L,F,B){const Y=r.attributes,W=L.attributes;let $=0;const J=F.getAttributes();for(const G in J)if(J[G].location>=0){const ut=Y[G];let Et=W[G];if(Et===void 0&&(G==="instanceMatrix"&&b.instanceMatrix&&(Et=b.instanceMatrix),G==="instanceColor"&&b.instanceColor&&(Et=b.instanceColor)),ut===void 0||ut.attribute!==Et||Et&&ut.data!==Et.data)return!0;$++}return r.attributesNum!==$||r.index!==B}function g(b,L,F,B){const Y={},W=L.attributes;let $=0;const J=F.getAttributes();for(const G in J)if(J[G].location>=0){let ut=W[G];ut===void 0&&(G==="instanceMatrix"&&b.instanceMatrix&&(ut=b.instanceMatrix),G==="instanceColor"&&b.instanceColor&&(ut=b.instanceColor));const Et={};Et.attribute=ut,ut&&ut.data&&(Et.data=ut.data),Y[G]=Et,$++}r.attributes=Y,r.attributesNum=$,r.index=B}function v(){const b=r.newAttributes;for(let L=0,F=b.length;L<F;L++)b[L]=0}function m(b){p(b,0)}function p(b,L){const F=r.newAttributes,B=r.enabledAttributes,Y=r.attributeDivisors;F[b]=1,B[b]===0&&(s.enableVertexAttribArray(b),B[b]=1),Y[b]!==L&&(s.vertexAttribDivisor(b,L),Y[b]=L)}function w(){const b=r.newAttributes,L=r.enabledAttributes;for(let F=0,B=L.length;F<B;F++)L[F]!==b[F]&&(s.disableVertexAttribArray(F),L[F]=0)}function E(b,L,F,B,Y,W,$){$===!0?s.vertexAttribIPointer(b,L,F,Y,W):s.vertexAttribPointer(b,L,F,B,Y,W)}function _(b,L,F,B){v();const Y=B.attributes,W=F.getAttributes(),$=L.defaultAttributeValues;for(const J in W){const G=W[J];if(G.location>=0){let at=Y[J];if(at===void 0&&(J==="instanceMatrix"&&b.instanceMatrix&&(at=b.instanceMatrix),J==="instanceColor"&&b.instanceColor&&(at=b.instanceColor)),at!==void 0){const ut=at.normalized,Et=at.itemSize,Gt=t.get(at);if(Gt===void 0)continue;const se=Gt.buffer,oe=Gt.type,Kt=Gt.bytesPerElement,q=oe===s.INT||oe===s.UNSIGNED_INT||at.gpuType===Xo;if(at.isInterleavedBufferAttribute){const j=at.data,pt=j.stride,Dt=at.offset;if(j.isInstancedInterleavedBuffer){for(let bt=0;bt<G.locationSize;bt++)p(G.location+bt,j.meshPerAttribute);b.isInstancedMesh!==!0&&B._maxInstanceCount===void 0&&(B._maxInstanceCount=j.meshPerAttribute*j.count)}else for(let bt=0;bt<G.locationSize;bt++)m(G.location+bt);s.bindBuffer(s.ARRAY_BUFFER,se);for(let bt=0;bt<G.locationSize;bt++)E(G.location+bt,Et/G.locationSize,oe,ut,pt*Kt,(Dt+Et/G.locationSize*bt)*Kt,q)}else{if(at.isInstancedBufferAttribute){for(let j=0;j<G.locationSize;j++)p(G.location+j,at.meshPerAttribute);b.isInstancedMesh!==!0&&B._maxInstanceCount===void 0&&(B._maxInstanceCount=at.meshPerAttribute*at.count)}else for(let j=0;j<G.locationSize;j++)m(G.location+j);s.bindBuffer(s.ARRAY_BUFFER,se);for(let j=0;j<G.locationSize;j++)E(G.location+j,Et/G.locationSize,oe,ut,Et*Kt,Et/G.locationSize*j*Kt,q)}}else if($!==void 0){const ut=$[J];if(ut!==void 0)switch(ut.length){case 2:s.vertexAttrib2fv(G.location,ut);break;case 3:s.vertexAttrib3fv(G.location,ut);break;case 4:s.vertexAttrib4fv(G.location,ut);break;default:s.vertexAttrib1fv(G.location,ut)}}}}w()}function P(){T();for(const b in i){const L=i[b];for(const F in L){const B=L[F];for(const Y in B)h(B[Y].object),delete B[Y];delete L[F]}delete i[b]}}function R(b){if(i[b.id]===void 0)return;const L=i[b.id];for(const F in L){const B=L[F];for(const Y in B)h(B[Y].object),delete B[Y];delete L[F]}delete i[b.id]}function I(b){for(const L in i){const F=i[L];if(F[b.id]===void 0)continue;const B=F[b.id];for(const Y in B)h(B[Y].object),delete B[Y];delete F[b.id]}}function T(){S(),a=!0,r!==n&&(r=n,l(r.object))}function S(){n.geometry=null,n.program=null,n.wireframe=!1}return{setup:o,reset:T,resetDefaultState:S,dispose:P,releaseStatesOfGeometry:R,releaseStatesOfProgram:I,initAttributes:v,enableAttribute:m,disableUnusedAttributes:w}}function wg(s,t,e){let i;function n(l){i=l}function r(l,h){s.drawArrays(i,l,h),e.update(h,i,1)}function a(l,h,u){u!==0&&(s.drawArraysInstanced(i,l,h,u),e.update(h,i,u))}function o(l,h,u){if(u===0)return;t.get("WEBGL_multi_draw").multiDrawArraysWEBGL(i,l,0,h,0,u);let f=0;for(let g=0;g<u;g++)f+=h[g];e.update(f,i,1)}function c(l,h,u,d){if(u===0)return;const f=t.get("WEBGL_multi_draw");if(f===null)for(let g=0;g<l.length;g++)a(l[g],h[g],d[g]);else{f.multiDrawArraysInstancedWEBGL(i,l,0,h,0,d,0,u);let g=0;for(let v=0;v<u;v++)g+=h[v]*d[v];e.update(g,i,1)}}this.setMode=n,this.render=r,this.renderInstances=a,this.renderMultiDraw=o,this.renderMultiDrawInstances=c}function Tg(s,t,e,i){let n;function r(){if(n!==void 0)return n;if(t.has("EXT_texture_filter_anisotropic")===!0){const I=t.get("EXT_texture_filter_anisotropic");n=s.getParameter(I.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else n=0;return n}function a(I){return!(I!==ri&&i.convert(I)!==s.getParameter(s.IMPLEMENTATION_COLOR_READ_FORMAT))}function o(I){const T=I===Gs&&(t.has("EXT_color_buffer_half_float")||t.has("EXT_color_buffer_float"));return!(I!==mi&&i.convert(I)!==s.getParameter(s.IMPLEMENTATION_COLOR_READ_TYPE)&&I!==di&&!T)}function c(I){if(I==="highp"){if(s.getShaderPrecisionFormat(s.VERTEX_SHADER,s.HIGH_FLOAT).precision>0&&s.getShaderPrecisionFormat(s.FRAGMENT_SHADER,s.HIGH_FLOAT).precision>0)return"highp";I="mediump"}return I==="mediump"&&s.getShaderPrecisionFormat(s.VERTEX_SHADER,s.MEDIUM_FLOAT).precision>0&&s.getShaderPrecisionFormat(s.FRAGMENT_SHADER,s.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}let l=e.precision!==void 0?e.precision:"highp";const h=c(l);h!==l&&(console.warn("THREE.WebGLRenderer:",l,"not supported, using",h,"instead."),l=h);const u=e.logarithmicDepthBuffer===!0,d=e.reversedDepthBuffer===!0&&t.has("EXT_clip_control"),f=s.getParameter(s.MAX_TEXTURE_IMAGE_UNITS),g=s.getParameter(s.MAX_VERTEX_TEXTURE_IMAGE_UNITS),v=s.getParameter(s.MAX_TEXTURE_SIZE),m=s.getParameter(s.MAX_CUBE_MAP_TEXTURE_SIZE),p=s.getParameter(s.MAX_VERTEX_ATTRIBS),w=s.getParameter(s.MAX_VERTEX_UNIFORM_VECTORS),E=s.getParameter(s.MAX_VARYING_VECTORS),_=s.getParameter(s.MAX_FRAGMENT_UNIFORM_VECTORS),P=g>0,R=s.getParameter(s.MAX_SAMPLES);return{isWebGL2:!0,getMaxAnisotropy:r,getMaxPrecision:c,textureFormatReadable:a,textureTypeReadable:o,precision:l,logarithmicDepthBuffer:u,reversedDepthBuffer:d,maxTextures:f,maxVertexTextures:g,maxTextureSize:v,maxCubemapSize:m,maxAttributes:p,maxVertexUniforms:w,maxVaryings:E,maxFragmentUniforms:_,vertexTextures:P,maxSamples:R}}function Ag(s){const t=this;let e=null,i=0,n=!1,r=!1;const a=new nn,o=new Ot,c={value:null,needsUpdate:!1};this.uniform=c,this.numPlanes=0,this.numIntersection=0,this.init=function(u,d){const f=u.length!==0||d||i!==0||n;return n=d,i=u.length,f},this.beginShadows=function(){r=!0,h(null)},this.endShadows=function(){r=!1},this.setGlobalState=function(u,d){e=h(u,d,0)},this.setState=function(u,d,f){const g=u.clippingPlanes,v=u.clipIntersection,m=u.clipShadows,p=s.get(u);if(!n||g===null||g.length===0||r&&!m)r?h(null):l();else{const w=r?0:i,E=w*4;let _=p.clippingState||null;c.value=_,_=h(g,d,E,f);for(let P=0;P!==E;++P)_[P]=e[P];p.clippingState=_,this.numIntersection=v?this.numPlanes:0,this.numPlanes+=w}};function l(){c.value!==e&&(c.value=e,c.needsUpdate=i>0),t.numPlanes=i,t.numIntersection=0}function h(u,d,f,g){const v=u!==null?u.length:0;let m=null;if(v!==0){if(m=c.value,g!==!0||m===null){const p=f+v*4,w=d.matrixWorldInverse;o.getNormalMatrix(w),(m===null||m.length<p)&&(m=new Float32Array(p));for(let E=0,_=f;E!==v;++E,_+=4)a.copy(u[E]).applyMatrix4(w,o),a.normal.toArray(m,_),m[_+3]=a.constant}c.value=m,c.needsUpdate=!0}return t.numPlanes=v,t.numIntersection=0,m}}function Rg(s){let t=new WeakMap;function e(a,o){return o===Ja?a.mapping=es:o===ja&&(a.mapping=is),a}function i(a){if(a&&a.isTexture){const o=a.mapping;if(o===Ja||o===ja)if(t.has(a)){const c=t.get(a).texture;return e(c,a.mapping)}else{const c=a.image;if(c&&c.height>0){const l=new Kd(c.height);return l.fromEquirectangularTexture(s,a),t.set(a,l),a.addEventListener("dispose",n),e(l.texture,a.mapping)}else return null}}return a}function n(a){const o=a.target;o.removeEventListener("dispose",n);const c=t.get(o);c!==void 0&&(t.delete(o),c.dispose())}function r(){t=new WeakMap}return{get:i,dispose:r}}const Xn=4,pc=[.125,.215,.35,.446,.526,.582],an=20,Ta=new sl,mc=new Ht;let Aa=null,Ra=0,Ca=0,Pa=!1;const sn=(1+Math.sqrt(5))/2,Hn=1/sn,gc=[new C(-sn,Hn,0),new C(sn,Hn,0),new C(-Hn,0,sn),new C(Hn,0,sn),new C(0,sn,-Hn),new C(0,sn,Hn),new C(-1,1,-1),new C(1,1,-1),new C(-1,1,1),new C(1,1,1)],Cg=new C;class vc{constructor(t){this._renderer=t,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(t,e=0,i=.1,n=100,r={}){const{size:a=256,position:o=Cg}=r;Aa=this._renderer.getRenderTarget(),Ra=this._renderer.getActiveCubeFace(),Ca=this._renderer.getActiveMipmapLevel(),Pa=this._renderer.xr.enabled,this._renderer.xr.enabled=!1,this._setSize(a);const c=this._allocateTargets();return c.depthBuffer=!0,this._sceneToCubeUV(t,i,n,c,o),e>0&&this._blur(c,0,0,e),this._applyPMREM(c),this._cleanup(c),c}fromEquirectangular(t,e=null){return this._fromTexture(t,e)}fromCubemap(t,e=null){return this._fromTexture(t,e)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=yc(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=xc(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(t){this._lodMax=Math.floor(Math.log2(t)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let t=0;t<this._lodPlanes.length;t++)this._lodPlanes[t].dispose()}_cleanup(t){this._renderer.setRenderTarget(Aa,Ra,Ca),this._renderer.xr.enabled=Pa,t.scissorTest=!1,Sr(t,0,0,t.width,t.height)}_fromTexture(t,e){t.mapping===es||t.mapping===is?this._setSize(t.image.length===0?16:t.image[0].width||t.image[0].image.width):this._setSize(t.image.width/4),Aa=this._renderer.getRenderTarget(),Ra=this._renderer.getActiveCubeFace(),Ca=this._renderer.getActiveMipmapLevel(),Pa=this._renderer.xr.enabled,this._renderer.xr.enabled=!1;const i=e||this._allocateTargets();return this._textureToCubeUV(t,i),this._applyPMREM(i),this._cleanup(i),i}_allocateTargets(){const t=3*Math.max(this._cubeSize,112),e=4*this._cubeSize,i={magFilter:ui,minFilter:ui,generateMipmaps:!1,type:Gs,format:ri,colorSpace:ns,depthBuffer:!1},n=_c(t,e,i);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==t||this._pingPongRenderTarget.height!==e){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=_c(t,e,i);const{_lodMax:r}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=Pg(r)),this._blurMaterial=Ig(r,t,e)}return n}_compileMaterial(t){const e=new Vt(this._lodPlanes[0],t);this._renderer.compile(e,Ta)}_sceneToCubeUV(t,e,i,n,r){const c=new si(90,1,e,i),l=[1,-1,1,1,1,1],h=[1,1,1,-1,-1,-1],u=this._renderer,d=u.autoClear,f=u.toneMapping;u.getClearColor(mc),u.toneMapping=Wi,u.autoClear=!1,u.state.buffers.depth.getReversed()&&(u.setRenderTarget(n),u.clearDepth(),u.setRenderTarget(null));const v=new Ge({name:"PMREM.Background",side:Oe,depthWrite:!1,depthTest:!1}),m=new Vt(new $i,v);let p=!1;const w=t.background;w?w.isColor&&(v.color.copy(w),t.background=null,p=!0):(v.color.copy(mc),p=!0);for(let E=0;E<6;E++){const _=E%3;_===0?(c.up.set(0,l[E],0),c.position.set(r.x,r.y,r.z),c.lookAt(r.x+h[E],r.y,r.z)):_===1?(c.up.set(0,0,l[E]),c.position.set(r.x,r.y,r.z),c.lookAt(r.x,r.y+h[E],r.z)):(c.up.set(0,l[E],0),c.position.set(r.x,r.y,r.z),c.lookAt(r.x,r.y,r.z+h[E]));const P=this._cubeSize;Sr(n,_*P,E>2?P:0,P,P),u.setRenderTarget(n),p&&u.render(m,c),u.render(t,c)}m.geometry.dispose(),m.material.dispose(),u.toneMapping=f,u.autoClear=d,t.background=w}_textureToCubeUV(t,e){const i=this._renderer,n=t.mapping===es||t.mapping===is;n?(this._cubemapMaterial===null&&(this._cubemapMaterial=yc()),this._cubemapMaterial.uniforms.flipEnvMap.value=t.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=xc());const r=n?this._cubemapMaterial:this._equirectMaterial,a=new Vt(this._lodPlanes[0],r),o=r.uniforms;o.envMap.value=t;const c=this._cubeSize;Sr(e,0,0,3*c,2*c),i.setRenderTarget(e),i.render(a,Ta)}_applyPMREM(t){const e=this._renderer,i=e.autoClear;e.autoClear=!1;const n=this._lodPlanes.length;for(let r=1;r<n;r++){const a=Math.sqrt(this._sigmas[r]*this._sigmas[r]-this._sigmas[r-1]*this._sigmas[r-1]),o=gc[(n-r-1)%gc.length];this._blur(t,r-1,r,a,o)}e.autoClear=i}_blur(t,e,i,n,r){const a=this._pingPongRenderTarget;this._halfBlur(t,a,e,i,n,"latitudinal",r),this._halfBlur(a,t,i,i,n,"longitudinal",r)}_halfBlur(t,e,i,n,r,a,o){const c=this._renderer,l=this._blurMaterial;a!=="latitudinal"&&a!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");const h=3,u=new Vt(this._lodPlanes[n],l),d=l.uniforms,f=this._sizeLods[i]-1,g=isFinite(r)?Math.PI/(2*f):2*Math.PI/(2*an-1),v=r/g,m=isFinite(r)?1+Math.floor(h*v):an;m>an&&console.warn(`sigmaRadians, ${r}, is too large and will clip, as it requested ${m} samples when the maximum is set to ${an}`);const p=[];let w=0;for(let I=0;I<an;++I){const T=I/v,S=Math.exp(-T*T/2);p.push(S),I===0?w+=S:I<m&&(w+=2*S)}for(let I=0;I<p.length;I++)p[I]=p[I]/w;d.envMap.value=t.texture,d.samples.value=m,d.weights.value=p,d.latitudinal.value=a==="latitudinal",o&&(d.poleAxis.value=o);const{_lodMax:E}=this;d.dTheta.value=g,d.mipInt.value=E-i;const _=this._sizeLods[n],P=3*_*(n>E-Xn?n-E+Xn:0),R=4*(this._cubeSize-_);Sr(e,P,R,3*_,2*_),c.setRenderTarget(e),c.render(u,Ta)}}function Pg(s){const t=[],e=[],i=[];let n=s;const r=s-Xn+1+pc.length;for(let a=0;a<r;a++){const o=Math.pow(2,n);e.push(o);let c=1/o;a>s-Xn?c=pc[a-s+Xn-1]:a===0&&(c=0),i.push(c);const l=1/(o-2),h=-l,u=1+l,d=[h,h,u,h,u,u,h,h,u,u,h,u],f=6,g=6,v=3,m=2,p=1,w=new Float32Array(v*g*f),E=new Float32Array(m*g*f),_=new Float32Array(p*g*f);for(let R=0;R<f;R++){const I=R%3*2/3-1,T=R>2?0:-1,S=[I,T,0,I+2/3,T,0,I+2/3,T+1,0,I,T,0,I+2/3,T+1,0,I,T+1,0];w.set(S,v*g*R),E.set(d,m*g*R);const b=[R,R,R,R,R,R];_.set(b,p*g*R)}const P=new we;P.setAttribute("position",new $e(w,v)),P.setAttribute("uv",new $e(E,m)),P.setAttribute("faceIndex",new $e(_,p)),t.push(P),n>Xn&&n--}return{lodPlanes:t,sizeLods:e,sigmas:i}}function _c(s,t,e){const i=new vn(s,t,e);return i.texture.mapping=Gr,i.texture.name="PMREM.cubeUv",i.scissorTest=!0,i}function Sr(s,t,e,i,n){s.viewport.set(t,e,i,n),s.scissor.set(t,e,i,n)}function Ig(s,t,e){const i=new Float32Array(an),n=new C(0,1,0);return new Yi({name:"SphericalGaussianBlur",defines:{n:an,CUBEUV_TEXEL_WIDTH:1/t,CUBEUV_TEXEL_HEIGHT:1/e,CUBEUV_MAX_MIP:`${s}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:i},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:n}},vertexShader:rl(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;
			uniform int samples;
			uniform float weights[ n ];
			uniform bool latitudinal;
			uniform float dTheta;
			uniform float mipInt;
			uniform vec3 poleAxis;

			#define ENVMAP_TYPE_CUBE_UV
			#include <cube_uv_reflection_fragment>

			vec3 getSample( float theta, vec3 axis ) {

				float cosTheta = cos( theta );
				// Rodrigues' axis-angle rotation
				vec3 sampleDirection = vOutputDirection * cosTheta
					+ cross( axis, vOutputDirection ) * sin( theta )
					+ axis * dot( axis, vOutputDirection ) * ( 1.0 - cosTheta );

				return bilinearCubeUV( envMap, sampleDirection, mipInt );

			}

			void main() {

				vec3 axis = latitudinal ? poleAxis : cross( poleAxis, vOutputDirection );

				if ( all( equal( axis, vec3( 0.0 ) ) ) ) {

					axis = vec3( vOutputDirection.z, 0.0, - vOutputDirection.x );

				}

				axis = normalize( axis );

				gl_FragColor = vec4( 0.0, 0.0, 0.0, 1.0 );
				gl_FragColor.rgb += weights[ 0 ] * getSample( 0.0, axis );

				for ( int i = 1; i < n; i++ ) {

					if ( i >= samples ) {

						break;

					}

					float theta = dTheta * float( i );
					gl_FragColor.rgb += weights[ i ] * getSample( -1.0 * theta, axis );
					gl_FragColor.rgb += weights[ i ] * getSample( theta, axis );

				}

			}
		`,blending:Vi,depthTest:!1,depthWrite:!1})}function xc(){return new Yi({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:rl(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;

			#include <common>

			void main() {

				vec3 outputDirection = normalize( vOutputDirection );
				vec2 uv = equirectUv( outputDirection );

				gl_FragColor = vec4( texture2D ( envMap, uv ).rgb, 1.0 );

			}
		`,blending:Vi,depthTest:!1,depthWrite:!1})}function yc(){return new Yi({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:rl(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:Vi,depthTest:!1,depthWrite:!1})}function rl(){return`

		precision mediump float;
		precision mediump int;

		attribute float faceIndex;

		varying vec3 vOutputDirection;

		// RH coordinate system; PMREM face-indexing convention
		vec3 getDirection( vec2 uv, float face ) {

			uv = 2.0 * uv - 1.0;

			vec3 direction = vec3( uv, 1.0 );

			if ( face == 0.0 ) {

				direction = direction.zyx; // ( 1, v, u ) pos x

			} else if ( face == 1.0 ) {

				direction = direction.xzy;
				direction.xz *= -1.0; // ( -u, 1, -v ) pos y

			} else if ( face == 2.0 ) {

				direction.x *= -1.0; // ( -u, v, 1 ) pos z

			} else if ( face == 3.0 ) {

				direction = direction.zyx;
				direction.xz *= -1.0; // ( -1, v, -u ) neg x

			} else if ( face == 4.0 ) {

				direction = direction.xzy;
				direction.xy *= -1.0; // ( -u, -1, v ) neg y

			} else if ( face == 5.0 ) {

				direction.z *= -1.0; // ( u, v, -1 ) neg z

			}

			return direction;

		}

		void main() {

			vOutputDirection = getDirection( uv, faceIndex );
			gl_Position = vec4( position, 1.0 );

		}
	`}function Dg(s){let t=new WeakMap,e=null;function i(o){if(o&&o.isTexture){const c=o.mapping,l=c===Ja||c===ja,h=c===es||c===is;if(l||h){let u=t.get(o);const d=u!==void 0?u.texture.pmremVersion:0;if(o.isRenderTargetTexture&&o.pmremVersion!==d)return e===null&&(e=new vc(s)),u=l?e.fromEquirectangular(o,u):e.fromCubemap(o,u),u.texture.pmremVersion=o.pmremVersion,t.set(o,u),u.texture;if(u!==void 0)return u.texture;{const f=o.image;return l&&f&&f.height>0||h&&f&&n(f)?(e===null&&(e=new vc(s)),u=l?e.fromEquirectangular(o):e.fromCubemap(o),u.texture.pmremVersion=o.pmremVersion,t.set(o,u),o.addEventListener("dispose",r),u.texture):null}}}return o}function n(o){let c=0;const l=6;for(let h=0;h<l;h++)o[h]!==void 0&&c++;return c===l}function r(o){const c=o.target;c.removeEventListener("dispose",r);const l=t.get(c);l!==void 0&&(t.delete(c),l.dispose())}function a(){t=new WeakMap,e!==null&&(e.dispose(),e=null)}return{get:i,dispose:a}}function Lg(s){const t={};function e(i){if(t[i]!==void 0)return t[i];let n;switch(i){case"WEBGL_depth_texture":n=s.getExtension("WEBGL_depth_texture")||s.getExtension("MOZ_WEBGL_depth_texture")||s.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":n=s.getExtension("EXT_texture_filter_anisotropic")||s.getExtension("MOZ_EXT_texture_filter_anisotropic")||s.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":n=s.getExtension("WEBGL_compressed_texture_s3tc")||s.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||s.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":n=s.getExtension("WEBGL_compressed_texture_pvrtc")||s.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:n=s.getExtension(i)}return t[i]=n,n}return{has:function(i){return e(i)!==null},init:function(){e("EXT_color_buffer_float"),e("WEBGL_clip_cull_distance"),e("OES_texture_float_linear"),e("EXT_color_buffer_half_float"),e("WEBGL_multisampled_render_to_texture"),e("WEBGL_render_shared_exponent")},get:function(i){const n=e(i);return n===null&&Fs("THREE.WebGLRenderer: "+i+" extension not supported."),n}}}function Ng(s,t,e,i){const n={},r=new WeakMap;function a(u){const d=u.target;d.index!==null&&t.remove(d.index);for(const g in d.attributes)t.remove(d.attributes[g]);d.removeEventListener("dispose",a),delete n[d.id];const f=r.get(d);f&&(t.remove(f),r.delete(d)),i.releaseStatesOfGeometry(d),d.isInstancedBufferGeometry===!0&&delete d._maxInstanceCount,e.memory.geometries--}function o(u,d){return n[d.id]===!0||(d.addEventListener("dispose",a),n[d.id]=!0,e.memory.geometries++),d}function c(u){const d=u.attributes;for(const f in d)t.update(d[f],s.ARRAY_BUFFER)}function l(u){const d=[],f=u.index,g=u.attributes.position;let v=0;if(f!==null){const w=f.array;v=f.version;for(let E=0,_=w.length;E<_;E+=3){const P=w[E+0],R=w[E+1],I=w[E+2];d.push(P,R,R,I,I,P)}}else if(g!==void 0){const w=g.array;v=g.version;for(let E=0,_=w.length/3-1;E<_;E+=3){const P=E+0,R=E+1,I=E+2;d.push(P,R,R,I,I,P)}}else return;const m=new(ph(d)?yh:xh)(d,1);m.version=v;const p=r.get(u);p&&t.remove(p),r.set(u,m)}function h(u){const d=r.get(u);if(d){const f=u.index;f!==null&&d.version<f.version&&l(u)}else l(u);return r.get(u)}return{get:o,update:c,getWireframeAttribute:h}}function Ug(s,t,e){let i;function n(d){i=d}let r,a;function o(d){r=d.type,a=d.bytesPerElement}function c(d,f){s.drawElements(i,f,r,d*a),e.update(f,i,1)}function l(d,f,g){g!==0&&(s.drawElementsInstanced(i,f,r,d*a,g),e.update(f,i,g))}function h(d,f,g){if(g===0)return;t.get("WEBGL_multi_draw").multiDrawElementsWEBGL(i,f,0,r,d,0,g);let m=0;for(let p=0;p<g;p++)m+=f[p];e.update(m,i,1)}function u(d,f,g,v){if(g===0)return;const m=t.get("WEBGL_multi_draw");if(m===null)for(let p=0;p<d.length;p++)l(d[p]/a,f[p],v[p]);else{m.multiDrawElementsInstancedWEBGL(i,f,0,r,d,0,v,0,g);let p=0;for(let w=0;w<g;w++)p+=f[w]*v[w];e.update(p,i,1)}}this.setMode=n,this.setIndex=o,this.render=c,this.renderInstances=l,this.renderMultiDraw=h,this.renderMultiDrawInstances=u}function Og(s){const t={geometries:0,textures:0},e={frame:0,calls:0,triangles:0,points:0,lines:0};function i(r,a,o){switch(e.calls++,a){case s.TRIANGLES:e.triangles+=o*(r/3);break;case s.LINES:e.lines+=o*(r/2);break;case s.LINE_STRIP:e.lines+=o*(r-1);break;case s.LINE_LOOP:e.lines+=o*r;break;case s.POINTS:e.points+=o*r;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",a);break}}function n(){e.calls=0,e.triangles=0,e.points=0,e.lines=0}return{memory:t,render:e,programs:null,autoReset:!0,reset:n,update:i}}function Fg(s,t,e){const i=new WeakMap,n=new pe;function r(a,o,c){const l=a.morphTargetInfluences,h=o.morphAttributes.position||o.morphAttributes.normal||o.morphAttributes.color,u=h!==void 0?h.length:0;let d=i.get(o);if(d===void 0||d.count!==u){let b=function(){T.dispose(),i.delete(o),o.removeEventListener("dispose",b)};var f=b;d!==void 0&&d.texture.dispose();const g=o.morphAttributes.position!==void 0,v=o.morphAttributes.normal!==void 0,m=o.morphAttributes.color!==void 0,p=o.morphAttributes.position||[],w=o.morphAttributes.normal||[],E=o.morphAttributes.color||[];let _=0;g===!0&&(_=1),v===!0&&(_=2),m===!0&&(_=3);let P=o.attributes.position.count*_,R=1;P>t.maxTextureSize&&(R=Math.ceil(P/t.maxTextureSize),P=t.maxTextureSize);const I=new Float32Array(P*R*4*u),T=new mh(I,P,R,u);T.type=di,T.needsUpdate=!0;const S=_*4;for(let L=0;L<u;L++){const F=p[L],B=w[L],Y=E[L],W=P*R*4*L;for(let $=0;$<F.count;$++){const J=$*S;g===!0&&(n.fromBufferAttribute(F,$),I[W+J+0]=n.x,I[W+J+1]=n.y,I[W+J+2]=n.z,I[W+J+3]=0),v===!0&&(n.fromBufferAttribute(B,$),I[W+J+4]=n.x,I[W+J+5]=n.y,I[W+J+6]=n.z,I[W+J+7]=0),m===!0&&(n.fromBufferAttribute(Y,$),I[W+J+8]=n.x,I[W+J+9]=n.y,I[W+J+10]=n.z,I[W+J+11]=Y.itemSize===4?n.w:1)}}d={count:u,texture:T,size:new ht(P,R)},i.set(o,d),o.addEventListener("dispose",b)}if(a.isInstancedMesh===!0&&a.morphTexture!==null)c.getUniforms().setValue(s,"morphTexture",a.morphTexture,e);else{let g=0;for(let m=0;m<l.length;m++)g+=l[m];const v=o.morphTargetsRelative?1:1-g;c.getUniforms().setValue(s,"morphTargetBaseInfluence",v),c.getUniforms().setValue(s,"morphTargetInfluences",l)}c.getUniforms().setValue(s,"morphTargetsTexture",d.texture,e),c.getUniforms().setValue(s,"morphTargetsTextureSize",d.size)}return{update:r}}function kg(s,t,e,i){let n=new WeakMap;function r(c){const l=i.render.frame,h=c.geometry,u=t.get(c,h);if(n.get(u)!==l&&(t.update(u),n.set(u,l)),c.isInstancedMesh&&(c.hasEventListener("dispose",o)===!1&&c.addEventListener("dispose",o),n.get(c)!==l&&(e.update(c.instanceMatrix,s.ARRAY_BUFFER),c.instanceColor!==null&&e.update(c.instanceColor,s.ARRAY_BUFFER),n.set(c,l))),c.isSkinnedMesh){const d=c.skeleton;n.get(d)!==l&&(d.update(),n.set(d,l))}return u}function a(){n=new WeakMap}function o(c){const l=c.target;l.removeEventListener("dispose",o),e.remove(l.instanceMatrix),l.instanceColor!==null&&e.remove(l.instanceColor)}return{update:r,dispose:a}}const Hh=new Ce,Mc=new Ah(1,1),Gh=new mh,Vh=new Dd,Wh=new bh,Sc=[],bc=[],Ec=new Float32Array(16),wc=new Float32Array(9),Tc=new Float32Array(4);function cs(s,t,e){const i=s[0];if(i<=0||i>0)return s;const n=t*e;let r=Sc[n];if(r===void 0&&(r=new Float32Array(n),Sc[n]=r),t!==0){i.toArray(r,0);for(let a=1,o=0;a!==t;++a)o+=e,s[a].toArray(r,o)}return r}function ye(s,t){if(s.length!==t.length)return!1;for(let e=0,i=s.length;e<i;e++)if(s[e]!==t[e])return!1;return!0}function Me(s,t){for(let e=0,i=t.length;e<i;e++)s[e]=t[e]}function Vr(s,t){let e=bc[t];e===void 0&&(e=new Int32Array(t),bc[t]=e);for(let i=0;i!==t;++i)e[i]=s.allocateTextureUnit();return e}function zg(s,t){const e=this.cache;e[0]!==t&&(s.uniform1f(this.addr,t),e[0]=t)}function Bg(s,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(s.uniform2f(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(ye(e,t))return;s.uniform2fv(this.addr,t),Me(e,t)}}function Hg(s,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(s.uniform3f(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else if(t.r!==void 0)(e[0]!==t.r||e[1]!==t.g||e[2]!==t.b)&&(s.uniform3f(this.addr,t.r,t.g,t.b),e[0]=t.r,e[1]=t.g,e[2]=t.b);else{if(ye(e,t))return;s.uniform3fv(this.addr,t),Me(e,t)}}function Gg(s,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(s.uniform4f(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(ye(e,t))return;s.uniform4fv(this.addr,t),Me(e,t)}}function Vg(s,t){const e=this.cache,i=t.elements;if(i===void 0){if(ye(e,t))return;s.uniformMatrix2fv(this.addr,!1,t),Me(e,t)}else{if(ye(e,i))return;Tc.set(i),s.uniformMatrix2fv(this.addr,!1,Tc),Me(e,i)}}function Wg(s,t){const e=this.cache,i=t.elements;if(i===void 0){if(ye(e,t))return;s.uniformMatrix3fv(this.addr,!1,t),Me(e,t)}else{if(ye(e,i))return;wc.set(i),s.uniformMatrix3fv(this.addr,!1,wc),Me(e,i)}}function Xg(s,t){const e=this.cache,i=t.elements;if(i===void 0){if(ye(e,t))return;s.uniformMatrix4fv(this.addr,!1,t),Me(e,t)}else{if(ye(e,i))return;Ec.set(i),s.uniformMatrix4fv(this.addr,!1,Ec),Me(e,i)}}function $g(s,t){const e=this.cache;e[0]!==t&&(s.uniform1i(this.addr,t),e[0]=t)}function Yg(s,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(s.uniform2i(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(ye(e,t))return;s.uniform2iv(this.addr,t),Me(e,t)}}function qg(s,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(s.uniform3i(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if(ye(e,t))return;s.uniform3iv(this.addr,t),Me(e,t)}}function Kg(s,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(s.uniform4i(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(ye(e,t))return;s.uniform4iv(this.addr,t),Me(e,t)}}function Zg(s,t){const e=this.cache;e[0]!==t&&(s.uniform1ui(this.addr,t),e[0]=t)}function Jg(s,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(s.uniform2ui(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(ye(e,t))return;s.uniform2uiv(this.addr,t),Me(e,t)}}function jg(s,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(s.uniform3ui(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if(ye(e,t))return;s.uniform3uiv(this.addr,t),Me(e,t)}}function Qg(s,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(s.uniform4ui(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(ye(e,t))return;s.uniform4uiv(this.addr,t),Me(e,t)}}function t0(s,t,e){const i=this.cache,n=e.allocateTextureUnit();i[0]!==n&&(s.uniform1i(this.addr,n),i[0]=n);let r;this.type===s.SAMPLER_2D_SHADOW?(Mc.compareFunction=dh,r=Mc):r=Hh,e.setTexture2D(t||r,n)}function e0(s,t,e){const i=this.cache,n=e.allocateTextureUnit();i[0]!==n&&(s.uniform1i(this.addr,n),i[0]=n),e.setTexture3D(t||Vh,n)}function i0(s,t,e){const i=this.cache,n=e.allocateTextureUnit();i[0]!==n&&(s.uniform1i(this.addr,n),i[0]=n),e.setTextureCube(t||Wh,n)}function n0(s,t,e){const i=this.cache,n=e.allocateTextureUnit();i[0]!==n&&(s.uniform1i(this.addr,n),i[0]=n),e.setTexture2DArray(t||Gh,n)}function s0(s){switch(s){case 5126:return zg;case 35664:return Bg;case 35665:return Hg;case 35666:return Gg;case 35674:return Vg;case 35675:return Wg;case 35676:return Xg;case 5124:case 35670:return $g;case 35667:case 35671:return Yg;case 35668:case 35672:return qg;case 35669:case 35673:return Kg;case 5125:return Zg;case 36294:return Jg;case 36295:return jg;case 36296:return Qg;case 35678:case 36198:case 36298:case 36306:case 35682:return t0;case 35679:case 36299:case 36307:return e0;case 35680:case 36300:case 36308:case 36293:return i0;case 36289:case 36303:case 36311:case 36292:return n0}}function r0(s,t){s.uniform1fv(this.addr,t)}function a0(s,t){const e=cs(t,this.size,2);s.uniform2fv(this.addr,e)}function o0(s,t){const e=cs(t,this.size,3);s.uniform3fv(this.addr,e)}function l0(s,t){const e=cs(t,this.size,4);s.uniform4fv(this.addr,e)}function c0(s,t){const e=cs(t,this.size,4);s.uniformMatrix2fv(this.addr,!1,e)}function h0(s,t){const e=cs(t,this.size,9);s.uniformMatrix3fv(this.addr,!1,e)}function u0(s,t){const e=cs(t,this.size,16);s.uniformMatrix4fv(this.addr,!1,e)}function d0(s,t){s.uniform1iv(this.addr,t)}function f0(s,t){s.uniform2iv(this.addr,t)}function p0(s,t){s.uniform3iv(this.addr,t)}function m0(s,t){s.uniform4iv(this.addr,t)}function g0(s,t){s.uniform1uiv(this.addr,t)}function v0(s,t){s.uniform2uiv(this.addr,t)}function _0(s,t){s.uniform3uiv(this.addr,t)}function x0(s,t){s.uniform4uiv(this.addr,t)}function y0(s,t,e){const i=this.cache,n=t.length,r=Vr(e,n);ye(i,r)||(s.uniform1iv(this.addr,r),Me(i,r));for(let a=0;a!==n;++a)e.setTexture2D(t[a]||Hh,r[a])}function M0(s,t,e){const i=this.cache,n=t.length,r=Vr(e,n);ye(i,r)||(s.uniform1iv(this.addr,r),Me(i,r));for(let a=0;a!==n;++a)e.setTexture3D(t[a]||Vh,r[a])}function S0(s,t,e){const i=this.cache,n=t.length,r=Vr(e,n);ye(i,r)||(s.uniform1iv(this.addr,r),Me(i,r));for(let a=0;a!==n;++a)e.setTextureCube(t[a]||Wh,r[a])}function b0(s,t,e){const i=this.cache,n=t.length,r=Vr(e,n);ye(i,r)||(s.uniform1iv(this.addr,r),Me(i,r));for(let a=0;a!==n;++a)e.setTexture2DArray(t[a]||Gh,r[a])}function E0(s){switch(s){case 5126:return r0;case 35664:return a0;case 35665:return o0;case 35666:return l0;case 35674:return c0;case 35675:return h0;case 35676:return u0;case 5124:case 35670:return d0;case 35667:case 35671:return f0;case 35668:case 35672:return p0;case 35669:case 35673:return m0;case 5125:return g0;case 36294:return v0;case 36295:return _0;case 36296:return x0;case 35678:case 36198:case 36298:case 36306:case 35682:return y0;case 35679:case 36299:case 36307:return M0;case 35680:case 36300:case 36308:case 36293:return S0;case 36289:case 36303:case 36311:case 36292:return b0}}class w0{constructor(t,e,i){this.id=t,this.addr=i,this.cache=[],this.type=e.type,this.setValue=s0(e.type)}}class T0{constructor(t,e,i){this.id=t,this.addr=i,this.cache=[],this.type=e.type,this.size=e.size,this.setValue=E0(e.type)}}class A0{constructor(t){this.id=t,this.seq=[],this.map={}}setValue(t,e,i){const n=this.seq;for(let r=0,a=n.length;r!==a;++r){const o=n[r];o.setValue(t,e[o.id],i)}}}const Ia=/(\w+)(\])?(\[|\.)?/g;function Ac(s,t){s.seq.push(t),s.map[t.id]=t}function R0(s,t,e){const i=s.name,n=i.length;for(Ia.lastIndex=0;;){const r=Ia.exec(i),a=Ia.lastIndex;let o=r[1];const c=r[2]==="]",l=r[3];if(c&&(o=o|0),l===void 0||l==="["&&a+2===n){Ac(e,l===void 0?new w0(o,s,t):new T0(o,s,t));break}else{let u=e.map[o];u===void 0&&(u=new A0(o),Ac(e,u)),e=u}}}class Ir{constructor(t,e){this.seq=[],this.map={};const i=t.getProgramParameter(e,t.ACTIVE_UNIFORMS);for(let n=0;n<i;++n){const r=t.getActiveUniform(e,n),a=t.getUniformLocation(e,r.name);R0(r,a,this)}}setValue(t,e,i,n){const r=this.map[e];r!==void 0&&r.setValue(t,i,n)}setOptional(t,e,i){const n=e[i];n!==void 0&&this.setValue(t,i,n)}static upload(t,e,i,n){for(let r=0,a=e.length;r!==a;++r){const o=e[r],c=i[o.id];c.needsUpdate!==!1&&o.setValue(t,c.value,n)}}static seqWithValue(t,e){const i=[];for(let n=0,r=t.length;n!==r;++n){const a=t[n];a.id in e&&i.push(a)}return i}}function Rc(s,t,e){const i=s.createShader(t);return s.shaderSource(i,e),s.compileShader(i),i}const C0=37297;let P0=0;function I0(s,t){const e=s.split(`
`),i=[],n=Math.max(t-6,0),r=Math.min(t+6,e.length);for(let a=n;a<r;a++){const o=a+1;i.push(`${o===t?">":" "} ${o}: ${e[a]}`)}return i.join(`
`)}const Cc=new Ot;function D0(s){qt._getMatrix(Cc,qt.workingColorSpace,s);const t=`mat3( ${Cc.elements.map(e=>e.toFixed(4))} )`;switch(qt.getTransfer(s)){case Nr:return[t,"LinearTransferOETF"];case te:return[t,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space: ",s),[t,"LinearTransferOETF"]}}function Pc(s,t,e){const i=s.getShaderParameter(t,s.COMPILE_STATUS),r=(s.getShaderInfoLog(t)||"").trim();if(i&&r==="")return"";const a=/ERROR: 0:(\d+)/.exec(r);if(a){const o=parseInt(a[1]);return e.toUpperCase()+`

`+r+`

`+I0(s.getShaderSource(t),o)}else return r}function L0(s,t){const e=D0(t);return[`vec4 ${s}( vec4 value ) {`,`	return ${e[1]}( vec4( value.rgb * ${e[0]}, value.a ) );`,"}"].join(`
`)}function N0(s,t){let e;switch(t){case od:e="Linear";break;case ld:e="Reinhard";break;case cd:e="Cineon";break;case ih:e="ACESFilmic";break;case ud:e="AgX";break;case dd:e="Neutral";break;case hd:e="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",t),e="Linear"}return"vec3 "+s+"( vec3 color ) { return "+e+"ToneMapping( color ); }"}const br=new C;function U0(){qt.getLuminanceCoefficients(br);const s=br.x.toFixed(4),t=br.y.toFixed(4),e=br.z.toFixed(4);return["float luminance( const in vec3 rgb ) {",`	const vec3 weights = vec3( ${s}, ${t}, ${e} );`,"	return dot( weights, rgb );","}"].join(`
`)}function O0(s){return[s.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":"",s.extensionMultiDraw?"#extension GL_ANGLE_multi_draw : require":""].filter(Ts).join(`
`)}function F0(s){const t=[];for(const e in s){const i=s[e];i!==!1&&t.push("#define "+e+" "+i)}return t.join(`
`)}function k0(s,t){const e={},i=s.getProgramParameter(t,s.ACTIVE_ATTRIBUTES);for(let n=0;n<i;n++){const r=s.getActiveAttrib(t,n),a=r.name;let o=1;r.type===s.FLOAT_MAT2&&(o=2),r.type===s.FLOAT_MAT3&&(o=3),r.type===s.FLOAT_MAT4&&(o=4),e[a]={type:r.type,location:s.getAttribLocation(t,a),locationSize:o}}return e}function Ts(s){return s!==""}function Ic(s,t){const e=t.numSpotLightShadows+t.numSpotLightMaps-t.numSpotLightShadowsWithMaps;return s.replace(/NUM_DIR_LIGHTS/g,t.numDirLights).replace(/NUM_SPOT_LIGHTS/g,t.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,t.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,e).replace(/NUM_RECT_AREA_LIGHTS/g,t.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,t.numPointLights).replace(/NUM_HEMI_LIGHTS/g,t.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,t.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,t.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,t.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,t.numPointLightShadows)}function Dc(s,t){return s.replace(/NUM_CLIPPING_PLANES/g,t.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,t.numClippingPlanes-t.numClipIntersection)}const z0=/^[ \t]*#include +<([\w\d./]+)>/gm;function Fo(s){return s.replace(z0,H0)}const B0=new Map;function H0(s,t){let e=kt[t];if(e===void 0){const i=B0.get(t);if(i!==void 0)e=kt[i],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',t,i);else throw new Error("Can not resolve #include <"+t+">")}return Fo(e)}const G0=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function Lc(s){return s.replace(G0,V0)}function V0(s,t,e,i){let n="";for(let r=parseInt(t);r<parseInt(e);r++)n+=i.replace(/\[\s*i\s*\]/g,"[ "+r+" ]").replace(/UNROLLED_LOOP_INDEX/g,r);return n}function Nc(s){let t=`precision ${s.precision} float;
	precision ${s.precision} int;
	precision ${s.precision} sampler2D;
	precision ${s.precision} samplerCube;
	precision ${s.precision} sampler3D;
	precision ${s.precision} sampler2DArray;
	precision ${s.precision} sampler2DShadow;
	precision ${s.precision} samplerCubeShadow;
	precision ${s.precision} sampler2DArrayShadow;
	precision ${s.precision} isampler2D;
	precision ${s.precision} isampler3D;
	precision ${s.precision} isamplerCube;
	precision ${s.precision} isampler2DArray;
	precision ${s.precision} usampler2D;
	precision ${s.precision} usampler3D;
	precision ${s.precision} usamplerCube;
	precision ${s.precision} usampler2DArray;
	`;return s.precision==="highp"?t+=`
#define HIGH_PRECISION`:s.precision==="mediump"?t+=`
#define MEDIUM_PRECISION`:s.precision==="lowp"&&(t+=`
#define LOW_PRECISION`),t}function W0(s){let t="SHADOWMAP_TYPE_BASIC";return s.shadowMapType===Qc?t="SHADOWMAP_TYPE_PCF":s.shadowMapType===th?t="SHADOWMAP_TYPE_PCF_SOFT":s.shadowMapType===wi&&(t="SHADOWMAP_TYPE_VSM"),t}function X0(s){let t="ENVMAP_TYPE_CUBE";if(s.envMap)switch(s.envMapMode){case es:case is:t="ENVMAP_TYPE_CUBE";break;case Gr:t="ENVMAP_TYPE_CUBE_UV";break}return t}function $0(s){let t="ENVMAP_MODE_REFLECTION";return s.envMap&&s.envMapMode===is&&(t="ENVMAP_MODE_REFRACTION"),t}function Y0(s){let t="ENVMAP_BLENDING_NONE";if(s.envMap)switch(s.combine){case eh:t="ENVMAP_BLENDING_MULTIPLY";break;case rd:t="ENVMAP_BLENDING_MIX";break;case ad:t="ENVMAP_BLENDING_ADD";break}return t}function q0(s){const t=s.envMapCubeUVHeight;if(t===null)return null;const e=Math.log2(t)-2,i=1/t;return{texelWidth:1/(3*Math.max(Math.pow(2,e),112)),texelHeight:i,maxMip:e}}function K0(s,t,e,i){const n=s.getContext(),r=e.defines;let a=e.vertexShader,o=e.fragmentShader;const c=W0(e),l=X0(e),h=$0(e),u=Y0(e),d=q0(e),f=O0(e),g=F0(r),v=n.createProgram();let m,p,w=e.glslVersion?"#version "+e.glslVersion+`
`:"";e.isRawShaderMaterial?(m=["#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g].filter(Ts).join(`
`),m.length>0&&(m+=`
`),p=["#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g].filter(Ts).join(`
`),p.length>0&&(p+=`
`)):(m=[Nc(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g,e.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",e.batching?"#define USE_BATCHING":"",e.batchingColor?"#define USE_BATCHING_COLOR":"",e.instancing?"#define USE_INSTANCING":"",e.instancingColor?"#define USE_INSTANCING_COLOR":"",e.instancingMorph?"#define USE_INSTANCING_MORPH":"",e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.map?"#define USE_MAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+h:"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.displacementMap?"#define USE_DISPLACEMENTMAP":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.mapUv?"#define MAP_UV "+e.mapUv:"",e.alphaMapUv?"#define ALPHAMAP_UV "+e.alphaMapUv:"",e.lightMapUv?"#define LIGHTMAP_UV "+e.lightMapUv:"",e.aoMapUv?"#define AOMAP_UV "+e.aoMapUv:"",e.emissiveMapUv?"#define EMISSIVEMAP_UV "+e.emissiveMapUv:"",e.bumpMapUv?"#define BUMPMAP_UV "+e.bumpMapUv:"",e.normalMapUv?"#define NORMALMAP_UV "+e.normalMapUv:"",e.displacementMapUv?"#define DISPLACEMENTMAP_UV "+e.displacementMapUv:"",e.metalnessMapUv?"#define METALNESSMAP_UV "+e.metalnessMapUv:"",e.roughnessMapUv?"#define ROUGHNESSMAP_UV "+e.roughnessMapUv:"",e.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+e.anisotropyMapUv:"",e.clearcoatMapUv?"#define CLEARCOATMAP_UV "+e.clearcoatMapUv:"",e.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+e.clearcoatNormalMapUv:"",e.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+e.clearcoatRoughnessMapUv:"",e.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+e.iridescenceMapUv:"",e.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+e.iridescenceThicknessMapUv:"",e.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+e.sheenColorMapUv:"",e.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+e.sheenRoughnessMapUv:"",e.specularMapUv?"#define SPECULARMAP_UV "+e.specularMapUv:"",e.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+e.specularColorMapUv:"",e.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+e.specularIntensityMapUv:"",e.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+e.transmissionMapUv:"",e.thicknessMapUv?"#define THICKNESSMAP_UV "+e.thicknessMapUv:"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.flatShading?"#define FLAT_SHADED":"",e.skinning?"#define USE_SKINNING":"",e.morphTargets?"#define USE_MORPHTARGETS":"",e.morphNormals&&e.flatShading===!1?"#define USE_MORPHNORMALS":"",e.morphColors?"#define USE_MORPHCOLORS":"",e.morphTargetsCount>0?"#define MORPHTARGETS_TEXTURE_STRIDE "+e.morphTextureStride:"",e.morphTargetsCount>0?"#define MORPHTARGETS_COUNT "+e.morphTargetsCount:"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+c:"",e.sizeAttenuation?"#define USE_SIZEATTENUATION":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.logarithmicDepthBuffer?"#define USE_LOGARITHMIC_DEPTH_BUFFER":"",e.reversedDepthBuffer?"#define USE_REVERSED_DEPTH_BUFFER":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","#ifdef USE_INSTANCING_MORPH","	uniform sampler2D morphTexture;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(Ts).join(`
`),p=[Nc(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g,e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.alphaToCoverage?"#define ALPHA_TO_COVERAGE":"",e.map?"#define USE_MAP":"",e.matcap?"#define USE_MATCAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+l:"",e.envMap?"#define "+h:"",e.envMap?"#define "+u:"",d?"#define CUBEUV_TEXEL_WIDTH "+d.texelWidth:"",d?"#define CUBEUV_TEXEL_HEIGHT "+d.texelHeight:"",d?"#define CUBEUV_MAX_MIP "+d.maxMip+".0":"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoat?"#define USE_CLEARCOAT":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.dispersion?"#define USE_DISPERSION":"",e.iridescence?"#define USE_IRIDESCENCE":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaTest?"#define USE_ALPHATEST":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.sheen?"#define USE_SHEEN":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors||e.instancingColor||e.batchingColor?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.gradientMap?"#define USE_GRADIENTMAP":"",e.flatShading?"#define FLAT_SHADED":"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+c:"",e.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",e.decodeVideoTextureEmissive?"#define DECODE_VIDEO_TEXTURE_EMISSIVE":"",e.logarithmicDepthBuffer?"#define USE_LOGARITHMIC_DEPTH_BUFFER":"",e.reversedDepthBuffer?"#define USE_REVERSED_DEPTH_BUFFER":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",e.toneMapping!==Wi?"#define TONE_MAPPING":"",e.toneMapping!==Wi?kt.tonemapping_pars_fragment:"",e.toneMapping!==Wi?N0("toneMapping",e.toneMapping):"",e.dithering?"#define DITHERING":"",e.opaque?"#define OPAQUE":"",kt.colorspace_pars_fragment,L0("linearToOutputTexel",e.outputColorSpace),U0(),e.useDepthPacking?"#define DEPTH_PACKING "+e.depthPacking:"",`
`].filter(Ts).join(`
`)),a=Fo(a),a=Ic(a,e),a=Dc(a,e),o=Fo(o),o=Ic(o,e),o=Dc(o,e),a=Lc(a),o=Lc(o),e.isRawShaderMaterial!==!0&&(w=`#version 300 es
`,m=[f,"#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+m,p=["#define varying in",e.glslVersion===Dl?"":"layout(location = 0) out highp vec4 pc_fragColor;",e.glslVersion===Dl?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+p);const E=w+m+a,_=w+p+o,P=Rc(n,n.VERTEX_SHADER,E),R=Rc(n,n.FRAGMENT_SHADER,_);n.attachShader(v,P),n.attachShader(v,R),e.index0AttributeName!==void 0?n.bindAttribLocation(v,0,e.index0AttributeName):e.morphTargets===!0&&n.bindAttribLocation(v,0,"position"),n.linkProgram(v);function I(L){if(s.debug.checkShaderErrors){const F=n.getProgramInfoLog(v)||"",B=n.getShaderInfoLog(P)||"",Y=n.getShaderInfoLog(R)||"",W=F.trim(),$=B.trim(),J=Y.trim();let G=!0,at=!0;if(n.getProgramParameter(v,n.LINK_STATUS)===!1)if(G=!1,typeof s.debug.onShaderError=="function")s.debug.onShaderError(n,v,P,R);else{const ut=Pc(n,P,"vertex"),Et=Pc(n,R,"fragment");console.error("THREE.WebGLProgram: Shader Error "+n.getError()+" - VALIDATE_STATUS "+n.getProgramParameter(v,n.VALIDATE_STATUS)+`

Material Name: `+L.name+`
Material Type: `+L.type+`

Program Info Log: `+W+`
`+ut+`
`+Et)}else W!==""?console.warn("THREE.WebGLProgram: Program Info Log:",W):($===""||J==="")&&(at=!1);at&&(L.diagnostics={runnable:G,programLog:W,vertexShader:{log:$,prefix:m},fragmentShader:{log:J,prefix:p}})}n.deleteShader(P),n.deleteShader(R),T=new Ir(n,v),S=k0(n,v)}let T;this.getUniforms=function(){return T===void 0&&I(this),T};let S;this.getAttributes=function(){return S===void 0&&I(this),S};let b=e.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return b===!1&&(b=n.getProgramParameter(v,C0)),b},this.destroy=function(){i.releaseStatesOfProgram(this),n.deleteProgram(v),this.program=void 0},this.type=e.shaderType,this.name=e.shaderName,this.id=P0++,this.cacheKey=t,this.usedTimes=1,this.program=v,this.vertexShader=P,this.fragmentShader=R,this}let Z0=0;class J0{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(t){const e=t.vertexShader,i=t.fragmentShader,n=this._getShaderStage(e),r=this._getShaderStage(i),a=this._getShaderCacheForMaterial(t);return a.has(n)===!1&&(a.add(n),n.usedTimes++),a.has(r)===!1&&(a.add(r),r.usedTimes++),this}remove(t){const e=this.materialCache.get(t);for(const i of e)i.usedTimes--,i.usedTimes===0&&this.shaderCache.delete(i.code);return this.materialCache.delete(t),this}getVertexShaderID(t){return this._getShaderStage(t.vertexShader).id}getFragmentShaderID(t){return this._getShaderStage(t.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(t){const e=this.materialCache;let i=e.get(t);return i===void 0&&(i=new Set,e.set(t,i)),i}_getShaderStage(t){const e=this.shaderCache;let i=e.get(t);return i===void 0&&(i=new j0(t),e.set(t,i)),i}}class j0{constructor(t){this.id=Z0++,this.code=t,this.usedTimes=0}}function Q0(s,t,e,i,n,r,a){const o=new vh,c=new J0,l=new Set,h=[],u=n.logarithmicDepthBuffer,d=n.vertexTextures;let f=n.precision;const g={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function v(S){return l.add(S),S===0?"uv":`uv${S}`}function m(S,b,L,F,B){const Y=F.fog,W=B.geometry,$=S.isMeshStandardMaterial?F.environment:null,J=(S.isMeshStandardMaterial?e:t).get(S.envMap||$),G=J&&J.mapping===Gr?J.image.height:null,at=g[S.type];S.precision!==null&&(f=n.getMaxPrecision(S.precision),f!==S.precision&&console.warn("THREE.WebGLProgram.getParameters:",S.precision,"not supported, using",f,"instead."));const ut=W.morphAttributes.position||W.morphAttributes.normal||W.morphAttributes.color,Et=ut!==void 0?ut.length:0;let Gt=0;W.morphAttributes.position!==void 0&&(Gt=1),W.morphAttributes.normal!==void 0&&(Gt=2),W.morphAttributes.color!==void 0&&(Gt=3);let se,oe,Kt,q;if(at){const Zt=ci[at];se=Zt.vertexShader,oe=Zt.fragmentShader}else se=S.vertexShader,oe=S.fragmentShader,c.update(S),Kt=c.getVertexShaderID(S),q=c.getFragmentShaderID(S);const j=s.getRenderTarget(),pt=s.state.buffers.depth.getReversed(),Dt=B.isInstancedMesh===!0,bt=B.isBatchedMesh===!0,$t=!!S.map,Te=!!S.matcap,D=!!J,le=!!S.aoMap,Nt=!!S.lightMap,Pt=!!S.bumpMap,vt=!!S.normalMap,ce=!!S.displacementMap,_t=!!S.emissiveMap,Ft=!!S.metalnessMap,Se=!!S.roughnessMap,me=S.anisotropy>0,A=S.clearcoat>0,x=S.dispersion>0,k=S.iridescence>0,X=S.sheen>0,Z=S.transmission>0,V=me&&!!S.anisotropyMap,St=A&&!!S.clearcoatMap,nt=A&&!!S.clearcoatNormalMap,xt=A&&!!S.clearcoatRoughnessMap,yt=k&&!!S.iridescenceMap,et=k&&!!S.iridescenceThicknessMap,ct=X&&!!S.sheenColorMap,Ct=X&&!!S.sheenRoughnessMap,Mt=!!S.specularMap,ot=!!S.specularColorMap,Ut=!!S.specularIntensityMap,N=Z&&!!S.transmissionMap,it=Z&&!!S.thicknessMap,st=!!S.gradientMap,ft=!!S.alphaMap,Q=S.alphaTest>0,K=!!S.alphaHash,gt=!!S.extensions;let Lt=Wi;S.toneMapped&&(j===null||j.isXRRenderTarget===!0)&&(Lt=s.toneMapping);const re={shaderID:at,shaderType:S.type,shaderName:S.name,vertexShader:se,fragmentShader:oe,defines:S.defines,customVertexShaderID:Kt,customFragmentShaderID:q,isRawShaderMaterial:S.isRawShaderMaterial===!0,glslVersion:S.glslVersion,precision:f,batching:bt,batchingColor:bt&&B._colorsTexture!==null,instancing:Dt,instancingColor:Dt&&B.instanceColor!==null,instancingMorph:Dt&&B.morphTexture!==null,supportsVertexTextures:d,outputColorSpace:j===null?s.outputColorSpace:j.isXRRenderTarget===!0?j.texture.colorSpace:ns,alphaToCoverage:!!S.alphaToCoverage,map:$t,matcap:Te,envMap:D,envMapMode:D&&J.mapping,envMapCubeUVHeight:G,aoMap:le,lightMap:Nt,bumpMap:Pt,normalMap:vt,displacementMap:d&&ce,emissiveMap:_t,normalMapObjectSpace:vt&&S.normalMapType===gd,normalMapTangentSpace:vt&&S.normalMapType===uh,metalnessMap:Ft,roughnessMap:Se,anisotropy:me,anisotropyMap:V,clearcoat:A,clearcoatMap:St,clearcoatNormalMap:nt,clearcoatRoughnessMap:xt,dispersion:x,iridescence:k,iridescenceMap:yt,iridescenceThicknessMap:et,sheen:X,sheenColorMap:ct,sheenRoughnessMap:Ct,specularMap:Mt,specularColorMap:ot,specularIntensityMap:Ut,transmission:Z,transmissionMap:N,thicknessMap:it,gradientMap:st,opaque:S.transparent===!1&&S.blending===Jn&&S.alphaToCoverage===!1,alphaMap:ft,alphaTest:Q,alphaHash:K,combine:S.combine,mapUv:$t&&v(S.map.channel),aoMapUv:le&&v(S.aoMap.channel),lightMapUv:Nt&&v(S.lightMap.channel),bumpMapUv:Pt&&v(S.bumpMap.channel),normalMapUv:vt&&v(S.normalMap.channel),displacementMapUv:ce&&v(S.displacementMap.channel),emissiveMapUv:_t&&v(S.emissiveMap.channel),metalnessMapUv:Ft&&v(S.metalnessMap.channel),roughnessMapUv:Se&&v(S.roughnessMap.channel),anisotropyMapUv:V&&v(S.anisotropyMap.channel),clearcoatMapUv:St&&v(S.clearcoatMap.channel),clearcoatNormalMapUv:nt&&v(S.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:xt&&v(S.clearcoatRoughnessMap.channel),iridescenceMapUv:yt&&v(S.iridescenceMap.channel),iridescenceThicknessMapUv:et&&v(S.iridescenceThicknessMap.channel),sheenColorMapUv:ct&&v(S.sheenColorMap.channel),sheenRoughnessMapUv:Ct&&v(S.sheenRoughnessMap.channel),specularMapUv:Mt&&v(S.specularMap.channel),specularColorMapUv:ot&&v(S.specularColorMap.channel),specularIntensityMapUv:Ut&&v(S.specularIntensityMap.channel),transmissionMapUv:N&&v(S.transmissionMap.channel),thicknessMapUv:it&&v(S.thicknessMap.channel),alphaMapUv:ft&&v(S.alphaMap.channel),vertexTangents:!!W.attributes.tangent&&(vt||me),vertexColors:S.vertexColors,vertexAlphas:S.vertexColors===!0&&!!W.attributes.color&&W.attributes.color.itemSize===4,pointsUvs:B.isPoints===!0&&!!W.attributes.uv&&($t||ft),fog:!!Y,useFog:S.fog===!0,fogExp2:!!Y&&Y.isFogExp2,flatShading:S.flatShading===!0&&S.wireframe===!1,sizeAttenuation:S.sizeAttenuation===!0,logarithmicDepthBuffer:u,reversedDepthBuffer:pt,skinning:B.isSkinnedMesh===!0,morphTargets:W.morphAttributes.position!==void 0,morphNormals:W.morphAttributes.normal!==void 0,morphColors:W.morphAttributes.color!==void 0,morphTargetsCount:Et,morphTextureStride:Gt,numDirLights:b.directional.length,numPointLights:b.point.length,numSpotLights:b.spot.length,numSpotLightMaps:b.spotLightMap.length,numRectAreaLights:b.rectArea.length,numHemiLights:b.hemi.length,numDirLightShadows:b.directionalShadowMap.length,numPointLightShadows:b.pointShadowMap.length,numSpotLightShadows:b.spotShadowMap.length,numSpotLightShadowsWithMaps:b.numSpotLightShadowsWithMaps,numLightProbes:b.numLightProbes,numClippingPlanes:a.numPlanes,numClipIntersection:a.numIntersection,dithering:S.dithering,shadowMapEnabled:s.shadowMap.enabled&&L.length>0,shadowMapType:s.shadowMap.type,toneMapping:Lt,decodeVideoTexture:$t&&S.map.isVideoTexture===!0&&qt.getTransfer(S.map.colorSpace)===te,decodeVideoTextureEmissive:_t&&S.emissiveMap.isVideoTexture===!0&&qt.getTransfer(S.emissiveMap.colorSpace)===te,premultipliedAlpha:S.premultipliedAlpha,doubleSided:S.side===Ne,flipSided:S.side===Oe,useDepthPacking:S.depthPacking>=0,depthPacking:S.depthPacking||0,index0AttributeName:S.index0AttributeName,extensionClipCullDistance:gt&&S.extensions.clipCullDistance===!0&&i.has("WEBGL_clip_cull_distance"),extensionMultiDraw:(gt&&S.extensions.multiDraw===!0||bt)&&i.has("WEBGL_multi_draw"),rendererExtensionParallelShaderCompile:i.has("KHR_parallel_shader_compile"),customProgramCacheKey:S.customProgramCacheKey()};return re.vertexUv1s=l.has(1),re.vertexUv2s=l.has(2),re.vertexUv3s=l.has(3),l.clear(),re}function p(S){const b=[];if(S.shaderID?b.push(S.shaderID):(b.push(S.customVertexShaderID),b.push(S.customFragmentShaderID)),S.defines!==void 0)for(const L in S.defines)b.push(L),b.push(S.defines[L]);return S.isRawShaderMaterial===!1&&(w(b,S),E(b,S),b.push(s.outputColorSpace)),b.push(S.customProgramCacheKey),b.join()}function w(S,b){S.push(b.precision),S.push(b.outputColorSpace),S.push(b.envMapMode),S.push(b.envMapCubeUVHeight),S.push(b.mapUv),S.push(b.alphaMapUv),S.push(b.lightMapUv),S.push(b.aoMapUv),S.push(b.bumpMapUv),S.push(b.normalMapUv),S.push(b.displacementMapUv),S.push(b.emissiveMapUv),S.push(b.metalnessMapUv),S.push(b.roughnessMapUv),S.push(b.anisotropyMapUv),S.push(b.clearcoatMapUv),S.push(b.clearcoatNormalMapUv),S.push(b.clearcoatRoughnessMapUv),S.push(b.iridescenceMapUv),S.push(b.iridescenceThicknessMapUv),S.push(b.sheenColorMapUv),S.push(b.sheenRoughnessMapUv),S.push(b.specularMapUv),S.push(b.specularColorMapUv),S.push(b.specularIntensityMapUv),S.push(b.transmissionMapUv),S.push(b.thicknessMapUv),S.push(b.combine),S.push(b.fogExp2),S.push(b.sizeAttenuation),S.push(b.morphTargetsCount),S.push(b.morphAttributeCount),S.push(b.numDirLights),S.push(b.numPointLights),S.push(b.numSpotLights),S.push(b.numSpotLightMaps),S.push(b.numHemiLights),S.push(b.numRectAreaLights),S.push(b.numDirLightShadows),S.push(b.numPointLightShadows),S.push(b.numSpotLightShadows),S.push(b.numSpotLightShadowsWithMaps),S.push(b.numLightProbes),S.push(b.shadowMapType),S.push(b.toneMapping),S.push(b.numClippingPlanes),S.push(b.numClipIntersection),S.push(b.depthPacking)}function E(S,b){o.disableAll(),b.supportsVertexTextures&&o.enable(0),b.instancing&&o.enable(1),b.instancingColor&&o.enable(2),b.instancingMorph&&o.enable(3),b.matcap&&o.enable(4),b.envMap&&o.enable(5),b.normalMapObjectSpace&&o.enable(6),b.normalMapTangentSpace&&o.enable(7),b.clearcoat&&o.enable(8),b.iridescence&&o.enable(9),b.alphaTest&&o.enable(10),b.vertexColors&&o.enable(11),b.vertexAlphas&&o.enable(12),b.vertexUv1s&&o.enable(13),b.vertexUv2s&&o.enable(14),b.vertexUv3s&&o.enable(15),b.vertexTangents&&o.enable(16),b.anisotropy&&o.enable(17),b.alphaHash&&o.enable(18),b.batching&&o.enable(19),b.dispersion&&o.enable(20),b.batchingColor&&o.enable(21),b.gradientMap&&o.enable(22),S.push(o.mask),o.disableAll(),b.fog&&o.enable(0),b.useFog&&o.enable(1),b.flatShading&&o.enable(2),b.logarithmicDepthBuffer&&o.enable(3),b.reversedDepthBuffer&&o.enable(4),b.skinning&&o.enable(5),b.morphTargets&&o.enable(6),b.morphNormals&&o.enable(7),b.morphColors&&o.enable(8),b.premultipliedAlpha&&o.enable(9),b.shadowMapEnabled&&o.enable(10),b.doubleSided&&o.enable(11),b.flipSided&&o.enable(12),b.useDepthPacking&&o.enable(13),b.dithering&&o.enable(14),b.transmission&&o.enable(15),b.sheen&&o.enable(16),b.opaque&&o.enable(17),b.pointsUvs&&o.enable(18),b.decodeVideoTexture&&o.enable(19),b.decodeVideoTextureEmissive&&o.enable(20),b.alphaToCoverage&&o.enable(21),S.push(o.mask)}function _(S){const b=g[S.type];let L;if(b){const F=ci[b];L=Xd.clone(F.uniforms)}else L=S.uniforms;return L}function P(S,b){let L;for(let F=0,B=h.length;F<B;F++){const Y=h[F];if(Y.cacheKey===b){L=Y,++L.usedTimes;break}}return L===void 0&&(L=new K0(s,b,S,r),h.push(L)),L}function R(S){if(--S.usedTimes===0){const b=h.indexOf(S);h[b]=h[h.length-1],h.pop(),S.destroy()}}function I(S){c.remove(S)}function T(){c.dispose()}return{getParameters:m,getProgramCacheKey:p,getUniforms:_,acquireProgram:P,releaseProgram:R,releaseShaderCache:I,programs:h,dispose:T}}function tv(){let s=new WeakMap;function t(a){return s.has(a)}function e(a){let o=s.get(a);return o===void 0&&(o={},s.set(a,o)),o}function i(a){s.delete(a)}function n(a,o,c){s.get(a)[o]=c}function r(){s=new WeakMap}return{has:t,get:e,remove:i,update:n,dispose:r}}function ev(s,t){return s.groupOrder!==t.groupOrder?s.groupOrder-t.groupOrder:s.renderOrder!==t.renderOrder?s.renderOrder-t.renderOrder:s.material.id!==t.material.id?s.material.id-t.material.id:s.z!==t.z?s.z-t.z:s.id-t.id}function Uc(s,t){return s.groupOrder!==t.groupOrder?s.groupOrder-t.groupOrder:s.renderOrder!==t.renderOrder?s.renderOrder-t.renderOrder:s.z!==t.z?t.z-s.z:s.id-t.id}function Oc(){const s=[];let t=0;const e=[],i=[],n=[];function r(){t=0,e.length=0,i.length=0,n.length=0}function a(u,d,f,g,v,m){let p=s[t];return p===void 0?(p={id:u.id,object:u,geometry:d,material:f,groupOrder:g,renderOrder:u.renderOrder,z:v,group:m},s[t]=p):(p.id=u.id,p.object=u,p.geometry=d,p.material=f,p.groupOrder=g,p.renderOrder=u.renderOrder,p.z=v,p.group=m),t++,p}function o(u,d,f,g,v,m){const p=a(u,d,f,g,v,m);f.transmission>0?i.push(p):f.transparent===!0?n.push(p):e.push(p)}function c(u,d,f,g,v,m){const p=a(u,d,f,g,v,m);f.transmission>0?i.unshift(p):f.transparent===!0?n.unshift(p):e.unshift(p)}function l(u,d){e.length>1&&e.sort(u||ev),i.length>1&&i.sort(d||Uc),n.length>1&&n.sort(d||Uc)}function h(){for(let u=t,d=s.length;u<d;u++){const f=s[u];if(f.id===null)break;f.id=null,f.object=null,f.geometry=null,f.material=null,f.group=null}}return{opaque:e,transmissive:i,transparent:n,init:r,push:o,unshift:c,finish:h,sort:l}}function iv(){let s=new WeakMap;function t(i,n){const r=s.get(i);let a;return r===void 0?(a=new Oc,s.set(i,[a])):n>=r.length?(a=new Oc,r.push(a)):a=r[n],a}function e(){s=new WeakMap}return{get:t,dispose:e}}function nv(){const s={};return{get:function(t){if(s[t.id]!==void 0)return s[t.id];let e;switch(t.type){case"DirectionalLight":e={direction:new C,color:new Ht};break;case"SpotLight":e={position:new C,direction:new C,color:new Ht,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":e={position:new C,color:new Ht,distance:0,decay:0};break;case"HemisphereLight":e={direction:new C,skyColor:new Ht,groundColor:new Ht};break;case"RectAreaLight":e={color:new Ht,position:new C,halfWidth:new C,halfHeight:new C};break}return s[t.id]=e,e}}}function sv(){const s={};return{get:function(t){if(s[t.id]!==void 0)return s[t.id];let e;switch(t.type){case"DirectionalLight":e={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ht};break;case"SpotLight":e={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ht};break;case"PointLight":e={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new ht,shadowCameraNear:1,shadowCameraFar:1e3};break}return s[t.id]=e,e}}}let rv=0;function av(s,t){return(t.castShadow?2:0)-(s.castShadow?2:0)+(t.map?1:0)-(s.map?1:0)}function ov(s){const t=new nv,e=sv(),i={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let l=0;l<9;l++)i.probe.push(new C);const n=new C,r=new ie,a=new ie;function o(l){let h=0,u=0,d=0;for(let S=0;S<9;S++)i.probe[S].set(0,0,0);let f=0,g=0,v=0,m=0,p=0,w=0,E=0,_=0,P=0,R=0,I=0;l.sort(av);for(let S=0,b=l.length;S<b;S++){const L=l[S],F=L.color,B=L.intensity,Y=L.distance,W=L.shadow&&L.shadow.map?L.shadow.map.texture:null;if(L.isAmbientLight)h+=F.r*B,u+=F.g*B,d+=F.b*B;else if(L.isLightProbe){for(let $=0;$<9;$++)i.probe[$].addScaledVector(L.sh.coefficients[$],B);I++}else if(L.isDirectionalLight){const $=t.get(L);if($.color.copy(L.color).multiplyScalar(L.intensity),L.castShadow){const J=L.shadow,G=e.get(L);G.shadowIntensity=J.intensity,G.shadowBias=J.bias,G.shadowNormalBias=J.normalBias,G.shadowRadius=J.radius,G.shadowMapSize=J.mapSize,i.directionalShadow[f]=G,i.directionalShadowMap[f]=W,i.directionalShadowMatrix[f]=L.shadow.matrix,w++}i.directional[f]=$,f++}else if(L.isSpotLight){const $=t.get(L);$.position.setFromMatrixPosition(L.matrixWorld),$.color.copy(F).multiplyScalar(B),$.distance=Y,$.coneCos=Math.cos(L.angle),$.penumbraCos=Math.cos(L.angle*(1-L.penumbra)),$.decay=L.decay,i.spot[v]=$;const J=L.shadow;if(L.map&&(i.spotLightMap[P]=L.map,P++,J.updateMatrices(L),L.castShadow&&R++),i.spotLightMatrix[v]=J.matrix,L.castShadow){const G=e.get(L);G.shadowIntensity=J.intensity,G.shadowBias=J.bias,G.shadowNormalBias=J.normalBias,G.shadowRadius=J.radius,G.shadowMapSize=J.mapSize,i.spotShadow[v]=G,i.spotShadowMap[v]=W,_++}v++}else if(L.isRectAreaLight){const $=t.get(L);$.color.copy(F).multiplyScalar(B),$.halfWidth.set(L.width*.5,0,0),$.halfHeight.set(0,L.height*.5,0),i.rectArea[m]=$,m++}else if(L.isPointLight){const $=t.get(L);if($.color.copy(L.color).multiplyScalar(L.intensity),$.distance=L.distance,$.decay=L.decay,L.castShadow){const J=L.shadow,G=e.get(L);G.shadowIntensity=J.intensity,G.shadowBias=J.bias,G.shadowNormalBias=J.normalBias,G.shadowRadius=J.radius,G.shadowMapSize=J.mapSize,G.shadowCameraNear=J.camera.near,G.shadowCameraFar=J.camera.far,i.pointShadow[g]=G,i.pointShadowMap[g]=W,i.pointShadowMatrix[g]=L.shadow.matrix,E++}i.point[g]=$,g++}else if(L.isHemisphereLight){const $=t.get(L);$.skyColor.copy(L.color).multiplyScalar(B),$.groundColor.copy(L.groundColor).multiplyScalar(B),i.hemi[p]=$,p++}}m>0&&(s.has("OES_texture_float_linear")===!0?(i.rectAreaLTC1=rt.LTC_FLOAT_1,i.rectAreaLTC2=rt.LTC_FLOAT_2):(i.rectAreaLTC1=rt.LTC_HALF_1,i.rectAreaLTC2=rt.LTC_HALF_2)),i.ambient[0]=h,i.ambient[1]=u,i.ambient[2]=d;const T=i.hash;(T.directionalLength!==f||T.pointLength!==g||T.spotLength!==v||T.rectAreaLength!==m||T.hemiLength!==p||T.numDirectionalShadows!==w||T.numPointShadows!==E||T.numSpotShadows!==_||T.numSpotMaps!==P||T.numLightProbes!==I)&&(i.directional.length=f,i.spot.length=v,i.rectArea.length=m,i.point.length=g,i.hemi.length=p,i.directionalShadow.length=w,i.directionalShadowMap.length=w,i.pointShadow.length=E,i.pointShadowMap.length=E,i.spotShadow.length=_,i.spotShadowMap.length=_,i.directionalShadowMatrix.length=w,i.pointShadowMatrix.length=E,i.spotLightMatrix.length=_+P-R,i.spotLightMap.length=P,i.numSpotLightShadowsWithMaps=R,i.numLightProbes=I,T.directionalLength=f,T.pointLength=g,T.spotLength=v,T.rectAreaLength=m,T.hemiLength=p,T.numDirectionalShadows=w,T.numPointShadows=E,T.numSpotShadows=_,T.numSpotMaps=P,T.numLightProbes=I,i.version=rv++)}function c(l,h){let u=0,d=0,f=0,g=0,v=0;const m=h.matrixWorldInverse;for(let p=0,w=l.length;p<w;p++){const E=l[p];if(E.isDirectionalLight){const _=i.directional[u];_.direction.setFromMatrixPosition(E.matrixWorld),n.setFromMatrixPosition(E.target.matrixWorld),_.direction.sub(n),_.direction.transformDirection(m),u++}else if(E.isSpotLight){const _=i.spot[f];_.position.setFromMatrixPosition(E.matrixWorld),_.position.applyMatrix4(m),_.direction.setFromMatrixPosition(E.matrixWorld),n.setFromMatrixPosition(E.target.matrixWorld),_.direction.sub(n),_.direction.transformDirection(m),f++}else if(E.isRectAreaLight){const _=i.rectArea[g];_.position.setFromMatrixPosition(E.matrixWorld),_.position.applyMatrix4(m),a.identity(),r.copy(E.matrixWorld),r.premultiply(m),a.extractRotation(r),_.halfWidth.set(E.width*.5,0,0),_.halfHeight.set(0,E.height*.5,0),_.halfWidth.applyMatrix4(a),_.halfHeight.applyMatrix4(a),g++}else if(E.isPointLight){const _=i.point[d];_.position.setFromMatrixPosition(E.matrixWorld),_.position.applyMatrix4(m),d++}else if(E.isHemisphereLight){const _=i.hemi[v];_.direction.setFromMatrixPosition(E.matrixWorld),_.direction.transformDirection(m),v++}}}return{setup:o,setupView:c,state:i}}function Fc(s){const t=new ov(s),e=[],i=[];function n(h){l.camera=h,e.length=0,i.length=0}function r(h){e.push(h)}function a(h){i.push(h)}function o(){t.setup(e)}function c(h){t.setupView(e,h)}const l={lightsArray:e,shadowsArray:i,camera:null,lights:t,transmissionRenderTarget:{}};return{init:n,state:l,setupLights:o,setupLightsView:c,pushLight:r,pushShadow:a}}function lv(s){let t=new WeakMap;function e(n,r=0){const a=t.get(n);let o;return a===void 0?(o=new Fc(s),t.set(n,[o])):r>=a.length?(o=new Fc(s),a.push(o)):o=a[r],o}function i(){t=new WeakMap}return{get:e,dispose:i}}const cv=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,hv=`uniform sampler2D shadow_pass;
uniform vec2 resolution;
uniform float radius;
#include <packing>
void main() {
	const float samples = float( VSM_SAMPLES );
	float mean = 0.0;
	float squared_mean = 0.0;
	float uvStride = samples <= 1.0 ? 0.0 : 2.0 / ( samples - 1.0 );
	float uvStart = samples <= 1.0 ? 0.0 : - 1.0;
	for ( float i = 0.0; i < samples; i ++ ) {
		float uvOffset = uvStart + i * uvStride;
		#ifdef HORIZONTAL_PASS
			vec2 distribution = unpackRGBATo2Half( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( uvOffset, 0.0 ) * radius ) / resolution ) );
			mean += distribution.x;
			squared_mean += distribution.y * distribution.y + distribution.x * distribution.x;
		#else
			float depth = unpackRGBAToDepth( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( 0.0, uvOffset ) * radius ) / resolution ) );
			mean += depth;
			squared_mean += depth * depth;
		#endif
	}
	mean = mean / samples;
	squared_mean = squared_mean / samples;
	float std_dev = sqrt( squared_mean - mean * mean );
	gl_FragColor = pack2HalfToRGBA( vec2( mean, std_dev ) );
}`;function uv(s,t,e){let i=new tl;const n=new ht,r=new ht,a=new pe,o=new Uf({depthPacking:md}),c=new Of,l={},h=e.maxTextureSize,u={[Xi]:Oe,[Oe]:Xi,[Ne]:Ne},d=new Yi({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new ht},radius:{value:4}},vertexShader:cv,fragmentShader:hv}),f=d.clone();f.defines.HORIZONTAL_PASS=1;const g=new we;g.setAttribute("position",new $e(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));const v=new Vt(g,d),m=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=Qc;let p=this.type;this.render=function(R,I,T){if(m.enabled===!1||m.autoUpdate===!1&&m.needsUpdate===!1||R.length===0)return;const S=s.getRenderTarget(),b=s.getActiveCubeFace(),L=s.getActiveMipmapLevel(),F=s.state;F.setBlending(Vi),F.buffers.depth.getReversed()===!0?F.buffers.color.setClear(0,0,0,0):F.buffers.color.setClear(1,1,1,1),F.buffers.depth.setTest(!0),F.setScissorTest(!1);const B=p!==wi&&this.type===wi,Y=p===wi&&this.type!==wi;for(let W=0,$=R.length;W<$;W++){const J=R[W],G=J.shadow;if(G===void 0){console.warn("THREE.WebGLShadowMap:",J,"has no shadow.");continue}if(G.autoUpdate===!1&&G.needsUpdate===!1)continue;n.copy(G.mapSize);const at=G.getFrameExtents();if(n.multiply(at),r.copy(G.mapSize),(n.x>h||n.y>h)&&(n.x>h&&(r.x=Math.floor(h/at.x),n.x=r.x*at.x,G.mapSize.x=r.x),n.y>h&&(r.y=Math.floor(h/at.y),n.y=r.y*at.y,G.mapSize.y=r.y)),G.map===null||B===!0||Y===!0){const Et=this.type!==wi?{minFilter:Xe,magFilter:Xe}:{};G.map!==null&&G.map.dispose(),G.map=new vn(n.x,n.y,Et),G.map.texture.name=J.name+".shadowMap",G.camera.updateProjectionMatrix()}s.setRenderTarget(G.map),s.clear();const ut=G.getViewportCount();for(let Et=0;Et<ut;Et++){const Gt=G.getViewport(Et);a.set(r.x*Gt.x,r.y*Gt.y,r.x*Gt.z,r.y*Gt.w),F.viewport(a),G.updateMatrices(J,Et),i=G.getFrustum(),_(I,T,G.camera,J,this.type)}G.isPointLightShadow!==!0&&this.type===wi&&w(G,T),G.needsUpdate=!1}p=this.type,m.needsUpdate=!1,s.setRenderTarget(S,b,L)};function w(R,I){const T=t.update(v);d.defines.VSM_SAMPLES!==R.blurSamples&&(d.defines.VSM_SAMPLES=R.blurSamples,f.defines.VSM_SAMPLES=R.blurSamples,d.needsUpdate=!0,f.needsUpdate=!0),R.mapPass===null&&(R.mapPass=new vn(n.x,n.y)),d.uniforms.shadow_pass.value=R.map.texture,d.uniforms.resolution.value=R.mapSize,d.uniforms.radius.value=R.radius,s.setRenderTarget(R.mapPass),s.clear(),s.renderBufferDirect(I,null,T,d,v,null),f.uniforms.shadow_pass.value=R.mapPass.texture,f.uniforms.resolution.value=R.mapSize,f.uniforms.radius.value=R.radius,s.setRenderTarget(R.map),s.clear(),s.renderBufferDirect(I,null,T,f,v,null)}function E(R,I,T,S){let b=null;const L=T.isPointLight===!0?R.customDistanceMaterial:R.customDepthMaterial;if(L!==void 0)b=L;else if(b=T.isPointLight===!0?c:o,s.localClippingEnabled&&I.clipShadows===!0&&Array.isArray(I.clippingPlanes)&&I.clippingPlanes.length!==0||I.displacementMap&&I.displacementScale!==0||I.alphaMap&&I.alphaTest>0||I.map&&I.alphaTest>0||I.alphaToCoverage===!0){const F=b.uuid,B=I.uuid;let Y=l[F];Y===void 0&&(Y={},l[F]=Y);let W=Y[B];W===void 0&&(W=b.clone(),Y[B]=W,I.addEventListener("dispose",P)),b=W}if(b.visible=I.visible,b.wireframe=I.wireframe,S===wi?b.side=I.shadowSide!==null?I.shadowSide:I.side:b.side=I.shadowSide!==null?I.shadowSide:u[I.side],b.alphaMap=I.alphaMap,b.alphaTest=I.alphaToCoverage===!0?.5:I.alphaTest,b.map=I.map,b.clipShadows=I.clipShadows,b.clippingPlanes=I.clippingPlanes,b.clipIntersection=I.clipIntersection,b.displacementMap=I.displacementMap,b.displacementScale=I.displacementScale,b.displacementBias=I.displacementBias,b.wireframeLinewidth=I.wireframeLinewidth,b.linewidth=I.linewidth,T.isPointLight===!0&&b.isMeshDistanceMaterial===!0){const F=s.properties.get(b);F.light=T}return b}function _(R,I,T,S,b){if(R.visible===!1)return;if(R.layers.test(I.layers)&&(R.isMesh||R.isLine||R.isPoints)&&(R.castShadow||R.receiveShadow&&b===wi)&&(!R.frustumCulled||i.intersectsObject(R))){R.modelViewMatrix.multiplyMatrices(T.matrixWorldInverse,R.matrixWorld);const B=t.update(R),Y=R.material;if(Array.isArray(Y)){const W=B.groups;for(let $=0,J=W.length;$<J;$++){const G=W[$],at=Y[G.materialIndex];if(at&&at.visible){const ut=E(R,at,S,b);R.onBeforeShadow(s,R,I,T,B,ut,G),s.renderBufferDirect(T,null,B,ut,R,G),R.onAfterShadow(s,R,I,T,B,ut,G)}}}else if(Y.visible){const W=E(R,Y,S,b);R.onBeforeShadow(s,R,I,T,B,W,null),s.renderBufferDirect(T,null,B,W,R,null),R.onAfterShadow(s,R,I,T,B,W,null)}}const F=R.children;for(let B=0,Y=F.length;B<Y;B++)_(F[B],I,T,S,b)}function P(R){R.target.removeEventListener("dispose",P);for(const T in l){const S=l[T],b=R.target.uuid;b in S&&(S[b].dispose(),delete S[b])}}}const dv={[Wa]:Xa,[$a]:Ka,[Ya]:Za,[ts]:qa,[Xa]:Wa,[Ka]:$a,[Za]:Ya,[qa]:ts};function fv(s,t){function e(){let N=!1;const it=new pe;let st=null;const ft=new pe(0,0,0,0);return{setMask:function(Q){st!==Q&&!N&&(s.colorMask(Q,Q,Q,Q),st=Q)},setLocked:function(Q){N=Q},setClear:function(Q,K,gt,Lt,re){re===!0&&(Q*=Lt,K*=Lt,gt*=Lt),it.set(Q,K,gt,Lt),ft.equals(it)===!1&&(s.clearColor(Q,K,gt,Lt),ft.copy(it))},reset:function(){N=!1,st=null,ft.set(-1,0,0,0)}}}function i(){let N=!1,it=!1,st=null,ft=null,Q=null;return{setReversed:function(K){if(it!==K){const gt=t.get("EXT_clip_control");K?gt.clipControlEXT(gt.LOWER_LEFT_EXT,gt.ZERO_TO_ONE_EXT):gt.clipControlEXT(gt.LOWER_LEFT_EXT,gt.NEGATIVE_ONE_TO_ONE_EXT),it=K;const Lt=Q;Q=null,this.setClear(Lt)}},getReversed:function(){return it},setTest:function(K){K?j(s.DEPTH_TEST):pt(s.DEPTH_TEST)},setMask:function(K){st!==K&&!N&&(s.depthMask(K),st=K)},setFunc:function(K){if(it&&(K=dv[K]),ft!==K){switch(K){case Wa:s.depthFunc(s.NEVER);break;case Xa:s.depthFunc(s.ALWAYS);break;case $a:s.depthFunc(s.LESS);break;case ts:s.depthFunc(s.LEQUAL);break;case Ya:s.depthFunc(s.EQUAL);break;case qa:s.depthFunc(s.GEQUAL);break;case Ka:s.depthFunc(s.GREATER);break;case Za:s.depthFunc(s.NOTEQUAL);break;default:s.depthFunc(s.LEQUAL)}ft=K}},setLocked:function(K){N=K},setClear:function(K){Q!==K&&(it&&(K=1-K),s.clearDepth(K),Q=K)},reset:function(){N=!1,st=null,ft=null,Q=null,it=!1}}}function n(){let N=!1,it=null,st=null,ft=null,Q=null,K=null,gt=null,Lt=null,re=null;return{setTest:function(Zt){N||(Zt?j(s.STENCIL_TEST):pt(s.STENCIL_TEST))},setMask:function(Zt){it!==Zt&&!N&&(s.stencilMask(Zt),it=Zt)},setFunc:function(Zt,_i,oi){(st!==Zt||ft!==_i||Q!==oi)&&(s.stencilFunc(Zt,_i,oi),st=Zt,ft=_i,Q=oi)},setOp:function(Zt,_i,oi){(K!==Zt||gt!==_i||Lt!==oi)&&(s.stencilOp(Zt,_i,oi),K=Zt,gt=_i,Lt=oi)},setLocked:function(Zt){N=Zt},setClear:function(Zt){re!==Zt&&(s.clearStencil(Zt),re=Zt)},reset:function(){N=!1,it=null,st=null,ft=null,Q=null,K=null,gt=null,Lt=null,re=null}}}const r=new e,a=new i,o=new n,c=new WeakMap,l=new WeakMap;let h={},u={},d=new WeakMap,f=[],g=null,v=!1,m=null,p=null,w=null,E=null,_=null,P=null,R=null,I=new Ht(0,0,0),T=0,S=!1,b=null,L=null,F=null,B=null,Y=null;const W=s.getParameter(s.MAX_COMBINED_TEXTURE_IMAGE_UNITS);let $=!1,J=0;const G=s.getParameter(s.VERSION);G.indexOf("WebGL")!==-1?(J=parseFloat(/^WebGL (\d)/.exec(G)[1]),$=J>=1):G.indexOf("OpenGL ES")!==-1&&(J=parseFloat(/^OpenGL ES (\d)/.exec(G)[1]),$=J>=2);let at=null,ut={};const Et=s.getParameter(s.SCISSOR_BOX),Gt=s.getParameter(s.VIEWPORT),se=new pe().fromArray(Et),oe=new pe().fromArray(Gt);function Kt(N,it,st,ft){const Q=new Uint8Array(4),K=s.createTexture();s.bindTexture(N,K),s.texParameteri(N,s.TEXTURE_MIN_FILTER,s.NEAREST),s.texParameteri(N,s.TEXTURE_MAG_FILTER,s.NEAREST);for(let gt=0;gt<st;gt++)N===s.TEXTURE_3D||N===s.TEXTURE_2D_ARRAY?s.texImage3D(it,0,s.RGBA,1,1,ft,0,s.RGBA,s.UNSIGNED_BYTE,Q):s.texImage2D(it+gt,0,s.RGBA,1,1,0,s.RGBA,s.UNSIGNED_BYTE,Q);return K}const q={};q[s.TEXTURE_2D]=Kt(s.TEXTURE_2D,s.TEXTURE_2D,1),q[s.TEXTURE_CUBE_MAP]=Kt(s.TEXTURE_CUBE_MAP,s.TEXTURE_CUBE_MAP_POSITIVE_X,6),q[s.TEXTURE_2D_ARRAY]=Kt(s.TEXTURE_2D_ARRAY,s.TEXTURE_2D_ARRAY,1,1),q[s.TEXTURE_3D]=Kt(s.TEXTURE_3D,s.TEXTURE_3D,1,1),r.setClear(0,0,0,1),a.setClear(1),o.setClear(0),j(s.DEPTH_TEST),a.setFunc(ts),Pt(!1),vt(Al),j(s.CULL_FACE),le(Vi);function j(N){h[N]!==!0&&(s.enable(N),h[N]=!0)}function pt(N){h[N]!==!1&&(s.disable(N),h[N]=!1)}function Dt(N,it){return u[N]!==it?(s.bindFramebuffer(N,it),u[N]=it,N===s.DRAW_FRAMEBUFFER&&(u[s.FRAMEBUFFER]=it),N===s.FRAMEBUFFER&&(u[s.DRAW_FRAMEBUFFER]=it),!0):!1}function bt(N,it){let st=f,ft=!1;if(N){st=d.get(it),st===void 0&&(st=[],d.set(it,st));const Q=N.textures;if(st.length!==Q.length||st[0]!==s.COLOR_ATTACHMENT0){for(let K=0,gt=Q.length;K<gt;K++)st[K]=s.COLOR_ATTACHMENT0+K;st.length=Q.length,ft=!0}}else st[0]!==s.BACK&&(st[0]=s.BACK,ft=!0);ft&&s.drawBuffers(st)}function $t(N){return g!==N?(s.useProgram(N),g=N,!0):!1}const Te={[rn]:s.FUNC_ADD,[Gu]:s.FUNC_SUBTRACT,[Vu]:s.FUNC_REVERSE_SUBTRACT};Te[Wu]=s.MIN,Te[Xu]=s.MAX;const D={[$u]:s.ZERO,[Yu]:s.ONE,[qu]:s.SRC_COLOR,[Ga]:s.SRC_ALPHA,[td]:s.SRC_ALPHA_SATURATE,[ju]:s.DST_COLOR,[Zu]:s.DST_ALPHA,[Ku]:s.ONE_MINUS_SRC_COLOR,[Va]:s.ONE_MINUS_SRC_ALPHA,[Qu]:s.ONE_MINUS_DST_COLOR,[Ju]:s.ONE_MINUS_DST_ALPHA,[ed]:s.CONSTANT_COLOR,[id]:s.ONE_MINUS_CONSTANT_COLOR,[nd]:s.CONSTANT_ALPHA,[sd]:s.ONE_MINUS_CONSTANT_ALPHA};function le(N,it,st,ft,Q,K,gt,Lt,re,Zt){if(N===Vi){v===!0&&(pt(s.BLEND),v=!1);return}if(v===!1&&(j(s.BLEND),v=!0),N!==Hu){if(N!==m||Zt!==S){if((p!==rn||_!==rn)&&(s.blendEquation(s.FUNC_ADD),p=rn,_=rn),Zt)switch(N){case Jn:s.blendFuncSeparate(s.ONE,s.ONE_MINUS_SRC_ALPHA,s.ONE,s.ONE_MINUS_SRC_ALPHA);break;case Rl:s.blendFunc(s.ONE,s.ONE);break;case Cl:s.blendFuncSeparate(s.ZERO,s.ONE_MINUS_SRC_COLOR,s.ZERO,s.ONE);break;case Pl:s.blendFuncSeparate(s.DST_COLOR,s.ONE_MINUS_SRC_ALPHA,s.ZERO,s.ONE);break;default:console.error("THREE.WebGLState: Invalid blending: ",N);break}else switch(N){case Jn:s.blendFuncSeparate(s.SRC_ALPHA,s.ONE_MINUS_SRC_ALPHA,s.ONE,s.ONE_MINUS_SRC_ALPHA);break;case Rl:s.blendFuncSeparate(s.SRC_ALPHA,s.ONE,s.ONE,s.ONE);break;case Cl:console.error("THREE.WebGLState: SubtractiveBlending requires material.premultipliedAlpha = true");break;case Pl:console.error("THREE.WebGLState: MultiplyBlending requires material.premultipliedAlpha = true");break;default:console.error("THREE.WebGLState: Invalid blending: ",N);break}w=null,E=null,P=null,R=null,I.set(0,0,0),T=0,m=N,S=Zt}return}Q=Q||it,K=K||st,gt=gt||ft,(it!==p||Q!==_)&&(s.blendEquationSeparate(Te[it],Te[Q]),p=it,_=Q),(st!==w||ft!==E||K!==P||gt!==R)&&(s.blendFuncSeparate(D[st],D[ft],D[K],D[gt]),w=st,E=ft,P=K,R=gt),(Lt.equals(I)===!1||re!==T)&&(s.blendColor(Lt.r,Lt.g,Lt.b,re),I.copy(Lt),T=re),m=N,S=!1}function Nt(N,it){N.side===Ne?pt(s.CULL_FACE):j(s.CULL_FACE);let st=N.side===Oe;it&&(st=!st),Pt(st),N.blending===Jn&&N.transparent===!1?le(Vi):le(N.blending,N.blendEquation,N.blendSrc,N.blendDst,N.blendEquationAlpha,N.blendSrcAlpha,N.blendDstAlpha,N.blendColor,N.blendAlpha,N.premultipliedAlpha),a.setFunc(N.depthFunc),a.setTest(N.depthTest),a.setMask(N.depthWrite),r.setMask(N.colorWrite);const ft=N.stencilWrite;o.setTest(ft),ft&&(o.setMask(N.stencilWriteMask),o.setFunc(N.stencilFunc,N.stencilRef,N.stencilFuncMask),o.setOp(N.stencilFail,N.stencilZFail,N.stencilZPass)),_t(N.polygonOffset,N.polygonOffsetFactor,N.polygonOffsetUnits),N.alphaToCoverage===!0?j(s.SAMPLE_ALPHA_TO_COVERAGE):pt(s.SAMPLE_ALPHA_TO_COVERAGE)}function Pt(N){b!==N&&(N?s.frontFace(s.CW):s.frontFace(s.CCW),b=N)}function vt(N){N!==zu?(j(s.CULL_FACE),N!==L&&(N===Al?s.cullFace(s.BACK):N===Bu?s.cullFace(s.FRONT):s.cullFace(s.FRONT_AND_BACK))):pt(s.CULL_FACE),L=N}function ce(N){N!==F&&($&&s.lineWidth(N),F=N)}function _t(N,it,st){N?(j(s.POLYGON_OFFSET_FILL),(B!==it||Y!==st)&&(s.polygonOffset(it,st),B=it,Y=st)):pt(s.POLYGON_OFFSET_FILL)}function Ft(N){N?j(s.SCISSOR_TEST):pt(s.SCISSOR_TEST)}function Se(N){N===void 0&&(N=s.TEXTURE0+W-1),at!==N&&(s.activeTexture(N),at=N)}function me(N,it,st){st===void 0&&(at===null?st=s.TEXTURE0+W-1:st=at);let ft=ut[st];ft===void 0&&(ft={type:void 0,texture:void 0},ut[st]=ft),(ft.type!==N||ft.texture!==it)&&(at!==st&&(s.activeTexture(st),at=st),s.bindTexture(N,it||q[N]),ft.type=N,ft.texture=it)}function A(){const N=ut[at];N!==void 0&&N.type!==void 0&&(s.bindTexture(N.type,null),N.type=void 0,N.texture=void 0)}function x(){try{s.compressedTexImage2D(...arguments)}catch(N){console.error("THREE.WebGLState:",N)}}function k(){try{s.compressedTexImage3D(...arguments)}catch(N){console.error("THREE.WebGLState:",N)}}function X(){try{s.texSubImage2D(...arguments)}catch(N){console.error("THREE.WebGLState:",N)}}function Z(){try{s.texSubImage3D(...arguments)}catch(N){console.error("THREE.WebGLState:",N)}}function V(){try{s.compressedTexSubImage2D(...arguments)}catch(N){console.error("THREE.WebGLState:",N)}}function St(){try{s.compressedTexSubImage3D(...arguments)}catch(N){console.error("THREE.WebGLState:",N)}}function nt(){try{s.texStorage2D(...arguments)}catch(N){console.error("THREE.WebGLState:",N)}}function xt(){try{s.texStorage3D(...arguments)}catch(N){console.error("THREE.WebGLState:",N)}}function yt(){try{s.texImage2D(...arguments)}catch(N){console.error("THREE.WebGLState:",N)}}function et(){try{s.texImage3D(...arguments)}catch(N){console.error("THREE.WebGLState:",N)}}function ct(N){se.equals(N)===!1&&(s.scissor(N.x,N.y,N.z,N.w),se.copy(N))}function Ct(N){oe.equals(N)===!1&&(s.viewport(N.x,N.y,N.z,N.w),oe.copy(N))}function Mt(N,it){let st=l.get(it);st===void 0&&(st=new WeakMap,l.set(it,st));let ft=st.get(N);ft===void 0&&(ft=s.getUniformBlockIndex(it,N.name),st.set(N,ft))}function ot(N,it){const ft=l.get(it).get(N);c.get(it)!==ft&&(s.uniformBlockBinding(it,ft,N.__bindingPointIndex),c.set(it,ft))}function Ut(){s.disable(s.BLEND),s.disable(s.CULL_FACE),s.disable(s.DEPTH_TEST),s.disable(s.POLYGON_OFFSET_FILL),s.disable(s.SCISSOR_TEST),s.disable(s.STENCIL_TEST),s.disable(s.SAMPLE_ALPHA_TO_COVERAGE),s.blendEquation(s.FUNC_ADD),s.blendFunc(s.ONE,s.ZERO),s.blendFuncSeparate(s.ONE,s.ZERO,s.ONE,s.ZERO),s.blendColor(0,0,0,0),s.colorMask(!0,!0,!0,!0),s.clearColor(0,0,0,0),s.depthMask(!0),s.depthFunc(s.LESS),a.setReversed(!1),s.clearDepth(1),s.stencilMask(4294967295),s.stencilFunc(s.ALWAYS,0,4294967295),s.stencilOp(s.KEEP,s.KEEP,s.KEEP),s.clearStencil(0),s.cullFace(s.BACK),s.frontFace(s.CCW),s.polygonOffset(0,0),s.activeTexture(s.TEXTURE0),s.bindFramebuffer(s.FRAMEBUFFER,null),s.bindFramebuffer(s.DRAW_FRAMEBUFFER,null),s.bindFramebuffer(s.READ_FRAMEBUFFER,null),s.useProgram(null),s.lineWidth(1),s.scissor(0,0,s.canvas.width,s.canvas.height),s.viewport(0,0,s.canvas.width,s.canvas.height),h={},at=null,ut={},u={},d=new WeakMap,f=[],g=null,v=!1,m=null,p=null,w=null,E=null,_=null,P=null,R=null,I=new Ht(0,0,0),T=0,S=!1,b=null,L=null,F=null,B=null,Y=null,se.set(0,0,s.canvas.width,s.canvas.height),oe.set(0,0,s.canvas.width,s.canvas.height),r.reset(),a.reset(),o.reset()}return{buffers:{color:r,depth:a,stencil:o},enable:j,disable:pt,bindFramebuffer:Dt,drawBuffers:bt,useProgram:$t,setBlending:le,setMaterial:Nt,setFlipSided:Pt,setCullFace:vt,setLineWidth:ce,setPolygonOffset:_t,setScissorTest:Ft,activeTexture:Se,bindTexture:me,unbindTexture:A,compressedTexImage2D:x,compressedTexImage3D:k,texImage2D:yt,texImage3D:et,updateUBOMapping:Mt,uniformBlockBinding:ot,texStorage2D:nt,texStorage3D:xt,texSubImage2D:X,texSubImage3D:Z,compressedTexSubImage2D:V,compressedTexSubImage3D:St,scissor:ct,viewport:Ct,reset:Ut}}function pv(s,t,e,i,n,r,a){const o=t.has("WEBGL_multisampled_render_to_texture")?t.get("WEBGL_multisampled_render_to_texture"):null,c=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),l=new ht,h=new WeakMap;let u;const d=new WeakMap;let f=!1;try{f=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function g(A,x){return f?new OffscreenCanvas(A,x):Or("canvas")}function v(A,x,k){let X=1;const Z=me(A);if((Z.width>k||Z.height>k)&&(X=k/Math.max(Z.width,Z.height)),X<1)if(typeof HTMLImageElement<"u"&&A instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&A instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&A instanceof ImageBitmap||typeof VideoFrame<"u"&&A instanceof VideoFrame){const V=Math.floor(X*Z.width),St=Math.floor(X*Z.height);u===void 0&&(u=g(V,St));const nt=x?g(V,St):u;return nt.width=V,nt.height=St,nt.getContext("2d").drawImage(A,0,0,V,St),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+Z.width+"x"+Z.height+") to ("+V+"x"+St+")."),nt}else return"data"in A&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+Z.width+"x"+Z.height+")."),A;return A}function m(A){return A.generateMipmaps}function p(A){s.generateMipmap(A)}function w(A){return A.isWebGLCubeRenderTarget?s.TEXTURE_CUBE_MAP:A.isWebGL3DRenderTarget?s.TEXTURE_3D:A.isWebGLArrayRenderTarget||A.isCompressedArrayTexture?s.TEXTURE_2D_ARRAY:s.TEXTURE_2D}function E(A,x,k,X,Z=!1){if(A!==null){if(s[A]!==void 0)return s[A];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+A+"'")}let V=x;if(x===s.RED&&(k===s.FLOAT&&(V=s.R32F),k===s.HALF_FLOAT&&(V=s.R16F),k===s.UNSIGNED_BYTE&&(V=s.R8)),x===s.RED_INTEGER&&(k===s.UNSIGNED_BYTE&&(V=s.R8UI),k===s.UNSIGNED_SHORT&&(V=s.R16UI),k===s.UNSIGNED_INT&&(V=s.R32UI),k===s.BYTE&&(V=s.R8I),k===s.SHORT&&(V=s.R16I),k===s.INT&&(V=s.R32I)),x===s.RG&&(k===s.FLOAT&&(V=s.RG32F),k===s.HALF_FLOAT&&(V=s.RG16F),k===s.UNSIGNED_BYTE&&(V=s.RG8)),x===s.RG_INTEGER&&(k===s.UNSIGNED_BYTE&&(V=s.RG8UI),k===s.UNSIGNED_SHORT&&(V=s.RG16UI),k===s.UNSIGNED_INT&&(V=s.RG32UI),k===s.BYTE&&(V=s.RG8I),k===s.SHORT&&(V=s.RG16I),k===s.INT&&(V=s.RG32I)),x===s.RGB_INTEGER&&(k===s.UNSIGNED_BYTE&&(V=s.RGB8UI),k===s.UNSIGNED_SHORT&&(V=s.RGB16UI),k===s.UNSIGNED_INT&&(V=s.RGB32UI),k===s.BYTE&&(V=s.RGB8I),k===s.SHORT&&(V=s.RGB16I),k===s.INT&&(V=s.RGB32I)),x===s.RGBA_INTEGER&&(k===s.UNSIGNED_BYTE&&(V=s.RGBA8UI),k===s.UNSIGNED_SHORT&&(V=s.RGBA16UI),k===s.UNSIGNED_INT&&(V=s.RGBA32UI),k===s.BYTE&&(V=s.RGBA8I),k===s.SHORT&&(V=s.RGBA16I),k===s.INT&&(V=s.RGBA32I)),x===s.RGB&&(k===s.UNSIGNED_INT_5_9_9_9_REV&&(V=s.RGB9_E5),k===s.UNSIGNED_INT_10F_11F_11F_REV&&(V=s.R11F_G11F_B10F)),x===s.RGBA){const St=Z?Nr:qt.getTransfer(X);k===s.FLOAT&&(V=s.RGBA32F),k===s.HALF_FLOAT&&(V=s.RGBA16F),k===s.UNSIGNED_BYTE&&(V=St===te?s.SRGB8_ALPHA8:s.RGBA8),k===s.UNSIGNED_SHORT_4_4_4_4&&(V=s.RGBA4),k===s.UNSIGNED_SHORT_5_5_5_1&&(V=s.RGB5_A1)}return(V===s.R16F||V===s.R32F||V===s.RG16F||V===s.RG32F||V===s.RGBA16F||V===s.RGBA32F)&&t.get("EXT_color_buffer_float"),V}function _(A,x){let k;return A?x===null||x===gn||x===Ns?k=s.DEPTH24_STENCIL8:x===di?k=s.DEPTH32F_STENCIL8:x===Ls&&(k=s.DEPTH24_STENCIL8,console.warn("DepthTexture: 16 bit depth attachment is not supported with stencil. Using 24-bit attachment.")):x===null||x===gn||x===Ns?k=s.DEPTH_COMPONENT24:x===di?k=s.DEPTH_COMPONENT32F:x===Ls&&(k=s.DEPTH_COMPONENT16),k}function P(A,x){return m(A)===!0||A.isFramebufferTexture&&A.minFilter!==Xe&&A.minFilter!==ui?Math.log2(Math.max(x.width,x.height))+1:A.mipmaps!==void 0&&A.mipmaps.length>0?A.mipmaps.length:A.isCompressedTexture&&Array.isArray(A.image)?x.mipmaps.length:1}function R(A){const x=A.target;x.removeEventListener("dispose",R),T(x),x.isVideoTexture&&h.delete(x)}function I(A){const x=A.target;x.removeEventListener("dispose",I),b(x)}function T(A){const x=i.get(A);if(x.__webglInit===void 0)return;const k=A.source,X=d.get(k);if(X){const Z=X[x.__cacheKey];Z.usedTimes--,Z.usedTimes===0&&S(A),Object.keys(X).length===0&&d.delete(k)}i.remove(A)}function S(A){const x=i.get(A);s.deleteTexture(x.__webglTexture);const k=A.source,X=d.get(k);delete X[x.__cacheKey],a.memory.textures--}function b(A){const x=i.get(A);if(A.depthTexture&&(A.depthTexture.dispose(),i.remove(A.depthTexture)),A.isWebGLCubeRenderTarget)for(let X=0;X<6;X++){if(Array.isArray(x.__webglFramebuffer[X]))for(let Z=0;Z<x.__webglFramebuffer[X].length;Z++)s.deleteFramebuffer(x.__webglFramebuffer[X][Z]);else s.deleteFramebuffer(x.__webglFramebuffer[X]);x.__webglDepthbuffer&&s.deleteRenderbuffer(x.__webglDepthbuffer[X])}else{if(Array.isArray(x.__webglFramebuffer))for(let X=0;X<x.__webglFramebuffer.length;X++)s.deleteFramebuffer(x.__webglFramebuffer[X]);else s.deleteFramebuffer(x.__webglFramebuffer);if(x.__webglDepthbuffer&&s.deleteRenderbuffer(x.__webglDepthbuffer),x.__webglMultisampledFramebuffer&&s.deleteFramebuffer(x.__webglMultisampledFramebuffer),x.__webglColorRenderbuffer)for(let X=0;X<x.__webglColorRenderbuffer.length;X++)x.__webglColorRenderbuffer[X]&&s.deleteRenderbuffer(x.__webglColorRenderbuffer[X]);x.__webglDepthRenderbuffer&&s.deleteRenderbuffer(x.__webglDepthRenderbuffer)}const k=A.textures;for(let X=0,Z=k.length;X<Z;X++){const V=i.get(k[X]);V.__webglTexture&&(s.deleteTexture(V.__webglTexture),a.memory.textures--),i.remove(k[X])}i.remove(A)}let L=0;function F(){L=0}function B(){const A=L;return A>=n.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+A+" texture units while this GPU supports only "+n.maxTextures),L+=1,A}function Y(A){const x=[];return x.push(A.wrapS),x.push(A.wrapT),x.push(A.wrapR||0),x.push(A.magFilter),x.push(A.minFilter),x.push(A.anisotropy),x.push(A.internalFormat),x.push(A.format),x.push(A.type),x.push(A.generateMipmaps),x.push(A.premultiplyAlpha),x.push(A.flipY),x.push(A.unpackAlignment),x.push(A.colorSpace),x.join()}function W(A,x){const k=i.get(A);if(A.isVideoTexture&&Ft(A),A.isRenderTargetTexture===!1&&A.isExternalTexture!==!0&&A.version>0&&k.__version!==A.version){const X=A.image;if(X===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(X.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{q(k,A,x);return}}else A.isExternalTexture&&(k.__webglTexture=A.sourceTexture?A.sourceTexture:null);e.bindTexture(s.TEXTURE_2D,k.__webglTexture,s.TEXTURE0+x)}function $(A,x){const k=i.get(A);if(A.isRenderTargetTexture===!1&&A.version>0&&k.__version!==A.version){q(k,A,x);return}e.bindTexture(s.TEXTURE_2D_ARRAY,k.__webglTexture,s.TEXTURE0+x)}function J(A,x){const k=i.get(A);if(A.isRenderTargetTexture===!1&&A.version>0&&k.__version!==A.version){q(k,A,x);return}e.bindTexture(s.TEXTURE_3D,k.__webglTexture,s.TEXTURE0+x)}function G(A,x){const k=i.get(A);if(A.version>0&&k.__version!==A.version){j(k,A,x);return}e.bindTexture(s.TEXTURE_CUBE_MAP,k.__webglTexture,s.TEXTURE0+x)}const at={[Qa]:s.REPEAT,[ln]:s.CLAMP_TO_EDGE,[to]:s.MIRRORED_REPEAT},ut={[Xe]:s.NEAREST,[fd]:s.NEAREST_MIPMAP_NEAREST,[qs]:s.NEAREST_MIPMAP_LINEAR,[ui]:s.LINEAR,[Kr]:s.LINEAR_MIPMAP_NEAREST,[cn]:s.LINEAR_MIPMAP_LINEAR},Et={[vd]:s.NEVER,[bd]:s.ALWAYS,[_d]:s.LESS,[dh]:s.LEQUAL,[xd]:s.EQUAL,[Sd]:s.GEQUAL,[yd]:s.GREATER,[Md]:s.NOTEQUAL};function Gt(A,x){if(x.type===di&&t.has("OES_texture_float_linear")===!1&&(x.magFilter===ui||x.magFilter===Kr||x.magFilter===qs||x.magFilter===cn||x.minFilter===ui||x.minFilter===Kr||x.minFilter===qs||x.minFilter===cn)&&console.warn("THREE.WebGLRenderer: Unable to use linear filtering with floating point textures. OES_texture_float_linear not supported on this device."),s.texParameteri(A,s.TEXTURE_WRAP_S,at[x.wrapS]),s.texParameteri(A,s.TEXTURE_WRAP_T,at[x.wrapT]),(A===s.TEXTURE_3D||A===s.TEXTURE_2D_ARRAY)&&s.texParameteri(A,s.TEXTURE_WRAP_R,at[x.wrapR]),s.texParameteri(A,s.TEXTURE_MAG_FILTER,ut[x.magFilter]),s.texParameteri(A,s.TEXTURE_MIN_FILTER,ut[x.minFilter]),x.compareFunction&&(s.texParameteri(A,s.TEXTURE_COMPARE_MODE,s.COMPARE_REF_TO_TEXTURE),s.texParameteri(A,s.TEXTURE_COMPARE_FUNC,Et[x.compareFunction])),t.has("EXT_texture_filter_anisotropic")===!0){if(x.magFilter===Xe||x.minFilter!==qs&&x.minFilter!==cn||x.type===di&&t.has("OES_texture_float_linear")===!1)return;if(x.anisotropy>1||i.get(x).__currentAnisotropy){const k=t.get("EXT_texture_filter_anisotropic");s.texParameterf(A,k.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(x.anisotropy,n.getMaxAnisotropy())),i.get(x).__currentAnisotropy=x.anisotropy}}}function se(A,x){let k=!1;A.__webglInit===void 0&&(A.__webglInit=!0,x.addEventListener("dispose",R));const X=x.source;let Z=d.get(X);Z===void 0&&(Z={},d.set(X,Z));const V=Y(x);if(V!==A.__cacheKey){Z[V]===void 0&&(Z[V]={texture:s.createTexture(),usedTimes:0},a.memory.textures++,k=!0),Z[V].usedTimes++;const St=Z[A.__cacheKey];St!==void 0&&(Z[A.__cacheKey].usedTimes--,St.usedTimes===0&&S(x)),A.__cacheKey=V,A.__webglTexture=Z[V].texture}return k}function oe(A,x,k){return Math.floor(Math.floor(A/k)/x)}function Kt(A,x,k,X){const V=A.updateRanges;if(V.length===0)e.texSubImage2D(s.TEXTURE_2D,0,0,0,x.width,x.height,k,X,x.data);else{V.sort((et,ct)=>et.start-ct.start);let St=0;for(let et=1;et<V.length;et++){const ct=V[St],Ct=V[et],Mt=ct.start+ct.count,ot=oe(Ct.start,x.width,4),Ut=oe(ct.start,x.width,4);Ct.start<=Mt+1&&ot===Ut&&oe(Ct.start+Ct.count-1,x.width,4)===ot?ct.count=Math.max(ct.count,Ct.start+Ct.count-ct.start):(++St,V[St]=Ct)}V.length=St+1;const nt=s.getParameter(s.UNPACK_ROW_LENGTH),xt=s.getParameter(s.UNPACK_SKIP_PIXELS),yt=s.getParameter(s.UNPACK_SKIP_ROWS);s.pixelStorei(s.UNPACK_ROW_LENGTH,x.width);for(let et=0,ct=V.length;et<ct;et++){const Ct=V[et],Mt=Math.floor(Ct.start/4),ot=Math.ceil(Ct.count/4),Ut=Mt%x.width,N=Math.floor(Mt/x.width),it=ot,st=1;s.pixelStorei(s.UNPACK_SKIP_PIXELS,Ut),s.pixelStorei(s.UNPACK_SKIP_ROWS,N),e.texSubImage2D(s.TEXTURE_2D,0,Ut,N,it,st,k,X,x.data)}A.clearUpdateRanges(),s.pixelStorei(s.UNPACK_ROW_LENGTH,nt),s.pixelStorei(s.UNPACK_SKIP_PIXELS,xt),s.pixelStorei(s.UNPACK_SKIP_ROWS,yt)}}function q(A,x,k){let X=s.TEXTURE_2D;(x.isDataArrayTexture||x.isCompressedArrayTexture)&&(X=s.TEXTURE_2D_ARRAY),x.isData3DTexture&&(X=s.TEXTURE_3D);const Z=se(A,x),V=x.source;e.bindTexture(X,A.__webglTexture,s.TEXTURE0+k);const St=i.get(V);if(V.version!==St.__version||Z===!0){e.activeTexture(s.TEXTURE0+k);const nt=qt.getPrimaries(qt.workingColorSpace),xt=x.colorSpace===Gi?null:qt.getPrimaries(x.colorSpace),yt=x.colorSpace===Gi||nt===xt?s.NONE:s.BROWSER_DEFAULT_WEBGL;s.pixelStorei(s.UNPACK_FLIP_Y_WEBGL,x.flipY),s.pixelStorei(s.UNPACK_PREMULTIPLY_ALPHA_WEBGL,x.premultiplyAlpha),s.pixelStorei(s.UNPACK_ALIGNMENT,x.unpackAlignment),s.pixelStorei(s.UNPACK_COLORSPACE_CONVERSION_WEBGL,yt);let et=v(x.image,!1,n.maxTextureSize);et=Se(x,et);const ct=r.convert(x.format,x.colorSpace),Ct=r.convert(x.type);let Mt=E(x.internalFormat,ct,Ct,x.colorSpace,x.isVideoTexture);Gt(X,x);let ot;const Ut=x.mipmaps,N=x.isVideoTexture!==!0,it=St.__version===void 0||Z===!0,st=V.dataReady,ft=P(x,et);if(x.isDepthTexture)Mt=_(x.format===Os,x.type),it&&(N?e.texStorage2D(s.TEXTURE_2D,1,Mt,et.width,et.height):e.texImage2D(s.TEXTURE_2D,0,Mt,et.width,et.height,0,ct,Ct,null));else if(x.isDataTexture)if(Ut.length>0){N&&it&&e.texStorage2D(s.TEXTURE_2D,ft,Mt,Ut[0].width,Ut[0].height);for(let Q=0,K=Ut.length;Q<K;Q++)ot=Ut[Q],N?st&&e.texSubImage2D(s.TEXTURE_2D,Q,0,0,ot.width,ot.height,ct,Ct,ot.data):e.texImage2D(s.TEXTURE_2D,Q,Mt,ot.width,ot.height,0,ct,Ct,ot.data);x.generateMipmaps=!1}else N?(it&&e.texStorage2D(s.TEXTURE_2D,ft,Mt,et.width,et.height),st&&Kt(x,et,ct,Ct)):e.texImage2D(s.TEXTURE_2D,0,Mt,et.width,et.height,0,ct,Ct,et.data);else if(x.isCompressedTexture)if(x.isCompressedArrayTexture){N&&it&&e.texStorage3D(s.TEXTURE_2D_ARRAY,ft,Mt,Ut[0].width,Ut[0].height,et.depth);for(let Q=0,K=Ut.length;Q<K;Q++)if(ot=Ut[Q],x.format!==ri)if(ct!==null)if(N){if(st)if(x.layerUpdates.size>0){const gt=fc(ot.width,ot.height,x.format,x.type);for(const Lt of x.layerUpdates){const re=ot.data.subarray(Lt*gt/ot.data.BYTES_PER_ELEMENT,(Lt+1)*gt/ot.data.BYTES_PER_ELEMENT);e.compressedTexSubImage3D(s.TEXTURE_2D_ARRAY,Q,0,0,Lt,ot.width,ot.height,1,ct,re)}x.clearLayerUpdates()}else e.compressedTexSubImage3D(s.TEXTURE_2D_ARRAY,Q,0,0,0,ot.width,ot.height,et.depth,ct,ot.data)}else e.compressedTexImage3D(s.TEXTURE_2D_ARRAY,Q,Mt,ot.width,ot.height,et.depth,0,ot.data,0,0);else console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()");else N?st&&e.texSubImage3D(s.TEXTURE_2D_ARRAY,Q,0,0,0,ot.width,ot.height,et.depth,ct,Ct,ot.data):e.texImage3D(s.TEXTURE_2D_ARRAY,Q,Mt,ot.width,ot.height,et.depth,0,ct,Ct,ot.data)}else{N&&it&&e.texStorage2D(s.TEXTURE_2D,ft,Mt,Ut[0].width,Ut[0].height);for(let Q=0,K=Ut.length;Q<K;Q++)ot=Ut[Q],x.format!==ri?ct!==null?N?st&&e.compressedTexSubImage2D(s.TEXTURE_2D,Q,0,0,ot.width,ot.height,ct,ot.data):e.compressedTexImage2D(s.TEXTURE_2D,Q,Mt,ot.width,ot.height,0,ot.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):N?st&&e.texSubImage2D(s.TEXTURE_2D,Q,0,0,ot.width,ot.height,ct,Ct,ot.data):e.texImage2D(s.TEXTURE_2D,Q,Mt,ot.width,ot.height,0,ct,Ct,ot.data)}else if(x.isDataArrayTexture)if(N){if(it&&e.texStorage3D(s.TEXTURE_2D_ARRAY,ft,Mt,et.width,et.height,et.depth),st)if(x.layerUpdates.size>0){const Q=fc(et.width,et.height,x.format,x.type);for(const K of x.layerUpdates){const gt=et.data.subarray(K*Q/et.data.BYTES_PER_ELEMENT,(K+1)*Q/et.data.BYTES_PER_ELEMENT);e.texSubImage3D(s.TEXTURE_2D_ARRAY,0,0,0,K,et.width,et.height,1,ct,Ct,gt)}x.clearLayerUpdates()}else e.texSubImage3D(s.TEXTURE_2D_ARRAY,0,0,0,0,et.width,et.height,et.depth,ct,Ct,et.data)}else e.texImage3D(s.TEXTURE_2D_ARRAY,0,Mt,et.width,et.height,et.depth,0,ct,Ct,et.data);else if(x.isData3DTexture)N?(it&&e.texStorage3D(s.TEXTURE_3D,ft,Mt,et.width,et.height,et.depth),st&&e.texSubImage3D(s.TEXTURE_3D,0,0,0,0,et.width,et.height,et.depth,ct,Ct,et.data)):e.texImage3D(s.TEXTURE_3D,0,Mt,et.width,et.height,et.depth,0,ct,Ct,et.data);else if(x.isFramebufferTexture){if(it)if(N)e.texStorage2D(s.TEXTURE_2D,ft,Mt,et.width,et.height);else{let Q=et.width,K=et.height;for(let gt=0;gt<ft;gt++)e.texImage2D(s.TEXTURE_2D,gt,Mt,Q,K,0,ct,Ct,null),Q>>=1,K>>=1}}else if(Ut.length>0){if(N&&it){const Q=me(Ut[0]);e.texStorage2D(s.TEXTURE_2D,ft,Mt,Q.width,Q.height)}for(let Q=0,K=Ut.length;Q<K;Q++)ot=Ut[Q],N?st&&e.texSubImage2D(s.TEXTURE_2D,Q,0,0,ct,Ct,ot):e.texImage2D(s.TEXTURE_2D,Q,Mt,ct,Ct,ot);x.generateMipmaps=!1}else if(N){if(it){const Q=me(et);e.texStorage2D(s.TEXTURE_2D,ft,Mt,Q.width,Q.height)}st&&e.texSubImage2D(s.TEXTURE_2D,0,0,0,ct,Ct,et)}else e.texImage2D(s.TEXTURE_2D,0,Mt,ct,Ct,et);m(x)&&p(X),St.__version=V.version,x.onUpdate&&x.onUpdate(x)}A.__version=x.version}function j(A,x,k){if(x.image.length!==6)return;const X=se(A,x),Z=x.source;e.bindTexture(s.TEXTURE_CUBE_MAP,A.__webglTexture,s.TEXTURE0+k);const V=i.get(Z);if(Z.version!==V.__version||X===!0){e.activeTexture(s.TEXTURE0+k);const St=qt.getPrimaries(qt.workingColorSpace),nt=x.colorSpace===Gi?null:qt.getPrimaries(x.colorSpace),xt=x.colorSpace===Gi||St===nt?s.NONE:s.BROWSER_DEFAULT_WEBGL;s.pixelStorei(s.UNPACK_FLIP_Y_WEBGL,x.flipY),s.pixelStorei(s.UNPACK_PREMULTIPLY_ALPHA_WEBGL,x.premultiplyAlpha),s.pixelStorei(s.UNPACK_ALIGNMENT,x.unpackAlignment),s.pixelStorei(s.UNPACK_COLORSPACE_CONVERSION_WEBGL,xt);const yt=x.isCompressedTexture||x.image[0].isCompressedTexture,et=x.image[0]&&x.image[0].isDataTexture,ct=[];for(let K=0;K<6;K++)!yt&&!et?ct[K]=v(x.image[K],!0,n.maxCubemapSize):ct[K]=et?x.image[K].image:x.image[K],ct[K]=Se(x,ct[K]);const Ct=ct[0],Mt=r.convert(x.format,x.colorSpace),ot=r.convert(x.type),Ut=E(x.internalFormat,Mt,ot,x.colorSpace),N=x.isVideoTexture!==!0,it=V.__version===void 0||X===!0,st=Z.dataReady;let ft=P(x,Ct);Gt(s.TEXTURE_CUBE_MAP,x);let Q;if(yt){N&&it&&e.texStorage2D(s.TEXTURE_CUBE_MAP,ft,Ut,Ct.width,Ct.height);for(let K=0;K<6;K++){Q=ct[K].mipmaps;for(let gt=0;gt<Q.length;gt++){const Lt=Q[gt];x.format!==ri?Mt!==null?N?st&&e.compressedTexSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,gt,0,0,Lt.width,Lt.height,Mt,Lt.data):e.compressedTexImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,gt,Ut,Lt.width,Lt.height,0,Lt.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):N?st&&e.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,gt,0,0,Lt.width,Lt.height,Mt,ot,Lt.data):e.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,gt,Ut,Lt.width,Lt.height,0,Mt,ot,Lt.data)}}}else{if(Q=x.mipmaps,N&&it){Q.length>0&&ft++;const K=me(ct[0]);e.texStorage2D(s.TEXTURE_CUBE_MAP,ft,Ut,K.width,K.height)}for(let K=0;K<6;K++)if(et){N?st&&e.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,0,0,0,ct[K].width,ct[K].height,Mt,ot,ct[K].data):e.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,0,Ut,ct[K].width,ct[K].height,0,Mt,ot,ct[K].data);for(let gt=0;gt<Q.length;gt++){const re=Q[gt].image[K].image;N?st&&e.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,gt+1,0,0,re.width,re.height,Mt,ot,re.data):e.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,gt+1,Ut,re.width,re.height,0,Mt,ot,re.data)}}else{N?st&&e.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,0,0,0,Mt,ot,ct[K]):e.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,0,Ut,Mt,ot,ct[K]);for(let gt=0;gt<Q.length;gt++){const Lt=Q[gt];N?st&&e.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,gt+1,0,0,Mt,ot,Lt.image[K]):e.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+K,gt+1,Ut,Mt,ot,Lt.image[K])}}}m(x)&&p(s.TEXTURE_CUBE_MAP),V.__version=Z.version,x.onUpdate&&x.onUpdate(x)}A.__version=x.version}function pt(A,x,k,X,Z,V){const St=r.convert(k.format,k.colorSpace),nt=r.convert(k.type),xt=E(k.internalFormat,St,nt,k.colorSpace),yt=i.get(x),et=i.get(k);if(et.__renderTarget=x,!yt.__hasExternalTextures){const ct=Math.max(1,x.width>>V),Ct=Math.max(1,x.height>>V);Z===s.TEXTURE_3D||Z===s.TEXTURE_2D_ARRAY?e.texImage3D(Z,V,xt,ct,Ct,x.depth,0,St,nt,null):e.texImage2D(Z,V,xt,ct,Ct,0,St,nt,null)}e.bindFramebuffer(s.FRAMEBUFFER,A),_t(x)?o.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,X,Z,et.__webglTexture,0,ce(x)):(Z===s.TEXTURE_2D||Z>=s.TEXTURE_CUBE_MAP_POSITIVE_X&&Z<=s.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&s.framebufferTexture2D(s.FRAMEBUFFER,X,Z,et.__webglTexture,V),e.bindFramebuffer(s.FRAMEBUFFER,null)}function Dt(A,x,k){if(s.bindRenderbuffer(s.RENDERBUFFER,A),x.depthBuffer){const X=x.depthTexture,Z=X&&X.isDepthTexture?X.type:null,V=_(x.stencilBuffer,Z),St=x.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,nt=ce(x);_t(x)?o.renderbufferStorageMultisampleEXT(s.RENDERBUFFER,nt,V,x.width,x.height):k?s.renderbufferStorageMultisample(s.RENDERBUFFER,nt,V,x.width,x.height):s.renderbufferStorage(s.RENDERBUFFER,V,x.width,x.height),s.framebufferRenderbuffer(s.FRAMEBUFFER,St,s.RENDERBUFFER,A)}else{const X=x.textures;for(let Z=0;Z<X.length;Z++){const V=X[Z],St=r.convert(V.format,V.colorSpace),nt=r.convert(V.type),xt=E(V.internalFormat,St,nt,V.colorSpace),yt=ce(x);k&&_t(x)===!1?s.renderbufferStorageMultisample(s.RENDERBUFFER,yt,xt,x.width,x.height):_t(x)?o.renderbufferStorageMultisampleEXT(s.RENDERBUFFER,yt,xt,x.width,x.height):s.renderbufferStorage(s.RENDERBUFFER,xt,x.width,x.height)}}s.bindRenderbuffer(s.RENDERBUFFER,null)}function bt(A,x){if(x&&x.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(e.bindFramebuffer(s.FRAMEBUFFER,A),!(x.depthTexture&&x.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");const X=i.get(x.depthTexture);X.__renderTarget=x,(!X.__webglTexture||x.depthTexture.image.width!==x.width||x.depthTexture.image.height!==x.height)&&(x.depthTexture.image.width=x.width,x.depthTexture.image.height=x.height,x.depthTexture.needsUpdate=!0),W(x.depthTexture,0);const Z=X.__webglTexture,V=ce(x);if(x.depthTexture.format===Us)_t(x)?o.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,s.DEPTH_ATTACHMENT,s.TEXTURE_2D,Z,0,V):s.framebufferTexture2D(s.FRAMEBUFFER,s.DEPTH_ATTACHMENT,s.TEXTURE_2D,Z,0);else if(x.depthTexture.format===Os)_t(x)?o.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,s.DEPTH_STENCIL_ATTACHMENT,s.TEXTURE_2D,Z,0,V):s.framebufferTexture2D(s.FRAMEBUFFER,s.DEPTH_STENCIL_ATTACHMENT,s.TEXTURE_2D,Z,0);else throw new Error("Unknown depthTexture format")}function $t(A){const x=i.get(A),k=A.isWebGLCubeRenderTarget===!0;if(x.__boundDepthTexture!==A.depthTexture){const X=A.depthTexture;if(x.__depthDisposeCallback&&x.__depthDisposeCallback(),X){const Z=()=>{delete x.__boundDepthTexture,delete x.__depthDisposeCallback,X.removeEventListener("dispose",Z)};X.addEventListener("dispose",Z),x.__depthDisposeCallback=Z}x.__boundDepthTexture=X}if(A.depthTexture&&!x.__autoAllocateDepthBuffer){if(k)throw new Error("target.depthTexture not supported in Cube render targets");const X=A.texture.mipmaps;X&&X.length>0?bt(x.__webglFramebuffer[0],A):bt(x.__webglFramebuffer,A)}else if(k){x.__webglDepthbuffer=[];for(let X=0;X<6;X++)if(e.bindFramebuffer(s.FRAMEBUFFER,x.__webglFramebuffer[X]),x.__webglDepthbuffer[X]===void 0)x.__webglDepthbuffer[X]=s.createRenderbuffer(),Dt(x.__webglDepthbuffer[X],A,!1);else{const Z=A.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,V=x.__webglDepthbuffer[X];s.bindRenderbuffer(s.RENDERBUFFER,V),s.framebufferRenderbuffer(s.FRAMEBUFFER,Z,s.RENDERBUFFER,V)}}else{const X=A.texture.mipmaps;if(X&&X.length>0?e.bindFramebuffer(s.FRAMEBUFFER,x.__webglFramebuffer[0]):e.bindFramebuffer(s.FRAMEBUFFER,x.__webglFramebuffer),x.__webglDepthbuffer===void 0)x.__webglDepthbuffer=s.createRenderbuffer(),Dt(x.__webglDepthbuffer,A,!1);else{const Z=A.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,V=x.__webglDepthbuffer;s.bindRenderbuffer(s.RENDERBUFFER,V),s.framebufferRenderbuffer(s.FRAMEBUFFER,Z,s.RENDERBUFFER,V)}}e.bindFramebuffer(s.FRAMEBUFFER,null)}function Te(A,x,k){const X=i.get(A);x!==void 0&&pt(X.__webglFramebuffer,A,A.texture,s.COLOR_ATTACHMENT0,s.TEXTURE_2D,0),k!==void 0&&$t(A)}function D(A){const x=A.texture,k=i.get(A),X=i.get(x);A.addEventListener("dispose",I);const Z=A.textures,V=A.isWebGLCubeRenderTarget===!0,St=Z.length>1;if(St||(X.__webglTexture===void 0&&(X.__webglTexture=s.createTexture()),X.__version=x.version,a.memory.textures++),V){k.__webglFramebuffer=[];for(let nt=0;nt<6;nt++)if(x.mipmaps&&x.mipmaps.length>0){k.__webglFramebuffer[nt]=[];for(let xt=0;xt<x.mipmaps.length;xt++)k.__webglFramebuffer[nt][xt]=s.createFramebuffer()}else k.__webglFramebuffer[nt]=s.createFramebuffer()}else{if(x.mipmaps&&x.mipmaps.length>0){k.__webglFramebuffer=[];for(let nt=0;nt<x.mipmaps.length;nt++)k.__webglFramebuffer[nt]=s.createFramebuffer()}else k.__webglFramebuffer=s.createFramebuffer();if(St)for(let nt=0,xt=Z.length;nt<xt;nt++){const yt=i.get(Z[nt]);yt.__webglTexture===void 0&&(yt.__webglTexture=s.createTexture(),a.memory.textures++)}if(A.samples>0&&_t(A)===!1){k.__webglMultisampledFramebuffer=s.createFramebuffer(),k.__webglColorRenderbuffer=[],e.bindFramebuffer(s.FRAMEBUFFER,k.__webglMultisampledFramebuffer);for(let nt=0;nt<Z.length;nt++){const xt=Z[nt];k.__webglColorRenderbuffer[nt]=s.createRenderbuffer(),s.bindRenderbuffer(s.RENDERBUFFER,k.__webglColorRenderbuffer[nt]);const yt=r.convert(xt.format,xt.colorSpace),et=r.convert(xt.type),ct=E(xt.internalFormat,yt,et,xt.colorSpace,A.isXRRenderTarget===!0),Ct=ce(A);s.renderbufferStorageMultisample(s.RENDERBUFFER,Ct,ct,A.width,A.height),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+nt,s.RENDERBUFFER,k.__webglColorRenderbuffer[nt])}s.bindRenderbuffer(s.RENDERBUFFER,null),A.depthBuffer&&(k.__webglDepthRenderbuffer=s.createRenderbuffer(),Dt(k.__webglDepthRenderbuffer,A,!0)),e.bindFramebuffer(s.FRAMEBUFFER,null)}}if(V){e.bindTexture(s.TEXTURE_CUBE_MAP,X.__webglTexture),Gt(s.TEXTURE_CUBE_MAP,x);for(let nt=0;nt<6;nt++)if(x.mipmaps&&x.mipmaps.length>0)for(let xt=0;xt<x.mipmaps.length;xt++)pt(k.__webglFramebuffer[nt][xt],A,x,s.COLOR_ATTACHMENT0,s.TEXTURE_CUBE_MAP_POSITIVE_X+nt,xt);else pt(k.__webglFramebuffer[nt],A,x,s.COLOR_ATTACHMENT0,s.TEXTURE_CUBE_MAP_POSITIVE_X+nt,0);m(x)&&p(s.TEXTURE_CUBE_MAP),e.unbindTexture()}else if(St){for(let nt=0,xt=Z.length;nt<xt;nt++){const yt=Z[nt],et=i.get(yt);let ct=s.TEXTURE_2D;(A.isWebGL3DRenderTarget||A.isWebGLArrayRenderTarget)&&(ct=A.isWebGL3DRenderTarget?s.TEXTURE_3D:s.TEXTURE_2D_ARRAY),e.bindTexture(ct,et.__webglTexture),Gt(ct,yt),pt(k.__webglFramebuffer,A,yt,s.COLOR_ATTACHMENT0+nt,ct,0),m(yt)&&p(ct)}e.unbindTexture()}else{let nt=s.TEXTURE_2D;if((A.isWebGL3DRenderTarget||A.isWebGLArrayRenderTarget)&&(nt=A.isWebGL3DRenderTarget?s.TEXTURE_3D:s.TEXTURE_2D_ARRAY),e.bindTexture(nt,X.__webglTexture),Gt(nt,x),x.mipmaps&&x.mipmaps.length>0)for(let xt=0;xt<x.mipmaps.length;xt++)pt(k.__webglFramebuffer[xt],A,x,s.COLOR_ATTACHMENT0,nt,xt);else pt(k.__webglFramebuffer,A,x,s.COLOR_ATTACHMENT0,nt,0);m(x)&&p(nt),e.unbindTexture()}A.depthBuffer&&$t(A)}function le(A){const x=A.textures;for(let k=0,X=x.length;k<X;k++){const Z=x[k];if(m(Z)){const V=w(A),St=i.get(Z).__webglTexture;e.bindTexture(V,St),p(V),e.unbindTexture()}}}const Nt=[],Pt=[];function vt(A){if(A.samples>0){if(_t(A)===!1){const x=A.textures,k=A.width,X=A.height;let Z=s.COLOR_BUFFER_BIT;const V=A.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,St=i.get(A),nt=x.length>1;if(nt)for(let yt=0;yt<x.length;yt++)e.bindFramebuffer(s.FRAMEBUFFER,St.__webglMultisampledFramebuffer),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+yt,s.RENDERBUFFER,null),e.bindFramebuffer(s.FRAMEBUFFER,St.__webglFramebuffer),s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0+yt,s.TEXTURE_2D,null,0);e.bindFramebuffer(s.READ_FRAMEBUFFER,St.__webglMultisampledFramebuffer);const xt=A.texture.mipmaps;xt&&xt.length>0?e.bindFramebuffer(s.DRAW_FRAMEBUFFER,St.__webglFramebuffer[0]):e.bindFramebuffer(s.DRAW_FRAMEBUFFER,St.__webglFramebuffer);for(let yt=0;yt<x.length;yt++){if(A.resolveDepthBuffer&&(A.depthBuffer&&(Z|=s.DEPTH_BUFFER_BIT),A.stencilBuffer&&A.resolveStencilBuffer&&(Z|=s.STENCIL_BUFFER_BIT)),nt){s.framebufferRenderbuffer(s.READ_FRAMEBUFFER,s.COLOR_ATTACHMENT0,s.RENDERBUFFER,St.__webglColorRenderbuffer[yt]);const et=i.get(x[yt]).__webglTexture;s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0,s.TEXTURE_2D,et,0)}s.blitFramebuffer(0,0,k,X,0,0,k,X,Z,s.NEAREST),c===!0&&(Nt.length=0,Pt.length=0,Nt.push(s.COLOR_ATTACHMENT0+yt),A.depthBuffer&&A.resolveDepthBuffer===!1&&(Nt.push(V),Pt.push(V),s.invalidateFramebuffer(s.DRAW_FRAMEBUFFER,Pt)),s.invalidateFramebuffer(s.READ_FRAMEBUFFER,Nt))}if(e.bindFramebuffer(s.READ_FRAMEBUFFER,null),e.bindFramebuffer(s.DRAW_FRAMEBUFFER,null),nt)for(let yt=0;yt<x.length;yt++){e.bindFramebuffer(s.FRAMEBUFFER,St.__webglMultisampledFramebuffer),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+yt,s.RENDERBUFFER,St.__webglColorRenderbuffer[yt]);const et=i.get(x[yt]).__webglTexture;e.bindFramebuffer(s.FRAMEBUFFER,St.__webglFramebuffer),s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0+yt,s.TEXTURE_2D,et,0)}e.bindFramebuffer(s.DRAW_FRAMEBUFFER,St.__webglMultisampledFramebuffer)}else if(A.depthBuffer&&A.resolveDepthBuffer===!1&&c){const x=A.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT;s.invalidateFramebuffer(s.DRAW_FRAMEBUFFER,[x])}}}function ce(A){return Math.min(n.maxSamples,A.samples)}function _t(A){const x=i.get(A);return A.samples>0&&t.has("WEBGL_multisampled_render_to_texture")===!0&&x.__useRenderToTexture!==!1}function Ft(A){const x=a.render.frame;h.get(A)!==x&&(h.set(A,x),A.update())}function Se(A,x){const k=A.colorSpace,X=A.format,Z=A.type;return A.isCompressedTexture===!0||A.isVideoTexture===!0||k!==ns&&k!==Gi&&(qt.getTransfer(k)===te?(X!==ri||Z!==mi)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",k)),x}function me(A){return typeof HTMLImageElement<"u"&&A instanceof HTMLImageElement?(l.width=A.naturalWidth||A.width,l.height=A.naturalHeight||A.height):typeof VideoFrame<"u"&&A instanceof VideoFrame?(l.width=A.displayWidth,l.height=A.displayHeight):(l.width=A.width,l.height=A.height),l}this.allocateTextureUnit=B,this.resetTextureUnits=F,this.setTexture2D=W,this.setTexture2DArray=$,this.setTexture3D=J,this.setTextureCube=G,this.rebindTextures=Te,this.setupRenderTarget=D,this.updateRenderTargetMipmap=le,this.updateMultisampleRenderTarget=vt,this.setupDepthRenderbuffer=$t,this.setupFrameBufferTexture=pt,this.useMultisampledRTT=_t}function mv(s,t){function e(i,n=Gi){let r;const a=qt.getTransfer(n);if(i===mi)return s.UNSIGNED_BYTE;if(i===$o)return s.UNSIGNED_SHORT_4_4_4_4;if(i===Yo)return s.UNSIGNED_SHORT_5_5_5_1;if(i===ah)return s.UNSIGNED_INT_5_9_9_9_REV;if(i===oh)return s.UNSIGNED_INT_10F_11F_11F_REV;if(i===sh)return s.BYTE;if(i===rh)return s.SHORT;if(i===Ls)return s.UNSIGNED_SHORT;if(i===Xo)return s.INT;if(i===gn)return s.UNSIGNED_INT;if(i===di)return s.FLOAT;if(i===Gs)return s.HALF_FLOAT;if(i===lh)return s.ALPHA;if(i===ch)return s.RGB;if(i===ri)return s.RGBA;if(i===Us)return s.DEPTH_COMPONENT;if(i===Os)return s.DEPTH_STENCIL;if(i===qo)return s.RED;if(i===Ko)return s.RED_INTEGER;if(i===hh)return s.RG;if(i===Zo)return s.RG_INTEGER;if(i===Jo)return s.RGBA_INTEGER;if(i===Ar||i===Rr||i===Cr||i===Pr)if(a===te)if(r=t.get("WEBGL_compressed_texture_s3tc_srgb"),r!==null){if(i===Ar)return r.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(i===Rr)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(i===Cr)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(i===Pr)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(r=t.get("WEBGL_compressed_texture_s3tc"),r!==null){if(i===Ar)return r.COMPRESSED_RGB_S3TC_DXT1_EXT;if(i===Rr)return r.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(i===Cr)return r.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(i===Pr)return r.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(i===eo||i===io||i===no||i===so)if(r=t.get("WEBGL_compressed_texture_pvrtc"),r!==null){if(i===eo)return r.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(i===io)return r.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(i===no)return r.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(i===so)return r.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(i===ro||i===ao||i===oo)if(r=t.get("WEBGL_compressed_texture_etc"),r!==null){if(i===ro||i===ao)return a===te?r.COMPRESSED_SRGB8_ETC2:r.COMPRESSED_RGB8_ETC2;if(i===oo)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:r.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(i===lo||i===co||i===ho||i===uo||i===fo||i===po||i===mo||i===go||i===vo||i===_o||i===xo||i===yo||i===Mo||i===So)if(r=t.get("WEBGL_compressed_texture_astc"),r!==null){if(i===lo)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:r.COMPRESSED_RGBA_ASTC_4x4_KHR;if(i===co)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:r.COMPRESSED_RGBA_ASTC_5x4_KHR;if(i===ho)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:r.COMPRESSED_RGBA_ASTC_5x5_KHR;if(i===uo)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:r.COMPRESSED_RGBA_ASTC_6x5_KHR;if(i===fo)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:r.COMPRESSED_RGBA_ASTC_6x6_KHR;if(i===po)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:r.COMPRESSED_RGBA_ASTC_8x5_KHR;if(i===mo)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:r.COMPRESSED_RGBA_ASTC_8x6_KHR;if(i===go)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:r.COMPRESSED_RGBA_ASTC_8x8_KHR;if(i===vo)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:r.COMPRESSED_RGBA_ASTC_10x5_KHR;if(i===_o)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:r.COMPRESSED_RGBA_ASTC_10x6_KHR;if(i===xo)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:r.COMPRESSED_RGBA_ASTC_10x8_KHR;if(i===yo)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:r.COMPRESSED_RGBA_ASTC_10x10_KHR;if(i===Mo)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:r.COMPRESSED_RGBA_ASTC_12x10_KHR;if(i===So)return a===te?r.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:r.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(i===bo||i===Eo||i===wo)if(r=t.get("EXT_texture_compression_bptc"),r!==null){if(i===bo)return a===te?r.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:r.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(i===Eo)return r.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(i===wo)return r.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(i===To||i===Ao||i===Ro||i===Co)if(r=t.get("EXT_texture_compression_rgtc"),r!==null){if(i===To)return r.COMPRESSED_RED_RGTC1_EXT;if(i===Ao)return r.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(i===Ro)return r.COMPRESSED_RED_GREEN_RGTC2_EXT;if(i===Co)return r.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return i===Ns?s.UNSIGNED_INT_24_8:s[i]!==void 0?s[i]:null}return{convert:e}}const gv=`
void main() {

	gl_Position = vec4( position, 1.0 );

}`,vv=`
uniform sampler2DArray depthColor;
uniform float depthWidth;
uniform float depthHeight;

void main() {

	vec2 coord = vec2( gl_FragCoord.x / depthWidth, gl_FragCoord.y / depthHeight );

	if ( coord.x >= 1.0 ) {

		gl_FragDepth = texture( depthColor, vec3( coord.x - 1.0, coord.y, 1 ) ).r;

	} else {

		gl_FragDepth = texture( depthColor, vec3( coord.x, coord.y, 0 ) ).r;

	}

}`;class _v{constructor(){this.texture=null,this.mesh=null,this.depthNear=0,this.depthFar=0}init(t,e){if(this.texture===null){const i=new Rh(t.texture);(t.depthNear!==e.depthNear||t.depthFar!==e.depthFar)&&(this.depthNear=t.depthNear,this.depthFar=t.depthFar),this.texture=i}}getMesh(t){if(this.texture!==null&&this.mesh===null){const e=t.cameras[0].viewport,i=new Yi({vertexShader:gv,fragmentShader:vv,uniforms:{depthColor:{value:this.texture},depthWidth:{value:e.z},depthHeight:{value:e.w}}});this.mesh=new Vt(new as(20,20),i)}return this.mesh}reset(){this.texture=null,this.mesh=null}getDepthTexture(){return this.texture}}class xv extends os{constructor(t,e){super();const i=this;let n=null,r=1,a=null,o="local-floor",c=1,l=null,h=null,u=null,d=null,f=null,g=null;const v=typeof XRWebGLBinding<"u",m=new _v,p={},w=e.getContextAttributes();let E=null,_=null;const P=[],R=[],I=new ht;let T=null;const S=new si;S.viewport=new pe;const b=new si;b.viewport=new pe;const L=[S,b],F=new Bf;let B=null,Y=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(q){let j=P[q];return j===void 0&&(j=new _a,P[q]=j),j.getTargetRaySpace()},this.getControllerGrip=function(q){let j=P[q];return j===void 0&&(j=new _a,P[q]=j),j.getGripSpace()},this.getHand=function(q){let j=P[q];return j===void 0&&(j=new _a,P[q]=j),j.getHandSpace()};function W(q){const j=R.indexOf(q.inputSource);if(j===-1)return;const pt=P[j];pt!==void 0&&(pt.update(q.inputSource,q.frame,l||a),pt.dispatchEvent({type:q.type,data:q.inputSource}))}function $(){n.removeEventListener("select",W),n.removeEventListener("selectstart",W),n.removeEventListener("selectend",W),n.removeEventListener("squeeze",W),n.removeEventListener("squeezestart",W),n.removeEventListener("squeezeend",W),n.removeEventListener("end",$),n.removeEventListener("inputsourceschange",J);for(let q=0;q<P.length;q++){const j=R[q];j!==null&&(R[q]=null,P[q].disconnect(j))}B=null,Y=null,m.reset();for(const q in p)delete p[q];t.setRenderTarget(E),f=null,d=null,u=null,n=null,_=null,Kt.stop(),i.isPresenting=!1,t.setPixelRatio(T),t.setSize(I.width,I.height,!1),i.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(q){r=q,i.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(q){o=q,i.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return l||a},this.setReferenceSpace=function(q){l=q},this.getBaseLayer=function(){return d!==null?d:f},this.getBinding=function(){return u===null&&v&&(u=new XRWebGLBinding(n,e)),u},this.getFrame=function(){return g},this.getSession=function(){return n},this.setSession=async function(q){if(n=q,n!==null){if(E=t.getRenderTarget(),n.addEventListener("select",W),n.addEventListener("selectstart",W),n.addEventListener("selectend",W),n.addEventListener("squeeze",W),n.addEventListener("squeezestart",W),n.addEventListener("squeezeend",W),n.addEventListener("end",$),n.addEventListener("inputsourceschange",J),w.xrCompatible!==!0&&await e.makeXRCompatible(),T=t.getPixelRatio(),t.getSize(I),v&&"createProjectionLayer"in XRWebGLBinding.prototype){let pt=null,Dt=null,bt=null;w.depth&&(bt=w.stencil?e.DEPTH24_STENCIL8:e.DEPTH_COMPONENT24,pt=w.stencil?Os:Us,Dt=w.stencil?Ns:gn);const $t={colorFormat:e.RGBA8,depthFormat:bt,scaleFactor:r};u=this.getBinding(),d=u.createProjectionLayer($t),n.updateRenderState({layers:[d]}),t.setPixelRatio(1),t.setSize(d.textureWidth,d.textureHeight,!1),_=new vn(d.textureWidth,d.textureHeight,{format:ri,type:mi,depthTexture:new Ah(d.textureWidth,d.textureHeight,Dt,void 0,void 0,void 0,void 0,void 0,void 0,pt),stencilBuffer:w.stencil,colorSpace:t.outputColorSpace,samples:w.antialias?4:0,resolveDepthBuffer:d.ignoreDepthValues===!1,resolveStencilBuffer:d.ignoreDepthValues===!1})}else{const pt={antialias:w.antialias,alpha:!0,depth:w.depth,stencil:w.stencil,framebufferScaleFactor:r};f=new XRWebGLLayer(n,e,pt),n.updateRenderState({baseLayer:f}),t.setPixelRatio(1),t.setSize(f.framebufferWidth,f.framebufferHeight,!1),_=new vn(f.framebufferWidth,f.framebufferHeight,{format:ri,type:mi,colorSpace:t.outputColorSpace,stencilBuffer:w.stencil,resolveDepthBuffer:f.ignoreDepthValues===!1,resolveStencilBuffer:f.ignoreDepthValues===!1})}_.isXRRenderTarget=!0,this.setFoveation(c),l=null,a=await n.requestReferenceSpace(o),Kt.setContext(n),Kt.start(),i.isPresenting=!0,i.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(n!==null)return n.environmentBlendMode},this.getDepthTexture=function(){return m.getDepthTexture()};function J(q){for(let j=0;j<q.removed.length;j++){const pt=q.removed[j],Dt=R.indexOf(pt);Dt>=0&&(R[Dt]=null,P[Dt].disconnect(pt))}for(let j=0;j<q.added.length;j++){const pt=q.added[j];let Dt=R.indexOf(pt);if(Dt===-1){for(let $t=0;$t<P.length;$t++)if($t>=R.length){R.push(pt),Dt=$t;break}else if(R[$t]===null){R[$t]=pt,Dt=$t;break}if(Dt===-1)break}const bt=P[Dt];bt&&bt.connect(pt)}}const G=new C,at=new C;function ut(q,j,pt){G.setFromMatrixPosition(j.matrixWorld),at.setFromMatrixPosition(pt.matrixWorld);const Dt=G.distanceTo(at),bt=j.projectionMatrix.elements,$t=pt.projectionMatrix.elements,Te=bt[14]/(bt[10]-1),D=bt[14]/(bt[10]+1),le=(bt[9]+1)/bt[5],Nt=(bt[9]-1)/bt[5],Pt=(bt[8]-1)/bt[0],vt=($t[8]+1)/$t[0],ce=Te*Pt,_t=Te*vt,Ft=Dt/(-Pt+vt),Se=Ft*-Pt;if(j.matrixWorld.decompose(q.position,q.quaternion,q.scale),q.translateX(Se),q.translateZ(Ft),q.matrixWorld.compose(q.position,q.quaternion,q.scale),q.matrixWorldInverse.copy(q.matrixWorld).invert(),bt[10]===-1)q.projectionMatrix.copy(j.projectionMatrix),q.projectionMatrixInverse.copy(j.projectionMatrixInverse);else{const me=Te+Ft,A=D+Ft,x=ce-Se,k=_t+(Dt-Se),X=le*D/A*me,Z=Nt*D/A*me;q.projectionMatrix.makePerspective(x,k,X,Z,me,A),q.projectionMatrixInverse.copy(q.projectionMatrix).invert()}}function Et(q,j){j===null?q.matrixWorld.copy(q.matrix):q.matrixWorld.multiplyMatrices(j.matrixWorld,q.matrix),q.matrixWorldInverse.copy(q.matrixWorld).invert()}this.updateCamera=function(q){if(n===null)return;let j=q.near,pt=q.far;m.texture!==null&&(m.depthNear>0&&(j=m.depthNear),m.depthFar>0&&(pt=m.depthFar)),F.near=b.near=S.near=j,F.far=b.far=S.far=pt,(B!==F.near||Y!==F.far)&&(n.updateRenderState({depthNear:F.near,depthFar:F.far}),B=F.near,Y=F.far),F.layers.mask=q.layers.mask|6,S.layers.mask=F.layers.mask&3,b.layers.mask=F.layers.mask&5;const Dt=q.parent,bt=F.cameras;Et(F,Dt);for(let $t=0;$t<bt.length;$t++)Et(bt[$t],Dt);bt.length===2?ut(F,S,b):F.projectionMatrix.copy(S.projectionMatrix),Gt(q,F,Dt)};function Gt(q,j,pt){pt===null?q.matrix.copy(j.matrixWorld):(q.matrix.copy(pt.matrixWorld),q.matrix.invert(),q.matrix.multiply(j.matrixWorld)),q.matrix.decompose(q.position,q.quaternion,q.scale),q.updateMatrixWorld(!0),q.projectionMatrix.copy(j.projectionMatrix),q.projectionMatrixInverse.copy(j.projectionMatrixInverse),q.isPerspectiveCamera&&(q.fov=Io*2*Math.atan(1/q.projectionMatrix.elements[5]),q.zoom=1)}this.getCamera=function(){return F},this.getFoveation=function(){if(!(d===null&&f===null))return c},this.setFoveation=function(q){c=q,d!==null&&(d.fixedFoveation=q),f!==null&&f.fixedFoveation!==void 0&&(f.fixedFoveation=q)},this.hasDepthSensing=function(){return m.texture!==null},this.getDepthSensingMesh=function(){return m.getMesh(F)},this.getCameraTexture=function(q){return p[q]};let se=null;function oe(q,j){if(h=j.getViewerPose(l||a),g=j,h!==null){const pt=h.views;f!==null&&(t.setRenderTargetFramebuffer(_,f.framebuffer),t.setRenderTarget(_));let Dt=!1;pt.length!==F.cameras.length&&(F.cameras.length=0,Dt=!0);for(let D=0;D<pt.length;D++){const le=pt[D];let Nt=null;if(f!==null)Nt=f.getViewport(le);else{const vt=u.getViewSubImage(d,le);Nt=vt.viewport,D===0&&(t.setRenderTargetTextures(_,vt.colorTexture,vt.depthStencilTexture),t.setRenderTarget(_))}let Pt=L[D];Pt===void 0&&(Pt=new si,Pt.layers.enable(D),Pt.viewport=new pe,L[D]=Pt),Pt.matrix.fromArray(le.transform.matrix),Pt.matrix.decompose(Pt.position,Pt.quaternion,Pt.scale),Pt.projectionMatrix.fromArray(le.projectionMatrix),Pt.projectionMatrixInverse.copy(Pt.projectionMatrix).invert(),Pt.viewport.set(Nt.x,Nt.y,Nt.width,Nt.height),D===0&&(F.matrix.copy(Pt.matrix),F.matrix.decompose(F.position,F.quaternion,F.scale)),Dt===!0&&F.cameras.push(Pt)}const bt=n.enabledFeatures;if(bt&&bt.includes("depth-sensing")&&n.depthUsage=="gpu-optimized"&&v){u=i.getBinding();const D=u.getDepthInformation(pt[0]);D&&D.isValid&&D.texture&&m.init(D,n.renderState)}if(bt&&bt.includes("camera-access")&&v){t.state.unbindTexture(),u=i.getBinding();for(let D=0;D<pt.length;D++){const le=pt[D].camera;if(le){let Nt=p[le];Nt||(Nt=new Rh,p[le]=Nt);const Pt=u.getCameraImage(le);Nt.sourceTexture=Pt}}}}for(let pt=0;pt<P.length;pt++){const Dt=R[pt],bt=P[pt];Dt!==null&&bt!==void 0&&bt.update(Dt,j,l||a)}se&&se(q,j),j.detectedPlanes&&i.dispatchEvent({type:"planesdetected",data:j}),g=null}const Kt=new Bh;Kt.setAnimationLoop(oe),this.setAnimationLoop=function(q){se=q},this.dispose=function(){}}}const en=new gi,yv=new ie;function Mv(s,t){function e(m,p){m.matrixAutoUpdate===!0&&m.updateMatrix(),p.value.copy(m.matrix)}function i(m,p){p.color.getRGB(m.fogColor.value,Mh(s)),p.isFog?(m.fogNear.value=p.near,m.fogFar.value=p.far):p.isFogExp2&&(m.fogDensity.value=p.density)}function n(m,p,w,E,_){p.isMeshBasicMaterial||p.isMeshLambertMaterial?r(m,p):p.isMeshToonMaterial?(r(m,p),u(m,p)):p.isMeshPhongMaterial?(r(m,p),h(m,p)):p.isMeshStandardMaterial?(r(m,p),d(m,p),p.isMeshPhysicalMaterial&&f(m,p,_)):p.isMeshMatcapMaterial?(r(m,p),g(m,p)):p.isMeshDepthMaterial?r(m,p):p.isMeshDistanceMaterial?(r(m,p),v(m,p)):p.isMeshNormalMaterial?r(m,p):p.isLineBasicMaterial?(a(m,p),p.isLineDashedMaterial&&o(m,p)):p.isPointsMaterial?c(m,p,w,E):p.isSpriteMaterial?l(m,p):p.isShadowMaterial?(m.color.value.copy(p.color),m.opacity.value=p.opacity):p.isShaderMaterial&&(p.uniformsNeedUpdate=!1)}function r(m,p){m.opacity.value=p.opacity,p.color&&m.diffuse.value.copy(p.color),p.emissive&&m.emissive.value.copy(p.emissive).multiplyScalar(p.emissiveIntensity),p.map&&(m.map.value=p.map,e(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.bumpMap&&(m.bumpMap.value=p.bumpMap,e(p.bumpMap,m.bumpMapTransform),m.bumpScale.value=p.bumpScale,p.side===Oe&&(m.bumpScale.value*=-1)),p.normalMap&&(m.normalMap.value=p.normalMap,e(p.normalMap,m.normalMapTransform),m.normalScale.value.copy(p.normalScale),p.side===Oe&&m.normalScale.value.negate()),p.displacementMap&&(m.displacementMap.value=p.displacementMap,e(p.displacementMap,m.displacementMapTransform),m.displacementScale.value=p.displacementScale,m.displacementBias.value=p.displacementBias),p.emissiveMap&&(m.emissiveMap.value=p.emissiveMap,e(p.emissiveMap,m.emissiveMapTransform)),p.specularMap&&(m.specularMap.value=p.specularMap,e(p.specularMap,m.specularMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest);const w=t.get(p),E=w.envMap,_=w.envMapRotation;E&&(m.envMap.value=E,en.copy(_),en.x*=-1,en.y*=-1,en.z*=-1,E.isCubeTexture&&E.isRenderTargetTexture===!1&&(en.y*=-1,en.z*=-1),m.envMapRotation.value.setFromMatrix4(yv.makeRotationFromEuler(en)),m.flipEnvMap.value=E.isCubeTexture&&E.isRenderTargetTexture===!1?-1:1,m.reflectivity.value=p.reflectivity,m.ior.value=p.ior,m.refractionRatio.value=p.refractionRatio),p.lightMap&&(m.lightMap.value=p.lightMap,m.lightMapIntensity.value=p.lightMapIntensity,e(p.lightMap,m.lightMapTransform)),p.aoMap&&(m.aoMap.value=p.aoMap,m.aoMapIntensity.value=p.aoMapIntensity,e(p.aoMap,m.aoMapTransform))}function a(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,p.map&&(m.map.value=p.map,e(p.map,m.mapTransform))}function o(m,p){m.dashSize.value=p.dashSize,m.totalSize.value=p.dashSize+p.gapSize,m.scale.value=p.scale}function c(m,p,w,E){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.size.value=p.size*w,m.scale.value=E*.5,p.map&&(m.map.value=p.map,e(p.map,m.uvTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function l(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.rotation.value=p.rotation,p.map&&(m.map.value=p.map,e(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,e(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function h(m,p){m.specular.value.copy(p.specular),m.shininess.value=Math.max(p.shininess,1e-4)}function u(m,p){p.gradientMap&&(m.gradientMap.value=p.gradientMap)}function d(m,p){m.metalness.value=p.metalness,p.metalnessMap&&(m.metalnessMap.value=p.metalnessMap,e(p.metalnessMap,m.metalnessMapTransform)),m.roughness.value=p.roughness,p.roughnessMap&&(m.roughnessMap.value=p.roughnessMap,e(p.roughnessMap,m.roughnessMapTransform)),p.envMap&&(m.envMapIntensity.value=p.envMapIntensity)}function f(m,p,w){m.ior.value=p.ior,p.sheen>0&&(m.sheenColor.value.copy(p.sheenColor).multiplyScalar(p.sheen),m.sheenRoughness.value=p.sheenRoughness,p.sheenColorMap&&(m.sheenColorMap.value=p.sheenColorMap,e(p.sheenColorMap,m.sheenColorMapTransform)),p.sheenRoughnessMap&&(m.sheenRoughnessMap.value=p.sheenRoughnessMap,e(p.sheenRoughnessMap,m.sheenRoughnessMapTransform))),p.clearcoat>0&&(m.clearcoat.value=p.clearcoat,m.clearcoatRoughness.value=p.clearcoatRoughness,p.clearcoatMap&&(m.clearcoatMap.value=p.clearcoatMap,e(p.clearcoatMap,m.clearcoatMapTransform)),p.clearcoatRoughnessMap&&(m.clearcoatRoughnessMap.value=p.clearcoatRoughnessMap,e(p.clearcoatRoughnessMap,m.clearcoatRoughnessMapTransform)),p.clearcoatNormalMap&&(m.clearcoatNormalMap.value=p.clearcoatNormalMap,e(p.clearcoatNormalMap,m.clearcoatNormalMapTransform),m.clearcoatNormalScale.value.copy(p.clearcoatNormalScale),p.side===Oe&&m.clearcoatNormalScale.value.negate())),p.dispersion>0&&(m.dispersion.value=p.dispersion),p.iridescence>0&&(m.iridescence.value=p.iridescence,m.iridescenceIOR.value=p.iridescenceIOR,m.iridescenceThicknessMinimum.value=p.iridescenceThicknessRange[0],m.iridescenceThicknessMaximum.value=p.iridescenceThicknessRange[1],p.iridescenceMap&&(m.iridescenceMap.value=p.iridescenceMap,e(p.iridescenceMap,m.iridescenceMapTransform)),p.iridescenceThicknessMap&&(m.iridescenceThicknessMap.value=p.iridescenceThicknessMap,e(p.iridescenceThicknessMap,m.iridescenceThicknessMapTransform))),p.transmission>0&&(m.transmission.value=p.transmission,m.transmissionSamplerMap.value=w.texture,m.transmissionSamplerSize.value.set(w.width,w.height),p.transmissionMap&&(m.transmissionMap.value=p.transmissionMap,e(p.transmissionMap,m.transmissionMapTransform)),m.thickness.value=p.thickness,p.thicknessMap&&(m.thicknessMap.value=p.thicknessMap,e(p.thicknessMap,m.thicknessMapTransform)),m.attenuationDistance.value=p.attenuationDistance,m.attenuationColor.value.copy(p.attenuationColor)),p.anisotropy>0&&(m.anisotropyVector.value.set(p.anisotropy*Math.cos(p.anisotropyRotation),p.anisotropy*Math.sin(p.anisotropyRotation)),p.anisotropyMap&&(m.anisotropyMap.value=p.anisotropyMap,e(p.anisotropyMap,m.anisotropyMapTransform))),m.specularIntensity.value=p.specularIntensity,m.specularColor.value.copy(p.specularColor),p.specularColorMap&&(m.specularColorMap.value=p.specularColorMap,e(p.specularColorMap,m.specularColorMapTransform)),p.specularIntensityMap&&(m.specularIntensityMap.value=p.specularIntensityMap,e(p.specularIntensityMap,m.specularIntensityMapTransform))}function g(m,p){p.matcap&&(m.matcap.value=p.matcap)}function v(m,p){const w=t.get(p).light;m.referencePosition.value.setFromMatrixPosition(w.matrixWorld),m.nearDistance.value=w.shadow.camera.near,m.farDistance.value=w.shadow.camera.far}return{refreshFogUniforms:i,refreshMaterialUniforms:n}}function Sv(s,t,e,i){let n={},r={},a=[];const o=s.getParameter(s.MAX_UNIFORM_BUFFER_BINDINGS);function c(w,E){const _=E.program;i.uniformBlockBinding(w,_)}function l(w,E){let _=n[w.id];_===void 0&&(g(w),_=h(w),n[w.id]=_,w.addEventListener("dispose",m));const P=E.program;i.updateUBOMapping(w,P);const R=t.render.frame;r[w.id]!==R&&(d(w),r[w.id]=R)}function h(w){const E=u();w.__bindingPointIndex=E;const _=s.createBuffer(),P=w.__size,R=w.usage;return s.bindBuffer(s.UNIFORM_BUFFER,_),s.bufferData(s.UNIFORM_BUFFER,P,R),s.bindBuffer(s.UNIFORM_BUFFER,null),s.bindBufferBase(s.UNIFORM_BUFFER,E,_),_}function u(){for(let w=0;w<o;w++)if(a.indexOf(w)===-1)return a.push(w),w;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function d(w){const E=n[w.id],_=w.uniforms,P=w.__cache;s.bindBuffer(s.UNIFORM_BUFFER,E);for(let R=0,I=_.length;R<I;R++){const T=Array.isArray(_[R])?_[R]:[_[R]];for(let S=0,b=T.length;S<b;S++){const L=T[S];if(f(L,R,S,P)===!0){const F=L.__offset,B=Array.isArray(L.value)?L.value:[L.value];let Y=0;for(let W=0;W<B.length;W++){const $=B[W],J=v($);typeof $=="number"||typeof $=="boolean"?(L.__data[0]=$,s.bufferSubData(s.UNIFORM_BUFFER,F+Y,L.__data)):$.isMatrix3?(L.__data[0]=$.elements[0],L.__data[1]=$.elements[1],L.__data[2]=$.elements[2],L.__data[3]=0,L.__data[4]=$.elements[3],L.__data[5]=$.elements[4],L.__data[6]=$.elements[5],L.__data[7]=0,L.__data[8]=$.elements[6],L.__data[9]=$.elements[7],L.__data[10]=$.elements[8],L.__data[11]=0):($.toArray(L.__data,Y),Y+=J.storage/Float32Array.BYTES_PER_ELEMENT)}s.bufferSubData(s.UNIFORM_BUFFER,F,L.__data)}}}s.bindBuffer(s.UNIFORM_BUFFER,null)}function f(w,E,_,P){const R=w.value,I=E+"_"+_;if(P[I]===void 0)return typeof R=="number"||typeof R=="boolean"?P[I]=R:P[I]=R.clone(),!0;{const T=P[I];if(typeof R=="number"||typeof R=="boolean"){if(T!==R)return P[I]=R,!0}else if(T.equals(R)===!1)return T.copy(R),!0}return!1}function g(w){const E=w.uniforms;let _=0;const P=16;for(let I=0,T=E.length;I<T;I++){const S=Array.isArray(E[I])?E[I]:[E[I]];for(let b=0,L=S.length;b<L;b++){const F=S[b],B=Array.isArray(F.value)?F.value:[F.value];for(let Y=0,W=B.length;Y<W;Y++){const $=B[Y],J=v($),G=_%P,at=G%J.boundary,ut=G+at;_+=at,ut!==0&&P-ut<J.storage&&(_+=P-ut),F.__data=new Float32Array(J.storage/Float32Array.BYTES_PER_ELEMENT),F.__offset=_,_+=J.storage}}}const R=_%P;return R>0&&(_+=P-R),w.__size=_,w.__cache={},this}function v(w){const E={boundary:0,storage:0};return typeof w=="number"||typeof w=="boolean"?(E.boundary=4,E.storage=4):w.isVector2?(E.boundary=8,E.storage=8):w.isVector3||w.isColor?(E.boundary=16,E.storage=12):w.isVector4?(E.boundary=16,E.storage=16):w.isMatrix3?(E.boundary=48,E.storage=48):w.isMatrix4?(E.boundary=64,E.storage=64):w.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",w),E}function m(w){const E=w.target;E.removeEventListener("dispose",m);const _=a.indexOf(E.__bindingPointIndex);a.splice(_,1),s.deleteBuffer(n[E.id]),delete n[E.id],delete r[E.id]}function p(){for(const w in n)s.deleteBuffer(n[w]);a=[],n={},r={}}return{bind:c,update:l,dispose:p}}class bv{constructor(t={}){const{canvas:e=wd(),context:i=null,depth:n=!0,stencil:r=!1,alpha:a=!1,antialias:o=!1,premultipliedAlpha:c=!0,preserveDrawingBuffer:l=!1,powerPreference:h="default",failIfMajorPerformanceCaveat:u=!1,reversedDepthBuffer:d=!1}=t;this.isWebGLRenderer=!0;let f;if(i!==null){if(typeof WebGLRenderingContext<"u"&&i instanceof WebGLRenderingContext)throw new Error("THREE.WebGLRenderer: WebGL 1 is not supported since r163.");f=i.getContextAttributes().alpha}else f=a;const g=new Uint32Array(4),v=new Int32Array(4);let m=null,p=null;const w=[],E=[];this.domElement=e,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this.toneMapping=Wi,this.toneMappingExposure=1,this.transmissionResolutionScale=1;const _=this;let P=!1;this._outputColorSpace=Ve;let R=0,I=0,T=null,S=-1,b=null;const L=new pe,F=new pe;let B=null;const Y=new Ht(0);let W=0,$=e.width,J=e.height,G=1,at=null,ut=null;const Et=new pe(0,0,$,J),Gt=new pe(0,0,$,J);let se=!1;const oe=new tl;let Kt=!1,q=!1;const j=new ie,pt=new C,Dt=new pe,bt={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0};let $t=!1;function Te(){return T===null?G:1}let D=i;function le(M,U){return e.getContext(M,U)}try{const M={alpha:!0,depth:n,stencil:r,antialias:o,premultipliedAlpha:c,preserveDrawingBuffer:l,powerPreference:h,failIfMajorPerformanceCaveat:u};if("setAttribute"in e&&e.setAttribute("data-engine",`three.js r${Wo}`),e.addEventListener("webglcontextlost",st,!1),e.addEventListener("webglcontextrestored",ft,!1),e.addEventListener("webglcontextcreationerror",Q,!1),D===null){const U="webgl2";if(D=le(U,M),D===null)throw le(U)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}}catch(M){throw console.error("THREE.WebGLRenderer: "+M.message),M}let Nt,Pt,vt,ce,_t,Ft,Se,me,A,x,k,X,Z,V,St,nt,xt,yt,et,ct,Ct,Mt,ot,Ut;function N(){Nt=new Lg(D),Nt.init(),Mt=new mv(D,Nt),Pt=new Tg(D,Nt,t,Mt),vt=new fv(D,Nt),Pt.reversedDepthBuffer&&d&&vt.buffers.depth.setReversed(!0),ce=new Og(D),_t=new tv,Ft=new pv(D,Nt,vt,_t,Pt,Mt,ce),Se=new Rg(_),me=new Dg(_),A=new Gf(D),ot=new Eg(D,A),x=new Ng(D,A,ce,ot),k=new kg(D,x,A,ce),et=new Fg(D,Pt,Ft),nt=new Ag(_t),X=new Q0(_,Se,me,Nt,Pt,ot,nt),Z=new Mv(_,_t),V=new iv,St=new lv(Nt),yt=new bg(_,Se,me,vt,k,f,c),xt=new uv(_,k,Pt),Ut=new Sv(D,ce,Pt,vt),ct=new wg(D,Nt,ce),Ct=new Ug(D,Nt,ce),ce.programs=X.programs,_.capabilities=Pt,_.extensions=Nt,_.properties=_t,_.renderLists=V,_.shadowMap=xt,_.state=vt,_.info=ce}N();const it=new xv(_,D);this.xr=it,this.getContext=function(){return D},this.getContextAttributes=function(){return D.getContextAttributes()},this.forceContextLoss=function(){const M=Nt.get("WEBGL_lose_context");M&&M.loseContext()},this.forceContextRestore=function(){const M=Nt.get("WEBGL_lose_context");M&&M.restoreContext()},this.getPixelRatio=function(){return G},this.setPixelRatio=function(M){M!==void 0&&(G=M,this.setSize($,J,!1))},this.getSize=function(M){return M.set($,J)},this.setSize=function(M,U,z=!0){if(it.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}$=M,J=U,e.width=Math.floor(M*G),e.height=Math.floor(U*G),z===!0&&(e.style.width=M+"px",e.style.height=U+"px"),this.setViewport(0,0,M,U)},this.getDrawingBufferSize=function(M){return M.set($*G,J*G).floor()},this.setDrawingBufferSize=function(M,U,z){$=M,J=U,G=z,e.width=Math.floor(M*z),e.height=Math.floor(U*z),this.setViewport(0,0,M,U)},this.getCurrentViewport=function(M){return M.copy(L)},this.getViewport=function(M){return M.copy(Et)},this.setViewport=function(M,U,z,H){M.isVector4?Et.set(M.x,M.y,M.z,M.w):Et.set(M,U,z,H),vt.viewport(L.copy(Et).multiplyScalar(G).round())},this.getScissor=function(M){return M.copy(Gt)},this.setScissor=function(M,U,z,H){M.isVector4?Gt.set(M.x,M.y,M.z,M.w):Gt.set(M,U,z,H),vt.scissor(F.copy(Gt).multiplyScalar(G).round())},this.getScissorTest=function(){return se},this.setScissorTest=function(M){vt.setScissorTest(se=M)},this.setOpaqueSort=function(M){at=M},this.setTransparentSort=function(M){ut=M},this.getClearColor=function(M){return M.copy(yt.getClearColor())},this.setClearColor=function(){yt.setClearColor(...arguments)},this.getClearAlpha=function(){return yt.getClearAlpha()},this.setClearAlpha=function(){yt.setClearAlpha(...arguments)},this.clear=function(M=!0,U=!0,z=!0){let H=0;if(M){let O=!1;if(T!==null){const tt=T.texture.format;O=tt===Jo||tt===Zo||tt===Ko}if(O){const tt=T.texture.type,lt=tt===mi||tt===gn||tt===Ls||tt===Ns||tt===$o||tt===Yo,mt=yt.getClearColor(),dt=yt.getClearAlpha(),Rt=mt.r,It=mt.g,wt=mt.b;lt?(g[0]=Rt,g[1]=It,g[2]=wt,g[3]=dt,D.clearBufferuiv(D.COLOR,0,g)):(v[0]=Rt,v[1]=It,v[2]=wt,v[3]=dt,D.clearBufferiv(D.COLOR,0,v))}else H|=D.COLOR_BUFFER_BIT}U&&(H|=D.DEPTH_BUFFER_BIT),z&&(H|=D.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),D.clear(H)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){e.removeEventListener("webglcontextlost",st,!1),e.removeEventListener("webglcontextrestored",ft,!1),e.removeEventListener("webglcontextcreationerror",Q,!1),yt.dispose(),V.dispose(),St.dispose(),_t.dispose(),Se.dispose(),me.dispose(),k.dispose(),ot.dispose(),Ut.dispose(),X.dispose(),it.dispose(),it.removeEventListener("sessionstart",oi),it.removeEventListener("sessionend",cl),qi.stop()};function st(M){M.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),P=!0}function ft(){console.log("THREE.WebGLRenderer: Context Restored."),P=!1;const M=ce.autoReset,U=xt.enabled,z=xt.autoUpdate,H=xt.needsUpdate,O=xt.type;N(),ce.autoReset=M,xt.enabled=U,xt.autoUpdate=z,xt.needsUpdate=H,xt.type=O}function Q(M){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",M.statusMessage)}function K(M){const U=M.target;U.removeEventListener("dispose",K),gt(U)}function gt(M){Lt(M),_t.remove(M)}function Lt(M){const U=_t.get(M).programs;U!==void 0&&(U.forEach(function(z){X.releaseProgram(z)}),M.isShaderMaterial&&X.releaseShaderCache(M))}this.renderBufferDirect=function(M,U,z,H,O,tt){U===null&&(U=bt);const lt=O.isMesh&&O.matrixWorld.determinant()<0,mt=qh(M,U,z,H,O);vt.setMaterial(H,lt);let dt=z.index,Rt=1;if(H.wireframe===!0){if(dt=x.getWireframeAttribute(z),dt===void 0)return;Rt=2}const It=z.drawRange,wt=z.attributes.position;let Xt=It.start*Rt,Qt=(It.start+It.count)*Rt;tt!==null&&(Xt=Math.max(Xt,tt.start*Rt),Qt=Math.min(Qt,(tt.start+tt.count)*Rt)),dt!==null?(Xt=Math.max(Xt,0),Qt=Math.min(Qt,dt.count)):wt!=null&&(Xt=Math.max(Xt,0),Qt=Math.min(Qt,wt.count));const fe=Qt-Xt;if(fe<0||fe===1/0)return;ot.setup(O,H,mt,z,dt);let ae,ne=ct;if(dt!==null&&(ae=A.get(dt),ne=Ct,ne.setIndex(ae)),O.isMesh)H.wireframe===!0?(vt.setLineWidth(H.wireframeLinewidth*Te()),ne.setMode(D.LINES)):ne.setMode(D.TRIANGLES);else if(O.isLine){let At=H.linewidth;At===void 0&&(At=1),vt.setLineWidth(At*Te()),O.isLineSegments?ne.setMode(D.LINES):O.isLineLoop?ne.setMode(D.LINE_LOOP):ne.setMode(D.LINE_STRIP)}else O.isPoints?ne.setMode(D.POINTS):O.isSprite&&ne.setMode(D.TRIANGLES);if(O.isBatchedMesh)if(O._multiDrawInstances!==null)Fs("THREE.WebGLRenderer: renderMultiDrawInstances has been deprecated and will be removed in r184. Append to renderMultiDraw arguments and use indirection."),ne.renderMultiDrawInstances(O._multiDrawStarts,O._multiDrawCounts,O._multiDrawCount,O._multiDrawInstances);else if(Nt.get("WEBGL_multi_draw"))ne.renderMultiDraw(O._multiDrawStarts,O._multiDrawCounts,O._multiDrawCount);else{const At=O._multiDrawStarts,he=O._multiDrawCounts,Yt=O._multiDrawCount,Fe=dt?A.get(dt).bytesPerElement:1,Mn=_t.get(H).currentProgram.getUniforms();for(let ke=0;ke<Yt;ke++)Mn.setValue(D,"_gl_DrawID",ke),ne.render(At[ke]/Fe,he[ke])}else if(O.isInstancedMesh)ne.renderInstances(Xt,fe,O.count);else if(z.isInstancedBufferGeometry){const At=z._maxInstanceCount!==void 0?z._maxInstanceCount:1/0,he=Math.min(z.instanceCount,At);ne.renderInstances(Xt,fe,he)}else ne.render(Xt,fe)};function re(M,U,z){M.transparent===!0&&M.side===Ne&&M.forceSinglePass===!1?(M.side=Oe,M.needsUpdate=!0,$s(M,U,z),M.side=Xi,M.needsUpdate=!0,$s(M,U,z),M.side=Ne):$s(M,U,z)}this.compile=function(M,U,z=null){z===null&&(z=M),p=St.get(z),p.init(U),E.push(p),z.traverseVisible(function(O){O.isLight&&O.layers.test(U.layers)&&(p.pushLight(O),O.castShadow&&p.pushShadow(O))}),M!==z&&M.traverseVisible(function(O){O.isLight&&O.layers.test(U.layers)&&(p.pushLight(O),O.castShadow&&p.pushShadow(O))}),p.setupLights();const H=new Set;return M.traverse(function(O){if(!(O.isMesh||O.isPoints||O.isLine||O.isSprite))return;const tt=O.material;if(tt)if(Array.isArray(tt))for(let lt=0;lt<tt.length;lt++){const mt=tt[lt];re(mt,z,O),H.add(mt)}else re(tt,z,O),H.add(tt)}),p=E.pop(),H},this.compileAsync=function(M,U,z=null){const H=this.compile(M,U,z);return new Promise(O=>{function tt(){if(H.forEach(function(lt){_t.get(lt).currentProgram.isReady()&&H.delete(lt)}),H.size===0){O(M);return}setTimeout(tt,10)}Nt.get("KHR_parallel_shader_compile")!==null?tt():setTimeout(tt,10)})};let Zt=null;function _i(M){Zt&&Zt(M)}function oi(){qi.stop()}function cl(){qi.start()}const qi=new Bh;qi.setAnimationLoop(_i),typeof self<"u"&&qi.setContext(self),this.setAnimationLoop=function(M){Zt=M,it.setAnimationLoop(M),M===null?qi.stop():qi.start()},it.addEventListener("sessionstart",oi),it.addEventListener("sessionend",cl),this.render=function(M,U){if(U!==void 0&&U.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(P===!0)return;if(M.matrixWorldAutoUpdate===!0&&M.updateMatrixWorld(),U.parent===null&&U.matrixWorldAutoUpdate===!0&&U.updateMatrixWorld(),it.enabled===!0&&it.isPresenting===!0&&(it.cameraAutoUpdate===!0&&it.updateCamera(U),U=it.getCamera()),M.isScene===!0&&M.onBeforeRender(_,M,U,T),p=St.get(M,E.length),p.init(U),E.push(p),j.multiplyMatrices(U.projectionMatrix,U.matrixWorldInverse),oe.setFromProjectionMatrix(j,fi,U.reversedDepth),q=this.localClippingEnabled,Kt=nt.init(this.clippingPlanes,q),m=V.get(M,w.length),m.init(),w.push(m),it.enabled===!0&&it.isPresenting===!0){const tt=_.xr.getDepthSensingMesh();tt!==null&&Wr(tt,U,-1/0,_.sortObjects)}Wr(M,U,0,_.sortObjects),m.finish(),_.sortObjects===!0&&m.sort(at,ut),$t=it.enabled===!1||it.isPresenting===!1||it.hasDepthSensing()===!1,$t&&yt.addToRenderList(m,M),this.info.render.frame++,Kt===!0&&nt.beginShadows();const z=p.state.shadowsArray;xt.render(z,M,U),Kt===!0&&nt.endShadows(),this.info.autoReset===!0&&this.info.reset();const H=m.opaque,O=m.transmissive;if(p.setupLights(),U.isArrayCamera){const tt=U.cameras;if(O.length>0)for(let lt=0,mt=tt.length;lt<mt;lt++){const dt=tt[lt];ul(H,O,M,dt)}$t&&yt.render(M);for(let lt=0,mt=tt.length;lt<mt;lt++){const dt=tt[lt];hl(m,M,dt,dt.viewport)}}else O.length>0&&ul(H,O,M,U),$t&&yt.render(M),hl(m,M,U);T!==null&&I===0&&(Ft.updateMultisampleRenderTarget(T),Ft.updateRenderTargetMipmap(T)),M.isScene===!0&&M.onAfterRender(_,M,U),ot.resetDefaultState(),S=-1,b=null,E.pop(),E.length>0?(p=E[E.length-1],Kt===!0&&nt.setGlobalState(_.clippingPlanes,p.state.camera)):p=null,w.pop(),w.length>0?m=w[w.length-1]:m=null};function Wr(M,U,z,H){if(M.visible===!1)return;if(M.layers.test(U.layers)){if(M.isGroup)z=M.renderOrder;else if(M.isLOD)M.autoUpdate===!0&&M.update(U);else if(M.isLight)p.pushLight(M),M.castShadow&&p.pushShadow(M);else if(M.isSprite){if(!M.frustumCulled||oe.intersectsSprite(M)){H&&Dt.setFromMatrixPosition(M.matrixWorld).applyMatrix4(j);const lt=k.update(M),mt=M.material;mt.visible&&m.push(M,lt,mt,z,Dt.z,null)}}else if((M.isMesh||M.isLine||M.isPoints)&&(!M.frustumCulled||oe.intersectsObject(M))){const lt=k.update(M),mt=M.material;if(H&&(M.boundingSphere!==void 0?(M.boundingSphere===null&&M.computeBoundingSphere(),Dt.copy(M.boundingSphere.center)):(lt.boundingSphere===null&&lt.computeBoundingSphere(),Dt.copy(lt.boundingSphere.center)),Dt.applyMatrix4(M.matrixWorld).applyMatrix4(j)),Array.isArray(mt)){const dt=lt.groups;for(let Rt=0,It=dt.length;Rt<It;Rt++){const wt=dt[Rt],Xt=mt[wt.materialIndex];Xt&&Xt.visible&&m.push(M,lt,Xt,z,Dt.z,wt)}}else mt.visible&&m.push(M,lt,mt,z,Dt.z,null)}}const tt=M.children;for(let lt=0,mt=tt.length;lt<mt;lt++)Wr(tt[lt],U,z,H)}function hl(M,U,z,H){const O=M.opaque,tt=M.transmissive,lt=M.transparent;p.setupLightsView(z),Kt===!0&&nt.setGlobalState(_.clippingPlanes,z),H&&vt.viewport(L.copy(H)),O.length>0&&Xs(O,U,z),tt.length>0&&Xs(tt,U,z),lt.length>0&&Xs(lt,U,z),vt.buffers.depth.setTest(!0),vt.buffers.depth.setMask(!0),vt.buffers.color.setMask(!0),vt.setPolygonOffset(!1)}function ul(M,U,z,H){if((z.isScene===!0?z.overrideMaterial:null)!==null)return;p.state.transmissionRenderTarget[H.id]===void 0&&(p.state.transmissionRenderTarget[H.id]=new vn(1,1,{generateMipmaps:!0,type:Nt.has("EXT_color_buffer_half_float")||Nt.has("EXT_color_buffer_float")?Gs:mi,minFilter:cn,samples:4,stencilBuffer:r,resolveDepthBuffer:!1,resolveStencilBuffer:!1,colorSpace:qt.workingColorSpace}));const tt=p.state.transmissionRenderTarget[H.id],lt=H.viewport||L;tt.setSize(lt.z*_.transmissionResolutionScale,lt.w*_.transmissionResolutionScale);const mt=_.getRenderTarget(),dt=_.getActiveCubeFace(),Rt=_.getActiveMipmapLevel();_.setRenderTarget(tt),_.getClearColor(Y),W=_.getClearAlpha(),W<1&&_.setClearColor(16777215,.5),_.clear(),$t&&yt.render(z);const It=_.toneMapping;_.toneMapping=Wi;const wt=H.viewport;if(H.viewport!==void 0&&(H.viewport=void 0),p.setupLightsView(H),Kt===!0&&nt.setGlobalState(_.clippingPlanes,H),Xs(M,z,H),Ft.updateMultisampleRenderTarget(tt),Ft.updateRenderTargetMipmap(tt),Nt.has("WEBGL_multisampled_render_to_texture")===!1){let Xt=!1;for(let Qt=0,fe=U.length;Qt<fe;Qt++){const ae=U[Qt],ne=ae.object,At=ae.geometry,he=ae.material,Yt=ae.group;if(he.side===Ne&&ne.layers.test(H.layers)){const Fe=he.side;he.side=Oe,he.needsUpdate=!0,dl(ne,z,H,At,he,Yt),he.side=Fe,he.needsUpdate=!0,Xt=!0}}Xt===!0&&(Ft.updateMultisampleRenderTarget(tt),Ft.updateRenderTargetMipmap(tt))}_.setRenderTarget(mt,dt,Rt),_.setClearColor(Y,W),wt!==void 0&&(H.viewport=wt),_.toneMapping=It}function Xs(M,U,z){const H=U.isScene===!0?U.overrideMaterial:null;for(let O=0,tt=M.length;O<tt;O++){const lt=M[O],mt=lt.object,dt=lt.geometry,Rt=lt.group;let It=lt.material;It.allowOverride===!0&&H!==null&&(It=H),mt.layers.test(z.layers)&&dl(mt,U,z,dt,It,Rt)}}function dl(M,U,z,H,O,tt){M.onBeforeRender(_,U,z,H,O,tt),M.modelViewMatrix.multiplyMatrices(z.matrixWorldInverse,M.matrixWorld),M.normalMatrix.getNormalMatrix(M.modelViewMatrix),O.onBeforeRender(_,U,z,H,M,tt),O.transparent===!0&&O.side===Ne&&O.forceSinglePass===!1?(O.side=Oe,O.needsUpdate=!0,_.renderBufferDirect(z,U,H,O,M,tt),O.side=Xi,O.needsUpdate=!0,_.renderBufferDirect(z,U,H,O,M,tt),O.side=Ne):_.renderBufferDirect(z,U,H,O,M,tt),M.onAfterRender(_,U,z,H,O,tt)}function $s(M,U,z){U.isScene!==!0&&(U=bt);const H=_t.get(M),O=p.state.lights,tt=p.state.shadowsArray,lt=O.state.version,mt=X.getParameters(M,O.state,tt,U,z),dt=X.getProgramCacheKey(mt);let Rt=H.programs;H.environment=M.isMeshStandardMaterial?U.environment:null,H.fog=U.fog,H.envMap=(M.isMeshStandardMaterial?me:Se).get(M.envMap||H.environment),H.envMapRotation=H.environment!==null&&M.envMap===null?U.environmentRotation:M.envMapRotation,Rt===void 0&&(M.addEventListener("dispose",K),Rt=new Map,H.programs=Rt);let It=Rt.get(dt);if(It!==void 0){if(H.currentProgram===It&&H.lightsStateVersion===lt)return pl(M,mt),It}else mt.uniforms=X.getUniforms(M),M.onBeforeCompile(mt,_),It=X.acquireProgram(mt,dt),Rt.set(dt,It),H.uniforms=mt.uniforms;const wt=H.uniforms;return(!M.isShaderMaterial&&!M.isRawShaderMaterial||M.clipping===!0)&&(wt.clippingPlanes=nt.uniform),pl(M,mt),H.needsLights=Zh(M),H.lightsStateVersion=lt,H.needsLights&&(wt.ambientLightColor.value=O.state.ambient,wt.lightProbe.value=O.state.probe,wt.directionalLights.value=O.state.directional,wt.directionalLightShadows.value=O.state.directionalShadow,wt.spotLights.value=O.state.spot,wt.spotLightShadows.value=O.state.spotShadow,wt.rectAreaLights.value=O.state.rectArea,wt.ltc_1.value=O.state.rectAreaLTC1,wt.ltc_2.value=O.state.rectAreaLTC2,wt.pointLights.value=O.state.point,wt.pointLightShadows.value=O.state.pointShadow,wt.hemisphereLights.value=O.state.hemi,wt.directionalShadowMap.value=O.state.directionalShadowMap,wt.directionalShadowMatrix.value=O.state.directionalShadowMatrix,wt.spotShadowMap.value=O.state.spotShadowMap,wt.spotLightMatrix.value=O.state.spotLightMatrix,wt.spotLightMap.value=O.state.spotLightMap,wt.pointShadowMap.value=O.state.pointShadowMap,wt.pointShadowMatrix.value=O.state.pointShadowMatrix),H.currentProgram=It,H.uniformsList=null,It}function fl(M){if(M.uniformsList===null){const U=M.currentProgram.getUniforms();M.uniformsList=Ir.seqWithValue(U.seq,M.uniforms)}return M.uniformsList}function pl(M,U){const z=_t.get(M);z.outputColorSpace=U.outputColorSpace,z.batching=U.batching,z.batchingColor=U.batchingColor,z.instancing=U.instancing,z.instancingColor=U.instancingColor,z.instancingMorph=U.instancingMorph,z.skinning=U.skinning,z.morphTargets=U.morphTargets,z.morphNormals=U.morphNormals,z.morphColors=U.morphColors,z.morphTargetsCount=U.morphTargetsCount,z.numClippingPlanes=U.numClippingPlanes,z.numIntersection=U.numClipIntersection,z.vertexAlphas=U.vertexAlphas,z.vertexTangents=U.vertexTangents,z.toneMapping=U.toneMapping}function qh(M,U,z,H,O){U.isScene!==!0&&(U=bt),Ft.resetTextureUnits();const tt=U.fog,lt=H.isMeshStandardMaterial?U.environment:null,mt=T===null?_.outputColorSpace:T.isXRRenderTarget===!0?T.texture.colorSpace:ns,dt=(H.isMeshStandardMaterial?me:Se).get(H.envMap||lt),Rt=H.vertexColors===!0&&!!z.attributes.color&&z.attributes.color.itemSize===4,It=!!z.attributes.tangent&&(!!H.normalMap||H.anisotropy>0),wt=!!z.morphAttributes.position,Xt=!!z.morphAttributes.normal,Qt=!!z.morphAttributes.color;let fe=Wi;H.toneMapped&&(T===null||T.isXRRenderTarget===!0)&&(fe=_.toneMapping);const ae=z.morphAttributes.position||z.morphAttributes.normal||z.morphAttributes.color,ne=ae!==void 0?ae.length:0,At=_t.get(H),he=p.state.lights;if(Kt===!0&&(q===!0||M!==b)){const Pe=M===b&&H.id===S;nt.setState(H,M,Pe)}let Yt=!1;H.version===At.__version?(At.needsLights&&At.lightsStateVersion!==he.state.version||At.outputColorSpace!==mt||O.isBatchedMesh&&At.batching===!1||!O.isBatchedMesh&&At.batching===!0||O.isBatchedMesh&&At.batchingColor===!0&&O.colorTexture===null||O.isBatchedMesh&&At.batchingColor===!1&&O.colorTexture!==null||O.isInstancedMesh&&At.instancing===!1||!O.isInstancedMesh&&At.instancing===!0||O.isSkinnedMesh&&At.skinning===!1||!O.isSkinnedMesh&&At.skinning===!0||O.isInstancedMesh&&At.instancingColor===!0&&O.instanceColor===null||O.isInstancedMesh&&At.instancingColor===!1&&O.instanceColor!==null||O.isInstancedMesh&&At.instancingMorph===!0&&O.morphTexture===null||O.isInstancedMesh&&At.instancingMorph===!1&&O.morphTexture!==null||At.envMap!==dt||H.fog===!0&&At.fog!==tt||At.numClippingPlanes!==void 0&&(At.numClippingPlanes!==nt.numPlanes||At.numIntersection!==nt.numIntersection)||At.vertexAlphas!==Rt||At.vertexTangents!==It||At.morphTargets!==wt||At.morphNormals!==Xt||At.morphColors!==Qt||At.toneMapping!==fe||At.morphTargetsCount!==ne)&&(Yt=!0):(Yt=!0,At.__version=H.version);let Fe=At.currentProgram;Yt===!0&&(Fe=$s(H,U,O));let Mn=!1,ke=!1,hs=!1;const ue=Fe.getUniforms(),Ye=At.uniforms;if(vt.useProgram(Fe.program)&&(Mn=!0,ke=!0,hs=!0),H.id!==S&&(S=H.id,ke=!0),Mn||b!==M){vt.buffers.depth.getReversed()&&M.reversedDepth!==!0&&(M._reversedDepth=!0,M.updateProjectionMatrix()),ue.setValue(D,"projectionMatrix",M.projectionMatrix),ue.setValue(D,"viewMatrix",M.matrixWorldInverse);const Ue=ue.map.cameraPosition;Ue!==void 0&&Ue.setValue(D,pt.setFromMatrixPosition(M.matrixWorld)),Pt.logarithmicDepthBuffer&&ue.setValue(D,"logDepthBufFC",2/(Math.log(M.far+1)/Math.LN2)),(H.isMeshPhongMaterial||H.isMeshToonMaterial||H.isMeshLambertMaterial||H.isMeshBasicMaterial||H.isMeshStandardMaterial||H.isShaderMaterial)&&ue.setValue(D,"isOrthographic",M.isOrthographicCamera===!0),b!==M&&(b=M,ke=!0,hs=!0)}if(O.isSkinnedMesh){ue.setOptional(D,O,"bindMatrix"),ue.setOptional(D,O,"bindMatrixInverse");const Pe=O.skeleton;Pe&&(Pe.boneTexture===null&&Pe.computeBoneTexture(),ue.setValue(D,"boneTexture",Pe.boneTexture,Ft))}O.isBatchedMesh&&(ue.setOptional(D,O,"batchingTexture"),ue.setValue(D,"batchingTexture",O._matricesTexture,Ft),ue.setOptional(D,O,"batchingIdTexture"),ue.setValue(D,"batchingIdTexture",O._indirectTexture,Ft),ue.setOptional(D,O,"batchingColorTexture"),O._colorsTexture!==null&&ue.setValue(D,"batchingColorTexture",O._colorsTexture,Ft));const qe=z.morphAttributes;if((qe.position!==void 0||qe.normal!==void 0||qe.color!==void 0)&&et.update(O,z,Fe),(ke||At.receiveShadow!==O.receiveShadow)&&(At.receiveShadow=O.receiveShadow,ue.setValue(D,"receiveShadow",O.receiveShadow)),H.isMeshGouraudMaterial&&H.envMap!==null&&(Ye.envMap.value=dt,Ye.flipEnvMap.value=dt.isCubeTexture&&dt.isRenderTargetTexture===!1?-1:1),H.isMeshStandardMaterial&&H.envMap===null&&U.environment!==null&&(Ye.envMapIntensity.value=U.environmentIntensity),ke&&(ue.setValue(D,"toneMappingExposure",_.toneMappingExposure),At.needsLights&&Kh(Ye,hs),tt&&H.fog===!0&&Z.refreshFogUniforms(Ye,tt),Z.refreshMaterialUniforms(Ye,H,G,J,p.state.transmissionRenderTarget[M.id]),Ir.upload(D,fl(At),Ye,Ft)),H.isShaderMaterial&&H.uniformsNeedUpdate===!0&&(Ir.upload(D,fl(At),Ye,Ft),H.uniformsNeedUpdate=!1),H.isSpriteMaterial&&ue.setValue(D,"center",O.center),ue.setValue(D,"modelViewMatrix",O.modelViewMatrix),ue.setValue(D,"normalMatrix",O.normalMatrix),ue.setValue(D,"modelMatrix",O.matrixWorld),H.isShaderMaterial||H.isRawShaderMaterial){const Pe=H.uniformsGroups;for(let Ue=0,Xr=Pe.length;Ue<Xr;Ue++){const Ki=Pe[Ue];Ut.update(Ki,Fe),Ut.bind(Ki,Fe)}}return Fe}function Kh(M,U){M.ambientLightColor.needsUpdate=U,M.lightProbe.needsUpdate=U,M.directionalLights.needsUpdate=U,M.directionalLightShadows.needsUpdate=U,M.pointLights.needsUpdate=U,M.pointLightShadows.needsUpdate=U,M.spotLights.needsUpdate=U,M.spotLightShadows.needsUpdate=U,M.rectAreaLights.needsUpdate=U,M.hemisphereLights.needsUpdate=U}function Zh(M){return M.isMeshLambertMaterial||M.isMeshToonMaterial||M.isMeshPhongMaterial||M.isMeshStandardMaterial||M.isShadowMaterial||M.isShaderMaterial&&M.lights===!0}this.getActiveCubeFace=function(){return R},this.getActiveMipmapLevel=function(){return I},this.getRenderTarget=function(){return T},this.setRenderTargetTextures=function(M,U,z){const H=_t.get(M);H.__autoAllocateDepthBuffer=M.resolveDepthBuffer===!1,H.__autoAllocateDepthBuffer===!1&&(H.__useRenderToTexture=!1),_t.get(M.texture).__webglTexture=U,_t.get(M.depthTexture).__webglTexture=H.__autoAllocateDepthBuffer?void 0:z,H.__hasExternalTextures=!0},this.setRenderTargetFramebuffer=function(M,U){const z=_t.get(M);z.__webglFramebuffer=U,z.__useDefaultFramebuffer=U===void 0};const Jh=D.createFramebuffer();this.setRenderTarget=function(M,U=0,z=0){T=M,R=U,I=z;let H=!0,O=null,tt=!1,lt=!1;if(M){const dt=_t.get(M);if(dt.__useDefaultFramebuffer!==void 0)vt.bindFramebuffer(D.FRAMEBUFFER,null),H=!1;else if(dt.__webglFramebuffer===void 0)Ft.setupRenderTarget(M);else if(dt.__hasExternalTextures)Ft.rebindTextures(M,_t.get(M.texture).__webglTexture,_t.get(M.depthTexture).__webglTexture);else if(M.depthBuffer){const wt=M.depthTexture;if(dt.__boundDepthTexture!==wt){if(wt!==null&&_t.has(wt)&&(M.width!==wt.image.width||M.height!==wt.image.height))throw new Error("WebGLRenderTarget: Attached DepthTexture is initialized to the incorrect size.");Ft.setupDepthRenderbuffer(M)}}const Rt=M.texture;(Rt.isData3DTexture||Rt.isDataArrayTexture||Rt.isCompressedArrayTexture)&&(lt=!0);const It=_t.get(M).__webglFramebuffer;M.isWebGLCubeRenderTarget?(Array.isArray(It[U])?O=It[U][z]:O=It[U],tt=!0):M.samples>0&&Ft.useMultisampledRTT(M)===!1?O=_t.get(M).__webglMultisampledFramebuffer:Array.isArray(It)?O=It[z]:O=It,L.copy(M.viewport),F.copy(M.scissor),B=M.scissorTest}else L.copy(Et).multiplyScalar(G).floor(),F.copy(Gt).multiplyScalar(G).floor(),B=se;if(z!==0&&(O=Jh),vt.bindFramebuffer(D.FRAMEBUFFER,O)&&H&&vt.drawBuffers(M,O),vt.viewport(L),vt.scissor(F),vt.setScissorTest(B),tt){const dt=_t.get(M.texture);D.framebufferTexture2D(D.FRAMEBUFFER,D.COLOR_ATTACHMENT0,D.TEXTURE_CUBE_MAP_POSITIVE_X+U,dt.__webglTexture,z)}else if(lt){const dt=U;for(let Rt=0;Rt<M.textures.length;Rt++){const It=_t.get(M.textures[Rt]);D.framebufferTextureLayer(D.FRAMEBUFFER,D.COLOR_ATTACHMENT0+Rt,It.__webglTexture,z,dt)}}else if(M!==null&&z!==0){const dt=_t.get(M.texture);D.framebufferTexture2D(D.FRAMEBUFFER,D.COLOR_ATTACHMENT0,D.TEXTURE_2D,dt.__webglTexture,z)}S=-1},this.readRenderTargetPixels=function(M,U,z,H,O,tt,lt,mt=0){if(!(M&&M.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let dt=_t.get(M).__webglFramebuffer;if(M.isWebGLCubeRenderTarget&&lt!==void 0&&(dt=dt[lt]),dt){vt.bindFramebuffer(D.FRAMEBUFFER,dt);try{const Rt=M.textures[mt],It=Rt.format,wt=Rt.type;if(!Pt.textureFormatReadable(It)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}if(!Pt.textureTypeReadable(wt)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}U>=0&&U<=M.width-H&&z>=0&&z<=M.height-O&&(M.textures.length>1&&D.readBuffer(D.COLOR_ATTACHMENT0+mt),D.readPixels(U,z,H,O,Mt.convert(It),Mt.convert(wt),tt))}finally{const Rt=T!==null?_t.get(T).__webglFramebuffer:null;vt.bindFramebuffer(D.FRAMEBUFFER,Rt)}}},this.readRenderTargetPixelsAsync=async function(M,U,z,H,O,tt,lt,mt=0){if(!(M&&M.isWebGLRenderTarget))throw new Error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");let dt=_t.get(M).__webglFramebuffer;if(M.isWebGLCubeRenderTarget&&lt!==void 0&&(dt=dt[lt]),dt)if(U>=0&&U<=M.width-H&&z>=0&&z<=M.height-O){vt.bindFramebuffer(D.FRAMEBUFFER,dt);const Rt=M.textures[mt],It=Rt.format,wt=Rt.type;if(!Pt.textureFormatReadable(It))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in RGBA or implementation defined format.");if(!Pt.textureTypeReadable(wt))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in UnsignedByteType or implementation defined type.");const Xt=D.createBuffer();D.bindBuffer(D.PIXEL_PACK_BUFFER,Xt),D.bufferData(D.PIXEL_PACK_BUFFER,tt.byteLength,D.STREAM_READ),M.textures.length>1&&D.readBuffer(D.COLOR_ATTACHMENT0+mt),D.readPixels(U,z,H,O,Mt.convert(It),Mt.convert(wt),0);const Qt=T!==null?_t.get(T).__webglFramebuffer:null;vt.bindFramebuffer(D.FRAMEBUFFER,Qt);const fe=D.fenceSync(D.SYNC_GPU_COMMANDS_COMPLETE,0);return D.flush(),await Td(D,fe,4),D.bindBuffer(D.PIXEL_PACK_BUFFER,Xt),D.getBufferSubData(D.PIXEL_PACK_BUFFER,0,tt),D.deleteBuffer(Xt),D.deleteSync(fe),tt}else throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: requested read bounds are out of range.")},this.copyFramebufferToTexture=function(M,U=null,z=0){const H=Math.pow(2,-z),O=Math.floor(M.image.width*H),tt=Math.floor(M.image.height*H),lt=U!==null?U.x:0,mt=U!==null?U.y:0;Ft.setTexture2D(M,0),D.copyTexSubImage2D(D.TEXTURE_2D,z,0,0,lt,mt,O,tt),vt.unbindTexture()};const jh=D.createFramebuffer(),Qh=D.createFramebuffer();this.copyTextureToTexture=function(M,U,z=null,H=null,O=0,tt=null){tt===null&&(O!==0?(Fs("WebGLRenderer: copyTextureToTexture function signature has changed to support src and dst mipmap levels."),tt=O,O=0):tt=0);let lt,mt,dt,Rt,It,wt,Xt,Qt,fe;const ae=M.isCompressedTexture?M.mipmaps[tt]:M.image;if(z!==null)lt=z.max.x-z.min.x,mt=z.max.y-z.min.y,dt=z.isBox3?z.max.z-z.min.z:1,Rt=z.min.x,It=z.min.y,wt=z.isBox3?z.min.z:0;else{const qe=Math.pow(2,-O);lt=Math.floor(ae.width*qe),mt=Math.floor(ae.height*qe),M.isDataArrayTexture?dt=ae.depth:M.isData3DTexture?dt=Math.floor(ae.depth*qe):dt=1,Rt=0,It=0,wt=0}H!==null?(Xt=H.x,Qt=H.y,fe=H.z):(Xt=0,Qt=0,fe=0);const ne=Mt.convert(U.format),At=Mt.convert(U.type);let he;U.isData3DTexture?(Ft.setTexture3D(U,0),he=D.TEXTURE_3D):U.isDataArrayTexture||U.isCompressedArrayTexture?(Ft.setTexture2DArray(U,0),he=D.TEXTURE_2D_ARRAY):(Ft.setTexture2D(U,0),he=D.TEXTURE_2D),D.pixelStorei(D.UNPACK_FLIP_Y_WEBGL,U.flipY),D.pixelStorei(D.UNPACK_PREMULTIPLY_ALPHA_WEBGL,U.premultiplyAlpha),D.pixelStorei(D.UNPACK_ALIGNMENT,U.unpackAlignment);const Yt=D.getParameter(D.UNPACK_ROW_LENGTH),Fe=D.getParameter(D.UNPACK_IMAGE_HEIGHT),Mn=D.getParameter(D.UNPACK_SKIP_PIXELS),ke=D.getParameter(D.UNPACK_SKIP_ROWS),hs=D.getParameter(D.UNPACK_SKIP_IMAGES);D.pixelStorei(D.UNPACK_ROW_LENGTH,ae.width),D.pixelStorei(D.UNPACK_IMAGE_HEIGHT,ae.height),D.pixelStorei(D.UNPACK_SKIP_PIXELS,Rt),D.pixelStorei(D.UNPACK_SKIP_ROWS,It),D.pixelStorei(D.UNPACK_SKIP_IMAGES,wt);const ue=M.isDataArrayTexture||M.isData3DTexture,Ye=U.isDataArrayTexture||U.isData3DTexture;if(M.isDepthTexture){const qe=_t.get(M),Pe=_t.get(U),Ue=_t.get(qe.__renderTarget),Xr=_t.get(Pe.__renderTarget);vt.bindFramebuffer(D.READ_FRAMEBUFFER,Ue.__webglFramebuffer),vt.bindFramebuffer(D.DRAW_FRAMEBUFFER,Xr.__webglFramebuffer);for(let Ki=0;Ki<dt;Ki++)ue&&(D.framebufferTextureLayer(D.READ_FRAMEBUFFER,D.COLOR_ATTACHMENT0,_t.get(M).__webglTexture,O,wt+Ki),D.framebufferTextureLayer(D.DRAW_FRAMEBUFFER,D.COLOR_ATTACHMENT0,_t.get(U).__webglTexture,tt,fe+Ki)),D.blitFramebuffer(Rt,It,lt,mt,Xt,Qt,lt,mt,D.DEPTH_BUFFER_BIT,D.NEAREST);vt.bindFramebuffer(D.READ_FRAMEBUFFER,null),vt.bindFramebuffer(D.DRAW_FRAMEBUFFER,null)}else if(O!==0||M.isRenderTargetTexture||_t.has(M)){const qe=_t.get(M),Pe=_t.get(U);vt.bindFramebuffer(D.READ_FRAMEBUFFER,jh),vt.bindFramebuffer(D.DRAW_FRAMEBUFFER,Qh);for(let Ue=0;Ue<dt;Ue++)ue?D.framebufferTextureLayer(D.READ_FRAMEBUFFER,D.COLOR_ATTACHMENT0,qe.__webglTexture,O,wt+Ue):D.framebufferTexture2D(D.READ_FRAMEBUFFER,D.COLOR_ATTACHMENT0,D.TEXTURE_2D,qe.__webglTexture,O),Ye?D.framebufferTextureLayer(D.DRAW_FRAMEBUFFER,D.COLOR_ATTACHMENT0,Pe.__webglTexture,tt,fe+Ue):D.framebufferTexture2D(D.DRAW_FRAMEBUFFER,D.COLOR_ATTACHMENT0,D.TEXTURE_2D,Pe.__webglTexture,tt),O!==0?D.blitFramebuffer(Rt,It,lt,mt,Xt,Qt,lt,mt,D.COLOR_BUFFER_BIT,D.NEAREST):Ye?D.copyTexSubImage3D(he,tt,Xt,Qt,fe+Ue,Rt,It,lt,mt):D.copyTexSubImage2D(he,tt,Xt,Qt,Rt,It,lt,mt);vt.bindFramebuffer(D.READ_FRAMEBUFFER,null),vt.bindFramebuffer(D.DRAW_FRAMEBUFFER,null)}else Ye?M.isDataTexture||M.isData3DTexture?D.texSubImage3D(he,tt,Xt,Qt,fe,lt,mt,dt,ne,At,ae.data):U.isCompressedArrayTexture?D.compressedTexSubImage3D(he,tt,Xt,Qt,fe,lt,mt,dt,ne,ae.data):D.texSubImage3D(he,tt,Xt,Qt,fe,lt,mt,dt,ne,At,ae):M.isDataTexture?D.texSubImage2D(D.TEXTURE_2D,tt,Xt,Qt,lt,mt,ne,At,ae.data):M.isCompressedTexture?D.compressedTexSubImage2D(D.TEXTURE_2D,tt,Xt,Qt,ae.width,ae.height,ne,ae.data):D.texSubImage2D(D.TEXTURE_2D,tt,Xt,Qt,lt,mt,ne,At,ae);D.pixelStorei(D.UNPACK_ROW_LENGTH,Yt),D.pixelStorei(D.UNPACK_IMAGE_HEIGHT,Fe),D.pixelStorei(D.UNPACK_SKIP_PIXELS,Mn),D.pixelStorei(D.UNPACK_SKIP_ROWS,ke),D.pixelStorei(D.UNPACK_SKIP_IMAGES,hs),tt===0&&U.generateMipmaps&&D.generateMipmap(he),vt.unbindTexture()},this.initRenderTarget=function(M){_t.get(M).__webglFramebuffer===void 0&&Ft.setupRenderTarget(M)},this.initTexture=function(M){M.isCubeTexture?Ft.setTextureCube(M,0):M.isData3DTexture?Ft.setTexture3D(M,0):M.isDataArrayTexture||M.isCompressedArrayTexture?Ft.setTexture2DArray(M,0):Ft.setTexture2D(M,0),vt.unbindTexture()},this.resetState=function(){R=0,I=0,T=null,vt.reset(),ot.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return fi}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(t){this._outputColorSpace=t;const e=this.getContext();e.drawingBufferColorSpace=qt._getDrawingBufferColorSpace(t),e.unpackColorSpace=qt._getUnpackColorSpace()}}const Qn={edge:14,gap:16,subjectPadding:42,subjectHeight:.7,pairAllowance:2.6},Ev=s=>Math.max(0,s.right-s.left)*Math.max(0,s.bottom-s.top);function wv(s,t,e,i=0){const{edge:n,gap:r}=Qn;let a=[{left:n,top:Math.max(n,i+r),right:s-n,bottom:t-n}];for(const c of e){const l={left:c.left-r,top:c.top-r,right:c.right+r,bottom:c.bottom+r};a=a.flatMap(h=>l.left>=h.right||l.right<=h.left||l.top>=h.bottom||l.bottom<=h.top?[h]:[{...h,right:Math.min(h.right,l.left)},{...h,left:Math.max(h.left,l.right)},{...h,bottom:Math.min(h.bottom,l.top)},{...h,top:Math.max(h.top,l.bottom)}].filter(u=>u.right>u.left&&u.bottom>u.top))}const o=c=>Ev(c)*Math.min(1,(c.right-c.left)/160)*Math.min(1,(c.bottom-c.top)/100);return a.sort((c,l)=>o(l)-o(c))[0]??{left:n,top:n,right:s-n,bottom:t-n}}function Xh(s){const t=Math.min(Qn.subjectPadding,(s.right-s.left)*.2,(s.bottom-s.top)*.2);return{left:s.left+t,right:s.right-t,top:s.top+t,bottom:s.bottom-t}}function $h(s,t,e){s.updateMatrixWorld();const i=s.matrixWorldInverse.elements,n=i[0]*i[9]-i[8]*i[1];return Math.abs(n)<1e-6?{x:0,z:0}:{x:(t*i[9]-i[8]*e)/n,z:(i[0]*e-t*i[1])/n}}function Tv(s,t,e,i,n,r,a){const o=Jt.camera,c=s.right-s.left,l=s.top-s.bottom,h=s.quaternion.clone().invert(),u=e?new C(e.x-t.x,0,e.z-t.z).applyQuaternion(h):new C,d=a*(n/r>1?o.minimumZoomRatio:o.portraitMinimumZoomRatio),f=e?Math.max(d,Math.min(a,c*((i.right-i.left)/n)/(Math.abs(u.x)+Qn.pairAllowance),l*((i.bottom-i.top)/r)/(Math.abs(u.y)+Qn.pairAllowance))):a,g=Xh(i),v=zt((i.left+i.right)/2-u.x*n*f/c/2,g.left,g.right),m=zt((i.top+i.bottom)/2+u.y*r*f/l/2,g.top,g.bottom),p=new C(0,Qn.subjectHeight,0).applyQuaternion(h),w=$h(s,p.x-(2*v/n-1)*c/(2*f),p.y-(1-2*m/r)*l/(2*f));return{look:{x:t.x+w.x,z:t.z+w.z},zoom:f}}function Av(s,t,e,i,n){s.updateMatrixWorld();const r=new C(t.x,Qn.subjectHeight,t.z).project(s),a=Xh(e),o=(r.x+1)*i/2,c=(1-r.y)*n/2;return $h(s,(o-zt(o,a.left,a.right))*(s.right-s.left)/(i*s.zoom),-(c-zt(c,a.top,a.bottom))*(s.top-s.bottom)/(n*s.zoom))}function Rv(s,t,e,i,n,r){const a=Jt.camera,o={x:(t.x+e.x)/2,z:(t.z+e.z)/2},c=s.quaternion.clone().invert(),l=new C(e.x-t.x,0,e.z-t.z).applyQuaternion(c),h=n<500?80:110,u=30,d=Math.min(a.visibilityPadding,i*.08),f=Math.abs(l.x)+1.8,g=Math.abs(l.y)+1.8,v=r*(i/n>1?a.minimumZoomRatio:a.portraitMinimumZoomRatio),m=Math.max(v,Math.min(r,(s.right-s.left)*(1-d*2/i)/f,(s.top-s.bottom)*(1-(h+u)/n)/g));s.updateMatrixWorld();const p=s.matrixWorldInverse.elements,w=(u-h)/n*s.top/m,E=p[1]*p[1]+p[9]*p[9],_=(1.2*p[5]-w)/Math.max(.01,E);return{look:{x:o.x+p[1]*_,z:o.z+p[9]*_},zoom:m}}class Cv{constructor(){y(this,"held",null);y(this,"release",0)}reset(){this.held=null,this.release=0}update(t,e,i,n){const r=Jt.camera;return n!==null?this.held===null||n<this.held?(this.held=n,this.release=0):n>this.held+r.hysteresisThreshold?(this.release+=t,this.release>r.returnDelay&&(this.held=n,this.release=0)):this.release=0:(this.release+=t,this.release>r.returnDelay&&(this.held=null)),xe(e,this.held===null?i:Math.min(i,this.held),r.interpolationSpeed,t)}}const kc=80*Math.PI/180,Pv=7;class Iv extends ge{constructor(){super();y(this,"body");y(this,"head",new ge);y(this,"tail",new ge);y(this,"legs",[]);y(this,"tucked",new ge);y(this,"wrapped");y(this,"eyes",[]);y(this,"loaf",1);y(this,"lastClock",null);const e=new Vn({color:2500909,roughness:.92}),i=new Vn({color:3619392,roughness:1}),n=(o,c,l,h,u,d,f,g=e)=>{const v=new Vt(new hn(1,20,14),g);return v.position.set(c,l,h),v.scale.set(u,d,f),v.castShadow=!0,o.add(v),v};this.body=n(this,-.04,.29,0,.49,.285,.3),n(this,-.27,.26,0,.29,.245,.29),n(this,.2,.29,0,.23,.25,.25),this.head.position.set(.31,.5,0),this.add(this.head),n(this.head,0,0,0,.23,.215,.21),n(this.head,.12,-.055,-.075,.13,.09,.09,i),n(this.head,.12,-.055,.075,.13,.09,.09,i);const r=new Vt(new pn(.034,.035,3),new Vn({color:9203063}));r.rotation.z=-Math.PI/2,r.position.set(.25,-.045,0),this.head.add(r);for(const o of[-.14,.14]){const c=new Vt(new pn(.115,.24,4),e);c.position.set(-.025,.205,o),c.rotation.x=o>0?.17:-.17,this.head.add(c);const l=new Vt(new pn(.064,.135,3),new Vn({color:7430248,roughness:1}));l.position.set(.035,.205,o),this.head.add(l);const h=n(this.head,.188,.02,o*.9,.028,.038,.04,new Vn({color:11777394}));this.eyes.push(h),n(h,.3,0,0,.55,.85,.22,new Ge({color:1448984}));for(let u=0;u<3;u++){const d=[new C(.19,-.065,o*.5),new C(.25+u*.035,-.05+(u-1)*.035,o*2.2)],f=new Th(new we().setFromPoints(d),new wh({color:10066577,transparent:!0,opacity:.55}));this.head.add(f)}}for(const o of[-.29,.25])for(const c of[-.18,.18])this.legs.push(n(this,o,.13,c,.09,.16,.08));this.add(this.tucked);for(const o of[-.16,.16])n(this.tucked,.24,.065,o,.12,.06,.085,i);const a=(o,c)=>new Vt(new nl(new Ch(o),24,c,8,!1),e);this.wrapped=a([new C(-.43,.13,-.13),new C(-.45,.09,-.27),new C(-.12,.065,-.32),new C(.18,.065,-.26)],.065),this.add(this.wrapped),this.tail.add(a([new C(-.4,.3,0),new C(-.66,.38,0),new C(-.77,.65,.03),new C(-.7,.8,.06)],.06)),this.add(this.tail)}update(e,i,n){const r=this.lastClock===null||i<this.lastClock,a=r?0:Math.min(.05,Math.max(0,i-this.lastClock));this.lastClock=i;const o=e.phase==="rest",c=["flee","roam","emerging"].includes(e.phase);this.loaf=xe(this.loaf,o?1:0,9,a),this.position.set(e.visualPosition.x,e.height+(c?Math.abs(Math.sin(i*14))*.035:0),e.visualPosition.z),this.rotation.y=-e.heading,this.body.scale.y=.285+Math.sin(i*(o?1.7:3))*.006,this.body.scale.z=.27+this.loaf*.03,this.head.position.y=.56-this.loaf*.09;const l=this.position.x+Math.cos(this.rotation.y)*this.head.position.x,h=this.position.z-Math.sin(this.rotation.y)*this.head.position.x,u=zt(Ci(-Math.atan2(n.z-h,n.x-l)-this.rotation.y),-kc,kc);this.head.rotation.y=r?u:xe(this.head.rotation.y,u,Pv,a),this.head.rotation.z=e.phase==="active"?.15:-this.loaf*.05,this.wrapped.rotation.y=Math.sin(i*1.8)*.1,this.wrapped.rotation.x=Math.sin(i*2.3+.5)*.035,this.wrapped.visible=this.loaf>.45,this.tucked.visible=this.loaf>.4,this.tail.visible=this.loaf<.55,this.tail.rotation.x=Math.sin(i*(e.phase==="flee"?12:2))*.15,this.legs.forEach((d,f)=>{d.scale.y=.16*(1-this.loaf*.8),d.position.y=.13-this.loaf*.075,d.visible=this.loaf<.9,d.rotation.z=c?Math.sin(i*(e.phase==="flee"?22:12)+f*Math.PI)*.5:0}),this.eyes.forEach(d=>d.scale.y=.038*(1-this.loaf*.85))}}class Dv{constructor(t){y(this,"shadowMap",{enabled:!1,type:0});y(this,"outputColorSpace","");y(this,"toneMapping",0);y(this,"toneMappingExposure",1);y(this,"domElement");y(this,"ctx");y(this,"width",1);y(this,"height",1);y(this,"last",0);if(this.domElement=t,this.ctx=t.getContext("2d"),!this.ctx)throw new Error("Canvas unavailable")}setPixelRatio(t){}setSize(t,e){this.width=t,this.height=e,this.domElement.width=t,this.domElement.height=e}render(t,e){const i=performance.now();if(i-this.last<65)return;this.last=i,t.updateMatrixWorld(),e.updateMatrixWorld();const n=this.ctx,r=this.width,a=this.height;n.fillStyle="#e9e9df",n.fillRect(0,0,r,a);const o=[],c=l=>(l.project(e),{x:(l.x*.5+.5)*r,y:(.5-l.y*.5)*a});t.traverseVisible(l=>{if(!(l instanceof Vt)||l instanceof kr)return;const h=Array.isArray(l.material)?l.material[0]:l.material;if(h.opacity<=0)return;if(l.userData.ribbon){const _=l.userData.points,P=l.userData.width*this.height/(25/e.zoom);o.push({depth:950,points:_.map(R=>c(new C(R.x,l.userData.height,R.z))),color:h.color.getStyle(),opacity:h.opacity,line:Math.max(1.5,P)});return}if(l.userData.arrow){const _=l.geometry.getAttribute("position"),P=l.geometry.index;for(let R=0;R<(P?.count??_.count);R+=3){const I=[];for(let T=0;T<3;T++)I.push(c(new C().fromBufferAttribute(_,P?P.getX(R+T):R+T).applyMatrix4(l.matrixWorld)));o.push({depth:900,points:I,color:h.color.getStyle(),opacity:1})}return}const u=l.geometry;u.boundingBox||u.computeBoundingBox();const d=u.boundingBox,f=d.getSize(new C),g=d.getCenter(new C).applyMatrix4(l.matrixWorld);if(f.x>80)return;const v=g.clone().applyMatrix4(e.matrixWorldInverse),m=g.y<.135?1e3-v.z:(Math.abs(g.x)>12.8||g.z<-8.8)&&f.y>.5?500-v.z:-v.z,p=h.color??new Ht(14604998),w=_=>p.clone().multiplyScalar(_).getStyle();if(u instanceof Ws||u instanceof pn){const _=u.parameters,P=_.radiusTop??0,R=_.radiusBottom??_.radius??.1,I=_.height,T=Math.min(24,_.radialSegments),S=[],b=[];for(let F=0;F<T;F++){const B=F/T*Math.PI*2;S.push(new C(Math.sin(B)*P,I/2,Math.cos(B)*P).applyMatrix4(l.matrixWorld)),b.push(new C(Math.sin(B)*R,-I/2,Math.cos(B)*R).applyMatrix4(l.matrixWorld))}const L=(F,B)=>{const Y=F.reduce((W,$)=>W.add($),new C).multiplyScalar(1/F.length).applyMatrix4(e.matrixWorldInverse);o.push({depth:m>500?m:-Y.z,points:F.map(W=>c(W.clone())),color:w(B),opacity:h.opacity})};L(S,1.08);for(let F=0;F<T;F++){const B=(F+1)%T;L([S[F],S[B],b[B],b[F]],.88+Math.cos(F/T*Math.PI*2)*.08)}return}if(u.type==="SphereGeometry"){const _=c(g.clone()),P=c(new C(d.max.x,0,0).applyMatrix4(l.matrixWorld)),R=c(new C(0,0,d.max.z).applyMatrix4(l.matrixWorld)),I=c(new C(0,d.max.y,0).applyMatrix4(l.matrixWorld)),T=Math.max(Math.abs(P.x-_.x),Math.abs(R.x-_.x),1),S=Math.max(Math.abs(I.y-_.y),Math.abs(R.y-_.y),2);o.push({depth:m,points:[_,{x:T,y:S}],color:w(1.04),opacity:h.opacity,ellipse:!0});return}if(u instanceof pi||u instanceof mn||u instanceof ks){const _=u instanceof pi?u.parameters.radius+u.parameters.tube:u instanceof mn?u.parameters.outerRadius:u.parameters.radius,P=u instanceof pi?u.parameters.radius-u.parameters.tube:u instanceof mn?u.parameters.innerRadius:0,R=u instanceof pi?u.parameters.arc:Math.PI*2,I=[];for(let T=0;T<=32;T++){const S=T/32*R;I.push(new C(Math.cos(S)*_,Math.sin(S)*_,0))}if(P)for(let T=32;T>=0;T--){const S=T/32*R;I.push(new C(Math.cos(S)*P,Math.sin(S)*P,0))}o.push({depth:m,points:I.map(T=>c(T.applyMatrix4(l.matrixWorld))),color:w(1),opacity:h.opacity});return}const E=[[d.min.x,d.min.y,d.min.z],[d.max.x,d.min.y,d.min.z],[d.max.x,d.min.y,d.max.z],[d.min.x,d.min.y,d.max.z],[d.min.x,d.max.y,d.min.z],[d.max.x,d.max.y,d.min.z],[d.max.x,d.max.y,d.max.z],[d.min.x,d.max.y,d.max.z]].map(_=>new C(..._).applyMatrix4(l.matrixWorld));for(const[_,P]of[[[0,3,7,4],.83],[[1,5,6,2],.91],[[3,2,6,7],.96],[[4,7,6,5],1.08]]){const R=_.map(T=>E[T]),I=R.reduce((T,S)=>T.add(S),new C).multiplyScalar(.25).applyMatrix4(e.matrixWorldInverse);o.push({depth:m>500?m:-I.z,points:R.map(T=>c(T.clone())),color:w(P),opacity:h.opacity})}}),o.sort((l,h)=>h.depth-l.depth);for(const l of o)n.globalAlpha=l.opacity,n.fillStyle=l.color,n.beginPath(),l.ellipse?n.ellipse(l.points[0].x,l.points[0].y,l.points[1].x,l.points[1].y,0,0,Math.PI*2):(l.points.forEach((h,u)=>u?n.lineTo(h.x,h.y):n.moveTo(h.x,h.y)),l.line||n.closePath()),l.line?(n.strokeStyle=l.color,n.lineWidth=l.line,n.lineCap="round",n.lineJoin="round",n.stroke()):n.fill();n.globalAlpha=1,t.traverseVisible(l=>{if(!(l instanceof Th))return;const h=l.geometry.getAttribute("position");if(!h)return;const u=l.material;n.strokeStyle=u.color.getStyle(),n.globalAlpha=u.opacity,n.lineWidth=1,n.setLineDash([4,6]),n.beginPath();for(let d=0;d<h.count;d++){const f=c(new C().fromBufferAttribute(h,d).applyMatrix4(l.matrixWorld));d?n.lineTo(f.x,f.y):n.moveTo(f.x,f.y)}n.stroke(),n.setLineDash([]),n.globalAlpha=1}),t.traverseVisible(l=>{if(!(l instanceof kr)||l.geometry.type!=="SphereGeometry")return;const h=new ie;n.fillStyle="#a68e6e",n.globalAlpha=.58;for(let u=0;u<l.count;u++){if(l.getMatrixAt(u,h),h.elements[0]===0)continue;const d=c(new C().setFromMatrixPosition(h).applyMatrix4(l.matrixWorld));n.beginPath(),n.arc(d.x,d.y,1,0,Math.PI*2),n.fill()}n.globalAlpha=1})}}const Ss=new C;function Ze(s,t,e,i,n,r){const a=2*Math.PI*n/4,o=Math.max(r-2*n,0),c=Math.PI/4;Ss.copy(t),Ss[i]=0,Ss.normalize();const l=.5*a/(a+o),h=1-Ss.angleTo(s)/c;return Math.sign(Ss[e])===1?h*l:o/(a+o)+l+l*(1-h)}class al extends $i{constructor(t=1,e=1,i=1,n=2,r=.1){const a=n*2+1;if(r=Math.min(t/2,e/2,i/2,r),super(1,1,1,a,a,a),this.type="RoundedBoxGeometry",this.parameters={width:t,height:e,depth:i,segments:n,radius:r},a===1)return;const o=this.toNonIndexed();this.index=null,this.attributes.position=o.attributes.position,this.attributes.normal=o.attributes.normal,this.attributes.uv=o.attributes.uv;const c=new C,l=new C,h=new C(t,e,i).divideScalar(2).subScalar(r),u=this.attributes.position.array,d=this.attributes.normal.array,f=this.attributes.uv.array,g=u.length/6,v=new C,m=.5/a;for(let p=0,w=0;p<u.length;p+=3,w+=2)switch(c.fromArray(u,p),l.copy(c),l.x-=Math.sign(l.x)*m,l.y-=Math.sign(l.y)*m,l.z-=Math.sign(l.z)*m,l.normalize(),u[p+0]=h.x*Math.sign(c.x)+l.x*r,u[p+1]=h.y*Math.sign(c.y)+l.y*r,u[p+2]=h.z*Math.sign(c.z)+l.z*r,d[p+0]=l.x,d[p+1]=l.y,d[p+2]=l.z,Math.floor(p/g)){case 0:v.set(1,0,0),f[w+0]=Ze(v,l,"z","y",r,i),f[w+1]=1-Ze(v,l,"y","z",r,e);break;case 1:v.set(-1,0,0),f[w+0]=1-Ze(v,l,"z","y",r,i),f[w+1]=1-Ze(v,l,"y","z",r,e);break;case 2:v.set(0,1,0),f[w+0]=1-Ze(v,l,"x","z",r,t),f[w+1]=Ze(v,l,"z","x",r,i);break;case 3:v.set(0,-1,0),f[w+0]=1-Ze(v,l,"x","z",r,t),f[w+1]=1-Ze(v,l,"z","x",r,i);break;case 4:v.set(0,0,1),f[w+0]=1-Ze(v,l,"x","y",r,t),f[w+1]=1-Ze(v,l,"y","x",r,e);break;case 5:v.set(0,0,-1),f[w+0]=Ze(v,l,"x","y",r,t),f[w+1]=1-Ze(v,l,"y","x",r,e);break}}static fromJSON(t){return new al(t.width,t.height,t.depth,t.segments,t.radius)}}class Bi extends Vt{constructor(e,i,n,r=1,a=5e3){const o=new we,c=new Float32Array(a*18);o.setAttribute("position",new $e(c,3).setUsage(fh)),o.setDrawRange(0,0);super(o,new Ge({color:e,transparent:!0,opacity:r,side:Ne,depthWrite:!1}));y(this,"width");y(this,"height");y(this,"buffer");y(this,"capacity");this.width=i,this.height=n,this.buffer=c,this.capacity=a,this.frustumCulled=!1,this.userData.ribbon=!0,this.userData.points=[],this.userData.width=i}setPoints(e){const i=Math.min(e.length-1,this.capacity);this.userData.points=e,this.userData.height=this.height;let n=0;for(let r=0;r<i;r++){const a=e[r],o=e[r+1],c=Math.hypot(o.x-a.x,o.z-a.z)||1,l=-(o.z-a.z)/c*this.width/2,h=(o.x-a.x)/c*this.width/2;for(const[u,d]of[[a,1],[a,-1],[o,1],[o,1],[a,-1],[o,-1]])this.buffer[n++]=u.x+l*d,this.buffer[n++]=this.height,this.buffer[n++]=u.z+h*d}this.geometry.setDrawRange(0,Math.max(0,i)*6),this.geometry.attributes.position.needsUpdate=!0}setDashes(e,i,n,r){const a=i+n,o=this.width/2;let c=0,l=0,h=0;this.userData.points=e,this.userData.height=this.height;const u=(d,f,g,v)=>{for(const[m,p]of[[d,1],[d,-1],[f,1],[f,1],[d,-1],[f,-1]])this.buffer[c++]=m.x+g*p,this.buffer[c++]=this.height,this.buffer[c++]=m.z+v*p;l++};for(let d=0;d<e.length-1&&l<this.capacity;d++){const f=e[d],g=e[d+1],v=Math.hypot(g.x-f.x,g.z-f.z);if(v<1e-6)continue;const m=(g.x-f.x)/v,p=(g.z-f.z)/v,w=-p*o,E=m*o;let _=0;for(;_<v&&l<this.capacity;){const P=((h+_-r)%a+a)%a,R=P<i,I=Math.min(v-_,R?i-P:a-P);R&&u({x:f.x+m*_,z:f.z+p*_},{x:f.x+m*(_+I),z:f.z+p*(_+I)},w,E),_+=I}h+=v}this.geometry.setDrawRange(0,l*6),this.geometry.attributes.position.needsUpdate=!0}}const Gn={duration:.28,amplitude:.16,fullRunUp:2.5,minimumStrength:.15},zc=[{x:.08,y:.3,z:.2},{x:-.06,y:.36,z:-.18},{x:.16,y:.46,z:.02}],bs={wood:13279864,wall:15525078,blue:4294577,amber:13274423};class Lv{constructor(t){y(this,"playableView",null);y(this,"rugs",new ge);y(this,"blackCat",new Iv);y(this,"goldZoom",new Cv);y(this,"renderer");y(this,"scene",new Jd);y(this,"camera");y(this,"root",new ge);y(this,"robot",new ge);y(this,"cat",new ge);y(this,"catHead",new ge);y(this,"catBody");y(this,"tail",new ge);y(this,"eyes",[]);y(this,"paw");y(this,"brush",new ge);y(this,"robotWheel");y(this,"dust");y(this,"dustCount",0);y(this,"floorRevision",-1);y(this,"routeLine");y(this,"playerPath",new Bi(4294577,.11,.19,.95,100));y(this,"acceptedPath",new Bi(16738816,.13,.175,.9,2500));y(this,"acceptedGlow",new Bi(16752714,.3,.172,.25,2500));y(this,"acceptedFlow",new Bi(16774358,.075,.178,1,2500));y(this,"obstacleHighlight",new Bi(16727450,.14,.162,.9,8));y(this,"obstacleGlow",new Bi(16748484,.5,.158,.3,8));y(this,"highlightKey","");y(this,"highlightLoop",[]);y(this,"activePath",new Bi(5541731,.09,.205,1,150));y(this,"bonusMeshes",new Map);y(this,"playerArc",new ge);y(this,"systemArc",new ge);y(this,"sharedArc",new ge);y(this,"furniture",new Map);y(this,"particlePool",[]);y(this,"magnetRing",new Vt(new mn(3.12,3.18,64),new Ge({color:8037263,transparent:!0,opacity:.23,side:Ne,depthWrite:!1})));y(this,"magnetMotes",[]);y(this,"look",new C(-5,0,2));y(this,"clock",0);y(this,"shake",0);y(this,"shakeStrength",1);y(this,"highQuality",!0);y(this,"lastSimulation",null);y(this,"robotShadow");y(this,"temp",new _e);y(this,"catZ");y(this,"angerMarks",[]);y(this,"sharedGlow");y(this,"sharedHeading",null);y(this,"shakeOffset",new C);y(this,"routeVisible",!0);y(this,"failTime",0);const e=t.getContext("webgl2",{antialias:!0,alpha:!1,powerPreference:"high-performance"});this.renderer=e?new bv({canvas:t,context:e,antialias:!0,alpha:!1,powerPreference:"high-performance"}):new Dv(t),this.renderer.setPixelRatio(Math.min(window.devicePixelRatio,1.65)),this.renderer.shadowMap.enabled=!0,this.renderer.shadowMap.type=th,this.renderer.outputColorSpace=Ve,this.renderer.toneMapping=ih,this.renderer.toneMappingExposure=1.25,this.scene.background=new Ht(15329759),this.scene.fog=new Qo(15329759,48,95),this.camera=new sl(-20,20,12,-12,.1,120),this.camera.position.set(16,23,22),this.camera.lookAt(0,0,0),this.scene.add(this.root),this.scene.add(new Ff(16775140,8029560,2.2));const i=new dc(16772555,3.1);i.position.set(-4,18,-6),i.castShadow=!0,i.shadow.mapSize.set(1024,1024),i.shadow.camera.left=-20,i.shadow.camera.right=20,i.shadow.camera.top=18,i.shadow.camera.bottom=-18,i.shadow.normalBias=.04,i.shadow.bias=-15e-5,this.scene.add(i);const n=new dc(16777215,.7);n.position.set(10,8,12),this.scene.add(n),this.root.add(this.blackCat),this.buildHouse(),this.buildRobot(),this.buildCat(),this.buildArcs(),this.root.add(this.obstacleGlow,this.obstacleHighlight,this.playerPath,this.acceptedGlow,this.acceptedPath,this.acceptedFlow,this.activePath);const r=this.material(14665364);for(let c=0;c<36;c++){const l=new Vt(new hn(.06,5,4),r);l.visible=!1,this.root.add(l),this.particlePool.push({mesh:l,vx:0,vz:0,vy:0,life:0})}this.magnetRing.rotation.x=-Math.PI/2,this.magnetRing.visible=!1,this.root.add(this.magnetRing);const a=new hn(.065,6,4),o=this.material(16047534);for(let c=0;c<40;c++){const l=new Vt(a,o);l.visible=!1,this.root.add(l),this.magnetMotes.push({mesh:l,life:0})}this.resize(),window.addEventListener("resize",()=>this.resize())}material(t,e=.75){return new Vn({color:t,roughness:e})}box(t,e,i,n,r,a,o,c,l=.07){const h=new Vt(l>0?new al(e,i,n,2,l):new $i(e,i,n),this.material(c));return h.position.set(r,a,o),h.castShadow=!0,h.receiveShadow=!0,t.add(h),h}sphere(t,e,i,n,r,a,o,c){const l=new Vt(new hn(1,16,12),this.material(c));return l.position.set(e,i,n),l.scale.set(r,a,o),l.castShadow=!0,t.add(l),l}cylinder(t,e,i,n,r,a,o,c){const l=new Vt(new Ws(e,i,n,24),this.material(c));return l.position.set(r,a,o),l.castShadow=!0,l.receiveShadow=!0,t.add(l),l}plant(t,e,i,n=1){const r=new ge;r.position.set(e,0,i),r.scale.setScalar(n),t.add(r),this.cylinder(r,.32,.24,.52,0,.26,0,12155221),this.cylinder(r,.27,.27,.03,0,.53,0,7362113);for(let a=0;a<9;a++){const o=a*2.4,c=this.cylinder(r,.025,.025,.85,Math.cos(o)*.12,.85,Math.sin(o)*.12,4943952);c.rotation.z=Math.cos(o)*.35,this.sphere(r,Math.cos(o)*.29,1.05+a%3*.18,Math.sin(o)*.29,.18,.37,.08,a%2?7114075:8888427).rotation.set(Math.sin(o)*.5,o,Math.cos(o)*.65)}return r}buildHouse(){const t=zo(741);this.box(this.root,dn+.4,.42,fn+.4,0,-.3,0,10188886,.16),this.box(this.root,dn,.12,fn,0,-.04,0,bs.wood,.02);const e=new $i(1.985,.045,.49),i=new kr(e,this.material(bs.wood),468);let n=0;const r=[13608573,13872259,13279351,14135433,13346432];for(let l=0;l<36;l++)for(let h=0;h<13;h++)this.temp.position.set(-12+h*2,.026,-8.75+l*.5),this.temp.scale.set(1,1,1),this.temp.updateMatrix(),i.setMatrixAt(n,this.temp.matrix),i.setColorAt(n++,new Ht(r[Math.floor(t()*r.length)]));i.receiveShadow=!0,this.root.add(i),this.box(this.root,26.45,2.7,.2,0,1.2,-9.1,bs.wall,.06),this.box(this.root,.2,2.7,18.3,-13.1,1.2,0,14737356,.06),this.box(this.root,26.1,.14,.12,0,.1,-8.95,16249054,.02),this.box(this.root,.12,.14,18,-12.96,.1,0,16249054,.02),this.box(this.root,3.25,1.9,.14,.5,1.6,-8.94,16446434,.03),this.box(this.root,2.9,1.65,.05,.5,1.62,-8.84,12505029,.02),this.box(this.root,.09,1.7,.09,.5,1.62,-8.77,16775144,.01),this.box(this.root,2.95,.08,.09,.5,1.62,-8.77,16775144,.01),this.box(this.root,3.5,.15,.45,.5,.65,-8.82,16051415,.025);for(const l of[-1.28,2.28])for(let h=0;h<4;h++)this.cylinder(this.root,.085,.085,2.15,l+h*.09,.99,-8.68,14734777);const a=new Vt(new as(4,3.5),new Ge({color:16771250,transparent:!0,opacity:.18,depthWrite:!1}));a.rotation.x=-Math.PI/2,a.rotation.z=-.28,a.position.set(.8,.058,-3.5),this.root.add(a),this.root.add(this.rugs),this.box(this.rugs,5.4,.035,3.5,-4,.08,-1.5,14274230,.15),this.box(this.rugs,5.05,.016,3.15,-4,.105,-1.5,12172956,.15);for(let l=0;l<11;l++)this.box(this.rugs,.025,.009,3.1,-6.4+l*.48,.12,-1.5,13225136,0);this.box(this.rugs,2,.045,4.5,1,.08,1.5,14472377,.14);for(let l=-.5;l<3.6;l+=.38)this.box(this.rugs,1.8,.008,.025,1,.11,l,13616292,0);for(const l of Ii){const h=new ge;if(h.position.set(l.x,0,l.z),this.root.add(h),this.furniture.set(l.kind+l.x,h),l.kind==="sofa"){for(const f of[-1.45,1.45])for(const g of[-.43,.43])this.cylinder(h,.075,.06,.26,f,.13,g,7691328);this.box(h,3.6,.52,1.35,0,.47,0,6455156,.18),this.box(h,3.55,.88,.33,0,.99,-.51,7113597,.13);for(const f of[-1.62,1.62])this.box(h,.34,.7,1.3,f,.76,0,7902599,.13);for(const f of[-1,0,1])this.box(h,.97,.23,1.06,f,.8,.07,8888723,.12);const u=this.box(h,.57,.57,.18,-1.05,1.02,-.18,15850668,.11);u.rotation.z=.16;const d=this.box(h,.56,.5,.2,1.07,1.03,-.2,13074784,.09);d.rotation.z=-.18,this.box(h,.68,.05,.9,.15,.94,.19,14141610,.025)}else if(l.kind==="coffee"||l.kind==="dining"||l.kind==="desk"){const u=l.kind==="coffee"?.62:1.12;for(const d of[-l.w*.38,l.w*.38])for(const f of[-l.d*.34,l.d*.34])this.cylinder(h,.06,.08,u,d,u/2,f,9203020);if(this.box(h,l.w,.14,l.d,0,u,0,l.kind==="coffee"?14137234:11898985,.14),l.kind==="coffee"&&(this.box(h,.52,.05,.38,-.34,u+.1,.1,5732461,.018),this.box(h,.44,.04,.34,-.3,u+.145,.08,15918533,.015),this.cylinder(h,.13,.11,.18,.44,u+.15,.07,15984846)),l.kind==="dining"){this.cylinder(h,.18,.14,.45,0,u+.24,0,14010021);for(let d=0;d<4;d++)this.sphere(h,Math.sin(d*2)*.13,u+.6+d*.03,Math.cos(d*2)*.13,.18,.1,.12,14860399);for(const d of[-.8,.8])this.cylinder(h,.27,.27,.04,d,u+.1,0,15260609)}l.kind==="desk"&&(this.box(h,.8,.04,.54,-.7,u+.1,0,15393996,.02),this.box(h,.06,.62,.6,-.7,u+.4,-.2,8622981,.03).rotation.z=-.2,this.cylinder(h,.25,.25,.05,.7,u+.1,0,5795158),this.cylinder(h,.03,.03,.64,.7,u+.42,0,5795158),this.cylinder(h,.32,.18,.25,.7,u+.81,0,9806468))}else if(l.kind==="chair"){for(const u of[-.33,.33])for(const d of[-.22,.22])this.box(h,.07,.6,.07,u,.3,d,9663053,.01);this.box(h,.95,.12,.62,0,.64,0,13149046,.07),this.box(h,.95,.65,.09,0,1,-.26,12163952,.04)}else if(l.kind==="plant")this.plant(h,0,0);else if(l.kind==="box")this.box(h,.85,.55,.73,0,.28,0,12029282,.045),this.box(h,.16,.025,.73,0,.56,0,14664593,.01),this.box(h,.3,.2,.02,.14,.29,.37,15391419,.015);else if(l.kind==="shelf"){h.rotation.y=Math.PI/2,this.box(h,.72,2.25,2.65,0,1.13,0,11373670,.035);for(const u of[.35,1,1.65]){this.box(h,.77,.07,2.7,.02,u,0,13084800,.02);for(let d=0;d<8;d++){const f=this.box(h,.46,.35+t()*.16,.12,.22,u+.24,-1.08+d*.28,[9149052,12424557,14862751,9084571][d%4],.01);f.rotation.x=(t()-.5)*.12}}}else l.kind==="catbed"&&(this.cylinder(h,.87,.79,.19,0,.11,0,12033915),this.cylinder(h,.72,.74,.13,0,.23,0,15326144))}this.box(this.root,1.1,1.25,.07,-5.1,1.9,-8.94,9992015,.025),this.box(this.root,.96,1.1,.05,-5.1,1.9,-8.88,15393224,.01),this.sphere(this.root,-5.1,1.97,-8.84,.29,.37,.017,8426615),this.sphere(this.root,-5.1,2.1,-8.82,.12,.12,.02,14072172),this.sphere(this.root,2.5,.15,4.6,.14,.14,.14,12287577),this.sphere(this.root,-6,.08,.7,.18,.065,.1,9540218);const o=this.cylinder(this.root,.055,.055,.64,-5.8,.07,.6,8293477);o.rotation.z=Math.PI/2;const c=new Vt(new as(200,200),this.material(15329759));c.rotation.x=-Math.PI/2,c.position.y=-.55,c.receiveShadow=!0,this.scene.add(c)}buildRobot(){this.root.add(this.robot),this.cylinder(this.robot,.43,.42,.24,0,.23,0,15460568),this.cylinder(this.robot,.44,.44,.09,0,.14,0,6517094),this.cylinder(this.robot,.38,.4,.06,0,.38,0,16315366),this.cylinder(this.robot,.17,.17,.055,0,.425,0,9282956),this.cylinder(this.robot,.038,.038,.015,.03,.46,0,15265231),this.box(this.robot,.06,.08,.23,.409,.25,0,4740683,.025);for(const n of[-.095,.095])this.sphere(this.robot,.445,.27,n,.014,.019,.025,16182216);this.robotWheel=this.cylinder(this.robot,.13,.13,.1,0,.1,.34,4608326),this.robotWheel.rotation.x=Math.PI/2,this.robot.add(this.brush),this.brush.position.set(.27,.075,-.26);for(let n=0;n<3;n++){const r=this.box(this.brush,.4,.015,.04,.12,.01,0,7897964,.005);r.rotation.y=n*Math.PI*2/3}const t=new Vt(new ks(.53,32),new Ge({color:5064504,transparent:!0,opacity:.15,depthWrite:!1}));t.rotation.x=-Math.PI/2,t.position.y=.08,this.robot.add(t),this.robotShadow=t;const e=new No;e.moveTo(1.06,0),e.lineTo(.77,.17),e.lineTo(.81,.055),e.lineTo(.63,.055),e.lineTo(.63,-.055),e.lineTo(.81,-.055),e.lineTo(.77,-.17),e.closePath();const i=new Vt(new Hr(e),new Ge({color:4685459,side:Ne}));i.rotation.x=-Math.PI/2,i.position.y=.2,i.name="Facing arrow",i.userData.arrow=!0,this.robot.add(i)}buildCat(){this.robot.add(this.cat),this.cat.position.set(0,.43,0),this.cat.scale.setScalar(.77),this.cat.rotation.y=Math.PI/2,this.catBody=this.sphere(this.cat,0,.27,0,.5,.27,.34,13011797),this.sphere(this.cat,.15,.25,.02,.32,.235,.29,14792576),this.cat.add(this.catHead),this.catHead.position.set(-.3,.32,.17),this.sphere(this.catHead,0,0,0,.25,.23,.23,13473118),this.sphere(this.catHead,0,-.08,.17,.15,.09,.08,15389092);for(const n of[-.15,.15]){const r=new Vt(new pn(.13,.24,3),this.material(13011797));r.position.set(n,.21,-.02),r.rotation.z=n>0?-.2:.2,this.catHead.add(r),this.sphere(this.catHead,n,.21,.025,.06,.07,.015,14001043);const a=this.sphere(this.catHead,n*.7,.01,.198,.05,.012,.023,5524278);this.eyes.push(a)}this.sphere(this.catHead,0,-.058,.242,.035,.026,.023,10975077),this.paw=this.sphere(this.cat,-.3,.06,.2,.12,.06,.085,14992534),this.cat.add(this.tail);for(let n=0;n<9;n++){const r=-1.5+n*.25;this.sphere(this.tail,Math.cos(r)*.48,.12,Math.sin(r)*.33,.13,.11,.115,n%3===0?11104067:12946777)}for(let n=0;n<3;n++)this.sphere(this.cat,-.06+n*.17,.49,-.09,.045,.014,.19,11368780);const t=document.createElement("canvas");t.width=128,t.height=128;const e=t.getContext("2d");e.fillStyle="#6d8776",e.font="bold 48px Georgia",e.fillText("z",28,99),e.font="bold 32px Georgia",e.fillText("z",69,60);const i=new sc(t);this.catZ=new jl(new Do({map:i,transparent:!0,depthTest:!1})),this.catZ.position.set(.2,1.25,0),this.catZ.scale.set(.8,.8,.8),this.cat.add(this.catZ),this.buildAngerMarks()}updateObstacleHighlight(t,e){const i=t.highlightedObstacle;if(this.obstacleHighlight.visible=this.obstacleGlow.visible=!e&&!!i,!i)return;const n=`${i.x},${i.z},${i.w},${i.d}`;if(n!==this.highlightKey){this.highlightKey=n;const a=i.w/2+.28,o=i.d/2+.28,c=[{x:i.x-a,z:i.z-o},{x:i.x+a,z:i.z-o},{x:i.x+a,z:i.z+o},{x:i.x-a,z:i.z+o},{x:i.x-a,z:i.z-o},{x:i.x+a,z:i.z-o}];this.obstacleHighlight.setPoints(c),this.highlightLoop=c}const r=.5+.5*Math.sin(this.clock*4);this.obstacleHighlight.material.opacity=.7+.3*r,this.obstacleGlow.material.opacity=.12+.45*r,this.obstacleGlow.width=.3+.45*r,this.obstacleGlow.setPoints(this.highlightLoop)}buildAngerMarks(){const t=document.createElement("canvas");t.width=128,t.height=128;const e=t.getContext("2d");e.textAlign="center",e.textBaseline="middle",e.font='96px "Segoe UI Emoji","Apple Color Emoji","Noto Color Emoji",sans-serif',e.fillText("💢",64,70);const i=new sc(t);for(const n of zc){const r=new jl(new Do({map:i,transparent:!0,depthTest:!1}));r.position.set(n.x,n.y,n.z),r.scale.setScalar(.22),r.visible=!1,r.renderOrder=6,r.name="Cat anger",this.catHead.add(r),this.angerMarks.push(r)}}updateAngerMarks(t){const e=jc(t);this.angerMarks.forEach((i,n)=>{if(i.visible=n<e,i.visible){const r=1+Math.max(0,Math.sin(this.clock*(5+n)+n*2))*.22;i.scale.setScalar(.22*r),i.position.y=zc[n].y+Math.sin(this.clock*2.4+n)*.02}})}updateSharedWeight(t,e,i){const n=this.sharedArc.rotation.y;if(i||this.sharedHeading===null||t.mode!=="collaborative")this.sharedHeading=n;else{const a=3+15*zt(t.steering.driverAuthority,0,1);this.sharedHeading=Ci(this.sharedHeading+Ci(n-this.sharedHeading)*(1-Math.exp(-a*e)))}this.sharedArc.rotation.y=this.sharedHeading;const r=!i&&t.mode==="collaborative"&&t.steering.inputMagnitude>.12?zt(t.obstacleArbitration/.9,0,1):0;this.sharedGlow.material.opacity=xe(this.sharedGlow.material.opacity,r*(.55+.25*Math.sin(this.clock*7)),10,e),this.sharedGlow.visible=this.sharedGlow.material.opacity>.01}buildArcs(){for(const[t,e,i]of[[this.playerArc,bs.blue,.72],[this.systemArc,bs.amber,.92],[this.sharedArc,16776435,1.12]]){const n=new Ge({color:e,transparent:!0,opacity:.98,depthTest:!1,side:Ne}),r=new Vt(new pi(i,.029,6,24,.8),n);r.rotation.x=Math.PI/2,r.rotation.z=-.4,t.add(r);const a=new No;a.moveTo(i+.17,0),a.lineTo(i-.05,.11),a.lineTo(i,0),a.lineTo(i-.05,-.11),a.closePath();const o=new Vt(new Hr(a),n);if(o.rotation.x=-Math.PI/2,o.userData.arrow=!0,t.add(o),t===this.sharedArc){const c=new Vt(new pi(i,.047,6,24,.8),new Ge({color:7892569,transparent:!0,opacity:.65,depthTest:!1}));c.rotation.copy(r.rotation),c.position.y=-.002,c.renderOrder=2,r.renderOrder=3,o.renderOrder=3,t.add(c),this.sharedGlow=new Vt(new pi(i,.11,8,32,.95),new Ge({color:16742938,transparent:!0,opacity:0,depthTest:!1,depthWrite:!1})),this.sharedGlow.rotation.x=Math.PI/2,this.sharedGlow.rotation.z=-.475,this.sharedGlow.position.y=-.004,this.sharedGlow.renderOrder=1,this.sharedGlow.name="Arbitration glow",t.add(this.sharedGlow)}t.position.y=.22,t.name=t===this.playerArc?"Player direction":t===this.systemArc?"Automation direction":"Shared direction",this.root.add(t)}}setSimulation(t){this.blackCat.visible=t.scenario.showBlackCat,this.rugs.visible=t.level==="level1";for(const[e,i]of this.furniture)i.visible=t.scenario.obstacles.some(n=>e===n.kind+n.x);this.lastSimulation=t,this.goldZoom.reset();for(const e of this.magnetMotes)e.life=0,e.mesh.visible=!1;this.floorRevision=-1,this.failTime=0,this.clock=0,this.brush.rotation.y=0,this.catHead.position.y=.32;for(const e of this.particlePool)e.life=0,e.mesh.visible=!1;this.dust&&(this.root.remove(this.dust),this.dust.geometry.dispose(),this.dust.material.dispose()),this.dustCount=t.scenario.grid.cells.length,this.dust=new kr(new hn(.045,5,3),this.material(10258025),this.dustCount*2),this.dust.instanceMatrix.setUsage(fh),this.root.add(this.dust),this.routeLine&&(this.root.remove(this.routeLine),this.routeLine.geometry.dispose(),this.routeLine.material.dispose()),this.routeLine=new Bi(9017729,.085,.15,.62),this.routeLine.setPoints(t.scenario.route),this.root.add(this.routeLine);for(const e of this.bonusMeshes.values())this.root.remove(e),e.traverse(i=>{i instanceof Vt&&(i.geometry.dispose(),(Array.isArray(i.material)?i.material:[i.material]).forEach(r=>r.dispose()))});this.bonusMeshes.clear();for(const e of t.scenario.bonuses){const i=new ge;i.position.set(e.x,.18,e.z),this.root.add(i),this.bonusMeshes.set(e.id,i);const n=e.kind==="gold",r=e.kind==="red",a=e.kind==="magnet";i.userData.gold=n;const o=new Vt(new mn(n?.64:.52,n?.79:.64,48),new Ge({color:a?2330246:n?13863686:r?14174265:14763665,side:Ne,transparent:!0,opacity:.9}));o.rotation.x=-Math.PI/2,o.position.y=-.015,o.name="halo",i.add(o);const c=new Vt(new ks(.5,32),new Ge({color:a?14024689:n?16772251:r?16770517:16771827,transparent:!0,opacity:.8,side:Ne,depthWrite:!1}));if(c.rotation.x=-Math.PI/2,c.position.y=-.025,i.add(c),a){const l=new Vt(new pi(.28,.09,8,24,Math.PI),this.material(4106913));l.rotation.x=-Math.PI/2,l.position.y=.15,i.add(l);for(const h of[-.28,.28])this.box(i,.18,.15,.32,h,.15,.15,4106913,.04),this.box(i,.18,.16,.12,h,.15,.34,16773836,.03);i.visible=!1;continue}for(let l=0;l<5;l++)this.sphere(i,Math.sin(l*2.4)*.18,.1+l*.025,Math.cos(l*2.4)*.16,.21,.17,.18,n?l%2?16767560:15312661:r?l%2?16747381:14174265:l%2?16221621:14634129);this.sphere(i,-.13,.34,0,.11,.24,.085,n?16768091:16027837).rotation.z=.25,this.sphere(i,.13,.34,0,.11,.23,.085,n?16768091:16027837).rotation.z=-.3;for(const l of[-.085,.085])this.sphere(i,l,.23,.22,.025,.03,.024,n?6833688:8468312);i.visible=!1}}bonusVisible(t){const e=this.project(t,1);return e.x>75&&e.x<window.innerWidth-75&&e.y>145&&e.y<window.innerHeight-45}resize(){const t=window.innerWidth,e=window.innerHeight;this.renderer.setSize(t,e);const i=12.5;this.camera.left=-i*t/e,this.camera.right=i*t/e,this.camera.top=i,this.camera.bottom=-i,this.camera.updateProjectionMatrix()}project(t,e=.8){const i=new C(t.x,e,t.z).project(this.camera);return{x:(i.x*.5+.5)*window.innerWidth,y:(-.5*i.y+.5)*window.innerHeight}}burst(t,e=!1){let i=0;for(const n of this.particlePool)if(n.life<=0&&i++<12){const r=i*2.4;n.life=.6,n.mesh.visible=!0,n.mesh.position.set(t.x,.25,t.z),n.vx=Math.cos(r)*(e?1.8:1),n.vz=Math.sin(r)*(e?1.8:1),n.vy=.7+i%3*.2}if(e){this.shake=Gn.duration,this.shakeStrength=zt((this.lastSimulation?.collisionRunUp??Gn.fullRunUp)/Gn.fullRunUp,Gn.minimumStrength,1);const n=this.lastSimulation?.nearbyObstacle;if(n){const r=this.furniture.get(n.kind+n.x);r&&(r.rotation.z=.06)}}}update(t,e,i,n){this.clock+=t,this.lastSimulation!==e&&this.setSimulation(e);const r=i?null:e.activeBonuses.find(f=>f.kind==="gold"),a=window.innerWidth/window.innerHeight,o=Jt.camera,c=i?a>1.2?.89:.64:a>1.2?window.innerHeight<500?o.shortLandscapeZoom:o.normalZoom:o.portraitZoom,l=i?new C(-5,0,2):new C(e.position.x,0,e.position.z),h=i?null:this.playableView,u=h?Tv(this.camera,e.position,r,h,window.innerWidth,window.innerHeight,c):r?Rv(this.camera,e.position,r,window.innerWidth,window.innerHeight,c):null;if(u&&l.set(u.look.x,0,u.look.z),this.look.lerp(l,1-Math.exp(-t*3)),this.camera.position.copy(this.look).add(new C(16,23,22)),this.shakeOffset.set(0,0,0),this.shake>0){this.shake=Math.max(0,this.shake-t);const f=Math.pow(this.shake/Gn.duration,2)*Gn.amplitude*this.shakeStrength;this.shakeOffset.set(Math.sin(this.clock*73)*f,0,Math.cos(this.clock*61)*f),this.camera.position.add(this.shakeOffset)}if(this.camera.lookAt(this.look.clone().add(this.shakeOffset)),this.camera.zoom=this.goldZoom.update(t,this.camera.zoom,c,u?.zoom??null),this.camera.updateProjectionMatrix(),h){const f=Av(this.camera,e.position,h,window.innerWidth,window.innerHeight);this.look.x+=f.x,this.look.z+=f.z,this.camera.position.x+=f.x,this.camera.position.z+=f.z,this.camera.lookAt(this.look),this.camera.updateMatrixWorld()}this.blackCat.update(e.critical,this.clock,i?{x:1.6,z:3.4}:e.position),this.robot.position.set(i?1.6:e.position.x,0,i?3.4:e.position.z),this.robot.rotation.y=-e.heading,n||(this.brush.rotation.y+=t*(e.speed+1)*8,this.robot.position.y=Math.sin(this.clock*13)*.005*e.speed);const d=i?0:e.cat.value;if(this.cat.position.set(0,.43+(i?0:e.cat.startle*.09),i?0:e.cat.sway*.08),this.cat.rotation.set(i?0:e.cat.lean+e.cat.sway,Math.PI/2,i?0:Math.sin(this.clock*26)*e.cat.startle*.11),this.catBody.scale.y=.27+Math.sin(this.clock*(d<40?1.8:3.8))*.018,this.catHead.position.y=xe(this.catHead.position.y,d>=60?.64:d>=40?.43:.32,3,t),this.catBody.rotation.z=xe(this.catBody.rotation.z,d>=60?-.55:0,3,t),this.catHead.rotation.z=(d>=20?Math.sin(this.clock*2)*.035:0)+(i?0:e.cat.startle*.18),this.catHead.rotation.y=d>=60?Math.sin(this.clock*.8)*.2:0,this.eyes.forEach((f,g)=>f.scale.y=d>=60?.05:d>=40&&g===0?.045:.012),this.tail.rotation.y=d>=80?Math.sin(this.clock*9)*.48:d>=20?Math.sin(this.clock*1.5)*.06:0,this.catZ.visible=d<60,this.catZ.position.y=1.23+Math.sin(this.clock*1.3)*.055,this.updateAngerMarks(d),e.status==="failed"&&!i?(this.failTime+=t,this.paw.position.x=-.36-Math.max(0,Math.sin(this.clock*12))*.28,this.robot.rotation.y+=Math.sin(this.clock*8)*.25,this.robot.rotation.z=Math.sin(this.clock*4)*.08,this.robotWheel.position.z=xe(this.robotWheel.position.z,.8,3,t)):(this.robot.rotation.z=0,this.robotWheel.position.z=.34,this.paw.position.x=-.3),this.floorRevision!==e.scenario.grid.revision){this.floorRevision=e.scenario.grid.revision;let f=0;for(const g of e.scenario.grid.cells)for(let v=0;v<2;v++)this.temp.position.set(g.x+(v?-.1:.13),.14,g.z+(v?.12:-.08)),this.temp.scale.setScalar(g.eligible&&!g.clean?1:0),this.temp.updateMatrix(),this.dust.setMatrixAt(f++,this.temp.matrix);this.dust.instanceMatrix.needsUpdate=!0}if(this.magnetRing.visible=!i&&e.status==="playing"&&e.magnetRemaining>0,this.magnetRing.position.set(e.position.x,.15,e.position.z),this.magnetRing.material.opacity=.19+Math.sin(this.clock*3)*.035,!n&&!i){let f=0;for(const g of this.magnetMotes){if(g.life<=0&&f<e.magnetDust.length){const v=e.magnetDust[f++];g.life=.45,g.mesh.position.set(v.x,.18,v.z),g.mesh.visible=!0}g.life>0&&(g.life-=t,g.mesh.position.x=xe(g.mesh.position.x,e.position.x,7,t),g.mesh.position.z=xe(g.mesh.position.z,e.position.z,7,t),g.mesh.position.y=.18+Math.sin((1-g.life/.45)*Math.PI)*.3,g.mesh.scale.setScalar(Math.max(0,g.life/.45)),g.mesh.visible=g.life>0)}e.magnetDust=[]}this.routeLine.visible=!i,this.updateObstacleHighlight(e,i),this.playerPath.visible=!i&&e.mode!=="automation"&&e.steering.inputMagnitude>.015,this.acceptedPath.visible=!i&&e.plannedPath.length>0,this.acceptedGlow.visible=this.acceptedFlow.visible=this.acceptedPath.visible,this.activePath.visible=!i,this.playerPath.visible&&this.playerPath.setPoints(e.playerPath),this.acceptedPath.visible&&(this.acceptedPath.setPoints(e.plannedPath),this.acceptedGlow.setPoints(e.plannedPath),this.acceptedGlow.material.opacity=.2+.14*(.5+.5*Math.sin(this.clock*3.2)),this.acceptedFlow.setDashes(e.plannedPath,.32,.5,this.clock*1.8)),this.activePath.visible&&this.activePath.setPoints([...e.trail,...e.activePath]);for(const f of e.scenario.bonuses){const g=this.bonusMeshes.get(f.id);if(g.visible=!i&&!f.collected&&!f.expired&&e.time>=f.spawn,g.visible){g.position.x=f.x,g.position.z=f.z,g.position.y=.21+Math.sin(this.clock*4)*.055,g.scale.setScalar((f.kind==="gold"?1.22:1)*(1+Math.sin(this.clock*5)*.075));const v=zt(1-(e.time-f.spawn)/f.duration,0,1);g.getObjectByName("halo").scale.setScalar(.4+v*.7)}}this.playerArc.visible=!i&&e.mode!=="automation"&&e.steering.inputMagnitude>.015,this.systemArc.visible=!i&&e.mode!=="manual",this.sharedArc.visible=!i;for(const[f,g]of[[this.playerArc,e.steering.playerDirection],[this.systemArc,e.steering.systemDirection],[this.sharedArc,e.steering.sharedDirection]])f.position.x=e.position.x,f.position.z=e.position.z,f.rotation.y=-Math.atan2(g.z,g.x);this.updateSharedWeight(e,t,i);for(const f of this.particlePool)f.life>0&&(f.life-=t,f.mesh.position.x+=f.vx*t,f.mesh.position.z+=f.vz*t,f.mesh.position.y+=f.vy*t,f.vy-=t*3,f.mesh.scale.setScalar(Math.max(0,f.life*1.4)),f.mesh.visible=f.life>0);for(const[f,g]of this.furniture)e.events.some(m=>m.type==="collision")&&f===e.nearbyObstacle.kind+e.nearbyObstacle.x&&(g.rotation.z=.035),g.rotation.z=xe(g.rotation.z,0,9,t);this.renderer.render(this.scene,this.camera)}}class Nv{constructor(){y(this,"samples",[]);y(this,"hazardId",null);y(this,"used",!1)}accept(){this.used=!0}reset(){this.samples=[],this.hazardId=null,this.used=!1}sample(t,e,i,n){if(n!==this.hazardId&&(this.samples=[],this.hazardId=n,this.used=!1),!n||this.used)return null;const r=Jt.emergency;this.samples=this.samples.filter(h=>i-h.t<=Math.min(r.maximumGestureDuration,r.windowSeconds));const a=this.samples.find(h=>{const u=i-h.t,d=Math.hypot(t-h.x,e-h.y);return u>0&&u<=r.maximumGestureDuration&&d>=r.swipeDistanceThreshold&&d/u>=r.swipeVelocityThreshold});if(this.samples.push({x:t,y:e,t:i}),!a)return null;const o=t-a.x,c=e-a.y,l=Math.hypot(o,c);return{x:o/l,y:c/l}}}class Uv{constructor(){y(this,"keys",new Set);y(this,"value",{x:0,y:0});y(this,"pointerValue",{x:0,y:0});y(this,"pointerId",null);y(this,"startX",0);y(this,"startY",0);y(this,"enabled",!1);y(this,"surface",null);y(this,"emergencyDetector",new Nv);y(this,"hazardId",()=>null);y(this,"onEmergency",()=>!1);y(this,"onPause",()=>{});y(this,"onEscape",()=>{});y(this,"onReengage",()=>{});y(this,"onModeSwitch",()=>{});y(this,"onDebug",()=>{});y(this,"onGesture",()=>{});window.addEventListener("keydown",t=>{if(t.code==="Escape"){t.repeat||this.onEscape(),this.onGesture();return}if(!t.target?.matches?.('input,textarea,select,[contenteditable="true"]')){if(this.enabled&&["ArrowLeft","ArrowRight","ArrowUp","ArrowDown","a","A","d","D","w","W","s","S","Escape"].includes(t.key)&&t.preventDefault(),t.code==="F9"){t.preventDefault(),t.repeat||this.onDebug();return}t.repeat||(this.keys.add(t.code),t.code==="KeyP"&&this.onPause(),t.code==="KeyE"&&this.onReengage(),t.code==="KeyM"&&this.onModeSwitch(),this.onGesture())}}),window.addEventListener("keyup",t=>this.keys.delete(t.code)),window.addEventListener("blur",()=>{this.reset(),this.enabled&&this.onPause()}),document.addEventListener("visibilitychange",()=>{document.hidden&&(this.reset(),this.enabled&&this.onPause())})}attach(t){this.surface=t,t.addEventListener("pointerdown",i=>{!this.enabled||this.pointerId!==null||(i.preventDefault(),this.onGesture(),this.pointerId=i.pointerId,this.startX=i.clientX,this.startY=i.clientY,this.pointerValue={x:0,y:0},this.value={x:0,y:0},this.emergencyDetector.sample(i.clientX,i.clientY,i.timeStamp/1e3,this.hazardId()),t.setPointerCapture(i.pointerId),t.style.setProperty("--touch-x",`${i.clientX}px`),t.style.setProperty("--touch-y",`${i.clientY}px`),t.style.setProperty("--thumb-x","0px"),t.style.setProperty("--thumb-y","0px"),t.classList.add("dragging"))}),t.addEventListener("pointermove",i=>{if(this.pointerId!==i.pointerId||!this.enabled)return;i.preventDefault();const n=this.emergencyDetector.sample(i.clientX,i.clientY,i.timeStamp/1e3,this.hazardId());n&&this.onEmergency(n)&&this.emergencyDetector.accept();const r=Math.min(108,Math.max(72,t.clientWidth*.14)),a=i.clientX-this.startX,o=i.clientY-this.startY,c=Math.hypot(a,o),l=c<=6?0:Math.min(1,(c-6)/(r-6));this.pointerValue={x:c?a/c*l:0,y:c?o/c*l:0},t.style.setProperty("--thumb-x",`${this.pointerValue.x*r}px`),t.style.setProperty("--thumb-y",`${this.pointerValue.y*r}px`)});const e=i=>{i.pointerId===this.pointerId&&(this.pointerId=null,this.pointerValue={x:0,y:0},t.classList.remove("dragging"))};t.addEventListener("pointerup",e),t.addEventListener("pointercancel",e),t.addEventListener("lostpointercapture",e)}update(t){const e=(this.keys.has("KeyD")||this.keys.has("ArrowRight")?1:0)-(this.keys.has("KeyA")||this.keys.has("ArrowLeft")?1:0),i=(this.keys.has("KeyS")||this.keys.has("ArrowDown")?1:0)-(this.keys.has("KeyW")||this.keys.has("ArrowUp")?1:0),n=Math.max(1,Math.hypot(e,i)),r=this.enabled?this.pointerId!==null?this.pointerValue:{x:e/n,y:i/n}:{x:0,y:0};return this.value=this.enabled&&this.pointerId!==null?{...r}:{x:xe(this.value.x,r.x,9,t),y:xe(this.value.y,r.y,9,t)},Math.hypot(this.value.x,this.value.y)<.001&&(this.value={x:0,y:0}),this.value}reset(){this.emergencyDetector.reset(),this.keys.clear(),this.value={x:0,y:0},this.pointerId=null,this.pointerValue={x:0,y:0},this.surface?.classList.remove("dragging")}}const Ov={tap:.065,page:.042,select:.12,back:.1,"bonk-wood":.19,"bonk-wall":.23,"bonk-soft":.16,"cat-stir":.28,"cat-grumble":.48,"cat-angry":.65,"cat-woke":.88,purr:2.4,cleaning:5.3},De=Math.PI*2;function Fv(s,t){const e=Ov[s],i=new Float32Array(Math.ceil(e*t));let n=71;for(const o of s)n=n*31+o.charCodeAt(0)|0;let r=0,a=0;for(let o=0;o<i.length;o++){const c=o/t,l=c/e;n^=n<<13,n^=n>>>17,n^=n<<5;const h=(n>>>0)/2147483648-1;r+=(h-r)*Math.min(1,2400/t);const u=Math.min(1,c/.003,(e-c)/.012);let d=0;if(s==="page")a+=De*(1450-650*l)/t,d=.3*Math.sin(a)*Math.exp(-c*145)+(h-r)*.24*Math.exp(-c*210);else if(s==="purr"){const f=Math.sin(Math.PI*l)**2,g=(.5+.5*Math.sin(De*65*l))**2;d=(.24*Math.sin(De*150*c)+.14*Math.sin(De*300*c)+r*.6)*(.18+.82*g)*f*.85}else if(s==="cleaning"){const f=Math.min(1,c/.06,(e-c)/.06);d=(r*.65+.055*Math.sin(De*180*c)+.025*Math.sin(De*360*c))*(.88+.12*Math.sin(De*17*c))*f}else if(s.startsWith("bonk")){const f=s==="bonk-soft",g=s==="bonk-wall",v=g?125:f?190:260,m=g?21:f?30:34;d=.52*Math.sin(De*v*c)*Math.exp(-c*m)+.22*Math.sin(De*v*2.63*c)*Math.exp(-c*60),d+=(h-r)*(f?.13:.36)*Math.exp(-c*(g?170:250)),f||(d+=.1*Math.sin(De*780*c)*Math.exp(-Math.max(0,c-.018)*85)*(c>.018?1:0))}else if(s.startsWith("cat")){const f=s==="cat-woke",g=s==="cat-angry",m=f?220+190*Math.sin(Math.PI*l)**2:g?145+32*Math.sin(Math.PI*l):s==="cat-grumble"?175-40*l:310-115*l;a+=De*m*(1+.025*Math.sin(De*19*c))/t;const p=Math.sin(a)*.25+Math.sin(a*2)*.16+Math.sin(a*3)*.11+Math.sin(a*5)*.07,w=.62+.38*Math.sin(De*(g?29:23)*c);d=Math.sin(Math.PI*l)**.7*(p*w+(g?(h-r)*.12:l>.67&&f?(h-r)*.08:r*.1))}else{const f=s==="select"?680:s==="back"?380:1050;d=(.4*Math.sin(De*f*c)+.14*Math.sin(De*f*2.02*c))*Math.exp(-c*(s==="select"?42:75)),d+=(h-r)*.15*Math.exp(-c*320)}i[o]=Math.tanh(d*1.4)*u}return i[0]=0,i[i.length-1]=0,i}const Bc={...nu,sound:!0,haptics:!0,route:!0,quality:!0,volume:.45,interventionShare:Ho,inclusionDelay:Go,takeoverHold:Vo};class kv{constructor(){y(this,"settings",{...Bc});y(this,"best",0);try{const t=JSON.parse(localStorage.getItem("catnap-settings")??"null");t&&(this.settings={...Bc,...t,...qn(t),interventionShare:Dr(t.interventionShare),inclusionDelay:Ba(t.inclusionDelay),takeoverHold:Ha(t.takeoverHold)}),this.best=Number(localStorage.getItem("catnap-best")??0)||0}catch{}}save(){try{localStorage.setItem("catnap-settings",JSON.stringify(this.settings)),localStorage.setItem("catnap-best",String(this.best))}catch{}}}class zv{constructor(t){y(this,"storage");y(this,"context",null);y(this,"master",null);y(this,"hum",null);y(this,"humGain",null);y(this,"purrGain",null);y(this,"cleaningGain",null);y(this,"nextDynamicPulse",0);y(this,"criticalQuietUntil",0);y(this,"buffers",new Map);y(this,"voices",new Set);y(this,"lastOption",-10);y(this,"catBand",0);y(this,"lastCatCue",-10);y(this,"lastImpact",-10);y(this,"vibrationStatus","unavailable");this.storage=t}unlock(){try{if(!this.context){const t=window.AudioContext||window.webkitAudioContext;this.context=new t,this.master=this.context.createGain();const e=this.context.createDynamicsCompressor();e.threshold.value=-16,e.knee.value=14,e.ratio.value=4,e.attack.value=.003,e.release.value=.15,this.master.connect(e),e.connect(this.context.destination),this.hum=this.context.createOscillator(),this.hum.type="sine",this.hum.frequency.value=83,this.humGain=this.context.createGain(),this.humGain.gain.value=0,this.hum.connect(this.humGain),this.humGain.connect(this.master),this.hum.start();const i=this.context.createBufferSource();i.buffer=this.buffer("purr"),i.loop=!0,this.purrGain=this.context.createGain(),this.purrGain.gain.value=0,i.connect(this.purrGain),this.purrGain.connect(this.master),i.start();const n=this.context.createBufferSource();n.buffer=this.buffer("cleaning"),n.loop=!0,this.cleaningGain=this.context.createGain(),this.cleaningGain.gain.value=0,n.connect(this.cleaningGain),this.cleaningGain.connect(this.master),n.start()}this.context.state==="suspended"&&this.context.resume().catch(()=>{}),this.master.gain.setTargetAtTime(this.storage.settings.sound?this.storage.settings.volume:0,this.context.currentTime,.025)}catch{}}buffer(t){let e=this.buffers.get(t);if(!e){const i=this.context,n=Fv(t,i.sampleRate);e=i.createBuffer(1,n.length,i.sampleRate),e.getChannelData(0).set(n),this.buffers.set(t,e)}return e}track(t,e){this.voices.add(t),t.onended=()=>{t.disconnect(),e.disconnect(),this.voices.delete(t)}}sound(t,e=.5,i=0){if(!this.context||!this.master||!this.storage.settings.sound||this.voices.size>=12)return;const n=this.context,r=n.createBufferSource(),a=n.createGain();r.buffer=this.buffer(t),a.gain.value=e,r.connect(a),a.connect(this.master),this.track(r,a),r.start(n.currentTime+i)}option(t="tap",e=!1){if(this.unlock(),!this.context)return;const i=this.context.currentTime;i-this.lastOption<(e?.11:.035)||(this.lastOption=i,this.sound(t==="slide-step"?"page":t.startsWith("select-")||["start","start-comparison","continue","resume"].includes(t)?"select":["back","pause","quit-request","cancel-quit","main"].includes(t)?"back":"tap",e?.22:.4))}resetRun(){this.nextDynamicPulse=0,this.criticalQuietUntil=0,this.catBand=0,this.lastCatCue=-10,this.lastImpact=-10;for(const t of this.voices)try{t.stop()}catch{}}silence(){if(!this.context)return;const t=this.context.currentTime;this.master?.gain.setTargetAtTime(0,t,.025),this.humGain?.gain.setTargetAtTime(0,t,.05),this.purrGain?.gain.setTargetAtTime(0,t,.06),this.cleaningGain?.gain.setTargetAtTime(0,t,.06)}update(t,e,i=0){if(!this.context||!this.master)return;const n=this.context.currentTime;if(this.master.gain.setTargetAtTime(this.storage.settings.sound?this.storage.settings.volume:0,n,.04),this.humGain?.gain.setTargetAtTime(t?.002+Math.min(4,Math.max(0,e))*5e-4:0,n,.16),this.hum?.frequency.setTargetAtTime(78+e*14,n,.1),this.cleaningGain?.gain.setTargetAtTime(t?.032+Math.min(4,Math.max(0,e))*.003:0,n,.16),this.purrGain?.gain.setTargetAtTime(t?Math.max(0,1-i/40)*.3:0,n,.22),!t)return;const r=i>=80?3:i>=60?2:i>=40?1:0;r<this.catBand&&i<[0,35,55,75][this.catBand]&&(this.catBand=r);const a=r>this.catBand,o=r===3&&n-this.lastCatCue>5;(a||o)&&n-this.lastCatCue>1.15&&(this.sound(r===3?"cat-angry":r===2?"cat-grumble":"cat-stir",r===3?.65:.5),this.catBand=r,this.lastCatCue=n)}note(t,e=.12,i="triangle",n=.09,r=0,a=.004){if(!this.context||!this.master||!this.storage.settings.sound||this.voices.size>=12)return;const o=this.context.currentTime+r,c=this.context.createOscillator(),l=this.context.createGain();c.type=i,c.frequency.setValueAtTime(t,o),l.gain.setValueAtTime(0,o),l.gain.linearRampToValueAtTime(n,o+a),l.gain.exponentialRampToValueAtTime(.001,o+e),c.connect(l),l.connect(this.master),this.track(c,l),c.start(o),c.stop(o+e+.02)}vibrate(t){if(!this.storage.settings.haptics){this.vibrationStatus="disabled";return}if(typeof navigator>"u"||typeof navigator.vibrate!="function"){this.vibrationStatus="unavailable";return}try{this.vibrationStatus=navigator.vibrate(t)?"requested":"blocked"}catch{this.vibrationStatus="blocked"}}testFeedback(){return this.unlock(),this.vibrate([65,60,65]),this.note(180,.12,"sine",.12),this.vibrationStatus}dynamic(t,e,i){if(!i||e<this.criticalQuietUntil||t<.025||e<this.nextDynamicPulse)return;const n=Jt.force;this.nextDynamicPulse=e+1/(n.pulseFrequencyMin+(n.pulseFrequencyMax-n.pulseFrequencyMin)*t),this.vibrate(Math.round(n.pulseDurationMin+(n.pulseDurationMax-n.pulseDurationMin)*t)),this.note(125+t*90,.055,"sine",.025+t*.08)}event(t,e="wall"){if(["player_deviation","conflict","intent_detected","sync","collision","bonus_collection"].includes(t)&&(this.criticalQuietUntil=Math.max(this.criticalQuietUntil,performance.now()/1e3+.1)),t==="critical_hazard")this.criticalQuietUntil=performance.now()/1e3+.7,this.note(880,.08,"triangle",.1),this.note(660,.12,"triangle",.08,.09),this.vibrate([22,35,22]);else if(t==="black_cat_collision")this.criticalQuietUntil=performance.now()/1e3+.7,this.sound("cat-angry",.8),this.sound("bonk-soft",.9),this.vibrate([50,35,35]);else if(t==="emergency_swipe")this.note(390,.08,"triangle",.07),this.vibrate(20);else if(t==="collision"){const i=this.context?.currentTime??0;i-this.lastImpact>.1&&(this.lastImpact=i,this.sound(e==="sofa"||e==="catbed"?"bonk-soft":e==="wall"?"bonk-wall":"bonk-wood",.9)),this.vibrate([40,20,20])}else t==="player_deviation"?(this.note(294,.065,"sine",.035),this.vibrate(6)):t==="conflict"?(this.note(220,.12,"sine",.045),this.vibrate(9)):t==="intent_detected"?(this.note(494,.1),this.vibrate([12,30,12])):t==="yielding"?this.note(587,.14,"sine",.07):t==="magnet_started"?(this.note(330,.15,"sine",.06),this.note(660,.25,"sine",.07,.09),this.vibrate(15)):t==="sync"?(this.note(659,.19),this.note(880,.25,"sine",.075,.075),this.vibrate(15)):t==="bonus_collection"||t==="perfect_return"?(this.note(784,.14),this.note(1047,.26,"sine",.085,.09),this.vibrate(12)):t==="bonus_spawn"?this.note(523,.16,"sine",.035):t==="automation_takeover"?(this.note(784,.2,"sine",.022,0,.03),this.note(587,.32,"sine",.02,.19,.03),this.vibrate(18)):t==="automation_reactivated"?(this.note(587,.18,"sine",.02,0,.03),this.note(880,.3,"sine",.022,.17,.03),this.vibrate(12)):t==="cat_woke"?(this.sound("cat-woke",.75),this.sound("bonk-soft",.4,.3),this.vibrate([60,50,80])):t==="complete"&&[523,659,784,1047].forEach((i,n)=>this.note(i,.32,"triangle",.09,n*.11))}}const Da=["manual","automation","collaborative"],Hc={manual:"Manual",automation:"Full Automation",collaborative:"Collaborative"},Bv=[{key:"score",label:"Total score",unit:"points"},{key:"cleaningTime",label:"Cleaning time",unit:"s"},{key:"bonusesCollected",label:"Bunnies collected",unit:""},{key:"peakCatDisturbance",label:"Peak cat disturbance",unit:"%",max:100},{key:"collisions",label:"Collisions",unit:""},{key:"smoothness",label:"Smoothness",unit:"/ 100",max:100},{key:"playerInvolvement",label:"Player involvement",unit:"%",max:100}];function Hv(s){return`<div class="comparison-chart" aria-label="Comparison of all three driving modes"><div class="comparison-legend">${Da.map(t=>`<span><i class="${t}"></i>${Hc[t]}</span>`).join("")}</div><p class="comparison-scale">Each metric has its own scale, shared by all three modes. Bars start at zero.</p><div class="comparison-metrics">${Bv.map(t=>{const e=t.max??Math.max(1,...Da.map(i=>Number(s[i]?.[t.key]??0)));return`<section class="comparison-metric"><h3>${t.label}<small>0–${Math.ceil(e).toLocaleString()} ${t.unit}</small></h3>${Da.map(i=>{const n=s[i],r=Number(n?.[t.key]??0),a=n?`${t.key==="cleaningTime"?r.toFixed(1):Math.round(r).toLocaleString()} ${t.unit}`:"No run";return`<div class="comparison-bar-row"><span>${Hc[i]}</span><div class="comparison-track"><i class="${i}" style="width:${Math.min(100,Math.max(0,r/e*100))}%"></i></div><strong>${a}</strong></div>`}).join("")}</section>`}).join("")}</div></div>`}const Is="#4187b1",$n="#c78c37",ol="#fffef7",un=(s,t,e,i,n=48)=>`<g transform="translate(${s} ${t}) rotate(${e})"><path d="M0 0V-${n}m-6 7 6-7 6 7" fill="none" stroke="${i}" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/></g>`,Er=(s,t,e,i,n)=>`<circle cx="${s}" cy="${t}" r="60" fill="#929783"/>${un(s-7,t+10,i,$n,49)}${un(s+7,t+10,e,Is,44)}${un(s,t+10,n,ol,36)}`,La=`<g font-size="12" fill="#425c49"><circle cx="30" cy="166" r="4" fill="${Is}"/><text x="40" y="170">Player</text><circle cx="126" cy="166" r="4" fill="${$n}"/><text x="136" y="170">System</text><circle cx="235" cy="166" r="4" fill="${ol}" stroke="#859079"/><text x="245" y="170">Shared</text></g>`;function Yh(s){const t=["Two direction contributions produce one shared direction.","Interaction: control shifts seamlessly from system guidance toward player steering, without a mode switch.","Arbitration sets the weight: 30/70 feels heavier, 90/10 lighter, and near obstacles the system share rises up to 90%. Blue is player direction, amber is system direction, white is shared.","Example: the accepted amber route leaves the original rows, avoids furniture and reaches a gold bunny.","More player authority means weaker obstacle feedback; more system authority means stronger feedback."],e=[`<g font-size="13" fill="#425c49" text-anchor="middle"><text x="60" y="36">PLAYER</text><text x="177" y="36">SYSTEM</text><text x="292" y="36">SHARED</text></g><circle cx="60" cy="98" r="40" fill="#929783"/>${un(60,120,45,Is,45)}<text x="111" y="105" font-size="25" fill="#60755d">+</text><circle cx="177" cy="98" r="40" fill="#929783"/>${un(177,120,0,$n,45)}<text x="225" y="105" font-size="25" fill="#60755d">=</text><circle cx="292" cy="98" r="40" fill="#929783"/>${un(292,120,22.5,ol,45)}${La}`,`${Er(91,91,55,-25,.9)}${Er(267,91,55,-25,24.5)}<g font-size="12" fill="#425c49" text-anchor="middle"><text x="91" y="23">SYSTEM GUIDES</text><text x="267" y="23">PLAYER STEERS</text></g><path d="M167 94h27m-7-5 7 5-7 5" stroke="#60755d" stroke-width="2" fill="none"/>${La}`,`${Er(91,91,55,-25,-3)}${Er(267,91,55,-25,49)}<g font-size="12" fill="#425c49" text-anchor="middle"><text x="91" y="23">HEAVIER · 30 / 70</text><text x="267" y="23">LIGHTER · 90 / 10</text></g>${La}`,`<path d="M28 40h170q20 0 20 20t-20 20H40q-16 0-16 20t16 20h162" stroke="#a9b39e" stroke-width="3" stroke-dasharray="6 6" fill="none"/><rect x="228" y="64" width="56" height="42" rx="9" fill="#beac89"/><path d="M70 120q80 0 110-4t40-66q0-26 36-26h47" fill="none" stroke="${$n}" stroke-width="5" stroke-linecap="round"/><circle cx="70" cy="120" r="17" fill="#fff9eb" stroke="#849480" stroke-width="3"/>${un(70,120,90,"#65805b",14)}<g fill="#dfb535"><ellipse cx="306" cy="26" rx="16" ry="11"/><ellipse cx="299" cy="15" rx="4" ry="10"/><ellipse cx="311" cy="14" rx="4" ry="10"/></g><g fill="#526950" font-size="12"><text x="26" y="162">Muted: original rows</text><text x="186" y="162">Amber: accepted plan</text></g>`,`<g font-size="13" fill="#425c49"><text x="25" y="35">MORE PLAYER</text><text x="25" y="111">MORE SYSTEM</text><text x="220" y="35">Gentle feedback</text><text x="220" y="111">Stronger feedback</text></g><rect x="25" y="46" width="160" height="16" rx="8" fill="${$n}"/><path d="M33 54h108" stroke="${Is}" stroke-width="16" stroke-linecap="round"/><rect x="25" y="122" width="160" height="16" rx="8" fill="${$n}"/><path d="M33 130h28" stroke="${Is}" stroke-width="16" stroke-linecap="round"/><g fill="#779469"><rect x="220" y="51" width="27" height="7" rx="3"/><rect x="220" y="127" width="100" height="7" rx="3"/></g><text x="25" y="171" fill="#526950" font-size="12">Same nearby obstacle · different control authority</text>`];return s<0||s>=e.length?"":`<figure class="pairdriver-illustration"><svg viewBox="0 0 360 185" role="img" aria-label="${t[s]}" xmlns="http://www.w3.org/2000/svg"><rect x="1" y="1" width="358" height="183" rx="18" fill="#e9eadb"/>${e[s]}</svg></figure>`}function Gv(s,t){const e=s.step;return`<aside class="tour-panel ${s.compact?"compact":""}" data-page-swipe aria-label="Pairdriver tour"><header><span>${e.label}</span>${e.live?`<button class="guide-toggle" data-action="tour-guide-toggle" aria-expanded="${!s.compact}">${s.compact?"Instructions":"Minimise"}</button>`:""}<button class="guide-close" data-action="pause" aria-label="Pause tour">Ⅱ</button></header><div class="tour-copy"><div class="tour-figure">${Yh(e.picture)}</div><h2>${e.title}</h2><p class="tour-description">${e.text}</p>${["light","weight-choice"].includes(e.id)?`<label class="tour-weight">Player weight <output id="tour-weight-value">${Math.round(t.interventionShare*100)}% player · ${Math.round((1-t.interventionShare)*100)}% system</output><input type="range" min="30" max="90" step="5" value="${Math.round(t.interventionShare*100)}" data-tour-weight aria-label="Tour player weight"><small>30 / 70 · heavy <span>90 / 10 · light</span></small></label>`:""}<div class="tour-authority"><span>Player <b id="tour-player-authority">0%</b></span><i><em id="tour-authority-bar"></em></i><span>System <b id="tour-system-authority">100%</b></span></div><p class="tour-task" id="tour-task">${e.task}</p><small class="tour-pace">${e.live?"Live practice · full speed":"Paused · read at your pace"}</small></div><footer><button class="button text" data-action="tour-prev" ${s.index===0?"disabled":""}>Previous</button><span>${s.index+1} / ${ka.length}</span><button class="button primary" data-action="${e.id==="done"?"tour-finish":"tour-next"}" ${s.ready?"":"disabled"}>${e.id==="done"?"Choose level":"Next"}</button></footer></aside>`}function Vv(s,t,e){return e<900&&Math.abs(s)>55&&Math.abs(s)>Math.abs(t)*1.5?s<0?"next":"prev":null}function Wv(s,t){let e=null;s.addEventListener("pointerdown",i=>{const n=i.target;!n.closest("[data-page-swipe]")||n.closest("button,input,select,a,textarea")||(e={x:i.clientX,y:i.clientY,time:i.timeStamp,id:i.pointerId})}),s.addEventListener("pointerup",i=>{if(!e||i.pointerId!==e.id)return;const n=Vv(i.clientX-e.x,i.clientY-e.y,i.timeStamp-e.time);e=null,n&&t(n)}),s.addEventListener("pointercancel",()=>{e=null})}const Gc={paper:"https://www.nature.com/articles/s44172-022-00051-2",jtekt:"https://www.jtekt.co.jp/e/engineering-journal/1020/1020-13.html"},ko=[{label:"THE IDEA",title:"Two contributors. One direction.",body:"<p>Pairdriver® is JTEKT’s approach to shared steering. The driver can influence an automated vehicle while automation stays involved.</p><p>In CATNAP CLEANER, you supply a direction and the robot supplies a plan. Collaborative mode illustrates three functions working together, not three separate modes.</p>",example:"Interaction · Arbitration · Inclusion"},{label:"01 · INTERACTION",title:"Share control without a switch.",body:"<p><strong>Pairdriver:</strong> driver and automation interact seamlessly. The driver can steer at any moment, without an explicit switch between automated and manual modes.</p><p><strong>Try it here:</strong> your steering contributes immediately in Collaborative mode; release and the system guides the Roomba again. Blue shows your input, amber the system, and white the shared result.</p><p>Compare Full Automation in this game: holding a steering input for the takeover time switches to Manual, and AD back on reactivates automation. Collaborative needs neither switch.</p>",example:"Interaction: seamless, immediate steering influence with no mode switch."},{label:"02 · ARBITRATION",title:"Decide who carries the weight.",body:"<p><strong>Pairdriver:</strong> arbitration allocates authority between driver and automation, and the driver feels that allocation as steering weight.</p><p><strong>In this game:</strong> a phone or PC cannot reproduce steering-wheel resistance, so Player share is our stand-in: 30/70 feels heavier toward the system’s plan; 90/10 gives you lighter, stronger control.</p><p><strong>Near obstacles:</strong> as furniture or a wall gets close ahead, the system share rises smoothly, up to 90%, then returns to your setting once the way is clear.</p>",example:"Arbitration: the weight between you and AD, leaning toward AD near obstacles."},{label:"03 · INCLUSION",title:"Your goal becomes our plan.",body:"<p><strong>Pairdriver:</strong> a sufficiently large, persistent intervention is included in trajectory planning.</p><p><strong>Try it here:</strong> aim at gold until INCLUSION · LOCKED. The robot accepts the destination and plans around furniture. Release your drag; small corrections preserve the plan.</p><p>After collection, it routes toward reachable floor that still needs cleaning. Muted rows show the original route; amber shows the accepted plan.</p>",example:"Inclusion changes the destination, beyond steering influence."},{label:"THE SETTINGS",title:"Tune the shared experience.",body:'<dl class="pairdriver-settings"><div><dt>Player share · 10–100%</dt><dd>Arbitration: steering weight stands in for haptic resistance. Higher player share feels lighter. Near obstacles, AD can still take up to 90%.</dd></div><div><dt>Inclusion time · 0.1–3 seconds</dt><dd>Inclusion: how long to aim at gold before its route locks.</dd></div><div><dt>Dynamic Force Feedback</dt><dd>Obstacle risk and system authority shape warning pulses. More player authority means weaker feedback. OFF preserves the original event feedback.</dd></div></dl>',example:"No learning, history-based personalization, or machine learning."},{label:"RESEARCH & LIMITS",title:"A playful analogy.",body:`<p>This independent game is inspired by published research, not JTEKT’s controller or an official validation. Phone vibration, sound and visual cues cannot apply steering-wheel resistance.</p><p>The game does not estimate driver arm impedance. Current-input thresholds are deterministic. The black-cat events and Full Automation failure probability are game experiment conditions, not Pairdriver performance claims.</p><nav class="pairdriver-references" aria-label="Pairdriver references"><a href="${Gc.paper}" target="_blank" rel="noopener noreferrer">Nakade et al. · Communications Engineering (2023) ↗</a><a href="${Gc.jtekt}" target="_blank" rel="noopener noreferrer">JTEKT engineering article (2024) ↗</a></nav>`,example:"Read the original work for the actual steering system."}],Yn=ko.length;function Xv(s){const t=e=>{const i=ko[e];return`<article class="pairdriver-page" data-slide="${e}"><span class="mini-label">${i.label}</span><h2>${i.title}</h2><div class="pairdriver-explainer">${Yh(e)}<div class="pairdriver-body">${i.body}</div></div><aside class="pairdriver-takeaway">${i.example}</aside></article>`};return s===void 0?ko.map((e,i)=>t(i)).join(""):t(Math.max(0,Math.min(Yn-1,s)))}const wr=4,Na=s=>`<svg viewBox="0 0 100 80" role="img" aria-label="Dust bunny"><g fill="${s}"><ellipse cx="50" cy="48" rx="29" ry="21"/><ellipse cx="37" cy="25" rx="8" ry="18" transform="rotate(-18 37 25)"/><ellipse cx="62" cy="23" rx="8" ry="18" transform="rotate(18 62 23)"/><circle cx="27" cy="47" r="12"/><circle cx="74" cy="47" r="12"/></g><path d="M40 47q4 5 8 0m7 0q4 5 8 0" fill="none" stroke="#674955" stroke-width="2" stroke-linecap="round"/></svg>`;function $v(s,t,e=0){const i=s==="automation"?"Watch the robot follow its cleaning plan. Hold a steering input for the takeover time (1 second by default, set in Settings) to switch to Manual; tap AD back on to reactivate. During a black-cat hazard, a quick sideways swipe is your immediate emergency response.":"Touch anywhere on the floor view, then drag toward your destination. All 360° work, regardless of which way the robot faces. Small drags steer gently.",n=["Hello, little cleaner.","Drag where you want to go.","A little temptation.","Your way to clean."],r=['<img class="welcome-art" src="/catnap-cleaner-site/tutorial/welcome.webp" alt="A sleeping ginger cat riding a robot vacuum, beside pink and gold dust bunnies and a turquoise magnet" width="900" height="600">',`<svg class="control-picture" viewBox="0 0 380 240" role="img" aria-label="Start a touch anywhere and drag toward a bonus; the robot smoothly turns toward that direction"><defs><marker id="drag-tip" markerWidth="7" markerHeight="7" refX="5" refY="3" orient="auto"><path d="M0 0 6 3 0 6" fill="#548cac"/></marker></defs><rect x="10" y="10" width="360" height="220" rx="24" fill="#e8eadb"/><circle cx="112" cy="155" r="43" fill="none" stroke="#adbca9" stroke-width="2" stroke-dasharray="5 6"/><circle cx="112" cy="155" r="12" fill="#fffdf5" stroke="#799485" stroke-width="2"/><path d="M130 144 267 63" stroke="#548cac" stroke-width="7" stroke-linecap="round" marker-end="url(#drag-tip)"/><circle cx="209" cy="98" r="20" fill="#8cb7cc" stroke="#fffdf5" stroke-width="4"/><g transform="translate(271 22) scale(.8)">${Na("#e285b0")}</g><text x="62" y="217" fill="#526c59" font-size="16">Touch anywhere</text><text x="139" y="46" fill="#426d87" font-size="16">Drag toward your goal</text></svg>`,`<div class="pickup-pictures"><div>${Na("#e285b0")}<strong>Pink</strong><span>+100 / +250 / +500</span><small>6 seconds</small></div><div>${Na("#d6aa3e")}<strong>Gold</strong><span>+1,500</span><small>18 seconds · farther away</small></div><div><svg viewBox="0 0 100 80" role="img" aria-label="Turquoise magnet"><path d="M27 18v25a23 23 0 0 0 46 0V18" fill="none" stroke="#56a89c" stroke-width="17"/><path d="M27 18v10m46-10v10" stroke="#fffdf5" stroke-width="17"/></svg><strong>Magnet</strong><span>Collect to activate</span><small>Attracts dust for 10 seconds</small></div></div>`,'<div class="sharing-picture" role="img" aria-label="Blue player direction plus amber system direction produces the white shared direction"><div><i class="share-player"></i><span>PLAYER</span><b>↗</b></div><span class="share-plus">+</span><div><i class="share-system"></i><span>SYSTEM</span><b>→</b></div><span class="share-plus">=</span><div><i class="share-result"></i><span>SHARED</span><b>↗</b></div></div>'],a=['<p>Your robot has a sleepy passenger. Clean the floor while keeping the cat comfortable.</p><div class="welcome-callout"><strong>100% cleaned</strong><span>Brush every reachable dusty patch.</span></div><p class="welcome-detail">Smooth turns keep the nap peaceful. Bonks and rough corrections raise disturbance. At 100%, the cat stops the run.</p>',`<p>${i}</p><div class="welcome-callout"><strong>Direction, not speed.</strong><span>The robot keeps a steady cruise.</span></div><p class="welcome-detail">${s==="automation"?"Emergency swipes work only while a critical hazard is active.":"Keyboard: W A S D or arrow keys. Release to coast in Manual or let the system assist in Collaborative."}</p>`,'<p>Drive over a bunny before its halo disappears. Gold rewards a longer detour; the rare turquoise magnet pulls nearby dust toward you.</p><div class="welcome-callout"><strong>New floor patch: +5</strong><span>Bonk: −100 + disturbance</span></div><p class="welcome-detail">A quiet, smooth finish earns extra points.</p>',s==="collaborative"?`<p>Your direction acts immediately alongside the robot’s plan.</p><div class="welcome-functions"><span><b>Interaction</b> · steer anytime, no switches</span><span><b>Arbitration</b> · steering weight, AD leans in near obstacles</span><span><b>Inclusion</b> · your goal becomes the plan</span></div><p class="welcome-detail">Hold a direction for about 0.8 seconds. For gold, aim for ${t.toFixed(1)} seconds until LOCKED, then release. The robot routes to gold and back toward remaining dust.</p><p class="welcome-detail">Intent +25 · yield +25 · sync +100<br>Smooth capture +150 · smooth return +100</p>`:s==="manual"?'<p>You decide and execute every turn. The robot supplies no steering assistance.</p><div class="welcome-callout"><strong>Follow your own heading.</strong><span>Chase bonuses or sweep the dusty floor.</span></div><p class="welcome-detail">Gentle corrections keep your passenger happy. Watch for the black cat if critical events are enabled.</p>':'<p>The robot cleans systematically and seeks remaining dust. It collects bunnies and magnets along its path, without detouring to chase them.</p><div class="welcome-callout"><strong>You’re the lookout.</strong><span>AD drives until you hold a takeover.</span></div><p class="welcome-detail">If black-cat events are enabled, be ready for an emergency swipe when a hazard becomes active.</p>'],o=Math.max(0,Math.min(3,e));return`<div class="welcome-slide" data-slide="${o}"><div class="welcome-visual">${r[o]}</div><article class="welcome-copy"><span class="mini-label">${["THE GOAL","YOUR CONTROLS","POINTS & PICKUPS","READY TO CLEAN"][o]}</span><h2>${n[o]}</h2>${a[o]}${o===3?'<p class="welcome-ready">Nothing moves until you press Start cleaning.</p>':""}</article></div>`}const Ei=s=>({manual:"Manual",automation:"Full automation",collaborative:"Collaborative"})[s],Yv='<svg class="pairdriver-mark" viewBox="0 0 24 24" fill="none" stroke-width="3.6" stroke-linecap="butt" aria-hidden="true"><path d="M4.4 22.5 6.6 12A6 6 0 0 1 17.2 8.1" stroke="#2fae9f"/><path d="M8 15.9A6 6 0 0 0 18.6 12L21.4 2.2" stroke="#a5a7a9"/></svg>',He=s=>{const t={play:'<path d="m9 5 11 7-11 7z"/>',pause:'<path d="M9 5v14M16 5v14"/>',back:'<path d="m14 5-7 7 7 7M7 12h14"/>',arrow:'<path d="M4 12h16m-6-6 6 6-6 6"/>',sound:'<path d="M4 9h4l5-4v14l-5-4H4zM17 8q6 4 0 8"/>',settings:'<path d="M4 7h16M4 17h16"/><circle cx="9" cy="7" r="3"/><circle cx="16" cy="17" r="3"/>',manual:'<circle cx="12" cy="8.5" r="4.5"/><path d="M4 21v-1a6 5 0 0 1 6-5h4a6 5 0 0 1 6 5v1"/><circle cx="10.2" cy="8.5" r=".5"/><circle cx="13.8" cy="8.5" r=".5"/>',automation:'<path d="M4 16V8h16v8zM9 4h6M12 4v4M8 16v3m8-3v3"/><circle cx="9" cy="12" r=".5"/><circle cx="15" cy="12" r=".5"/>',collaborative:'<path d="m3 13 5-5 5 5-5 5zM11 6l5-3 5 5-5 5M10 10l4 4"/>',moon:'<path d="M20 15A9 9 0 0 1 9 4a9 9 0 1 0 11 11z"/>',check:'<path d="m5 12 4 4L20 5"/>',repeat:'<path d="M20 7h-5m5 0V2M4 17h5m-5 0v5M20 7a9 9 0 0 0-16 3m0 7a9 9 0 0 0 16-3"/>'};return s==="collaborative"?Yv:`<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${t[s]??t.arrow}</svg>`},Bt=(s,t,e="primary",i="")=>`<button class="button ${e}" data-action="${t}">${i?He(i):""}<span>${s}</span>${e==="primary"?He("arrow"):""}</button>`,Vc=s=>`${Math.floor(s/60)}:${String(Math.floor(s%60)).padStart(2,"0")}`;class qv{constructor(t,e){y(this,"session");y(this,"storage");y(this,"lang","en");y(this,"tour",null);y(this,"selectedLevel","level1");y(this,"welcomeSlide",0);y(this,"pairdriverSlide",0);y(this,"root");y(this,"screen","MAIN_MENU");y(this,"selected","collaborative");y(this,"action",()=>{});y(this,"setting",()=>{});y(this,"hud",{});this.session=t,this.storage=e,this.root=document.querySelector("#app"),Wv(this.root,i=>this.action(this.screen==="PAIRDRIVER_INFO"?`pairdriver-${i}`:this.screen==="RUN_INTRO"?`welcome-${i}`:`tour-${i}`)),this.root.addEventListener("click",i=>{const n=i.target.closest("[data-action]");n&&this.action(n.dataset.action)}),this.root.addEventListener("change",i=>{const n=i.target;n.dataset.setting&&n.type!=="range"&&this.setting(n.dataset.setting,n.type==="checkbox"?n.checked:Number(n.value))}),this.root.addEventListener("change",i=>{const n=i.target;n.hasAttribute?.("data-tour-weight")&&n.blur()}),this.root.addEventListener("input",i=>{const n=i.target;if(n.hasAttribute("data-tour-weight")){this.action(`tour-weight-${n.value}`);return}if(!(n.type!=="range"||!n.dataset.setting)){if(this.setting(n.dataset.setting,Number(n.value)/(n.dataset.setting==="interventionShare"?100:1)),n.dataset.setting==="interventionShare"&&this.updateInterventionSetting(),n.dataset.setting==="inclusionDelay"){const r=this.root.querySelector("#inclusion-value");r&&(r.textContent=`${Number(n.value).toFixed(1)} seconds`)}if(n.dataset.setting==="takeoverHold"){const r=this.root.querySelector("#takeover-value");r&&(r.textContent=`${Number(n.value).toFixed(1)} seconds`)}}})}show(t,e){const i=this.screen;this.screen=t,this.hud={},document.body.classList.toggle("in-game",["PLAYING","PAUSED","QUIT_CONFIRM","MODE_SWITCH","GAME_OVER"].includes(t)),this.root.className=t==="PLAYING"?"playing":t==="MAIN_MENU"?"home":["LEVEL_SELECT","MODE_SELECT","PAIRDRIVER_INFO","RESULTS"].includes(t)?"paged-menu":"",this.root.classList.toggle("results-view",t==="RESULTS"||t==="GAME_OVER");const n=Bt("Back","back","text","back");if(t==="MAIN_MENU")this.root.innerHTML=`<main class="main-menu"><div class="brand-mark">${He("moon")} A QUIET LITTLE GAME</div><h1>CATNAP<br><span>CLEANER</span><i>✳</i></h1><p class="tagline">A clean floor. A sleeping cat.<br>How much control will you share?</p><div class="menu-actions">${Bt("Play","play","primary","play")}${Bt("Comparison run","comparison","secondary","collaborative")}</div><div class="menu-links">${Bt("How to play","help","text")}${Bt("Pairdriver & this game","pairdriver","text")}${Bt("Settings","settings","text","settings")}</div><p class="menu-note">${this.storage.best>0?`BEST CLEAN · ${Math.round(this.storage.best).toLocaleString()} POINTS`:"Take your time. Mind the furniture."}</p></main><div class="lang-switch" role="group" aria-label="Language / 言語">${[["en","EN"],["ja","日本語"]].map(([r,a])=>`<button class="${this.lang===r?"active":""}" data-action="lang-${r}" aria-pressed="${this.lang===r}" lang="${r}">${a}</button>`).join("")}</div><div class="nap-tag">${He("moon")}<span>NAP IN PROGRESS<small>Please clean quietly.</small></span></div><div class="home-footer"><span>THREE WAYS TO CLEAN. ONE SLEEPY CAT.</span><span>Made for a little downtime.</span></div>`;else if(t==="LEVEL_SELECT")this.root.innerHTML=`<section class="screen-panel level-screen"><header>${n}<span class="eyebrow">CHOOSE A LEVEL</span><span></span></header><div class="screen-title"><h2>A little room to learn.</h2><p>Take the tour, revisit home, or enjoy open floor.</p></div><div class="level-grid">${Jc.map((r,a)=>`<button class="level-card ${this.selectedLevel===r.id?"selected":""}" data-action="level-${r.id}"><span class="level-picture level-picture-${r.id}" aria-hidden="true"><i></i><b>✳</b>${r.id==="level1"?"<em></em>":""}</span><span class="mini-label">${r.subtitle}</span><h3>${r.label}</h3><p>${r.description}</p></button>`).join("")}</div><footer><p>${this.selectedLevel==="tour"?"Welcome and how-to-play are always included.":"Choose your mode next. Start immediately or open the Guide."}</p>${Bt(this.selectedLevel==="tour"?"Take the tour":"Choose mode","level-continue","primary","play")}</footer></section>`;else if(t==="RUN_INTRO")this.root.innerHTML=`<section class="screen-panel welcome-screen side-welcome" data-page-swipe><header>${Bt(this.session.flow==="comparison"&&this.session.index>0?"Quit comparison":"Back","intro-back","text","back")}<span class="eyebrow">${this.session.flow==="comparison"?`COMPARISON ${this.session.index+1} / 3`:"BEFORE YOU BEGIN"} · ${Ei(this.session.mode)}</span>${Bt("About Pairdriver","pairdriver","text")}</header>${$v(this.session.mode,this.storage.settings.inclusionDelay,this.welcomeSlide)}<footer class="welcome-navigation"><button class="button secondary" data-action="welcome-prev" ${this.welcomeSlide===0?"disabled":""}>${He("back")}<span>Previous</span></button><div class="welcome-progress" aria-label="Welcome slide ${this.welcomeSlide+1} of ${wr}"><span>${this.welcomeSlide+1} / ${wr}</span><div>${Array.from({length:wr},(r,a)=>`<i ${a===this.welcomeSlide?'aria-current="step"':""}></i>`).join("")}</div></div>${this.welcomeSlide===wr-1?Bt("Start cleaning","begin-run","primary","play"):Bt("Next","welcome-next","primary")}</footer></section>`;else if(t==="PAIRDRIVER_INFO")this.root.innerHTML=`<section class="screen-panel information-screen pairdriver-screen" data-page-swipe><header>${Bt("Back","info-back","text","back")}<span class="eyebrow">PAIRDRIVER & CATNAP CLEANER</span><span></span></header>${Xv(this.pairdriverSlide)}<footer class="welcome-navigation"><button class="button secondary" data-action="pairdriver-prev" ${this.pairdriverSlide===0?"disabled":""}>${He("back")}<span>Previous</span></button><div class="welcome-progress" aria-label="Information page ${this.pairdriverSlide+1} of ${Yn}"><span>${this.pairdriverSlide+1} / ${Yn}</span><div>${Array.from({length:Yn},(r,a)=>`<i ${a===this.pairdriverSlide?'aria-current="step"':""}></i>`).join("")}</div></div>${this.pairdriverSlide===Yn-1?Bt("Back to game","info-back","primary"):Bt("Next","pairdriver-next","primary")}</footer></section>`;else if(t==="MODE_SELECT")this.root.innerHTML=`<section class="screen-panel mode-screen"><header>${n}<span class="eyebrow">${Fa(this.session.level).label}</span><span></span></header><div class="screen-title"><h2>Who’s behind the brush?</h2><p>Choose one mode.</p></div><div class="mode-grid">${Lr.map((r,a)=>this.modeCard(r,a)).join("")}</div><div class="screen-bottom"><p>Full Automation: hold to take over, then tap AD back on.</p><div class="start-options">${Bt("Guide","guide","secondary")}${Bt("Start","start","primary","play")}</div></div></section>`;else if(t==="COMPARISON_INTRO")this.root.innerHTML=`<section class="screen-panel comparison-intro"><header>${n}<span class="eyebrow">COMPARISON RUN</span><span></span></header><div class="screen-title"><span class="mini-label">ONE HOUSE · ONE SCENARIO</span><h2>Same house.<br>Three ways to clean.</h2><p>Experience each mode in order. See what changes.</p></div><div class="intro-modes">${Lr.map((r,a)=>`<div><span class="step-number">0${a+1}</span>${He(r)}<h3>${Ei(r)}</h3><p>${["You control everything.","The robot follows its plan.","You can change the plan together."][a]}</p></div>`).join("")}</div><div class="screen-bottom"><p>Same floor, furniture and timed dust bunnies.<br>Only completed runs enter the comparison.</p>${Bt("Start comparison","start-comparison","primary","play")}</div></section>`;else if(t==="HOW_TO_PLAY")this.root.innerHTML=`<section class="screen-panel help-screen"><header>${n}<span class="eyebrow">HOW TO PLAY</span><span></span></header><div class="screen-title"><h2>Let sleeping cats lie.</h2><p>Clean every dusty patch. Keep the nap intact.</p></div><div class="help-grid"><article><span class="help-number">01</span><h3>Steer softly.</h3><p>When critical events are enabled, watch the black cat. At <strong>BLACK CAT · QUICK SWIPE</strong>, make a fast sideways swipe (or mouse drag) to evade. You can jerk an already-held joystick. Full Automation also supports a deliberate takeover hold to switch to Manual. Tap AD back on to reactivate automation.</p><p>Your cat rides along at a steady cruise. Touch anywhere on the floor view, then drag toward where you want to go. All 360° work: right means screen-right, even when the robot faces down. Your touch starts a fresh joystick center. Use <kbd>W A S D</kbd> or the four arrow keys on a keyboard.</p><p>Steering changes direction, never speed. A small drag gives a gentler influence. Release to coast in Manual or let the system assist in Collaborative.</p></article><article><span class="help-number">02</span><h3>Follow the temptation.</h3><p>Red dust bunnies appear at random times a short, reachable drive away from the robot in every mode. They are worth 100, 250 or 500 points. Gold bunnies are farther away, worth <strong>1,500 points</strong>, and last <strong>18 seconds</strong>. Their halo shrinks before they vanish. Look for a turquoise <strong>magnet pickup</strong>. It appears randomly nearby, less often than red bunnies. Drive over it to attract nearby dust and bunnies for 10 seconds; pause freezes the timer. Every mode collects bunnies and magnets on contact, and magnets attract nearby bunnies in Full Automation too. Collisions and rough corrections disturb the cat. A few smooth seconds help it settle.</p></article><article><span class="help-number">03</span><h3>Make your intent felt.</h3><p>In Collaborative, steer toward a bunny and hold the same direction for about <strong>0.8 seconds</strong>. Your input turns the robot immediately while the system resists. Sustained intent makes it yield. Keep aiming toward the bunny as it moves across your screen. At <strong>YIELDING</strong>, the system turns toward your chosen target. <strong>SYNC</strong> confirms agreement. Gold uses your chosen inclusion delay, as short as 0.1 seconds. Keep aiming until <strong>INCLUSION · LOCKED</strong>. You can then release: the robot routes around furniture, collects it, and returns to the reachable lane with dust left to clean. Small corrections do not cancel this route. A firm drag more than 65° away overrides it after your chosen inclusion delay, up to 0.8 seconds. You can also tap Release route at any time. Change the lock timing in Settings.</p><div class="legend"><span class="blue-dot"></span> Player <span class="amber-dot"></span> System <span class="white-dot"></span> Shared</div></article></div><div class="help-sequence">RESIST <span>→</span> PERSIST <span>→</span> YIELD <span>→</span> SYNC <span>→</span> ASSIST</div><div class="screen-bottom"><p>Paths: muted original · blue player · amber accepted · green active. No learning or personalization.</p>${Bt("Let’s play","play","primary","play")}</div></section>`;else if(t==="SETTINGS"){const r=(a,o,c)=>`<label class="setting-row"><span><strong>${o}</strong><small>${c}</small></span><input type="checkbox" data-setting="${a}" ${this.storage.settings[a]?"checked":""}><span class="switch"></span></label>`;this.root.innerHTML=`<section class="screen-panel settings-screen"><header>${n}<span class="eyebrow">MAKE YOURSELF COMFORTABLE</span><span></span></header><div class="screen-title"><h2>Little adjustments.</h2></div><div class="settings-info-link">${Bt("Pairdriver: what these settings mean","pairdriver","secondary")}</div><div class="settings-list"><section class="settings-group"><h3>Steering &amp; control</h3><p class="settings-group-note">How you and the robot share the wheel.</p><div class="setting-row intervention-setting"><span><label for="intervention-share"><strong>Player share during intervention <em class="setting-tag">Collaborative</em></strong></label><small id="intervention-help">Your share of the steering at full drag. Smaller drags count for less.</small></span><div class="range-control"><output id="intervention-value" for="intervention-share" aria-label="Intervention steering split">${Math.round(this.storage.settings.interventionShare*100)}% player · ${Math.round((1-this.storage.settings.interventionShare)*100)}% system</output><input id="intervention-share" aria-describedby="intervention-help yield-share" aria-label="Player share during intervention" aria-valuetext="${Math.round(this.storage.settings.interventionShare*100)} percent player" data-setting="interventionShare" type="range" min="10" max="100" step="5" value="${Math.round(this.storage.settings.interventionShare*100)}"><div class="range-limits"><span>10% · more assistance</span><span>100% · full control</span></div><small id="yield-share">After intent: ${Math.round(Tr(this.storage.settings.interventionShare)*100)}% player · ${Math.round((1-Tr(this.storage.settings.interventionShare))*100)}% system</small></div></div><div class="setting-row intervention-setting"><span><label for="inclusion-delay"><strong>Intervention → inclusion time <em class="setting-tag">Collaborative</em></strong></label><small id="inclusion-help">Hold your aim this long to lock a goal into the robot's plan. Also how quickly you can override it (up to 0.8 s).</small></span><div class="range-control"><output id="inclusion-value" for="inclusion-delay">${this.storage.settings.inclusionDelay.toFixed(1)} seconds</output><input id="inclusion-delay" aria-describedby="inclusion-help" data-setting="inclusionDelay" type="range" min="0.1" max="3" step="0.1" value="${this.storage.settings.inclusionDelay}"><div class="range-limits"><span>0.1s · quicker lock</span><span>3.0s · longer intervention</span></div></div></div><div class="setting-row intervention-setting"><span><label for="takeover-hold"><strong>AD → manual takeover time <em class="setting-tag">Full AD</em></strong></label><small id="takeover-help">Steer firmly this long to switch AD off and drive manually.</small></span><div class="range-control"><output id="takeover-value" for="takeover-hold">${this.storage.settings.takeoverHold.toFixed(1)} seconds</output><input id="takeover-hold" aria-describedby="takeover-help" data-setting="takeoverHold" type="range" min="0.5" max="5" step="0.1" value="${this.storage.settings.takeoverHold}"><div class="range-limits"><span>0.5s · quick takeover</span><span>5.0s · harder to disengage</span></div></div></div></section><section class="settings-group"><h3>Black-cat hazards</h3><p class="settings-group-note">Surprise crossings that test quick reactions.</p><label class="setting-row"><span><strong>Event frequency</strong><small>How often a black cat dashes across. Off keeps it passive.</small></span><select aria-label="Critical event frequency" data-setting="criticalFrequency">${["Off","Low · 28–42s","Medium · 20–30s","High · 14–22s"].map((a,o)=>`<option value="${o}" ${this.storage.settings.criticalFrequency===o?"selected":""}>${a}</option>`).join("")}</select></label><label class="setting-row"><span><strong>Full Auto avoidance probability <em class="setting-tag">Full AD</em></strong><small>Chance that AD dodges a black cat on its own. 0.10 = 10%.</small></span><input aria-label="Autonomous critical avoidance probability" data-setting="fullAutoCriticalAvoidSuccessProbability" type="number" min="0" max="1" step=".05" value="${this.storage.settings.fullAutoCriticalAvoidSuccessProbability}"></label></section><section class="settings-group"><h3>Feedback</h3><p class="settings-group-note">What you feel when the robot pushes back.</p>${r("dynamicForceFeedbackEnabled","Dynamic force feedback","Collaborative: obstacle risk and robot authority scale vibration, with a soft warning sound.")}${r("haptics","Haptics","Device vibration, where supported.")}<div class="setting-row"><span><strong>Test feedback</strong><small id="feedback-test-result" role="status">Obstacle feedback uses vibration where available, a soft pulse sound, and the direction indicator.</small></span><button class="button secondary" data-action="test-feedback">Test</button></div></section><section class="settings-group"><h3>Sound &amp; display</h3>${r("sound","Sound","Taps, bonks, a sleepy purr and an opinionated cat.")}<label class="setting-row"><span><strong>Volume</strong></span><input aria-label="Volume" data-setting="volume" type="range" min="0" max="1" step=".05" value="${this.storage.settings.volume}"></label>${r("quality","Soft shadows","Turn off for a lighter experience on older phones.")}</section></div><p class="settings-footnote">Settings and up to six recent trial exports stay on this device. Random black-cat events are independent of the shared cleaning scenario.<br>No learning or personalization.</p><button class="export-link" data-action="export">Download experiment telemetry</button></section>`}else t==="PLAYING"?this.root.innerHTML=`<div id="joystick-region" aria-label="Touch anywhere to steer; quick sideways swipe during a black-cat hazard to evade"><span class="joystick-origin"><i class="joystick-thumb"></i></span></div><div class="game-hud"><div class="hud-top"><div class="score-block"><small>SCORE</small><strong id="hud-score">0</strong></div><div class="clean-block"><small>CLEANED</small><strong id="hud-clean">0<span>%</span></strong><div class="clean-track"><i id="hud-clean-bar"></i></div></div><div class="hud-right"><div class="level-pill"><small>${this.session.flow==="comparison"?`COMPARISON ${this.session.index+1} / 3`:Fa(this.session.level).label}</small><button class="pause-button" data-action="pause" aria-label="Pause">${He("pause")}</button></div></div></div><div class="run-tools"><button id="hud-ad-back" class="run-tool" data-action="ad-back" hidden>AD back on <kbd class="key-hint">E</kbd></button><span id="hud-takeover" class="takeover-status" hidden></span><div id="hud-magnet" class="magnet-status" role="status" hidden><span id="hud-magnet-label"></span></div><button id="hud-release-route" class="run-tool" data-action="release-route" hidden>Release route <kbd class="key-hint">E</kbd></button></div><div id="bonus-labels"></div><div class="quiet-hud"><div class="nap-status" aria-label="Cat disturbance"${e.level==="tour"?' style="display:none"':""}><span id="hud-cat-anger" class="anger-badge" data-level="0" aria-hidden="true"><svg class="anger-rays" viewBox="0 0 40 40"><path class="ray r1" d="M20 3.5 V7.5" transform="rotate(-60 20 20)"/><path class="ray r2" d="M20 3.5 V7.5" transform="rotate(-20 20 20)"/><path class="ray r3" d="M20 3.5 V7.5" transform="rotate(25 20 20)"/><path class="ray r1" d="M20 3.5 V7.5" transform="rotate(70 20 20)"/><path class="ray r2" d="M20 3.5 V7.5" transform="rotate(115 20 20)"/><path class="ray r3" d="M20 3.5 V7.5" transform="rotate(160 20 20)"/><path class="ray r1" d="M20 3.5 V7.5" transform="rotate(205 20 20)"/><path class="ray r2" d="M20 3.5 V7.5" transform="rotate(250 20 20)"/></svg><i class="m1">💢</i><i class="m2">💢</i><i class="m3">💢</i></span>${He("moon")}<div><div class="nap-heading"><span>CAT DISTURBANCE</span><strong id="hud-cat-value">0%</strong></div><div class="cat-track"><i id="hud-cat-bar"></i></div><span id="hud-cat-state">DEEP SLEEP</span></div><strong class="nap-alert" role="alert">THE SUPERVISOR IS UP.</strong></div><div class="steering-status" aria-label="Shared direction"><svg class="direction-dial" viewBox="0 0 40 40" aria-hidden="true"><circle cx="20" cy="20" r="18" fill="#929783" fill-opacity=".45"/><path id="hud-direction-player" d="M20 24V8m-3 4 3-4 3 4" fill="none" stroke="#4187b1" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/><path id="hud-direction-system" d="M20 24V5m-3 4 3-4 3 4" fill="none" stroke="#c78c37" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"/><path id="hud-direction-shared" d="M20 25V11m-3 4 3-4 3 4" fill="none" stroke="#fffef7" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round"/></svg><div class="control-copy"><strong id="hud-mode">${Ei(e.mode)}</strong><span id="hud-control-state">SHARED DIRECTION</span></div></div></div><pre id="debug-panel" hidden></pre></div>`:t==="PAUSED"?this.root.innerHTML=this.modal(`<span class="mini-label">${Ei(e.mode)} · ${Vc(e.time)}</span><h2>A little breather.</h2><p>The floor can wait.</p><div class="modal-actions">${Bt("Resume","resume","primary","play")}${Bt("Restart run","restart","secondary","repeat")}${this.session.flow==="free_play"?Bt("Change mode","mode-switch-request","secondary","collaborative"):""}${Bt("Settings","pause-settings","text","settings")}${Bt("Quit run","quit-request","text")}${Bt("Quit to main menu","quit-main-request","text")}${Bt("Export telemetry","export","text")}</div>`):t==="QUIT_CONFIRM"?this.root.innerHTML=this.modal(`<h2>Quit this run?</h2><p>This unfinished attempt won’t be kept.</p><div class="modal-actions">${Bt("Quit","quit","primary")}${Bt("Cancel","cancel-quit","secondary")}</div>`):t==="MODE_SWITCH"?this.root.innerHTML=this.modal(`<span class="mini-label">CURRENTLY ${Ei(e.mode).toUpperCase()}</span><h2>Switch mode?</h2><p>The stage restarts from the beginning in the new mode. This attempt won’t be kept.</p><div class="modal-actions">${["manual","automation","collaborative"].filter(r=>r!==e.mode).map(r=>Bt(`Restart as ${Ei(r)}`,`switch-mode-${r}`,"secondary",r)).join("")}${Bt("Cancel","cancel-mode-switch","text")}</div>`):t==="RESULTS"||t==="GAME_OVER"?this.root.innerHTML=this.results(e,t==="GAME_OVER"):t==="COMPARISON_RESULTS"&&(this.root.innerHTML=`<section class="screen-panel final-screen"><header><span class="eyebrow">THE SAME HOUSE. YOUR THREE RUNS.</span><span></span></header><div class="screen-title"><h2>Different ways. Your results.</h2><p>Every number comes from your completed runs. Pickups appear relative to your position, so routes affect the opportunities you encounter.</p></div>${Hv(this.session.results)}<div class="screen-bottom"><button class="export-link" data-action="export">Download experiment telemetry</button><p>Involvement = time providing steering input.<br>Smoothness reflects actual turning and collisions.</p><div class="row-actions">${Bt("Compare again","comparison","secondary")}${Bt("Main menu","main","primary")}</div></div></section>`);i!==t&&!matchMedia("(prefers-reduced-motion: reduce)").matches&&this.root.animate([{opacity:.25,transform:"translateY(7px)"},{opacity:1,transform:"translateY(0)"}],{duration:240,easing:"cubic-bezier(.2,.7,.2,1)"}),t==="PLAYING"&&(this.root.querySelectorAll("[id]").forEach(r=>this.hud[r.id]=r),this.renderTour(e))}updateInterventionSetting(){const t=Math.round(this.storage.settings.interventionShare*100),e=Math.round(Tr(this.storage.settings.interventionShare)*100),i=this.root.querySelector("#intervention-value"),n=this.root.querySelector("#yield-share");i&&(i.textContent=`${t}% player · ${100-t}% system`),n&&(n.textContent=`After intent: ${e}% player · ${100-e}% system`),this.root.querySelector("#intervention-share")?.setAttribute("aria-valuetext",`${t} percent player, ${100-t} percent system`)}selectMode(t){this.selected=t,this.root.querySelectorAll(".mode-card").forEach(e=>{const i=e.dataset.action===`select-${t}`;e.classList.toggle("selected",i),e.setAttribute("aria-pressed",String(i)),e.querySelector(".selection-circle").innerHTML=i?He("check"):""})}cameraView(t,e){const i=Array.from(this.root.querySelectorAll(".tour-panel,.side-welcome")).map(r=>r.getBoundingClientRect());if(!i.length)return null;const n=Array.from(this.root.querySelectorAll(".hud-top,.nap-status,.steering-status")).map(r=>r.getBoundingClientRect());return wv(t,e,i,Math.max(0,...n.map(r=>r.bottom)))}renderTour(t){this.root.querySelector(".tour-panel")?.remove(),this.tour&&this.screen==="PLAYING"&&this.root.insertAdjacentHTML("beforeend",Gv(this.tour,t))}modal(t){return`<div class="modal-backdrop"><section class="modal" role="dialog" aria-modal="true">${t}</section></div>`}modeCard(t,e){const i=["Steer every turn and choose your own route.","Watch a systematic clean; hold to take over.","Steer together, request a detour, then let the robot help."];return`<button class="mode-card ${t} ${this.selected===t?"selected":""}" data-action="select-${t}" aria-pressed="${this.selected===t}"><div class="mode-icon">${He(t)}</div><div class="mode-card-copy"><h3>${Ei(t)}</h3><strong>${["You control everything.","The robot cleans by itself.","You guide. The system assists."][e]}</strong><p>${i[e]}</p></div><span class="selection-circle">${this.selected===t?He("check"):""}</span></button>`}metricList(t,e=!1,i="manual"){const n=[["Total score",Math.round(t.score).toLocaleString()],["Cleaning time",Vc(t.cleaningTime)],["Bonuses collected",String(t.bonusesCollected)],["Peak cat disturbance",`${Math.round(t.peakCatDisturbance)}%`],["Collisions",String(t.collisions)],["Smoothness",`${Math.round(t.smoothness)} / 100`]];return e&&n.push(["Player involvement",`${Math.round(t.playerInvolvement)}%`]),i==="collaborative"&&n.push(["Intents detected",String(t.intentsDetected)],["Successful yields",String(t.successfulYields)],["Sync events",String(t.syncEvents)],["Smooth returns",String(t.smoothReturns)]),`<dl class="metrics">${n.map(([r,a])=>`<div><dt>${r}</dt><dd>${a}</dd></div>`).join("")}</dl>`}results(t,e){const i=t.metrics,n=this.session.flow==="comparison";return`<section class="screen-panel results-screen"><div class="result-heading"><span class="result-seal">${He(e?"moon":"check")}</span><span class="eyebrow">${Ei(t.mode)}${n?` · COMPARISON ${this.session.index+1} / 3`:""}</span><h2>${e?"Cat woke up.":"Cleaning complete."}</h2><p>${e?"Your supervisor has filed a very paw-sonal complaint.":"Spotless floor. Undisturbed dreams."}</p></div><div class="result-summary"><div><span>SCORE</span><strong>${Math.round(i.score).toLocaleString()}</strong></div><div><span>CLEANED</span><strong>${i.cleaningProgress.toFixed(0)}<small>%</small></strong></div></div>${this.metricList(i,!1,t.mode)}<div class="result-actions">${n?`${e?"":Bt(this.session.index===2?"See comparison":"Continue","continue","primary","arrow")}${Bt("Retry this mode","restart",e?"primary":"secondary","repeat")}${Bt("Quit comparison","main","text")}`:`${Bt("Retry","restart","primary","repeat")}${Bt("Change mode","change-mode","secondary")}${Bt("Main menu","main","text")}`}</div><button class="export-link" data-action="export">Download run telemetry</button></section>`}updateDirections(t,e){if(this.screen!=="PLAYING")return;const i=t.project(e.position,0);for(const[n,r,a]of[["player",e.steering.playerDirection,e.mode!=="automation"&&e.steering.inputMagnitude>.015],["system",e.steering.systemDirection,e.mode!=="manual"],["shared",e.steering.sharedDirection,!0]]){const o=this.hud[`hud-direction-${n}`];if(!o)continue;const c=t.project({x:e.position.x+r.x,z:e.position.z+r.z},0),l=Math.atan2(c.y-i.y,c.x-i.x)*180/Math.PI+90;o.setAttribute("transform",`rotate(${l} 20 20)`),o.style.opacity=a?"1":"0"}}update(t,e){if(this.screen!=="PLAYING")return;const i=(c,l)=>{this.hud[c]&&(this.hud[c].textContent=l)};i("hud-score",Math.round(t.metrics.score).toLocaleString()),i("hud-clean",`${Math.floor(t.metrics.cleaningProgress)}%`),this.hud["hud-clean-bar"].style.width=`${t.metrics.cleaningProgress}%`,i("hud-cat-value",`${Math.round(t.cat.value)}%`),this.hud["hud-cat-bar"].style.width=`${t.cat.value}%`,this.hud["hud-cat-bar"].style.background=t.cat.value>=60?"#be7853":"#8f9f77",i("hud-cat-state",t.cat.label);const n=this.root.querySelector("#hud-cat-anger");if(n){const c=String(jc(t.cat.value));n.dataset.level!==c&&(n.dataset.level=c)}this.root.querySelector(".nap-status")?.classList.toggle("woke",t.status==="failed");const r=this.root.querySelector(".steering-status");if(r){const c=Math.max(t.forceSample?.generatedDynamicForce??0,t.tourInteractionForce);r.style.boxShadow=c?`0 0 0 ${c*6}px rgba(190,137,64,${c*.8})`:"";const l=r.querySelector("#hud-control-state");l&&(l.textContent=t.inclusionLocked||t.state==="inclusion"?t.state==="return_to_path"?"RETURNING · LOCKED":"INCLUSION · LOCKED":t.state==="return_to_path"?"RETURNING TO PATH":["intent_detected","yielding","sync","target_bonus"].includes(t.state)?"INTENT ACCEPTED":t.mode==="collaborative"&&t.steering.inputMagnitude>.12&&t.obstacleArbitration>.15?"OBSTACLE · AD LEANS IN":c>.06?`RESISTANCE ${Math.round(c*100)}%`:t.mode==="manual"?"PLAYER CONTROL":t.mode==="automation"?"SYSTEM CONTROL":"SHARED DIRECTION")}const a=this.hud["hud-magnet"];if(a.hidden=t.magnetRemaining<=0,i("hud-magnet-label",`Magnet · ${t.magnetRemaining.toFixed(1)}s`),this.hud["hud-release-route"].hidden=t.mode!=="collaborative"||!t.hasAcceptedRoute,i("hud-mode",Ei(t.mode)),this.hud["hud-ad-back"].hidden=!t.automationDisengaged,this.hud["hud-takeover"].hidden=t.takeoverTimer<=0,i("hud-takeover",`Takeover · ${Math.min(t.takeoverHold,t.takeoverTimer).toFixed(1)} / ${t.takeoverHold.toFixed(1)}s`),this.tour){const c=this.tour,l=Math.round(t.steering.driverAuthority*100),h=d=>this.root.querySelector("#"+d);h("tour-player-authority")&&(h("tour-player-authority").textContent=l+"%"),h("tour-system-authority")&&(h("tour-system-authority").textContent=100-l+"%"),h("tour-authority-bar")&&(h("tour-authority-bar").style.width=l+"%");const u=this.root.querySelector("[data-action=tour-next]");u&&(u.disabled=!c.ready),h("tour-task")&&(h("tour-task").textContent=c.ready&&c.step.live?"Done · tap Next when you are ready.":c.step.id==="takeover"&&t.automationDisengaged?"Manual active · steer, then tap AD back on.":c.step.task)}const o=this.hud["debug-panel"];if(o.hidden=!e,e){const c=t.steering,l=h=>`${h.x.toFixed(2)}, ${h.z.toFixed(2)}`;o.textContent=[["playerDirection",l(c.playerDirection)],["systemDirection",l(c.systemDirection)],["sharedDirection",l(c.sharedDirection)],["inputMagnitude",c.inputMagnitude.toFixed(2)],["driverIntent",c.driverIntent.toFixed(3)],["systemIntent",c.systemIntent.toFixed(3)],["finalSteering",c.finalSteering.toFixed(3)],["playerAuthority",c.driverAuthority.toFixed(3)],["systemAuthority",c.systemAuthority.toFixed(3)],["interventionShare",`${Math.round(t.interventionShare*100)}% player`],["intentTimer",`${c.intentTimer.toFixed(3)} s`],["catDisturbance",t.cat.value.toFixed(2)],["robotSpeed",t.speed.toFixed(2)],["turnRate",t.turnRate.toFixed(2)],["lateralAccel",t.cat.lateralAcceleration.toFixed(2)],["collaborationState",t.state],["currentTarget",t.currentTarget],["mode",t.mode],["scenarioSeed",String(t.seed)]].map(([h,u])=>h.padEnd(19)+u).join(`
`)}}}const Wc={Manual:"手動","Full automation":"完全自動","Full Automation":"完全自動",Collaborative:"協調","Full AD":"完全AD",Player:"プレイヤー",System:"システム",Shared:"共有",PLAYER:"プレイヤー",SYSTEM:"システム",SHARED:"共有",Interaction:"インタラクション",Arbitration:"アービトレーション",Inclusion:"インクルージョン",Back:"戻る",Next:"次へ",Previous:"前へ",Continue:"続ける",Cancel:"キャンセル",Quit:"終了",Retry:"リトライ",Start:"スタート",Guide:"ガイド",Test:"テスト",Off:"オフ","A QUIET LITTLE GAME":"静かな小さなゲーム","A clean floor. A sleeping cat.":"きれいな床。眠るネコ。","How much control will you share?":"操作を、どれだけ分け合う？",Play:"プレイ","Comparison run":"比較プレイ","How to play":"遊び方","Pairdriver & this game":"Pairdriverとこのゲーム",Settings:"設定","Take your time. Mind the furniture.":"ごゆっくり。家具に気をつけて。","NAP IN PROGRESS":"お昼寝中","Please clean quietly.":"静かにお掃除してね。","THREE WAYS TO CLEAN. ONE SLEEPY CAT.":"3つのお掃除方法。眠いネコが1匹。","Made for a little downtime.":"ひと息つきたいときに。","Level 0 · Pairdriver tour":"レベル0・Pairdriverツアー","LEARN BY DOING":"やってみて覚える","A guided, hands-on introduction to shared steering.":"共有ステアリングを実際に体験できるガイド付き入門。","Level 1 · Open room":"レベル1・広い部屋","ROOM TO EXPLORE":"のびのび探索","A large, mostly empty floor. Two obstacles, no black-cat intervention.":"ほとんど何もない広い床。障害物は2つ、黒猫は出ません。","Level 2 · The original home":"レベル2・いつものおうち","FAMILIAR FLOOR":"見慣れた床","The existing room, furniture, pickups and experiment settings.":"おなじみの部屋、家具、アイテム、実験設定。","CHOOSE A LEVEL":"レベルを選ぶ","A little room to learn.":"まずは小さな部屋から。","Take the tour, revisit home, or enjoy open floor.":"ツアーに参加するか、おうちに戻るか、広い床を楽しもう。","Welcome and how-to-play are always included.":"ようこそ画面と遊び方はいつでも見られます。","Choose your mode next. Start immediately or open the Guide.":"次にモードを選びます。すぐ始めるか、ガイドを開きましょう。","Take the tour":"ツアーに参加","Choose mode":"モードを選ぶ","Who’s behind the brush?":"ブラシを握るのは誰？","Choose one mode.":"モードを1つ選んでください。","Full Automation: hold to take over, then tap AD back on.":"完全自動：長押しで操作を引き継ぎ、「ADを再開」で戻ります。","You control everything.":"すべて自分で操作。","The robot follows its plan.":"ロボットが計画どおりに進みます。","You can change the plan together.":"一緒に計画を変えられます。","Steer every turn and choose your own route.":"すべての曲がり角を自分で操作し、ルートを選びます。","Watch a systematic clean; hold to take over.":"計画的なお掃除を見守り、長押しで引き継ぎます。","Steer together, request a detour, then let the robot help.":"一緒に操作し、寄り道をお願いして、あとはロボットにお任せ。","The robot cleans by itself.":"ロボットが自分でお掃除します。","You guide. The system assists.":"あなたが導き、システムが支えます。","Quit comparison":"比較をやめる","COMPARISON RUN":"比較プレイ","ONE HOUSE · ONE SCENARIO":"同じ家・同じシナリオ","Same house.":"同じ家。","Three ways to clean.":"3つのお掃除方法。","Experience each mode in order. See what changes.":"各モードを順番に体験して、違いを感じてください。","Same floor, furniture and timed dust bunnies.":"床も家具も、ホコリうさぎの出現タイミングも同じです。","Only completed runs enter the comparison.":"最後までクリアしたプレイだけが比較に入ります。","Start comparison":"比較を始める","BEFORE YOU BEGIN":"はじめる前に","About Pairdriver":"Pairdriverについて","Start cleaning":"お掃除スタート","Back to game":"ゲームに戻る","THE GOAL":"目標","YOUR CONTROLS":"操作方法","POINTS & PICKUPS":"得点とアイテム","READY TO CLEAN":"お掃除の準備OK","Your robot has a sleepy passenger. Clean the floor while keeping the cat comfortable.":"ロボットには眠たい乗客がいます。ネコを快適に保ちながら床をお掃除しましょう。","100% cleaned":"100%お掃除","Brush every reachable dusty patch.":"届く範囲のホコリをすべてお掃除。","Smooth turns keep the nap peaceful. Bonks and rough corrections raise disturbance. At 100%, the cat stops the run.":"なめらかに曲がればお昼寝は安泰。ぶつかったり急に修正したりすると眠りが乱れます。100%になるとネコがプレイを止めます。","Emergency swipes work only while a critical hazard is active.":"緊急スワイプは危険イベントの発生中だけ使えます。","Drag where you want to go.":"行きたい方へドラッグ。","Touch anywhere on the floor view, then drag toward your destination. All 360° work, regardless of which way the robot faces. Small drags steer gently.":"床のどこかに触れて、目的地の方へドラッグします。ロボットの向きに関係なく360°どの方向でもOK。小さなドラッグならやさしく曲がります。","Keyboard: W A S D or arrow keys. Release to coast in Manual or let the system assist in Collaborative.":"キーボード：W A S Dまたは矢印キー。離すと、手動ではそのまま直進、協調ではシステムがアシストします。","Touch anywhere":"どこでもタッチ","Drag toward your goal":"目標の方へドラッグ","Direction, not speed.":"変えるのは向き、速さではありません。","The robot keeps a steady cruise.":"ロボットは一定の速さで進みます。","A little temptation.":"ちょっとした誘惑。","Drive over a bunny before its halo disappears. Gold rewards a longer detour; the rare turquoise magnet pulls nearby dust toward you.":"光の輪が消える前にうさぎの上を通りましょう。金のうさぎは遠回りする価値あり。まれに出るターコイズのマグネットは近くのホコリを引き寄せます。",Pink:"ピンク","6 seconds":"6秒",Gold:"金","18 seconds · farther away":"18秒・遠め",Magnet:"マグネット","Collect to activate":"拾うと発動","Attracts dust for 10 seconds":"10秒間ホコリを引き寄せる","New floor patch: +5":"新しい床：+5","Bonk: −100 + disturbance":"ゴツン：−100＋眠りの乱れ","A quiet, smooth finish earns extra points.":"静かでなめらかにクリアするとボーナス。","Your way to clean.":"あなた流のお掃除。","Your direction acts immediately alongside the robot’s plan.":"あなたの操作は、ロボットの計画と並んですぐに反映されます。","· your goal becomes the plan":"・あなたの目標が計画になる","Intent +25 · yield +25 · sync +100":"意図 +25・譲歩 +25・同期 +100","Smooth capture +150 · smooth return +100":"なめらかに回収 +150・なめらかに復帰 +100","You decide and execute every turn. The robot supplies no steering assistance.":"曲がり角はすべて自分で判断して操作。ロボットのアシストはありません。","Follow your own heading.":"自分の進路を進もう。","Chase bonuses or sweep the dusty floor.":"ボーナスを追うか、ホコリの床を掃除しよう。","Gentle corrections keep your passenger happy. Watch for the black cat if critical events are enabled.":"やさしい修正なら乗客もご機嫌。危険イベントが有効なら黒猫に注意。","The robot cleans systematically and seeks remaining dust. It collects bunnies and magnets along its path, without detouring to chase them.":"ロボットは計画的に掃除し、残ったホコリを探します。通り道のうさぎやマグネットは拾いますが、寄り道はしません。","You’re the lookout.":"あなたは見張り役。","AD drives until you hold a takeover.":"長押しで引き継ぐまでADが運転します。","If black-cat events are enabled, be ready for an emergency swipe when a hazard becomes active.":"黒猫イベントが有効なら、危険が発生したときの緊急スワイプに備えましょう。","Watch the robot follow its cleaning plan. Hold a steering input for the takeover time (1 second by default, set in Settings) to switch to Manual; tap AD back on to reactivate. During a black-cat hazard, a quick sideways swipe is your immediate emergency response.":"ロボットがお掃除の計画どおりに進むのを見守ります。引き継ぎ時間（初期設定は1秒、設定で変更可）だけ操作を続けると手動に切り替わり、「ADを再開」で戻ります。黒猫の危険発生中は、すばやい横スワイプが緊急回避です。","Nothing moves until you press Start cleaning.":"「お掃除スタート」を押すまで何も動きません。","Let’s play":"遊ぼう","Let sleeping cats lie.":"眠るネコはそっとしておこう。","Clean every dusty patch. Keep the nap intact.":"ホコリをすべてお掃除。お昼寝はそのままに。","Steer softly.":"やさしく操作。","When critical events are enabled, watch the black cat. At":"危険イベントが有効なときは黒猫に注意。","BLACK CAT · QUICK SWIPE":"黒猫・すばやくスワイプ",", make a fast sideways swipe (or mouse drag) to evade. You can jerk an already-held joystick. Full Automation also supports a deliberate takeover hold to switch to Manual. Tap AD back on to reactivate automation.":"と表示されたら、すばやく横にスワイプ（またはマウスでドラッグ）して回避します。押したままのジョイスティックをぐいっと動かしてもOK。完全自動では、意図的な長押しで手動に切り替えることもできます。「ADを再開」で自動運転に戻ります。","Your cat rides along at a steady cruise. Touch anywhere on the floor view, then drag toward where you want to go. All 360° work: right means screen-right, even when the robot faces down. Your touch starts a fresh joystick center. Use":"ネコを乗せて一定の速さで進みます。床のどこかに触れて、行きたい方へドラッグしましょう。360°どの方向でもOK：ロボットが下を向いていても、右は画面の右です。触れた場所が新しいジョイスティックの中心になります。キーボードでは","or the four arrow keys on a keyboard.":"または4つの矢印キーを使います。","Steering changes direction, never speed. A small drag gives a gentler influence. Release to coast in Manual or let the system assist in Collaborative.":"操作で変わるのは向きだけで、速さは変わりません。小さくドラッグすると影響もやさしくなります。離すと、手動ではそのまま直進、協調ではシステムがアシストします。","Follow the temptation.":"誘惑に乗ってみよう。","Red dust bunnies appear at random times a short, reachable drive away from the robot in every mode. They are worth 100, 250 or 500 points. Gold bunnies are farther away, worth":"赤いホコリうさぎは、どのモードでもロボットから少し進めば届く場所にランダムなタイミングで現れます。得点は100・250・500点。金のうさぎはもっと遠くに現れ、得点は","1,500 points":"1,500点",", and last":"、表示時間は","18 seconds":"18秒",". Their halo shrinks before they vanish. Look for a turquoise":"です。消える前に光の輪が縮んでいきます。ターコイズ色の","magnet pickup":"マグネット",". It appears randomly nearby, less often than red bunnies. Drive over it to attract nearby dust and bunnies for 10 seconds; pause freezes the timer. Every mode collects bunnies and magnets on contact, and magnets attract nearby bunnies in Full Automation too. Collisions and rough corrections disturb the cat. A few smooth seconds help it settle.":"も探しましょう。赤いうさぎより出現は少なめで、近くにランダムに現れます。上を通ると10秒間、近くのホコリとうさぎを引き寄せます（一時停止中はタイマーも止まります）。どのモードでも触れるとうさぎやマグネットを拾い、完全自動でもマグネットは近くのうさぎを引き寄せます。衝突や急な修正はネコの眠りを乱します。数秒なめらかに走ればまた落ち着きます。","Make your intent felt.":"意図を伝えよう。","In Collaborative, steer toward a bunny and hold the same direction for about":"協調モードでは、うさぎの方へ操作して同じ方向を約","0.8 seconds":"0.8秒",". Your input turns the robot immediately while the system resists. Sustained intent makes it yield. Keep aiming toward the bunny as it moves across your screen. At":"保ちます。入力するとすぐにロボットが曲がり、システムは抵抗します。意図を持続させるとシステムが譲ります。画面上で動いていくうさぎを狙い続けましょう。",YIELDING:"譲歩中",", the system turns toward your chosen target.":"になると、システムはあなたが選んだ目標の方へ向きます。",SYNC:"同期","confirms agreement. Gold uses your chosen inclusion delay, as short as 0.1 seconds. Keep aiming until":"で合意が確認されます。金のうさぎには設定したインクルージョン時間（最短0.1秒）が使われます。","INCLUSION · LOCKED":"インクルージョン・ロック",". You can then release: the robot routes around furniture, collects it, and returns to the reachable lane with dust left to clean. Small corrections do not cancel this route. A firm drag more than 65° away overrides it after your chosen inclusion delay, up to 0.8 seconds. You can also tap Release route at any time. Change the lock timing in Settings.":"になるまで狙い続けてください。その後は指を離してOK：ロボットが家具をよけて回収し、まだホコリの残る届く範囲のレーンに戻ります。小さな修正ではこのルートは解除されません。65°以上離れた方向へしっかりドラッグすると、設定したインクルージョン時間（最大0.8秒）の後に上書きされます。「ルートを解除」はいつでも押せます。ロックのタイミングは設定で変更できます。",RESIST:"抵抗",PERSIST:"持続",YIELD:"譲歩",ASSIST:"アシスト","Paths: muted original · blue player · amber accepted · green active. No learning or personalization.":"ルート：くすんだ色＝元の計画・青＝プレイヤー・琥珀色＝採用された計画・緑＝実行中。学習やパーソナライズはありません。","PAIRDRIVER & CATNAP CLEANER":"PAIRDRIVER & CATNAP CLEANER","THE IDEA":"コンセプト","Two contributors. One direction.":"2人の担い手。1つの方向。","Pairdriver® is JTEKT’s approach to shared steering. The driver can influence an automated vehicle while automation stays involved.":"Pairdriver®は、JTEKTによる共有ステアリングの考え方です。自動運転が関与したまま、ドライバーが車両に影響を与えられます。","In CATNAP CLEANER, you supply a direction and the robot supplies a plan. Collaborative mode illustrates three functions working together, not three separate modes.":"CATNAP CLEANERでは、あなたが方向を、ロボットが計画を出します。協調モードは、3つの別々のモードではなく、3つの機能が一緒に働く様子を表しています。","Interaction · Arbitration · Inclusion":"インタラクション・アービトレーション・インクルージョン","01 · INTERACTION":"01・インタラクション","Pairdriver:":"Pairdriver：","In this game:":"このゲームでは：","Try it here:":"ここで試す：","02 · ARBITRATION":"02・アービトレーション","Share control without a switch.":"切り替えなしで操作を分け合う。","Compare Full Automation in this game: holding a steering input for the takeover time switches to Manual, and AD back on reactivates automation. Collaborative needs neither switch.":"このゲームの完全自動と比べてみましょう：引き継ぎ時間だけ操作を続けると手動に切り替わり、「ADを再開」で自動に戻ります。協調ではどちらの切り替えも不要です。","03 · INCLUSION":"03・インクルージョン","Your goal becomes our plan.":"あなたの目標が、2人の計画に。","a sufficiently large, persistent intervention is included in trajectory planning.":"十分に大きく持続的な介入は、軌道計画に取り込まれます。","aim at gold until INCLUSION · LOCKED. The robot accepts the destination and plans around furniture. Release your drag; small corrections preserve the plan.":"インクルージョン・ロックになるまで金のうさぎを狙います。ロボットは目的地を受け入れ、家具をよけて計画を立てます。ドラッグを離してもOK。小さな修正では計画は変わりません。","After collection, it routes toward reachable floor that still needs cleaning. Muted rows show the original route; amber shows the accepted plan.":"回収後は、まだお掃除が必要な届く範囲の床へ向かいます。くすんだ線が元のルート、琥珀色が採用された計画です。","Inclusion changes the destination, beyond steering influence.":"インクルージョンは、操作の影響を超えて目的地そのものを変えます。","THE SETTINGS":"設定について","Tune the shared experience.":"共有の体験を調整する。","Player share · 10–100%":"プレイヤー配分・10〜100%","Inclusion time · 0.1–3 seconds":"インクルージョン時間・0.1〜3秒","Inclusion: how long to aim at gold before its route locks.":"インクルージョン：ルートがロックされるまで金のうさぎを狙う時間。","Dynamic Force Feedback":"動的フォースフィードバック","Obstacle risk and system authority shape warning pulses. More player authority means weaker feedback. OFF preserves the original event feedback.":"障害物の危険度とシステムの権限に応じて警告パルスが変わります。プレイヤーの権限が大きいほどフィードバックは弱くなります。オフでは元のイベントフィードバックのままです。","No learning, history-based personalization, or machine learning.":"学習、履歴に基づくパーソナライズ、機械学習は使っていません。","RESEARCH & LIMITS":"研究と限界","A playful analogy.":"遊び心のある例え。","This independent game is inspired by published research, not JTEKT’s controller or an official validation. Phone vibration, sound and visual cues cannot apply steering-wheel resistance.":"このゲームは公開された研究に着想を得た独立した作品で、JTEKTの制御器や公式な検証ではありません。スマホの振動や音、画面表示ではハンドルの抵抗は再現できません。","The game does not estimate driver arm impedance. Current-input thresholds are deterministic. The black-cat events and Full Automation failure probability are game experiment conditions, not Pairdriver performance claims.":"このゲームはドライバーの腕のインピーダンスを推定しません。入力のしきい値は固定です。黒猫イベントや完全自動の失敗確率はゲーム内の実験条件で、Pairdriverの性能を示すものではありません。","Read the original work for the actual steering system.":"実際のステアリングシステムについては原著をご覧ください。","Two direction contributions produce one shared direction.":"2つの方向の寄与から、1つの共有方向が生まれます。","Example: the accepted amber route leaves the original rows, avoids furniture and reaches a gold bunny.":"例：採用された琥珀色のルートが元の線から外れ、家具をよけて金のうさぎに届きます。","More player authority means weaker obstacle feedback; more system authority means stronger feedback.":"プレイヤーの権限が大きいほど障害物のフィードバックは弱く、システムの権限が大きいほど強くなります。","HEAVIER · 30 / 70":"重め・30 / 70","LIGHTER · 90 / 10":"軽め・90 / 10","SYSTEM GUIDES":"システムが誘導","PLAYER STEERS":"プレイヤーが操作","Muted: original rows":"くすんだ色：元の線","Amber: accepted plan":"琥珀色：採用された計画","MORE PLAYER":"プレイヤー多め","MORE SYSTEM":"システム多め","Gentle feedback":"やさしいフィードバック","Stronger feedback":"強いフィードバック","Same nearby obstacle · different control authority":"同じ近くの障害物・異なる操作権限","MAKE YOURSELF COMFORTABLE":"くつろいでね","Little adjustments.":"ちょっとした調整。","Pairdriver: what these settings mean":"Pairdriver：各設定の意味","Steering & control":"操作と制御","How you and the robot share the wheel.":"あなたとロボットの操作の分け合い方。","Player share during intervention":"介入時のプレイヤー配分","Your share of the steering at full drag. Smaller drags count for less.":"いっぱいにドラッグしたときのあなたの操作の割合。小さなドラッグでは少なくなります。","10% · more assistance":"10%・アシスト多め","100% · full control":"100%・完全に自分で","Intervention → inclusion time":"介入 → インクルージョン時間","Hold your aim this long to lock a goal into the robot's plan. Also how quickly you can override it (up to 0.8 s).":"この時間だけ狙い続けると、目標がロボットの計画にロックされます。上書きできるまでの時間にもなります（最大0.8秒）。","0.1s · quicker lock":"0.1秒・すぐロック","3.0s · longer intervention":"3.0秒・長めの介入","AD → manual takeover time":"AD → 手動の引き継ぎ時間","Steer firmly this long to switch AD off and drive manually.":"この時間しっかり操作するとADがオフになり、手動で運転します。","0.5s · quick takeover":"0.5秒・すぐ引き継ぐ","5.0s · harder to disengage":"5.0秒・解除しにくい","Black-cat hazards":"黒猫の危険","Surprise crossings that test quick reactions.":"とっさの反応が試される、突然の横切り。","Event frequency":"発生頻度","How often a black cat dashes across. Off keeps it passive.":"黒猫が横切る頻度。オフにすると何もしません。","Low · 28–42s":"低・28〜42秒","Medium · 20–30s":"中・20〜30秒","High · 14–22s":"高・14〜22秒","Full Auto avoidance probability":"完全自動の回避確率","Chance that AD dodges a black cat on its own. 0.10 = 10%.":"ADが自力で黒猫をよける確率。0.10 = 10%。",Feedback:"フィードバック","What you feel when the robot pushes back.":"ロボットが押し返すときの感触。","Dynamic force feedback":"動的フォースフィードバック","Collaborative: obstacle risk and robot authority scale vibration, with a soft warning sound.":"協調：障害物の危険度とロボットの権限に応じて振動が変わり、やさしい警告音も鳴ります。",Haptics:"振動","Device vibration, where supported.":"対応端末で振動します。","Test feedback":"フィードバックを試す","Obstacle feedback uses vibration where available, a soft pulse sound, and the direction indicator.":"障害物のフィードバックには、振動（対応端末のみ）、やさしいパルス音、方向表示を使います。","Sound & display":"サウンドと表示",Sound:"サウンド","Taps, bonks, a sleepy purr and an opinionated cat.":"タップ音、ゴツン、眠たいゴロゴロ、主張の強いネコ。",Volume:"音量","Soft shadows":"やわらかい影","Turn off for a lighter experience on older phones.":"古いスマホではオフにすると軽くなります。","Settings and up to six recent trial exports stay on this device. Random black-cat events are independent of the shared cleaning scenario.":"設定と最近の試行データ（最大6件）はこの端末に保存されます。ランダムな黒猫イベントは共通のお掃除シナリオとは独立しています。","No learning or personalization.":"学習やパーソナライズはありません。","Download experiment telemetry":"実験データをダウンロード","This browser has no vibration API. Sound and visual obstacle feedback are available.":"このブラウザは振動に対応していません。音と画面表示のフィードバックは使えます。","Haptics are off. Enable Haptics below to request vibration.":"振動はオフです。下の「振動」をオンにしてください。","The browser blocked vibration. Sound and visual feedback are available.":"ブラウザが振動をブロックしました。音と画面表示のフィードバックは使えます。","Vibration requested. If you felt nothing, check device vibration settings. You should also hear a soft pulse.":"振動を送りました。何も感じない場合は端末の振動設定を確認してください。やさしいパルス音も聞こえるはずです。",SCORE:"スコア",CLEANED:"お掃除済み","CAT DISTURBANCE":"ネコの眠りの乱れ","THE SUPERVISOR IS UP.":"監督（ネコ）が起きた！","SHARED DIRECTION":"共有の方向","DEEP SLEEP":"熟睡","LIGHT SLEEP":"うとうと",STIRRING:"もぞもぞ",AWAKE:"起きた",ANGRY:"怒ってる","CAT WOKE UP":"ネコが起きた","AD back on":"ADを再開","Release route":"ルートを解除","RETURNING · LOCKED":"復帰中・ロック","RETURNING TO PATH":"ルートに復帰中",ARBITRATING:"調停中","PLAYER CONTROL":"プレイヤー操作","SYSTEM CONTROL":"システム操作",MAGNET:"マグネット",LOCKED:"ロック","A little breather.":"ちょっとひと休み。","The floor can wait.":"床は待ってくれます。",Resume:"再開","Restart run":"やり直す","Change mode":"モードを変更","Quit run":"プレイをやめる","Quit to main menu":"メインメニューに戻る","Export telemetry":"データを書き出す","Quit this run?":"このプレイをやめますか？","This unfinished attempt won’t be kept.":"途中のプレイは保存されません。","Switch mode?":"モードを切り替えますか？","The stage restarts from the beginning in the new mode. This attempt won’t be kept.":"新しいモードでステージを最初からやり直します。今回のプレイは保存されません。","Cat woke up.":"ネコが起きた。","Cleaning complete.":"お掃除完了。","Your supervisor has filed a very paw-sonal complaint.":"監督（ネコ）から、肉球つきの苦情が届きました。","Spotless floor. Undisturbed dreams.":"ピカピカの床。ぐっすりの夢。","See comparison":"比較を見る","Retry this mode":"このモードでリトライ","Download run telemetry":"プレイデータをダウンロード","Compare again":"もう一度比較","Main menu":"メインメニュー","THE SAME HOUSE. YOUR THREE RUNS.":"同じ家。あなたの3回のプレイ。","Different ways. Your results.":"違うやり方。あなたの結果。","Every number comes from your completed runs. Pickups appear relative to your position, so routes affect the opportunities you encounter.":"数値はすべて、あなたがクリアしたプレイのものです。アイテムはあなたの位置に応じて現れるため、ルートによって出会えるチャンスが変わります。","Involvement = time providing steering input.":"関与度 = 操作入力していた時間。","Smoothness reflects actual turning and collisions.":"なめらかさは、実際の曲がり方と衝突を反映します。","Total score":"合計スコア","Cleaning time":"お掃除時間","Bunnies collected":"回収したうさぎ","Peak cat disturbance":"ネコの眠りの乱れ（最大）",Collisions:"衝突",Smoothness:"なめらかさ","Player involvement":"プレイヤーの関与度","No run":"プレイなし","Each metric has its own scale, shared by all three modes. Bars start at zero.":"各指標の目盛りは3モード共通です。棒グラフはゼロから始まります。","Bonuses collected":"回収したボーナス","Intents detected":"検出した意図","Successful yields":"成功した譲歩","Sync events":"同期の回数","Smooth returns":"なめらかな復帰",Instructions:"説明",Minimise:"最小化","Player weight":"プレイヤーの重み","30 / 70 · heavy":"30 / 70・重め","90 / 10 · light":"90 / 10・軽め","Live practice · full speed":"ライブ練習・通常の速さ","Paused · read at your pace":"一時停止中・自分のペースで読もう","Choose level":"レベルを選ぶ","Done · tap Next when you are ready.":"完了・準備ができたら「次へ」を押してください。","Manual active · steer, then tap AD back on.":"手動中・操作してから「ADを再開」を押してください。",WELCOME:"ようこそ","Hello, little cleaner.":"こんにちは、小さなお掃除係さん。","HOW TO PLAY":"遊び方","Drag toward your destination.":"目的地の方へドラッグ。","Touch the floor, then drag in any direction. Blue is you, amber is automation, white is the shared result. Steering changes direction, not cruise speed.":"床に触れて、好きな方向へドラッグします。青があなた、琥珀色が自動運転、白が共有の結果です。操作で変わるのは向きで、速さは変わりません。","Try each function next. Keyboard: WASD or arrows.":"次は各機能を試してみましょう。キーボード：WASDまたは矢印キー。","Take over, then return to AD.":"引き継いでから、ADに戻す。","Choose your own balance.":"自分のバランスを選ぼう。","03 · INCLUSION · NEARBY":"03・インクルージョン・近く","03 · INCLUSION · FARTHER AWAY":"03・インクルージョン・遠く","Now make your goal the plan.":"今度はあなたの目標を計画に。","Aim, lock, then let go.":"狙って、ロックして、手を離す。","TOUR COMPLETE":"ツアー完了","Two contributors. One plan.":"2人の担い手。1つの計画。","· steer anytime, no switches":"・いつでも操作、切り替え不要","· steering weight, AD leans in near obstacles":"・操作の重さ、障害物の近くではADが強まる","driver and automation interact seamlessly. The driver can steer at any moment, without an explicit switch between automated and manual modes.":"ドライバーと自動運転はシームレスにやりとりします。自動と手動を明示的に切り替えることなく、ドライバーはいつでも操作できます。","your steering contributes immediately in Collaborative mode; release and the system guides the Roomba again. Blue shows your input, amber the system, and white the shared result.":"協調モードでは、あなたの操作がすぐに反映されます。離すと再びシステムがロボットを導きます。青があなたの入力、琥珀色がシステム、白が共有の結果です。","Interaction: seamless, immediate steering influence with no mode switch.":"インタラクション：モード切り替えなしで、すぐに効くシームレスな操作。","Decide who carries the weight.":"どちらが重みを担うかを決める。","arbitration allocates authority between driver and automation, and the driver feels that allocation as steering weight.":"アービトレーションはドライバーと自動運転の間で権限を配分し、ドライバーはその配分を操作の重さとして感じます。","a phone or PC cannot reproduce steering-wheel resistance, so Player share is our stand-in: 30/70 feels heavier toward the system’s plan; 90/10 gives you lighter, stronger control.":"スマホやPCではハンドルの抵抗を再現できないため、プレイヤー配分で代わりに表現しています：30/70はシステムの計画側に重く、90/10は軽く、あなたの操作が強く効きます。","Near obstacles:":"障害物の近くでは：","as furniture or a wall gets close ahead, the system share rises smoothly, up to 90%, then returns to your setting once the way is clear.":"前方に家具や壁が近づくと、システムの配分が最大90%までなめらかに上がり、進路が空くとあなたの設定に戻ります。","Arbitration: the weight between you and AD, leaning toward AD near obstacles.":"アービトレーション：あなたとADの間の重み。障害物の近くではADに寄ります。","Arbitration: steering weight stands in for haptic resistance. Higher player share feels lighter. Near obstacles, AD can still take up to 90%.":"アービトレーション：操作の重さが触覚の抵抗の代わりです。プレイヤー配分が高いほど軽く感じます。障害物の近くでは、ADが最大90%まで担います。","Interaction: control shifts seamlessly from system guidance toward player steering, without a mode switch.":"インタラクション：モードを切り替えることなく、操作がシステムの誘導からプレイヤーの操作へシームレスに移ります。","Arbitration sets the weight: 30/70 feels heavier, 90/10 lighter, and near obstacles the system share rises up to 90%. Blue is player direction, amber is system direction, white is shared.":"アービトレーションが重さを決めます：30/70は重く、90/10は軽く、障害物の近くではシステムの配分が最大90%まで上がります。青がプレイヤー、琥珀色がシステム、白が共有の方向です。","01 · INTERACTION · COMPARE":"01・インタラクション・比較","01 · PAIRDRIVER INTERACTION":"01・Pairdriverのインタラクション","Now steer, without a switch.":"今度は切り替えなしで操作する。","INTENT ACCEPTED":"意図を採用",OBSTACLE:"障害物","AD LEANS IN":"ADが強まる","30 / 70 · Feel the weight.":"30 / 70・重さを感じよう。","Change to 90 / 10 · Feel it lighten.":"90 / 10に変更・軽さを感じよう。","Set 90 / 10 and steer for one second.":"90 / 10にして1秒間操作します。","Clean the floor for points. Red bunnies give 250, gold gives 1,500.":"床をお掃除すると得点。赤いうさぎは250、金は1,500です。","Swipe or tap Next.":"スワイプするか「次へ」を押そう。","First, a normal takeover.":"まずは普通の引き継ぎ。","In Full Automation, hold a drag to switch AD off and drive yourself.":"完全自動でドラッグし続けると、ADがオフになり自分で運転します。","This switch is what we compare against.":"この切り替えが比べる基準です。","Hold a drag until Manual appears. Steer, then press AD back on (or E).":"手動になるまでドラッグし続けます。操作したら「ADを再開」（またはE）を押します。","Take over, then turn AD back on.":"引き継いでから、ADを再開しよう。","In Collaborative, your drag and AD blend into one direction. No takeover needed.":"協調モードでは、あなたのドラッグとADが1つの方向に混ざります。引き継ぎは不要です。","Next: try it yourself.":"次へ：自分で試してみよう。","Drag any way. AD stays on and blends with you.":"好きな方向へドラッグ。ADはオンのまま、あなたと混ざります。","Steer for one second.":"1秒間操作しよう。","AD carries 70%. Steer away from the amber arrow and feel it pull back.":"ADが70%を担います。琥珀色の矢印から離れるように操作して、引き戻しを感じましょう。","Move the slider to 90 / 10, then steer. It feels much lighter.":"スライダーを90 / 10にして操作します。ぐっと軽く感じます。","Pick any weight you like. It carries into the next exercises.":"好きな重さを選びましょう。次の練習にも引き継がれます。","Tap Next when it feels right.":"しっくりきたら「次へ」。","Close to an obstacle, AD takes up to 90% and steers you clear.":"障害物に近づくと、ADが最大90%を担ってよけてくれます。","Next: drive at the pink furniture.":"次へ：ピンクの家具に向かって進もう。","Steer straight at the pink outline and keep holding. AD will take over.":"ピンクの枠にまっすぐ向かって操作し続けます。ADが引き継ぎます。","Hold until AD takes over.":"ADが引き継ぐまで保とう。","A red bunny appears.":"赤いうさぎが現れます。","Steer toward it to ask for a short detour. It is worth 250.":"その方へ操作すると、短い寄り道をお願いできます。250点です。","Next shows a red bunny nearby.":"「次へ」で近くに赤いうさぎが現れます。","Take the red bunny.":"赤いうさぎを取ろう。","Keep aiming at it until you collect it.":"回収するまで狙い続けましょう。","Collect the red bunny.":"赤いうさぎを回収しよう。","Aim at the far gold bunny until LOCKED, then let go. AD plans the route there.":"遠くの金のうさぎをロックされるまで狙い、手を離します。ADがルートを計画します。","Next shows gold farther away.":"「次へ」で遠くに金のうさぎが現れます。","Once LOCKED, you can still steer and the route follows. Press E to release it.":"ロック後も操作でき、ルートがついてきます。Eで解除できます。","Lock gold, collect it, and let AD return to cleaning.":"金をロックして回収し、ADにお掃除を再開させよう。","Interaction: steer without a switch. Arbitration: share the weight. Inclusion: your goal becomes the plan.":"インタラクション：切り替えなしで操作。アービトレーション：重みを分け合う。インクルージョン：あなたの目標が計画に。","A simple game analogy, not real steering force.":"簡単なゲームの例えで、実際のハンドルの力ではありません。","Steer seamlessly.":"シームレスに操作しよう。","02 · ARBITRATION · OBSTACLES":"02・アービトレーション・障害物","Near furniture, AD leans in.":"家具の近くでは、ADが強まる。","Drive at the highlighted furniture.":"強調表示された家具に向かって進もう。"},Xc=s=>ll(s)??s,Kv=[[/^(\d+)% player · (\d+)% system$/,(s,t,e)=>`プレイヤー${t}%・システム${e}%`],[/^After intent: (\d+)% player · (\d+)% system$/,(s,t,e)=>`意図検出後：プレイヤー${t}%・システム${e}%`],[/^([\d.]+) seconds$/,(s,t)=>`${t}秒`],[/^([\d.]+)s$/,(s,t)=>`${t}秒`],[/^Restart as (.+)$/,(s,t)=>`${Xc(t)}でやり直す`],[/^CURRENTLY (.+)$/,(s,t)=>`現在：${Xc(t)}`],[/^BEST CLEAN · ([\d,]+) POINTS$/,(s,t)=>`ベスト記録・${t}点`],[/^COMPARISON (\d+) \/ 3$/,(s,t)=>`比較 ${t} / 3`],[/^Magnet · ([\d.]+)s$/,(s,t)=>`マグネット・${t}秒`],[/^RESISTANCE (\d+)%$/,(s,t)=>`抵抗 ${t}%`],[/^Takeover · ([\d.]+) \/ ([\d.]+)s$/,(s,t,e)=>`引き継ぎ・${t} / ${e}秒`],[/^HOLD ([\d.]+)\/([\d.]+)s$/,(s,t,e)=>`ホールド ${t}/${e}秒`],[/^Hold a direction for about 0\.8 seconds\. For gold, aim for ([\d.]+) seconds until LOCKED, then release\. The robot routes to gold and back toward remaining dust\.$/,(s,t)=>`約0.8秒、同じ方向を保ちます。金のうさぎはロックされるまで${t}秒狙ってから離します。ロボットが金のうさぎまで行き、残りのホコリの方へ戻ります。`],[/^[\d\s.,:/%+−\-✦]+$/,s=>s]];let Ua=null;function ll(s){const t=Wc[s];if(t!==void 0)return t;Ua??(Ua=new Map(Object.entries(Wc).map(([i,n])=>[i.toLowerCase(),n])));const e=Ua.get(s.toLowerCase());if(e!==void 0)return e;for(const[i,n]of Kv){const r=s.match(i);if(r)return n(...r)}if(s.includes(" · ")){const i=s.split(" · ").map(n=>ll(n.trim()));if(i.every(n=>n!==null))return i.join("・")}return null}const $c="catnap-lang";class Zv{constructor(t){y(this,"root");y(this,"lang","en");y(this,"observer",null);this.root=t;try{localStorage.getItem($c)==="ja"&&(this.lang="ja")}catch{}this.apply()}set(t){this.lang=t;try{localStorage.setItem($c,t)}catch{}this.apply()}apply(){document.documentElement.lang=this.lang,this.observer?.disconnect(),this.observer=null,this.lang==="ja"&&(this.translateTree(this.root),this.observer=new MutationObserver(t=>{for(const e of t)e.type==="characterData"?this.translateNode(e.target):e.addedNodes.forEach(i=>this.translateTree(i))}),this.observer.observe(this.root,{subtree:!0,childList:!0,characterData:!0}))}translateTree(t){if(t.nodeType===Node.TEXT_NODE){this.translateNode(t);return}const e=document.createTreeWalker(t,NodeFilter.SHOW_TEXT);for(let i=e.nextNode();i;i=e.nextNode())this.translateNode(i)}translateNode(t){const e=t.nodeValue;if(!e||!/[A-Za-z]/.test(e))return;const i=ll(e.replace(/\s+/g," ").trim());i!==null&&i!==e&&(t.nodeValue=i)}}class Jv{constructor(){y(this,"tour",null);y(this,"navigation",null);y(this,"archive",new xu);y(this,"session",new ku);y(this,"storage",new kv);y(this,"feedback",new zv(this.storage));y(this,"input",new Uv);y(this,"ui",new qv(this.session,this.storage));y(this,"i18n",new Zv(document.querySelector("#app")));y(this,"world");y(this,"sim",new Tl("collaborative",20260908));y(this,"state","MAIN_MENU");y(this,"debug",!1);y(this,"quitToMenu",!1);y(this,"settingsReturn","MAIN_MENU");y(this,"infoReturn","MAIN_MENU");y(this,"last",0);y(this,"accumulator",0);y(this,"gameOverTime",0);this.world=new Lv(document.querySelector("#world")),this.world.renderer.shadowMap.enabled=this.storage.settings.quality,this.ui.action=t=>this.action(t),this.ui.setting=(t,e)=>{this.storage.settings[t]=e,t==="interventionShare"&&this.sim.setInterventionShare(Number(e)),t==="inclusionDelay"&&this.sim.setInclusionDelay(Number(e)),t==="takeoverHold"&&this.sim.setTakeoverHold(Number(e)),["criticalFrequency","fullAutoCriticalAvoidSuccessProbability","dynamicForceFeedbackEnabled"].includes(t)&&(Object.assign(this.storage.settings,qn(this.storage.settings)),this.sim.setExperiment(qn(this.storage.settings))),this.storage.save(),this.feedback.option(t,t==="volume"||t==="interventionShare"||t==="inclusionDelay"||t==="takeoverHold"),this.world.renderer.shadowMap.enabled=this.storage.settings.quality},this.input.hazardId=()=>this.state==="PLAYING"?this.sim.critical.activeId:null,this.input.onEmergency=t=>{if(this.state==="PLAYING"){const e=ml(t,this.world.camera,window.innerWidth,window.innerHeight);if(e)return this.sim.emergencySwipe(e.direction)}return!1},this.input.onPause=()=>{this.state==="PLAYING"?this.action("pause"):this.state==="PAUSED"&&this.action("resume")},this.input.onEscape=()=>{this.state==="PLAYING"||this.state==="PAUSED"?this.input.onPause():this.state!=="MAIN_MENU"&&this.back()},this.input.onReengage=()=>{this.state==="PLAYING"&&(this.sim.automationDisengaged?this.action("ad-back"):this.sim.mode==="collaborative"&&this.sim.hasAcceptedRoute&&this.action("release-route"))},this.input.onModeSwitch=()=>{this.state==="PLAYING"&&this.session.flow==="free_play"&&this.show("MODE_SWITCH")},this.input.onDebug=()=>this.debug=!this.debug,this.input.onGesture=()=>this.feedback.unlock(),document.addEventListener("visibilitychange",()=>{document.hidden&&(this.feedback.silence(),this.state==="PLAYING"&&this.show("PAUSED"))}),window.addEventListener("blur",()=>{this.state==="PLAYING"&&this.show("PAUSED"),this.feedback.silence()}),this.navigation=new vu(()=>this.back()),this.ui.lang=this.i18n.lang,this.show("MAIN_MENU"),requestAnimationFrame(t=>this.frame(t))}show(t){if(this.state=t,this.navigation?.sync(t!=="MAIN_MENU"),this.input.enabled=t==="PLAYING",this.input.reset(),this.ui.show(t,this.sim),t==="PLAYING"){const e=document.querySelector("#joystick-region");e&&this.input.attach(e)}else this.accumulator=0,queueMicrotask(()=>this.ui.root.querySelector("button")?.focus({preventScroll:!0}))}back(){if(this.state==="PLAYING"){this.show("PAUSED");return}if(this.state==="PAUSED"){this.sim.abandon(),this.archive.save(this.sim),this.show(this.session.level==="tour"?"LEVEL_SELECT":"MODE_SELECT");return}if(this.state==="QUIT_CONFIRM"||this.state==="MODE_SWITCH"){this.show("PAUSED");return}if(this.state==="PAIRDRIVER_INFO"){this.action(this.ui.pairdriverSlide>0?"pairdriver-prev":"info-back");return}if(this.state==="RUN_INTRO"){this.action(this.ui.welcomeSlide>0?"welcome-prev":"intro-back");return}if(this.state==="SETTINGS"){this.show(this.settingsReturn);return}if(this.state==="MODE_SELECT"){this.show("LEVEL_SELECT");return}if(["RESULTS","GAME_OVER","COMPARISON_RESULTS"].includes(this.state)){this.show("LEVEL_SELECT");return}if(this.state==="MAIN_MENU"){this.navigation?.atHomeBack();return}this.show("MAIN_MENU")}start(){this.feedback.resetRun(),this.sim.abandon(),this.archive.save(this.sim),this.sim=new Tl(this.session.mode,this.session.seed,{level:this.session.level,flow:this.session.flow,interventionShare:this.storage.settings.interventionShare,inclusionDelay:this.storage.settings.inclusionDelay,takeoverHold:this.storage.settings.takeoverHold,experiment:qn(this.storage.settings)}),this.tour=this.session.level==="tour"?new gu:null,this.ui.tour=this.tour,this.tour&&this.tour.enter(this.sim),this.sim.bonusVisible=t=>this.world.bonusVisible(t),this.accumulator=0,this.gameOverTime=0,this.feedback.unlock(),this.show("PLAYING")}action(t){if(this.feedback.unlock(),t==="lang-en"||t==="lang-ja"){this.i18n.set(t==="lang-ja"?"ja":"en"),this.ui.lang=this.i18n.lang,this.show(this.state);return}let e=t;if(t==="test-feedback"&&this.state==="SETTINGS"){const i=this.feedback.testFeedback(),n=this.ui.root.querySelector("#feedback-test-result");n&&(n.textContent={unavailable:"This browser has no vibration API. Sound and visual obstacle feedback are available.",disabled:"Haptics are off. Enable Haptics below to request vibration.",blocked:"The browser blocked vibration. Sound and visual feedback are available.",requested:"Vibration requested. If you felt nothing, check device vibration settings. You should also hear a soft pulse."}[i]);return}if(t.startsWith("tour-weight-")&&this.tour&&this.state==="PLAYING"&&["light","weight-choice"].includes(this.tour.step.id)){const i=Math.max(.3,Math.min(.9,Number(t.slice(12))/100));this.sim.setInterventionShare(i);const n=this.ui.root.querySelector("#tour-weight-value");n&&(n.textContent=`${Math.round(i*100)}% player · ${Math.round((1-i)*100)}% system`);return}if(t==="tour-guide-toggle"&&this.tour&&this.state==="PLAYING"){this.tour.compact=!this.tour.compact,this.ui.renderTour(this.sim);return}if((t==="tour-next"||t==="tour-prev")&&this.tour&&this.state==="PLAYING"){(t==="tour-next"?this.tour.next(this.sim):this.tour.previous(this.sim))&&(this.input.reset(),this.accumulator=0,this.ui.renderTour(this.sim),this.feedback.option("slide-step"));return}if(t==="tour-finish"&&this.tour?.step.id==="done"){this.sim.emit("tour_completed"),this.sim.abandon(),this.archive.save(this.sim),this.tour=null,this.ui.tour=null,this.show("LEVEL_SELECT"),this.feedback.option("tour-finish");return}if(t==="ad-back"&&this.state==="PLAYING"){const i=this.sim.automationDisengaged;this.sim.reactivateAutomation(),i&&!this.sim.automationDisengaged&&this.feedback.event("automation_reactivated");return}if(t.startsWith("level-")&&this.state==="LEVEL_SELECT"&&t!=="level-continue"){const i=t.slice(6);["tour","level1","practice"].includes(i)&&(this.ui.selectedLevel=i,this.session.level=i,this.show("LEVEL_SELECT"),this.feedback.option("select-level"));return}if(t==="level-continue"&&this.state==="LEVEL_SELECT"){this.session.level=this.ui.selectedLevel,this.session.level==="tour"?(this.session.begin("tour",this.newSeed(),"collaborative"),this.start()):this.show("MODE_SELECT"),this.feedback.option("play");return}if(t.startsWith("select-")&&this.state==="MODE_SELECT"){this.ui.selectMode(t.slice(7)),this.feedback.option(t);return}if(t==="play")this.show("LEVEL_SELECT");else if(t==="start"&&this.state==="MODE_SELECT")this.session.begin("free_play",this.newSeed(),this.ui.selected),this.start();else if(t==="guide"&&this.state==="MODE_SELECT")this.session.begin("free_play",this.newSeed(),this.ui.selected),this.start(),this.ui.welcomeSlide=0,this.show("RUN_INTRO");else if((t==="welcome-next"||t==="welcome-prev")&&this.state==="RUN_INTRO"){const i=Math.max(0,Math.min(3,this.ui.welcomeSlide+(t==="welcome-next"?1:-1)));if(i===this.ui.welcomeSlide)return;this.ui.welcomeSlide=i,e="slide-step",this.show("RUN_INTRO"),queueMicrotask(()=>this.ui.root.querySelector("[data-action=welcome-next],[data-action=begin-run]")?.focus({preventScroll:!0}))}else if(t==="begin-run"&&this.state==="RUN_INTRO"&&this.ui.welcomeSlide===3)this.start();else if(t==="intro-back"&&this.state==="RUN_INTRO")this.show(this.session.flow==="comparison"?this.session.index>0?"MAIN_MENU":"COMPARISON_INTRO":"MODE_SELECT");else if(t==="pairdriver"&&["MAIN_MENU","SETTINGS","RUN_INTRO","HOW_TO_PLAY"].includes(this.state))this.infoReturn=this.state,this.ui.pairdriverSlide=0,this.show("PAIRDRIVER_INFO");else if((t==="pairdriver-next"||t==="pairdriver-prev")&&this.state==="PAIRDRIVER_INFO"){const i=Math.max(0,Math.min(Yn-1,this.ui.pairdriverSlide+(t==="pairdriver-next"?1:-1)));if(i===this.ui.pairdriverSlide)return;this.ui.pairdriverSlide=i,e="slide-step",this.show("PAIRDRIVER_INFO"),queueMicrotask(()=>this.ui.root.querySelector("[data-action=pairdriver-next],footer [data-action=info-back]")?.focus({preventScroll:!0}))}else if(t==="info-back"&&this.state==="PAIRDRIVER_INFO")this.show(this.infoReturn);else if(t==="comparison")this.show("COMPARISON_INTRO");else if(t==="start-comparison"&&this.state==="COMPARISON_INTRO")this.session.level="level1",this.session.begin("comparison",this.newSeed()),this.ui.welcomeSlide=0,this.show("RUN_INTRO");else if(t==="release-route"&&this.state==="PLAYING"&&this.sim.hasAcceptedRoute)this.sim.releaseRoute();else if(t==="pause"&&this.state==="PLAYING")this.show("PAUSED");else if(t==="resume"&&this.state==="PAUSED")this.show("PLAYING");else if(t==="restart"&&["PAUSED","RESULTS","GAME_OVER"].includes(this.state))delete this.session.results[this.session.mode],this.start();else if((t==="quit-request"||t==="quit-main-request")&&this.state==="PAUSED")this.quitToMenu=t==="quit-main-request",this.show("QUIT_CONFIRM");else if(t==="mode-switch-request"&&this.state==="PAUSED"&&this.session.flow==="free_play")this.show("MODE_SWITCH");else if(t==="cancel-mode-switch"&&this.state==="MODE_SWITCH")this.show("PAUSED");else if(t.startsWith("switch-mode-")&&this.state==="MODE_SWITCH"){const i=t.slice(12);["manual","automation","collaborative"].includes(i)&&(this.session.mode=i,this.session.results={},this.start())}else t==="cancel-quit"&&this.state==="QUIT_CONFIRM"?this.show("PAUSED"):t==="quit"&&this.state==="QUIT_CONFIRM"?(this.sim.abandon(),this.archive.save(this.sim),this.show(this.quitToMenu||this.session.flow==="comparison"?"MAIN_MENU":this.session.level==="tour"?"LEVEL_SELECT":"MODE_SELECT")):t==="change-mode"&&this.state==="RESULTS"||t==="change-mode"&&this.state==="GAME_OVER"?this.show("MODE_SELECT"):t==="main"?(this.sim.abandon(),this.archive.save(this.sim),this.show("MAIN_MENU")):t==="continue"&&this.state==="RESULTS"&&this.sim.metrics.completed?(this.session.accept(this.sim.metrics),this.session.comparisonComplete?this.show("COMPARISON_RESULTS"):this.session.next()&&(this.ui.welcomeSlide=0,this.show("RUN_INTRO"))):t==="help"?this.show("HOW_TO_PLAY"):t==="settings"||t==="pause-settings"?(this.settingsReturn=t==="pause-settings"?"PAUSED":"MAIN_MENU",this.show("SETTINGS")):t==="back"?this.back():t==="export"&&["RESULTS","GAME_OVER","PAUSED","SETTINGS","COMPARISON_RESULTS"].includes(this.state)&&this.exportTelemetry();this.feedback.option(e)}newSeed(){const t=new Uint32Array(1);return crypto.getRandomValues(t),t[0]}exportTelemetry(){this.archive.save(this.sim);const t=new Blob([JSON.stringify({game:"CATNAP CLEANER",version:11,currentTrialId:this.sim.time>0?this.sim.critical.trialId:null,archivePersistent:this.archive.persistent,trials:this.archive.trials},null,2)],{type:"application/json"}),e=URL.createObjectURL(t),i=document.createElement("a");i.href=e,i.download=`catnap-experiment-${this.sim.seed}.json`,i.click(),setTimeout(()=>URL.revokeObjectURL(e),1e3)}processEvents(){for(const t of this.sim.events){const e=this.sim.nearbyObstacle,i=Math.hypot(Math.max(0,Math.abs(this.sim.position.x-e.x)-e.w/2),Math.max(0,Math.abs(this.sim.position.z-e.z)-e.d/2))<.75;this.feedback.event(t.type,i?e.kind:"wall"),(t.type==="black_cat_collision"||t.type==="collision"||t.type==="bonus_collection"||t.type==="magnet_collection")&&this.world.burst(this.sim.position,t.type==="collision"||t.type==="black_cat_collision")}}frame(t){const e=this.last?Math.min((t-this.last)/1e3,.05):.016666666666666666;if(this.last=t,this.state==="PLAYING"&&this.sim.status==="playing"){const n=this.tour?.timeScale??1;for(this.input.enabled=n>0,this.accumulator+=e*n;this.accumulator>=1/60&&this.sim.status==="playing";){const r=this.input.update(.016666666666666666),a=ml(r,this.world.camera,window.innerWidth,window.innerHeight);this.sim.step(1/60,a,1/60/Math.max(.01,n)),this.tour?.update(1/60/Math.max(.01,n),a,this.sim),this.processEvents(),this.accumulator-=1/60}this.sim.status==="success"&&(this.session.accept(this.sim.metrics),this.storage.best=Math.max(this.storage.best,this.sim.metrics.score),this.storage.save(),this.archive.save(this.sim),this.show("RESULTS"))}this.state==="PLAYING"&&this.sim.status==="failed"&&(this.input.enabled=!1,this.gameOverTime+=e,this.sim.critical.animateExit(e),this.gameOverTime>1.65&&(this.archive.save(this.sim),this.show("GAME_OVER")));const i=["MAIN_MENU","LEVEL_SELECT","MODE_SELECT","COMPARISON_INTRO","PAIRDRIVER_INFO","HOW_TO_PLAY","SETTINGS","COMPARISON_RESULTS"].includes(this.state);this.world.playableView=this.ui.cameraView(window.innerWidth,window.innerHeight),this.world.update(e,this.sim,i,this.state!=="PLAYING"||this.tour?.timeScale===0),this.feedback.update(this.state==="PLAYING"&&this.sim.status==="playing"&&!document.hidden,this.sim.speed,this.sim.cat.value),this.feedback.dynamic(Math.max(this.sim.forceSample?.generatedDynamicForce??0,this.sim.tourInteractionForce),t/1e3,this.state==="PLAYING"&&this.sim.status==="playing"&&!document.hidden&&(this.sim.experiment.dynamicForceFeedbackEnabled||this.sim.tourWeightOnly)&&this.sim.mode==="collaborative"&&this.tour?.timeScale!==0),this.ui.update(this.sim,this.debug),this.state==="PLAYING"&&(this.ui.updateDirections(this.world,this.sim),this.ui.hud["bonus-labels"].innerHTML=this.sim.activeBonuses.map(n=>{const r=this.world.project(n,1);return`<span class="${n.kind==="gold"?"gold-bonus":n.kind==="red"?"red-bonus":n.kind==="magnet"?"magnet-pickup":""}" style="left:${r.x}px;top:${r.y}px">${n.kind==="magnet"?"MAGNET":`${n.kind==="gold"?"✦ ":""}+${n.value}`} · ${Math.max(0,n.spawn+n.duration-this.sim.time).toFixed(1)}s${n.kind==="gold"&&this.sim.mode==="collaborative"?this.sim.targetBonus?.id===n.id&&this.sim.inclusionLocked?" · LOCKED":this.sim.inclusionTimer>0&&this.sim.targetBonus?.id===n.id?` · HOLD ${Math.min(this.sim.inclusionTimer,this.sim.inclusionDelay).toFixed(1)}/${this.sim.inclusionDelay.toFixed(1)}s`:"":""}</span>`}).join("")),requestAnimationFrame(n=>this.frame(n))}}try{new Jv}catch(s){document.querySelector("#app").innerHTML='<div class="modal-backdrop"><section class="modal"><h2>A little hiccup.</h2><p>This game needs WebGL. Try enabling hardware acceleration or opening it in a current browser.</p><button class="button primary" onclick="location.reload()">Try again</button></section></div>',console.error(s)}"serviceWorker"in navigator&&window.addEventListener("load",()=>{navigator.serviceWorker.register("/catnap-cleaner-site/sw.js").catch(()=>{})});
