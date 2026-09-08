// Focused geometry/render checks for the JS embedded in the vision console.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const source = fs.readFileSync('ov9281_debug/ov9281_unified_service.py', 'utf8');
const html = source.match(/HTML = r"""([\s\S]*?)"""\.encode\(\)/)[1];
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const labels = [];
const ctx = new Proxy({}, {get: (_, k) => k === 'measureText' ? t => ({width:t.length*12}) : (...args) => {if(k === 'fillText') labels.push(args[0]);},set:()=>true});
const elements = new Map();
const get = id => {if(!elements.has(id))elements.set(id,{getContext:()=>ctx,addEventListener(){}});return elements.get(id);};
const sandbox = {document:{getElementById:get,addEventListener(){}},window:{addEventListener(){}},setInterval(){},setTimeout(){},clearTimeout(){},Date};
vm.createContext(sandbox);
vm.runInContext(script.slice(0,script.indexOf('async function update()')),sandbox);
function project(x,y){const w=1+.0004*x+.0007*y;return [(1.1*x+.15*y+600)/w,(.12*x+.9*y+370)/w];}
function marker(inner,angle,tilted){
  const t=(angle+(inner?-45:0))*Math.PI/180,size=inner?30:150;
  const map=([x,y])=>{const X=(x*Math.cos(t)-y*Math.sin(t))*size,Y=(x*Math.sin(t)+y*Math.cos(t))*size;return tilted?project(X,Y):[640+X,400+Y];};
  const heading=angle*Math.PI/180;
  const origin=tilted?project(0,0):[640,400];
  const tip=tilted?project(30*Math.sin(heading),-30*Math.cos(heading)):[640+30*Math.sin(heading),400-30*Math.cos(heading)];
  return {tag_id:inner?1:0,role:inner?'inner':'outer',quality_passed:true,corners_px:[[1,-1],[-1,-1],[-1,1],[1,1]].map(map),orientation:{valid:true,frame:'BODY_FRD',source:'PNP_TAG_TO_PAD_TO_BODY',arrow_image_px:[origin,tip],pad_heading_body_deg:angle,pad_forward_body_frd:[Math.cos(heading),Math.sin(heading),0]}};
}
let count=0;
for(const angle of [0,30,90,180,270])for(const tilted of [false,true]){
  const outer=sandbox.tagDirection(marker(false,angle,tilted)),inner=sandbox.tagDirection(marker(true,angle,tilted));
  assert.ok(outer.unit[0]*inner.unit[0]+outer.unit[1]*inner.unit[1]>.999999,'inner compensation must match outer under perspective');count++;
}
// No client-side fallback: corners alone cannot claim a BODY_FRD direction.
const actual=sandbox.tagDirection({tag_id:0,corners_px:[[299.875,99.9251],[99.9249,99.8651],[99.8649,299.875],[299.875,299.875]]});
assert.equal(actual,null);
const invalid=marker(true,0,false);invalid.orientation.valid=false;assert.equal(sandbox.tagDirection(invalid),null);
assert.equal(sandbox.tagDirection({tag_id:1,corners_px:[[0,0],[0,0],[0,0],[0,0]]}),null);
assert.equal(sandbox.tagDirection({tag_id:1,corners_px:[[NaN,0],[0,0],[0,0],[0,0]]}),null);
vm.runInContext('directionReceivedAt=Date.now()',sandbox);
const state={mode:'apriltag',frame_age_ms:20,detections:[marker(false,0,false),marker(true,0,false)]};
sandbox.draw(state);assert.deepEqual(labels,['画面上方','大 Tag 上方','小 Tag 校正']);
labels.length=0;sandbox.draw({...state,frame_age_ms:800});assert.deepEqual(labels,['画面上方']);
labels.length=0;sandbox.draw({...state,detections:[]});assert.deepEqual(labels,['画面上方']);
console.log(`PASS: ${count} server-projected pairs, no UI orientation fallback, invalid geometry, three arrows, stale and lost tags`);
const flight = v => ({mode:'GUIDED',flight_controller_connected:true,motion_command:{state:'READY',age_s:.02,body_frame:'FLU',body_velocity_mps:v}});
for(const [v,expected] of [[{x:.1,y:0,z:0},[0,-1]],[{x:0,y:-.1,z:0},[1,0]],[{x:-.1,y:.1,z:0},[-Math.SQRT1_2,Math.SQRT1_2]]]){
  const m=sandbox.motionDirection(flight(v),100);assert.ok(Math.abs(m.unit[0]-expected[0])<1e-9&&Math.abs(m.unit[1]-expected[1])<1e-9);
}
assert.equal(sandbox.motionDirection(flight({x:1,y:0,z:0}),800),null);
assert.equal(sandbox.motionDirection({...flight({x:1,y:0,z:0}),mode:'LOITER'},0),null);
assert.equal(sandbox.motionDirection(flight({x:0,y:0,z:.1}),0).unit,null);
labels.length=0;sandbox.draw({...state,flight:flight({x:.1,y:0,z:-.05})});
assert.ok(labels.includes('准备运动 0.100 m/s'));assert.ok(labels.includes('下降 0.050 m/s'));
console.log('PASS: sent-command display directions, stale/mode gating, vertical label');
