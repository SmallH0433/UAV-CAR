// Run: node tools/test_console_directions.cjs. No camera, ROS or flight link.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../ov9281_debug/ov9281_unified_service.py'), 'utf8');
const script = source.match(/<script>([\s\S]*?)<\/script>/)[1];
const elements = new Map();
const context2d = new Proxy({}, {get: (_, key) => key === 'measureText' ? () => ({width: 100}) : () => {}});
const element = id => {
  if (!elements.has(id)) elements.set(id, {
    textContent: '', style: {}, classList: {toggle() {}}, addEventListener() {},
    getContext: () => context2d,
  });
  return elements.get(id);
};
const sandbox = {
  document: {getElementById: element, addEventListener() {}},
  window: {addEventListener() {}}, setInterval() {}, setTimeout() {}, clearTimeout() {},
  Date: {now: () => 10000},
  // Block startup polling without network activity.
  fetch: () => new Promise(() => {}),
};
vm.createContext(sandbox);
vm.runInContext(script, sandbox);
const run = code => vm.runInContext(code, sandbox);
const close = (actual, expected) => actual.forEach((n, i) => assert.ok(Math.abs(n - expected[i]) < 1e-9));
const detection = body => ({
  tag_id: 1, role: 'inner', quality_passed: true,
  orientation: {valid: true, frame: 'BODY_FRD', source: 'PNP_TAG_TO_PAD_TO_BODY',
    // Deliberately different camera projection: catches undoing body calibration.
    arrow_image_px: [[640, 400], [680, 400]], pad_forward_body_frd: body, pad_heading_body_deg: 0},
});
const command = (x, y, z = 0) => ({flight_controller_connected: true, mode: 'GUIDED',
  motion_command: {state: 'READY', body_frame: 'FLU', age_s: .1, body_velocity_mps: {x, y, z}}});
for (const [body, expected] of [
  [[1, 0, 0], [0, -1]], [[0, 1, 0], [1, 0]],
  [[-1, 0, 0], [0, 1]], [[0, -1, 0], [-1, 0]],
  [[1, 1, .3], [Math.SQRT1_2, -Math.SQRT1_2]],
]) {
  close(sandbox.tagDirection(detection(body)).unit, expected);
  close(sandbox.motionDirection(command(body[0], -body[1]), 0).unit, expected);
}
const outer = detection([1, 0, 0]); outer.role = 'outer'; outer.tag_id = 0;
close(sandbox.tagDirection(outer).unit, [1, 0]);
for (const bad of [null, [], [NaN, 1, 0], [0, 0, 1]]) assert.equal(sandbox.tagDirection(detection(bad)), null);
const rejected = detection([1, 0, 0]); rejected.quality_passed = false;
assert.equal(sandbox.tagDirection(rejected), null);
assert.equal(sandbox.motionDirection(command(1, 0), 600), null);
for (const mode of ['LAND', 'LOITER']) assert.equal(sandbox.motionDirection({...command(1, 0), mode}, 0), null);
assert.equal(sandbox.motionDirection({...command(1, 0), flight_controller_connected: false}, 0), null);
assert.equal(sandbox.motionDirection(command(NaN, 0), 0), null);
assert.equal(sandbox.motionDirection(command(0, 0, .2), 0).unit, null);
assert.equal(sandbox.motionDirection(command(0, 0, .2), 0).up, .2);
sandbox.fixture = {mode: 'apriltag', frame_age_ms: 500, detections: [outer, detection([1, 0, 0])], flight: command(1, 0)};
run('var arrows=[]; directionArrow=(origin,unit,length,color,label)=>arrows.push({origin,unit,color,label}); directionReceivedAt=10000; draw(fixture)');
assert.deepEqual(Array.from(run('arrows.map(a=>a.color)')), ['#00d9ff', '#ffb020', '#ff54d9', '#ff1528']);
run('arrows=[]; directionReceivedAt=9750; draw(fixture)');
assert.deepEqual(Array.from(run('arrows.map(a=>a.color)')), ['#00d9ff', '#ff1528']);
run('arrows=[]; directionReceivedAt=9000; draw(fixture)');
assert.deepEqual(Array.from(run('arrows.map(a=>a.color)')), ['#00d9ff']);
console.log('PASS: body/camera directions, FRD/FLU agreement, four arrow colors, quality rejection, stale frames/commands, mode exit and vertical motion.');
