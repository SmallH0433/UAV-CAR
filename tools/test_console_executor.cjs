// Read-only console tests; no ROS, camera or flight link required.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../ov9281_debug/ov9281_unified_service.py'), 'utf8');
const elements = new Map();
const element = id => {
  if (!elements.has(id)) elements.set(id, {textContent: '', style: {},
    classList: {toggle() {}}, addEventListener() {}, getContext: () => ({})});
  return elements.get(id);
};
let now = 10000;
const sandbox = {document: {getElementById: element, addEventListener() {}},
  window: {addEventListener() {}}, Date: {now: () => now},
  setInterval() {}, setTimeout() {}, clearTimeout() {}, fetch: () => new Promise(() => {})};
// Match browser named elements used by the existing flight panel.
for (const [, id] of source.matchAll(/id="([^"]+)"/g)) sandbox[id] = element(id);
vm.createContext(sandbox);
vm.runInContext(source.match(/<script>([\s\S]*?)<\/script>/)[1], sandbox);
const text = id => element(id).textContent;
const fresh = {guided_executor_age_s: .1, action_state: 'RUNNING', action: 'FOLLOW',
  executor_version: 'v2', control_owner: 'ACTION_EXECUTOR_V2', follow_active: true,
  landing_active: false, mode_gate: 'GUIDED', action_detail: 'TRACKING'};
sandbox.updateFlight(fresh);
assert.equal(text('executorState'), '执行中');
assert.equal(text('executorAction'), 'FOLLOW');
assert.equal(text('executorActivity'), '活动 / 未活动');
assert.equal(text('executorDetail'), 'TRACKING');
sandbox.updateFlight({...fresh, action_state: 'REJECTED', action_detail: 'REJECTED_EKF_UNHEALTHY'});
assert.equal(text('executorState'), '已拒绝');
assert.equal(element('executorState').className, 'value bad');
assert.equal(text('executorDetail'), 'REJECTED_EKF_UNHEALTHY');
now += 2000;
sandbox.renderExecutor();
assert.equal(text('executorState'), '状态已过期');
for (const id of ['executorAction', 'executorOwner', 'executorActivity', 'executorDetail', 'executorGate']) assert.equal(text(id), '—');
for (const age of [undefined, null, -1, NaN, '0.1']) {
  sandbox.updateFlight({...fresh, guided_executor_age_s: age});
  assert.equal(text('executorState'), '等待状态');
}
sandbox.updateFlight({guided_executor_age_s: .2, control_owner: 'GUIDED_EXECUTOR', mode_gate: 'RC_LOW'});
assert.equal(text('executorState'), '状态已接收');
assert.equal(text('executorAction'), '—');
assert.equal(text('executorDetail'), 'RC_LOW');
sandbox.updateFlight({...fresh, action_detail: '<img src=x onerror=alert(1)>'});
assert.equal(text('executorDetail'), '<img src=x onerror=alert(1)>');
sandbox.updateFlight({});
assert.equal(text('executorState'), '等待状态');
assert.equal(text('executorAction'), '—');
console.log('PASS: executor integration, rejection details, legacy data, stale watchdog, recovery and missing data.');
