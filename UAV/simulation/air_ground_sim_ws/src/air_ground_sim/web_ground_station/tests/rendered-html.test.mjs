import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

async function render() {
  const workerUrl = new URL("../dist/server/index.js", import.meta.url);
  workerUrl.searchParams.set("test", `${process.pid}-${Date.now()}`);
  const { default: worker } = await import(workerUrl.href);

  return worker.fetch(
    new Request("http://localhost/", { headers: { accept: "text/html" } }),
    { ASSETS: { fetch: async () => new Response("Not found", { status: 404 }) } },
    { waitUntil() {}, passThroughOnException() {} },
  );
}

test("server-renders the air-ground operations console", async () => {
  const response = await render();
  assert.equal(response.status, 200);
  assert.match(response.headers.get("content-type") ?? "", /^text\/html\b/i);

  const html = await response.text();
  assert.match(html, /<html lang="zh-CN">/i);
  assert.match(html, /<title>ASTRA 空地联合任务台<\/title>/i);
  assert.match(html, /联合态势 \/ MISSION MAP/);
  assert.match(html, /任务执行链/);
  assert.match(html, /视觉与对接/);
  assert.match(html, /机载感知健康/);
  assert.match(html, /2D LiDAR/);
  assert.match(html, /3D LiDAR/);
  assert.match(html, /双目深度/);
  assert.match(html, /超声·下/);
  assert.match(html, /暂停 Gazebo/);
  assert.match(html, /UGV 按住式点动/);
  assert.match(html, /aria-label="空地协同任务二维态势图"/);
  assert.doesNotMatch(html, /Your site is taking shape|Building your site/);
});

test("keeps live controls, safety gates, and responsive layouts wired", async () => {
  const [page, css, layout, packageJson] = await Promise.all([
    readFile(new URL("../app/page.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/globals.css", import.meta.url), "utf8"),
    readFile(new URL("../app/layout.tsx", import.meta.url), "utf8"),
    readFile(new URL("../package.json", import.meta.url), "utf8"),
  ]);

  assert.match(page, /new EventSource\(`\$\{apiBase\}\/api\/events`\)/);
  assert.match(page, /\/api\/camera\/\$\{name\}\.jpg/);
  assert.match(page, /\/api\/mission\/start/);
  assert.match(page, /\/api\/mission\/abort/);
  assert.match(page, /\/api\/sim\/pause/);
  assert.match(page, /\/api\/ugv\/teleop/);
  assert.match(page, /\/api\/safety\/emergency-stop/);
  assert.match(page, /\/api\/safety\/reset/);
  assert.match(page, /X-Request-ID/);
  assert.match(page, /X-Operator-ID/);
  assert.match(page, /systemReady && !systemEmergency|!systemReady \|\| systemEmergency/);
  assert.match(page, /pause_active_mission_before_teleop|missionActive && !missionPaused/);
  assert.match(page, /teleopStopPending/);
  assert.match(page, /onPointerCancel=\{stopTeleop\}/);
  assert.match(page, /X-Control-Token/);
  assert.match(page, /aria-modal="true"/);

  assert.match(css, /touch-action:\s*none/);
  assert.match(css, /@media \(max-width:\s*1180px\)/);
  assert.match(css, /@media \(max-width:\s*820px\)/);
  assert.match(css, /@media \(max-width:\s*540px\)/);
  assert.match(layout, /lang="zh-CN"/);
  assert.match(layout, /ASTRA 空地联合任务台/);
  assert.match(packageJson, /"lint":\s*"eslint/);
});
