# Landing observation interruptions — 2026-10-06

This change fixes two software paths that unnecessarily interrupt landing alignment. It has not yet been deployed or flight-validated.

## Evidence

The second reviewed LAND action ran for about 68 seconds. It spent approximately 32 seconds centring and 31 seconds in target/pose holds. No `GUIDED_TRACK_DESCENT` phase or negative commanded vertical velocity was recorded. The IBVS stream contained 87 HTTP timeouts, 60 stale-frame reports and 18 actual target-not-found reports. Only 2 of 317 new valid landing observations met the 20-pixel centre tolerance; none established a continuous 0.5-second alignment window.

These records describe interrupted alignment before program-authorized descent, rather than completed alignment followed by repeated commanded descent. Aircraft vertical motion and the eventual collision must be investigated separately; low voltage is not established as a cause in this report.

## Changes

- Add `/api/vision/status`, which returns the existing vision snapshot without querying flight telemetry. Both vision adapters now use this endpoint. The console's `/api/status` remains compatible. Previously, a vision request waited for the flight-status service, while both requests had a 150-millisecond timeout.
- A repeated camera sequence remains a duplicate after the adapter's feature-age limit; it is no longer misclassified as a newly rejected stale frame. During an existing guided landing, the executor may keep its original observation only until its existing 300-millisecond deadline. Repeated requests publish no new velocity candidate and renew no observation timestamp. FOLLOW still loses authorization at its existing adapter age limit.
- Actual target loss, rejected new poses, new stale frames, clock reversal and observations exceeding their original deadline still revoke evidence. The strict centre/heading/rate gates, stable interval, descent speed and native LAND handoff are unchanged.
- Preserve both the console executor display and the previously deployed capture-timestamp evidence. The legacy control-age basis is unchanged; the additional sensor timestamp remains separate evidence for the vertical monitor.

## Validation and deployment

The old adapter reproduces the regression: a duplicate at 160 milliseconds revokes still-valid landing evidence. The patched adapter retains the original evidence until its deadline and rejects it after 300 milliseconds. Tests cover frozen sequences, actual loss and new-frame rejection, FOLLOW expiry, timeout recovery and stable-window progress only on new observations.

The camera HTTP tests use the production handler and a real local threaded HTTP server. A blocked or failed flight-status request does not block the independent vision endpoint; the original console endpoint still reports telemetry.

Installation requires the matching camera endpoint and adapter changes. First verify the aircraft is disarmed and flight testing has ended, back up the active files, load the camera service with the new endpoint, then load the adapter source and runtime parameter overrides. These service restarts must not occur in flight. Missing new endpoints fail explicitly; adapters do not silently fall back to the blocking console endpoint. Verify runtime URLs, freshness, failure handling and logged alignment continuity before further flight validation.

Passing these tests does not prove the aircraft will remain centred or that all real image losses have been eliminated. Actual target loss and failure to meet the centre/heading gate are expected to pause descent.
