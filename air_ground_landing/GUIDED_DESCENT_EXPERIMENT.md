# Companion GUIDED descent — experimental implementation

2026-09-08 status: the policy, executor integration, pilot-session lockout and
ordinary-disarm gate are covered by local unit tests. ROS/SITL, MAVROS
coordinate conversion, actual flight-controller touchdown behaviour and armed
remote-control race testing are not yet accepted. The hardware configuration
keeps `companion_descent_enabled=false`; the separate SITL overlay enables it
only for simulation. Hardware actuation additionally requires explicit flight
approval, while observation-only hardware startup remains available.

## New path

Load config/adapters.guided-descent-sitl.yaml last over the normal SITL config.
This overlay does not itself authorize setpoint/mode outputs. Existing execution
and RC authorization checks still apply. Never relabel real hardware as SITL.

Only ALT_HOLD/LOITER can initiate follow; confirmed GUIDED is required before
CH8 high can request descent. STABILIZE and other nonallowed modes release the
manager. Current code requires fresh RC6 low samples spanning 0.4 seconds,
then high in ALT_HOLD/LOITER, for initial authorization and reentry after override.
CH8 edge semantics have not been changed by this session fix.
The coordinator receives false for native LAND request, retaining IBVS ownership.
There are no native LAND requests, attitude commands, motor writes or forced
DISARM commands in the new execution path. After independently fresh landing
evidence is confirmed for 0.5 seconds, the adapter may issue one ordinary
DISARM request; rejection or confirmation timeout requires operator attention.
Existing LANDING_TARGET telemetry may still be published by its separate
adapter, but does not drive this GUIDED policy.

Inputs required for descent: fresh connected/armed state and RC authorization,
healthy range median, pose, velocity, extended landed state, fresh IBVS candidate
and healthy/aligned IBVS status. Missing telemetry is a hold, never assumed zero.
Tag/candidate freshness limit is 0.3s on this new path, distinct from the old
1s following candidate setting. Velocity/pose arrival-age limit is also 0.3s;
state/extended-state arrival-age limit is 2s. End-to-end sensor timestamp and
transport-delay validation remains required before deployment.

Experimental policy values: alignment dwell 0.4s, measured horizontal speed
<=0.10m/s, tilt <=10 degrees, normal descent 0.10m/s. Healthy range <=0.10m
for 0.4s while aligned enters terminal descent at 0.05m/s, with no Tag tracking.
Unlike the legacy latch, this experimental version requires range evidence and
does not accept small-Tag-only proximity. This conservative limitation avoids
trusting the target-height discrepancies observed in log70.

Before terminal entry, missing Tag immediately requests zero velocity; after
the follow dropout grace, existing mode rollback applies. During terminal
descent, Tag loss alone does not stop descent; lost range/telemetry, excessive
tilt/speed, increasing range or 8s timeout ends descent into a fault hold. Faults
do not automatically resume descent until request/authority is cleared. This is
not a promise that zero velocity is mechanically stationary with a bad EKF.

The output uses LOCAL_NED message type with ROS ENU velocity values as expected
by MAVROS. Body FLU horizontal candidate is yaw-rotated to ENU; vertical command
is independent (negative ROS Z is down), avoiding tilted body-axis descent.
Existing horizontal speed/acceleration limiter applies in earth coordinates.
This conversion must be verified end-to-end in SITL before removing the gate.

On fresh FC ON_GROUND indication, motion commands stop. Ordinary DISARM is
eligible only after the aircraft was observed airborne, terminal descent was
entered, ON_GROUND plus pose/velocity/range evidence remain fresh and safe for
0.5 seconds, and throttle is low. It is sent at most once per authorized
descent session and is never forced. Do not claim autonomous
landing-to-motor-stop acceptance until the full path is validated in hardware.

New executor status includes `descent_backend`, phase, vertical command,
terminal state, landing request, disarm state, pilot-session state, mode and
whether a setpoint was transmitted. The read-only status bridge exposes fresh
outgoing motion commands to the camera console.

Next acceptance gates: isolated ROS startup; CH8 high after heartbeat-confirmed
GUIDED; no LAND request; correct ENU/NED velocity signs at multiple headings;
Tag loss before/after terminal; stale telemetry; RC override; exact touchdown
and disarm sequence. Then prop-off hardware acceptance, before any prop-on use.
