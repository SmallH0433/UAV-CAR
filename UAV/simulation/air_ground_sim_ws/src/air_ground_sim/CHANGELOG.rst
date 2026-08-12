Changelog for air_ground_sim
============================

0.4.0 (unreleased)
------------------
* Added an independent latched system supervisor, ROS diagnostics, safety events,
  external E-stop heartbeat monitoring and guarded reset.
* Added UGV control-authority arbitration and Nav2 Collision Monitor as the last
  software velocity safety filter.
* Made chassis motion gating fail closed with freshness and emergency watchdogs.
* Hardened the browser gateway with production validation, request/operator IDs,
  constant-time token checks, rate limiting, readiness interlocks and durable audit.
* Added production traceability, safety/security boundaries, fault tests and CI gates.
* Guarded stationary and moving capture with fresh vision, deck-relative range,
  flight-mode/landed state and a time-bounded normal-disarm transition.
* Made standalone command nodes fail closed and removed legacy command paths that
  bypassed authority arbitration or Collision Monitor.
* Added commissioned site-plan gating, configurable real-world/airspace metadata
  and separately authorized flight-mode, LAND, arm and takeoff operations.
* Hardened the local console deployment with same-origin TLS proxying, mTLS device
  identity, writable audit paths, session-only browser secrets and offline fonts.
* Pinned the validated pymavlink dependency and expanded production baseline gates.
* Added a fail-closed physical docking gateway with redundant contact/lock feedback,
  independent motion interlocks, non-extendable operation timeouts and supervision.
* Added map-frame heading and yaw-rate permission for moving-deck descent, explicit
  go-around/fault behavior, distance-based ride deceleration and fast Nav2 failure handling.
* Unified all SIL custom nodes, Nav2 and TF on Gazebo time while retaining monotonic
  wall-clock watchdogs, and added timestamp-validated map-to-base TF tracking.

0.3.0
-----
* Added the complete UAV sensor suite, airspace-aware navigation, visual docking,
  physical Gazebo latching, cooperative mission orchestration and operations console.
