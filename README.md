# kotoba-lang/floorplan-lab

Algorithms for estimating a building's interior floorplan (room shape /
wall outline) from phone-sensor data: dead-reckoning from motion samples,
acoustic chirp-echo sonar, and BLE RSSI trilateration, fused into a wall
point cloud and a simplified room polygon.

Design: `90-docs/adr/2607140600-indoor-floorplan-lab-compiler-phase3-device-capability-bridge.md`
in [`com-junkawasaki/root`](https://github.com/com-junkawasaki/root)
(status `proposed`, Phase 3a implementation authorized by the repo owner).

## Scope: algorithm-only, no real sensors (yet)

**This repo currently contains no native code and talks to no real
sensor.** Every algorithm here operates on plain data structures (motion
sample seqs, PCM float buffers, BLE `{:id :rssi}` readings) and is
verified against **synthetic/mock data** — a synthetic square-room walk
for dead-reckoning, a synthetic chirp + known-delay echo for sonar, and
known beacon positions + exact geometric distances for BLE trilateration.
No CoreMotion / CoreBluetooth / AVAudioEngine / any other native OS API
is called anywhere in this repo, by design (see the ADR's "native shim"
section — that boundary is explicitly out of this repo's scope and is
implemented separately, human-authored or separately-authorized, as a
judgment-free glue layer).

Real-device connection is a **driver injection point**, not a rewrite:
`src/floorplan_lab/sensing_bridge.cljc` defines the driver-map contract
(mirroring `kotoba.sensing-host`'s shape) that a real native shim would
implement; wiring in a real driver there does not require touching
`motion.cljc` / `sonar.cljc` / `ble.cljc` / `fusion.cljc` at all.

## A discrepancy worth knowing about

`sensing_bridge.cljc`'s docstring has the full account, but the short
version: the task that scaffolded this repo was told
`kotoba-lang/device` already registered `:motion`/`:audio-io`/
`:ble-scan`/`:wifi-info` capability tokens wired to
`kotoba-lang/kotoba`'s `kotoba.sensing-host`. Reading the actual
`device.cljc` source at scaffold time showed this is **not** true —
`device` only has `:bluetooth`/`:wifi`/`:display`/`:geolocation`/
`:camera`. `kotoba-core-contracts` (capability ids 234-237) and
`kotoba.sensing-host` (the deterministic-stub driver-dispatch layer) do
exist and are real; the `device`-side integration (ADR-2607140600
section 3) does not. This repo works around that with a locally-defined
driver contract that mirrors `kotoba.sensing-host`'s shape instead of
depending on it directly (see `sensing_bridge.cljc` for the full
reasoning, including why this repo avoids a direct dependency on the
much-heavier `kotoba-lang/kotoba` repo).

## Algorithms

- **`floorplan-lab.motion`** — dead-reckoning: integrates a walking
  trajectory (position + heading) from accelerometer + gyroscope
  samples. Naive double integration (semi-implicit Euler), explicitly
  not a Kalman filter.
- **`floorplan-lab.sonar`** — chirp-echo acoustic sonar: generates a
  linear-sweep chirp, cross-correlates it against a recorded buffer to
  find the round-trip echo delay, converts to a wall distance via
  time-of-flight at the speed of sound (`343 m/s` default).
- **`floorplan-lab.ble`** — BLE RSSI trilateration: converts RSSI to
  distance via the free-space log-distance path-loss model, then
  trilaterates a 2D position from >= 3 known-position beacons via linear
  least squares.
- **`floorplan-lab.fusion`** — combines the above: BLE-corrects a
  dead-reckoning pose (simple weighted average, not a Kalman gain),
  projects sonar readings into world-frame wall points along the
  walker's heading, and estimates a room outline as the convex hull of
  the collected wall points.
- **`floorplan-lab.sensing-bridge`** — the driver injection point
  (mirrors `kotoba.sensing-host`'s driver-map contract) plus a
  `kotoba.lang.device/IDevice` adapter, this repo's one dependency
  (`kotoba-lang/device`).

## Verify

```sh
clojure -M:test
clojure -M:lint
```

## Future work (explicitly out of scope for this first increment)

- Real sensor fusion (EKF/UKF with per-source noise covariance) in place
  of `fusion/correct-with-ble`'s plain weighted average; heading drift
  correction (currently only x/y are corrected).
- Dead-reckoning bias correction / zero-velocity updates (ZUPT) instead
  of raw double integration.
- FFT-based matched filter for chirp-echo detection instead of direct
  O(n·m) cross-correlation; multiple simultaneous echoes / multipath.
- Per-beacon/per-site calibration of the BLE path-loss model constants
  (`ble/default-tx-power-at-1m-dbm`, `ble/default-path-loss-exponent`)
  instead of fixed defaults.
- Concave room outlines (alpha-shape / occupancy-grid) instead of
  `fusion/convex-hull`, which clips L-shaped and other non-convex rooms.
- Wiring `sensing-bridge`'s mirrored driver contract to a real native
  shim (ADR-2607030900 `aiueos-device-provider` pattern) and/or to the
  real `kotoba.sensing-host` once `kotoba-lang/device` registers the
  matching capability surfaces (closing ADR-2607140600 section 3) — see
  `sensing_bridge.cljc`'s docstring for the exact swap point.
- A `kotoba-ui`/`shitsuke.hig`-stack PWA UI (per the ADR's "lab 本体"
  section) — this repo is algorithms only, no UI yet.
