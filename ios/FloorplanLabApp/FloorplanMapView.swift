import SwiftUI
import WebKit

/// Thin `WKWebView` host for the `web/` cljs 3D viewer
/// (`floorplan-lab.viewer`, rendering through `kotoba-lang/webgpu`'s
/// declarative render-IR executor). ALL floorplan estimation
/// (dead-reckoning / chirp-echo sonar / BLE trilateration / sensor
/// fusion) runs in that cljs bundle, not here -- see
/// `web/src/floorplan_lab/viewer.cljs`'s docstring "RESPONSIBILITY
/// BOUNDARY". This file has exactly two jobs:
///
///   1. Load `public/index.html` (bundled as a folder resource -- see
///      `ios/project.yml`'s `public` source entry, built by
///      `web/README`'s `clojure -M:build` + `npx shadow-cljs release app`)
///      into a `WKWebView`.
///   2. Periodically pull RAW samples from `SensingBridge` and hand them,
///      completely unprocessed, to `window.FloorplanLabViewer`'s ingest
///      functions (`ingestMotion`/`ingestSonar`/`ingestBle`) via
///      `evaluateJavaScript`. No judgment, no filtering, no fusion math
///      happens in this file -- it only reshapes Swift arrays/tuples into
///      JS literal source text.
struct FloorplanMapView: UIViewRepresentable {
    /// Known BLE beacon positions (peripheral id -> world [x y] meters),
    /// this lab's own beacon survey/config -- out of scope for
    /// auto-discovery, same caveat as
    /// `floorplan-lab.sensing-bridge/ble-readings->beacons`'s doc
    /// comment. Empty by default: `ingestBle` still runs every scan, it
    /// just finds zero known-position beacons, so BLE correction simply
    /// never fires until this is configured for a real site.
    var bleBeaconPositions: [String: (x: Double, y: Double)] = [:]

    func makeUIView(context: Context) -> WKWebView {
        let webView = WKWebView(frame: .zero)
        webView.isOpaque = false
        webView.backgroundColor = .black

        if let indexURL = Bundle.main.url(forResource: "index", withExtension: "html", subdirectory: "public") {
            webView.loadFileURL(indexURL, allowingReadAccessTo: indexURL.deletingLastPathComponent())
        } else {
            webView.loadHTMLString(
                "<p style=\"font-family:-apple-system;padding:24px\">public/index.html not found in the app bundle "
                    + "-- build web/ first (see web/README section of the repo README: "
                    + "`clojure -M:build` then `npx shadow-cljs release app` from web/), then re-run "
                    + "`xcodegen generate`.</p>",
                baseURL: nil
            )
        }

        context.coordinator.webView = webView
        context.coordinator.start()
        return webView
    }

    func updateUIView(_ uiView: WKWebView, context: Context) {}

    func makeCoordinator() -> Coordinator {
        Coordinator(bleBeaconPositions: bleBeaconPositions)
    }

    /// Owns the raw-sensor polling loops. `@MainActor` so the 3 loops
    /// below (motion/sonar/ble) never truly run concurrently against each
    /// other or `sampleIndex` -- Swift concurrency interleaves them at
    /// `await` points but never overlaps two bodies on the same actor.
    @MainActor
    final class Coordinator {
        weak var webView: WKWebView?
        private let bleBeaconPositions: [String: (x: Double, y: Double)]
        private var motionTask: Task<Void, Never>?
        private var sonarTask: Task<Void, Never>?
        private var bleTask: Task<Void, Never>?

        /// A running count of motion samples ingested so far -- handed to
        /// `ingestSonar` as `sampleIndex` (which trajectory pose, per
        /// `floorplan-lab.motion/walk-trajectory`'s indexing contract, a
        /// ping was fired from). This is only an APPROXIMATION of the
        /// cljs-side trajectory length at the instant a ping fires
        /// (`evaluateJavaScript` calls are async and cljs-side state
        /// updates asynchronously relative to this counter) -- acceptable
        /// for this lab per the task's own "implementation grain/timing
        /// is your call" allowance; a production version would instead
        /// have cljs echo back its own current sample count.
        private var sampleIndex = 0

        init(bleBeaconPositions: [String: (x: Double, y: Double)]) {
            self.bleBeaconPositions = bleBeaconPositions
        }

        func start() {
            motionTask = Task { await self.motionLoop() }
            sonarTask = Task { await self.sonarLoop() }
            bleTask = Task { await self.bleLoop() }
        }

        func stop() {
            motionTask?.cancel()
            sonarTask?.cancel()
            bleTask?.cancel()
        }

        // MARK: - raw-data polling loops (no fusion/estimation here)

        /// Raw motion samples at ~20Hz -- Swift's own polling interval,
        /// not a fusion decision (dead-reckoning integration happens
        /// entirely in cljs's `floorplan-lab.motion/walk-trajectory`).
        private func motionLoop() async {
            let dtSeconds = 0.05
            while !Task.isCancelled {
                let raw9 = await SensingBridge.motionRead()
                sampleIndex += 1
                callJS("window.FloorplanLabViewer && window.FloorplanLabViewer.ingestMotion(\(jsArray(raw9)), \(dtSeconds));")
                try? await Task.sleep(nanoseconds: UInt64(dtSeconds * 1_000_000_000))
            }
        }

        /// One chirp-echo ping every 3s: play a chirp, record the echo
        /// window, hand BOTH raw buffers to cljs (`floorplan-lab.sonar`
        /// does the cross-correlation / time-of-flight math there, not
        /// here).
        private func sonarLoop() async {
            let frequencyHz = 2_000.0
            let playDurationMs = 300
            let recordDurationMs = 500 // >= play duration, so the echo has room to land
            let sampleRateHz = 44_100.0
            // Let the first few motion samples land before the first ping
            // fires, so `sampleIndex` isn't 0 for the whole first window.
            try? await Task.sleep(nanoseconds: 500_000_000)
            while !Task.isCancelled {
                let chirp = SensingBridge.audioPlay(frequencyHz: frequencyHz, durationMs: playDurationMs)
                let recorded = await SensingBridge.audioRecord(durationMs: recordDurationMs)
                let firedAtIndex = sampleIndex
                callJS(
                    "window.FloorplanLabViewer && window.FloorplanLabViewer.ingestSonar("
                        + "\(jsArray(chirp)), \(jsArray(recorded)), \(sampleRateHz), \(firedAtIndex));"
                )
                try? await Task.sleep(nanoseconds: 3_000_000_000)
            }
        }

        /// One BLE scan every 5s.
        private func bleLoop() async {
            while !Task.isCancelled {
                let results = await SensingBridge.bleScan(durationMs: 2_000)
                let readingsJS = results
                    .map { "{id:\(jsString($0.peripheralId)),rssi:\($0.rssi)}" }
                    .joined(separator: ",")
                let beaconsJS = bleBeaconPositions
                    .map { id, pos in "{id:\(jsString(id)),x:\(pos.x),y:\(pos.y)}" }
                    .joined(separator: ",")
                callJS(
                    "window.FloorplanLabViewer && window.FloorplanLabViewer.ingestBle([\(readingsJS)],[\(beaconsJS)]);"
                )
                try? await Task.sleep(nanoseconds: 5_000_000_000)
            }
        }

        // MARK: - JS bridge plumbing (no sensing/fusion decisions below this line)

        private func callJS(_ script: String) {
            webView?.evaluateJavaScript(script) { _, error in
                if let error {
                    print("[FloorplanMapView] evaluateJavaScript error: \(error)")
                }
            }
        }

        private func jsArray(_ values: [Double]) -> String {
            "[" + values.map { String($0) }.joined(separator: ",") + "]"
        }

        private func jsArray(_ values: [Float]) -> String {
            "[" + values.map { String($0) }.joined(separator: ",") + "]"
        }

        private func jsString(_ s: String) -> String {
            "\"" + s.replacingOccurrences(of: "\"", with: "\\\"") + "\""
        }
    }
}

#Preview {
    FloorplanMapView()
}
