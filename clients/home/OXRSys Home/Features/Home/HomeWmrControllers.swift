// SPDX-License-Identifier: MPL-2.0
//
// Home's side of Windows Mixed Reality motion controllers on a USB Bluetooth adapter: keeps
// wmr_btstack running while the wired headset uses it, and forwards pairing actions.
// See WmrControllerSupport.swift.

import Foundation

extension HomeAppModel {
    var isWmrControllerAdapterActive: Bool {
        serverConfig.wiredHeadset && serverConfig.wiredControllerAdapter
    }

    /// The selected runtime first, then the registered one: the headset helper that uses the
    /// tool runs from whichever runtime the game loads.
    var resolvedWmrControllerToolPath: String? {
        WmrControllerServiceLauncher.toolPath(runtimeManifestPath: activeLaunchRuntimeManifestPath)
            ?? WmrControllerServiceLauncher.toolPath(runtimeManifestPath: activeRuntimePath)
    }

    var supportedUsbBluetoothAdapter: UsbBluetoothAdapter? {
        usbBluetoothAdapters.first { $0.support == .supported }
    }

    /// Runs from the one-second poll: adapters every five seconds, status every tick, and
    /// starts wmr_btstack when the setting is on, an adapter is plugged in, and it isn't running.
    func refreshWmrControllers() {
        guard serverConfig.wiredHeadset else {
            return
        }
        if Date().timeIntervalSince(lastUsbBluetoothAdapterScan) >= 5 {
            rescanUsbBluetoothAdapters()
        }
        let status = WmrControllerServiceStatus.read()
        if wmrControllerService != status {
            wmrControllerService = status
        }
        let toolPath = resolvedWmrControllerToolPath
        if wmrControllerToolPath != toolPath {
            wmrControllerToolPath = toolPath
        }

        guard isWmrControllerAdapterActive,
              status?.isRunning != true,
              supportedUsbBluetoothAdapter != nil,
              toolPath != nil,
              Date().timeIntervalSince(lastWmrControllerServiceStart) >= 10 else {
            return
        }
        startWmrControllerService()
    }

    func rescanUsbBluetoothAdapters() {
        lastUsbBluetoothAdapterScan = Date()
        let adapters = UsbBluetoothAdapterScanner.scan()
        if usbBluetoothAdapters != adapters {
            usbBluetoothAdapters = adapters
        }
    }

    func startWmrControllerService() {
        lastWmrControllerServiceStart = Date()
        guard let toolPath = resolvedWmrControllerToolPath else {
            errorMessage = "wmr_btstack was not found next to the selected runtime. Build drivers/tools/wmr_btstack and copy it next to liboxrsys-runtime.dylib."
            return
        }
        do {
            try WmrControllerServiceLauncher.start(toolPath: toolPath)
            statusMessage = "Started the motion controller service."
        } catch {
            errorMessage = "Could not start wmr_btstack: \(error.localizedDescription)"
        }
    }

    func stopWmrControllerService() {
        guard let status = wmrControllerService, status.isRunning else {
            return
        }
        // Stopped on purpose: don't let the poll restart it right away.
        lastWmrControllerServiceStart = Date()
        WmrControllerServiceLauncher.stop(processID: status.processID)
        statusMessage = "Stopped the motion controller service."
    }

    func restartWmrControllerService() {
        stopWmrControllerService()
        Task { @MainActor [weak self] in
            try? await Task.sleep(for: .seconds(3))
            self?.startWmrControllerService()
        }
    }

    func pairWmrController(seconds: Int = 60) {
        if WmrControllerServiceClient.pair(seconds: seconds) {
            statusMessage = seconds > 0
                ? "Looking for motion controllers in pairing mode for \(seconds) seconds."
                : "Stopped looking for new motion controllers."
        } else {
            errorMessage = "The motion controller service isn't running."
        }
        refreshWmrControllers()
    }

    func forgetWmrControllers() {
        if WmrControllerServiceClient.forgetAll() {
            statusMessage = "Forgot all paired motion controllers."
        } else {
            errorMessage = "The motion controller service isn't running."
        }
        refreshWmrControllers()
    }
}
