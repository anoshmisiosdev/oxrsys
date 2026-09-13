// SPDX-License-Identifier: MPL-2.0
//
// oxrsys-wmr-pair: find, pair and connect Windows Mixed Reality motion controllers.
//
// The original WMR controllers (Acer/Dell/HP/Lenovo/Samsung, "Motion controller -
// Left/Right") are Bluetooth Classic HID devices with Secure Simple Pairing. macOS's
// normal pairing path (System Settings, IOBluetoothDevicePair) runs SDP before it
// authenticates; the controller stalls mid-SDP and drops the link after ~5 s, so the
// pairing never starts and the controller "just disappears". This tool pages the
// controller with authentication required up front (the order Windows uses), accepts
// the Just Works / numeric-comparison confirmation, then lets service discovery run.
//
// Modes:
//   oxrsys-wmr-pair                 scan, pair every controller in pairing mode, keep
//                                   paired ones connected; runs until Ctrl-C
//   oxrsys-wmr-pair --once          exit after the first controller pairs
//   oxrsys-wmr-pair --list          list known/paired controllers and exit
//   oxrsys-wmr-pair --unpair ADDR   forget a controller
//   oxrsys-wmr-pair --strategy pair use IOBluetoothDevicePair only (for comparison)
//
// Put a controller in pairing mode: battery cover off, hold the small button by the
// batteries until the LED ring flashes.

import Foundation
import IOBluetooth
import IOKit.hid

let kMicrosoftVID = 0x045E
let kControllerPIDs: Set<Int> = [0x065B /* WMR */, 0x065D /* Odyssey */, 0x066A /* Reverb G2 */]
let kNamePrefix = "Motion controller"

func ts() -> String {
    let f = DateFormatter()
    f.dateFormat = "HH:mm:ss.SSS"
    return f.string(from: Date())
}

func log(_ s: String) {
    print("[\(ts())] \(s)")
    fflush(stdout)
}

func ioReturnString(_ r: IOReturn) -> String {
    if r == kIOReturnSuccess { return "success" }
    if let c = mach_error_string(r) { return String(format: "0x%08x (%@)", UInt32(bitPattern: r), String(cString: c)) }
    return String(format: "0x%08x", UInt32(bitPattern: r))
}

func isController(_ d: IOBluetoothDevice) -> Bool {
    return (d.name ?? "").hasPrefix(kNamePrefix)
}

// MARK: - HID presence (what the runtime's hidapi enumeration will see)

func hidControllersPresent() -> [String] {
    let mgr = IOHIDManagerCreate(kCFAllocatorDefault, IOOptionBits(kIOHIDOptionsTypeNone))
    IOHIDManagerSetDeviceMatching(mgr, [kIOHIDVendorIDKey: kMicrosoftVID] as CFDictionary)
    guard let set = IOHIDManagerCopyDevices(mgr) as? Set<IOHIDDevice> else { return [] }
    var out: [String] = []
    for dev in set {
        let pid = (IOHIDDeviceGetProperty(dev, kIOHIDProductIDKey as CFString) as? Int) ?? 0
        guard kControllerPIDs.contains(pid) else { continue }
        let name = (IOHIDDeviceGetProperty(dev, kIOHIDProductKey as CFString) as? String) ?? "?"
        let transport = (IOHIDDeviceGetProperty(dev, kIOHIDTransportKey as CFString) as? String) ?? "?"
        let serial = (IOHIDDeviceGetProperty(dev, kIOHIDSerialNumberKey as CFString) as? String) ?? ""
        out.append(String(format: "%@ pid=0x%04x transport=%@ %@", name, pid, transport, serial))
    }
    return out
}

// MARK: - Pairing

final class Pairer: NSObject, IOBluetoothDeviceInquiryDelegate, IOBluetoothDevicePairDelegate {
    let once: Bool
    let strategy: String
    var inquiry: IOBluetoothDeviceInquiry?
    var busy = Set<String>()          // addresses currently being paired
    var pairs: [String: IOBluetoothDevicePair] = [:]
    var failures: [String: Int] = [:]

    init(once: Bool, strategy: String) {
        self.once = once
        self.strategy = strategy
        super.init()
    }

    func start() {
        reconnectPaired()
        startInquiry()
        // Periodically re-check paired controllers (they reconnect when switched on).
        Timer.scheduledTimer(withTimeInterval: 5, repeats: true) { [weak self] _ in self?.reconnectPaired() }
    }

    func startInquiry() {
        let inq = IOBluetoothDeviceInquiry(delegate: self)!
        inq.updateNewDeviceNames = true
        inq.inquiryLength = 8
        inquiry = inq
        let r = inq.start()
        if r != kIOReturnSuccess { log("inquiry start failed: \(ioReturnString(r))") }
    }

    func reconnectPaired() {
        for case let d as IOBluetoothDevice in (IOBluetoothDevice.pairedDevices() ?? []) where isController(d) {
            if !d.isConnected() && !busy.contains(d.addressString) {
                // Controllers page the host themselves when woken; a host-side open
                // only succeeds while they're on. Quietly try.
                if d.openConnection(nil, withPageTimeout: 0x1000, authenticationRequired: true) == kIOReturnSuccess {
                    log("reconnecting paired '\(d.name ?? "?")' \(d.addressString)")
                }
            }
        }
    }

    // Inquiry delegate

    func deviceInquiryDeviceFound(_ sender: IOBluetoothDeviceInquiry!, device: IOBluetoothDevice!) {
        consider(device, why: "found")
    }

    func deviceInquiryDeviceNameUpdated(_ sender: IOBluetoothDeviceInquiry!, device: IOBluetoothDevice!, devicesRemaining: UInt32) {
        consider(device, why: "name")
    }

    func deviceInquiryComplete(_ sender: IOBluetoothDeviceInquiry!, error: IOReturn, aborted: Bool) {
        // Restart immediately so a controller put in pairing mode later is caught.
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { [weak self] in self?.startInquiry() }
    }

    func consider(_ d: IOBluetoothDevice, why: String) {
        guard isController(d) else { return }
        let addr = d.addressString ?? "?"
        if d.isPaired() {
            if !d.isConnected() { log("'\(d.name!)' \(addr) is paired; connecting") ; _ = d.openConnection() }
            return
        }
        guard !busy.contains(addr) else { return }
        if (failures[addr] ?? 0) >= 4 { return }
        busy.insert(addr)
        log("pairing-mode controller '\(d.name!)' \(addr) (\(why)); pairing with strategy '\(strategy)'")
        inquiry?.stop()   // paging and inquiry share the radio; stop scanning while pairing
        if strategy == "pair" {
            startDevicePair(d)
        } else if strategy == "race" {
            raceAuth(d)
        } else {
            authFirst(d)
        }
    }

    // Strategy "race": let IOBluetoothDevicePair page the controller (its agent answers
    // SSP confirmation), and the instant the ACL link appears request authentication
    // ourselves, so LMP pairing starts while bluetoothd is still on its first SDP
    // request instead of after the whole SDP pass the controller doesn't survive.
    var raceTarget: IOBluetoothDevice?
    var raceTimer: Timer?
    var raceFired = false
    var connectNote: IOBluetoothUserNotification?

    func raceAuth(_ d: IOBluetoothDevice) {
        raceTarget = d
        raceFired = false
        if connectNote == nil {
            connectNote = IOBluetoothDevice.register(forConnectNotifications: self, selector: #selector(deviceConnected(_:device:)))
        }
        // Notifications are delivered via the run loop; also poll in case they lag.
        raceTimer = Timer.scheduledTimer(withTimeInterval: 0.002, repeats: true) { [weak self] _ in
            guard let self = self, let t = self.raceTarget, t.isConnected() else { return }
            self.fireAuth(t, via: "poll")
        }
        startDevicePair(d)
    }

    @objc func deviceConnected(_ note: IOBluetoothUserNotification, device: IOBluetoothDevice) {
        guard let t = raceTarget, device.addressString == t.addressString else { return }
        fireAuth(device, via: "notification")
    }

    func fireAuth(_ d: IOBluetoothDevice, via: String) {
        guard !raceFired else { return }
        raceFired = true
        raceTimer?.invalidate()
        log("  link up (\(via)); requesting authentication immediately")
        let a = d.requestAuthentication()
        log("  requestAuthentication -> \(ioReturnString(a)); paired=\(d.isPaired()) encryption=\(d.getEncryptionMode())")
        raceTarget = nil
    }

    // Strategy "auth" (default): baseband connection with authentication required, so
    // SSP runs before any SDP traffic; then IOBluetoothDevicePair to record the pairing
    // if the link-level bond didn't already mark it paired.
    func authFirst(_ d: IOBluetoothDevice) {
        let addr = d.addressString!
        log("  paging \(addr) with authenticationRequired=YES")
        let r = d.openConnection(nil, withPageTimeout: 0x2000, authenticationRequired: true)
        log("  openConnection(auth) -> \(ioReturnString(r)); connected=\(d.isConnected()) paired=\(d.isPaired()) encryption=\(d.getEncryptionMode())")
        if r == kIOReturnSuccess && d.isPaired() {
            finished(d, ok: true, note: "bonded during authenticated connect")
            return
        }
        if d.isConnected() {
            // Link is up; ask for authentication explicitly before anything else happens.
            let a = d.requestAuthentication()
            log("  requestAuthentication -> \(ioReturnString(a)); paired=\(d.isPaired())")
            if a == kIOReturnSuccess && d.isPaired() {
                finished(d, ok: true, note: "bonded via requestAuthentication")
                return
            }
        }
        // Fall back to the pairing object, which also handles confirmation requests.
        startDevicePair(d)
    }

    func startDevicePair(_ d: IOBluetoothDevice) {
        let addr = d.addressString!
        guard let p = IOBluetoothDevicePair(device: d) else {
            finished(d, ok: false, note: "could not create IOBluetoothDevicePair")
            return
        }
        p.delegate = self
        pairs[addr] = p
        let r = p.start()
        log("  IOBluetoothDevicePair.start -> \(ioReturnString(r))")
        if r != kIOReturnSuccess { finished(d, ok: false, note: "pair start failed") }
    }

    // Pair delegate

    func devicePairingStarted(_ sender: Any!) { log("  pairing started") }
    func devicePairingConnecting(_ sender: Any!) { log("  pairing: connecting") }
    func devicePairingConnected(_ sender: Any!) { log("  pairing: connected") }

    func devicePairingPINCodeRequest(_ sender: Any!) {
        log("  PIN requested; replying 0000")
        var pin = BluetoothPINCode()
        withUnsafeMutableBytes(of: &pin) { buf in for (i, c) in "0000".utf8.enumerated() { buf[i] = c } }
        (sender as? IOBluetoothDevicePair)?.replyPINCode(4, pinCode: &pin)
    }

    func devicePairingUserConfirmationRequest(_ sender: Any!, numericValue: BluetoothNumericValue) {
        log("  SSP confirmation requested (\(numericValue)); accepting")
        (sender as? IOBluetoothDevicePair)?.replyUserConfirmation(true)
    }

    func devicePairingUserPasskeyNotification(_ sender: Any!, passkey: BluetoothPasskey) {
        log("  passkey notification \(passkey)")
    }

    func deviceSimplePairingComplete(_ sender: Any!, status: BluetoothHCIEventStatus) {
        log("  simple pairing complete, HCI status 0x\(String(status, radix: 16))")
    }

    func devicePairingFinished(_ sender: Any!, error: IOReturn) {
        guard let p = sender as? IOBluetoothDevicePair, let d = p.device() else { return }
        pairs[d.addressString] = nil
        finished(d, ok: error == kIOReturnSuccess && d.isPaired(), note: "IOBluetoothDevicePair finished: \(ioReturnString(error))")
    }

    func finished(_ d: IOBluetoothDevice, ok: Bool, note: String) {
        let addr = d.addressString ?? "?"
        busy.remove(addr)
        if ok {
            log("PAIRED '\(d.name ?? "?")' \(addr) — \(note)")
            if !d.isConnected() { _ = d.openConnection() }
            // Give the HID stack a moment to publish the device.
            DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in
                let hid = hidControllersPresent()
                if hid.isEmpty {
                    log("  warning: paired but no Microsoft motion-controller HID device yet (press the Windows button to wake it)")
                } else {
                    hid.forEach { log("  HID ready: \($0)") }
                }
                if self?.once == true { exit(0) }
                self?.startInquiry()
            }
        } else {
            failures[addr, default: 0] += 1
            log("FAILED '\(d.name ?? "?")' \(addr) — \(note) (attempt \(failures[addr]!)/4); still scanning")
            startInquiry()
        }
    }
}

// MARK: - main

let args = Array(CommandLine.arguments.dropFirst())

if args.contains("--help") || args.contains("-h") {
    print("usage: oxrsys-wmr-pair [--once] [--list] [--unpair ADDR] [--strategy auth|pair]")
    exit(0)
}

if args.contains("--list") {
    let known = (IOBluetoothDevice.pairedDevices() ?? []).compactMap { $0 as? IOBluetoothDevice }.filter(isController)
    if known.isEmpty { print("no paired motion controllers") }
    for d in known {
        print("\(d.addressString ?? "?")  '\(d.name ?? "?")'  paired=\(d.isPaired()) connected=\(d.isConnected())")
    }
    let hid = hidControllersPresent()
    print(hid.isEmpty ? "no motion-controller HID devices" : hid.map { "HID: " + $0 }.joined(separator: "\n"))
    exit(0)
}

if let i = args.firstIndex(of: "--unpair"), i + 1 < args.count {
    guard let d = IOBluetoothDevice(addressString: args[i + 1]) else { print("bad address"); exit(2) }
    // IOBluetoothDevice has no public unpair; the private `remove` selector is what
    // System Settings' "Forget" uses.
    let sel = Selector(("remove"))
    if d.responds(to: sel) { d.perform(sel); print("forgot \(args[i + 1])") } else { print("unpair not available; use System Settings") }
    exit(0)
}

let strategy = args.firstIndex(of: "--strategy").flatMap { $0 + 1 < args.count ? args[$0 + 1] : nil } ?? "auth"
let pairer = Pairer(once: args.contains("--once"), strategy: strategy)

guard IOBluetoothHostController.default()?.powerState == kBluetoothHCIPowerStateON else {
    log("Bluetooth is off")
    exit(1)
}
log("scanning for WMR motion controllers (strategy '\(strategy)'); put one in pairing mode: battery cover off, hold the button until the LEDs flash")
pairer.start()
signal(SIGINT) { _ in print(""); exit(0) }
RunLoop.main.run()
