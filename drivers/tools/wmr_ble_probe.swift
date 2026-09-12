// SPDX-License-Identifier: MPL-2.0
//
// Bluetooth LE pairing diagnostic for Windows Mixed Reality motion controllers.
//
// Scans for advertising peripherals, connects to the first "Motion controller"
// (or the name given as the first argument), walks its services, and reads
// the HID Report Map, which is encrypted and therefore forces the pairing
// (bonding) step. Every CoreBluetooth error and the disconnect reason are
// printed, which is what System Settings hides when a controller "just
// disappears" after Connect.
//
//   swift drivers/tools/wmr_ble_probe.swift [name-substring] [seconds]
//
// The first run asks for Bluetooth permission for the terminal.

import CoreBluetooth
import Foundation

let wantedName = CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : "Motion controller"
let seconds = CommandLine.arguments.count > 2 ? Double(CommandLine.arguments[2]) ?? 90 : 90

func ts() -> String {
    let f = DateFormatter()
    f.dateFormat = "HH:mm:ss.SSS"
    return f.string(from: Date())
}

func log(_ s: String) {
    print("[\(ts())] \(s)")
    fflush(stdout)
}

final class Probe: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    var central: CBCentralManager!
    var target: CBPeripheral?
    var seen = Set<UUID>()
    // Anonymous advertisers (no name, no manufacturer data) to identify by
    // connecting and reading their GAP name; a HoG device in pairing mode
    // often looks like this.
    var candidates: [CBPeripheral] = []
    var probing: CBPeripheral?
    var identified = Set<UUID>()

    func probeNextCandidate() {
        guard target == nil, probing == nil, let p = candidates.first else { return }
        candidates.removeFirst()
        probing = p
        p.delegate = self
        log("identifying anonymous \(p.identifier) (rssi seen earlier)")
        central.connect(p, options: nil)
        // Give up on it after a few seconds.
        DispatchQueue.main.asyncAfter(deadline: .now() + 6) { [weak self] in
            guard let self = self, self.probing === p, self.target !== p else { return }
            log("  no answer from \(p.identifier); cancelling")
            self.central.cancelPeripheralConnection(p)
            self.probing = nil
            self.probeNextCandidate()
        }
    }

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: nil)
    }

    func centralManagerDidUpdateState(_ c: CBCentralManager) {
        log("central state: \(c.state.rawValue) (5 = powered on)")
        if c.state == .poweredOn {
            log("scanning for peripherals named like '\(wantedName)' ... put the controller in pairing mode now")
            c.scanForPeripherals(withServices: nil, options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
        } else if c.state == .unauthorized {
            log("Bluetooth permission denied for this process; allow it in System Settings > Privacy & Security > Bluetooth")
        }
    }

    func centralManager(_ c: CBCentralManager, didDiscover p: CBPeripheral, advertisementData: [String: Any], rssi: NSNumber) {
        let name = p.name ?? (advertisementData[CBAdvertisementDataLocalNameKey] as? String) ?? "(no name)"
        if !seen.contains(p.identifier) {
            seen.insert(p.identifier)
            let svcs = (advertisementData[CBAdvertisementDataServiceUUIDsKey] as? [CBUUID])?.map { $0.uuidString } ?? []
            var mfg = ""
            if let d = advertisementData[CBAdvertisementDataManufacturerDataKey] as? Data {
                mfg = " mfg=" + d.map { String(format: "%02x", $0) }.joined()
            }
            log("seen '\(name)' rssi=\(rssi) services=\(svcs)\(mfg)")
            if name == "(no name)" && mfg.isEmpty && svcs.isEmpty && !identified.contains(p.identifier)
                && rssi.intValue > -75 {
                candidates.append(p)
                probeNextCandidate()
            }
        }
        guard target == nil, name.lowercased().contains(wantedName.lowercased()) else { return }
        target = p
        p.delegate = self
        log("connecting to '\(name)' \(p.identifier)")
        c.stopScan()
        c.connect(p, options: nil)
    }

    func centralManager(_ c: CBCentralManager, didConnect p: CBPeripheral) {
        if p === probing {
            log("  connected to anonymous \(p.identifier); reading its name")
            p.discoverServices([CBUUID(string: "1800"), CBUUID(string: "1812")])
            return
        }
        log("connected; discovering services")
        p.discoverServices(nil)
    }

    func centralManager(_ c: CBCentralManager, didFailToConnect p: CBPeripheral, error: Error?) {
        if p === probing {
            log("  anonymous \(p.identifier) did not connect: \(describe(error))")
            probing = nil
            probeNextCandidate()
            return
        }
        log("FAILED to connect: \(describe(error))")
    }

    func centralManager(_ c: CBCentralManager, didDisconnectPeripheral p: CBPeripheral, error: Error?) {
        if p === probing && p !== target {
            probing = nil
            probeNextCandidate()
            return
        }
        log("DISCONNECTED: \(describe(error))")
        target = nil
        probing = nil
        log("scanning again")
        c.scanForPeripherals(withServices: nil, options: nil)
    }

    func peripheral(_ p: CBPeripheral, didDiscoverServices error: Error?) {
        if let e = error { log("service discovery error: \(describe(e))"); return }
        for s in p.services ?? [] {
            log("service \(s.uuid) (\(s.uuid.uuidString))")
            p.discoverCharacteristics(nil, for: s)
        }
    }

    func peripheral(_ p: CBPeripheral, didDiscoverCharacteristicsFor s: CBService, error: Error?) {
        if let e = error { log("characteristic discovery error for \(s.uuid): \(describe(e))"); return }
        let isTarget = p === target
        for ch in s.characteristics ?? [] {
            if isTarget { log("  characteristic \(ch.uuid) props=\(props(ch.properties))") }
            // GAP Device Name is readable without pairing on any device.
            if ch.uuid == CBUUID(string: "2A00") || (isTarget && ch.uuid == CBUUID(string: "2A29")) {
                p.readValue(for: ch)
            }
            guard isTarget else { continue }
            // HID Report Map (2A4B) and Report (2A4D) are encrypted on HoG
            // devices: reading them makes CoreBluetooth start pairing.
            if ch.uuid == CBUUID(string: "2A4B") {
                log("  reading \(ch.uuid) (triggers pairing if encrypted)")
                p.readValue(for: ch)
            }
            if ch.uuid == CBUUID(string: "2A4D"), ch.properties.contains(.notify) {
                log("  subscribing to input report \(ch.uuid)")
                p.setNotifyValue(true, for: ch)
            }
        }
    }

    func peripheral(_ p: CBPeripheral, didUpdateValueFor ch: CBCharacteristic, error: Error?) {
        if let e = error { log("read/notify error on \(ch.uuid): \(describe(e))"); return }
        let bytes = ch.value?.map { String(format: "%02x", $0) }.joined() ?? ""
        let text = ch.value.flatMap { String(data: $0, encoding: .utf8) } ?? ""
        if p === probing && p !== target && ch.uuid == CBUUID(string: "2A00") {
            identified.insert(p.identifier)
            let services = (p.services ?? []).map { $0.uuid.uuidString }.joined(separator: ",")
            log("  anonymous \(p.identifier) is '\(text)' services=[\(services)]")
            if text.lowercased().contains(wantedName.lowercased()) {
                log("that is the controller; continuing with full discovery and pairing")
                target = p
                central.stopScan()
                p.discoverServices(nil)
            } else {
                central.cancelPeripheralConnection(p)
                probing = nil
                probeNextCandidate()
            }
            return
        }
        log("  value \(ch.uuid): \(bytes.prefix(80))\(bytes.count > 80 ? "..." : "") \(text.isEmpty ? "" : "'" + text + "'")")
    }

    func peripheral(_ p: CBPeripheral, didUpdateNotificationStateFor ch: CBCharacteristic, error: Error?) {
        if let e = error { log("notify state error on \(ch.uuid): \(describe(e))"); return }
        log("  notifications \(ch.isNotifying ? "on" : "off") for \(ch.uuid)")
    }

    func describe(_ error: Error?) -> String {
        guard let e = error as NSError? else { return "no error" }
        return "\(e.domain) code \(e.code): \(e.localizedDescription)"
    }

    func props(_ p: CBCharacteristicProperties) -> String {
        var out: [String] = []
        if p.contains(.read) { out.append("read") }
        if p.contains(.write) { out.append("write") }
        if p.contains(.writeWithoutResponse) { out.append("writeNR") }
        if p.contains(.notify) { out.append("notify") }
        if p.contains(.indicate) { out.append("indicate") }
        if p.contains(.authenticatedSignedWrites) { out.append("signed") }
        return out.joined(separator: ",")
    }
}

let probe = Probe()
log("running for \(Int(seconds)) s")
RunLoop.main.run(until: Date(timeIntervalSinceNow: seconds))
log("done")
