// SPDX-License-Identifier: MPL-2.0
//
// Windows Mixed Reality motion controllers through a USB Bluetooth adapter.
//
// macOS's Bluetooth stack can't pair 1st-gen WMR motion controllers, so OXRSys can drive a
// separate USB Bluetooth adapter with BTstack instead (drivers/tools/wmr_btstack). This file
// holds what Home needs for that: finding adapters over IOKit, reading the tool's status
// file, sending it commands over its socket, and locating/starting the tool.
// Protocol: drivers/monado/wmr_bt_bridge_protocol.h.

import Darwin
import Foundation
import IOKit

struct UsbBluetoothAdapter: Identifiable, Equatable {
    enum Support: Equatable {
        case supported
        /// Realtek and Intel adapters need a vendor firmware upload that wmr_btstack doesn't do.
        case needsFirmware
    }

    let locationID: UInt32
    let vendorID: Int
    let productID: Int
    let name: String

    var id: UInt32 { locationID }

    var usbID: String {
        String(format: "%04x:%04x", vendorID, productID)
    }

    var support: Support {
        switch vendorID {
        case 0x0bda, 0x8087:
            return .needsFirmware
        default:
            return .supported
        }
    }
}

enum UsbBluetoothAdapterScanner {
    private static let appleVendorID = 0x05ac

    /// USB Bluetooth HCI devices: class E0, subclass 01, protocol 01, either on the device itself
    /// (single-function dongles such as the CSR8510, which get no interface entries in the
    /// registry until something opens them) or on one of its interfaces (composite adapters).
    static func scan() -> [UsbBluetoothAdapter] {
        guard let matching = IOServiceMatching("IOUSBHostDevice") else {
            return []
        }
        var iterator: io_iterator_t = 0
        guard IOServiceGetMatchingServices(kIOMainPortDefault, matching, &iterator) == KERN_SUCCESS else {
            return []
        }
        defer { IOObjectRelease(iterator) }

        var adapters: [UsbBluetoothAdapter] = []
        var device = IOIteratorNext(iterator)
        while device != 0 {
            defer {
                IOObjectRelease(device)
                device = IOIteratorNext(iterator)
            }
            guard let vendorID = intProperty(device, "idVendor"),
                  let productID = intProperty(device, "idProduct"),
                  vendorID != appleVendorID,
                  isBluetoothHci(device, prefix: "bDevice") || hasBluetoothHciInterface(device) else {
                continue
            }
            adapters.append(UsbBluetoothAdapter(
                locationID: UInt32(truncatingIfNeeded: intProperty(device, "locationID") ?? 0),
                vendorID: vendorID,
                productID: productID,
                name: stringProperty(device, "USB Product Name")
                    ?? stringProperty(device, "kUSBProductString")
                    ?? "USB Bluetooth adapter"
            ))
        }
        return adapters.sorted { $0.locationID < $1.locationID }
    }

    private static func isBluetoothHci(_ entry: io_registry_entry_t, prefix: String) -> Bool {
        intProperty(entry, prefix + "Class") == 0xE0 &&
            intProperty(entry, prefix + "SubClass") == 0x01 &&
            intProperty(entry, prefix + "Protocol") == 0x01
    }

    private static func hasBluetoothHciInterface(_ device: io_registry_entry_t) -> Bool {
        var children: io_iterator_t = 0
        guard IORegistryEntryGetChildIterator(device, kIOServicePlane, &children) == KERN_SUCCESS else {
            return false
        }
        defer { IOObjectRelease(children) }
        var child = IOIteratorNext(children)
        while child != 0 {
            defer {
                IOObjectRelease(child)
                child = IOIteratorNext(children)
            }
            if IOObjectConformsTo(child, "IOUSBHostInterface") != 0 && isBluetoothHci(child, prefix: "bInterface") {
                return true
            }
        }
        return false
    }

    private static func intProperty(_ entry: io_registry_entry_t, _ key: String) -> Int? {
        guard let value = IORegistryEntryCreateCFProperty(entry, key as CFString, kCFAllocatorDefault, 0)?
            .takeRetainedValue() else {
            return nil
        }
        return (value as? NSNumber)?.intValue
    }

    private static func stringProperty(_ entry: io_registry_entry_t, _ key: String) -> String? {
        IORegistryEntryCreateCFProperty(entry, key as CFString, kCFAllocatorDefault, 0)?
            .takeRetainedValue() as? String
    }
}

/// wmr_btstack's wmr_controllers_status.json.
struct WmrControllerServiceStatus: Equatable {
    enum AdapterState: String {
        case ready
        case noAdapter = "no_adapter"
    }

    struct Controller: Equatable, Identifiable {
        enum State: String {
            case pairing
            case connecting
            case connected
        }

        let hand: String
        let name: String
        let address: String
        let state: State
        let reportsPerSecond: Int

        var id: String { address }
    }

    struct PairedController: Equatable, Identifiable {
        let hand: String
        let name: String
        let address: String

        var id: String { address }
    }

    var processID = 0
    var isRunning = false
    var adapterState: AdapterState = .ready
    var adapterAddress = ""
    var pairingSecondsLeft = 0
    var runtimeAttached: [String] = []
    var controllers: [Controller] = []
    var paired: [PairedController] = []

    static let statusPath = (HomePaths.appSupportDirectory as NSString)
        .appendingPathComponent("wmr_controllers_status.json")

    /// nil when wmr_btstack has never run or its status file is gone.
    static func read(from path: String = statusPath) -> WmrControllerServiceStatus? {
        guard let data = try? Data(contentsOf: URL(fileURLWithPath: path)) else {
            return nil
        }
        return parse(data)
    }

    static func parse(_ data: Data) -> WmrControllerServiceStatus? {
        guard let object = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else {
            return nil
        }
        var status = WmrControllerServiceStatus()
        status.processID = (object["process_id"] as? NSNumber)?.intValue ?? 0
        status.adapterState = (object["adapter_state"] as? String).flatMap(AdapterState.init(rawValue:)) ?? .ready
        status.isRunning = status.adapterState == .ready && isProcessRunning(status.processID)
        status.adapterAddress = object["adapter_address"] as? String ?? ""
        status.pairingSecondsLeft = (object["pairing_seconds_left"] as? NSNumber)?.intValue ?? 0
        status.runtimeAttached = object["runtime_attached"] as? [String] ?? []
        status.controllers = (object["controllers"] as? [[String: Any]] ?? []).compactMap { entry in
            guard let hand = entry["hand"] as? String,
                  let address = entry["address"] as? String,
                  let state = (entry["state"] as? String).flatMap(Controller.State.init(rawValue:)) else {
                return nil
            }
            return Controller(
                hand: hand,
                name: entry["name"] as? String ?? "Motion controller",
                address: address,
                state: state,
                reportsPerSecond: (entry["reports_per_second"] as? NSNumber)?.intValue ?? 0
            )
        }
        status.paired = (object["paired"] as? [[String: Any]] ?? []).compactMap { entry in
            guard let hand = entry["hand"] as? String, let address = entry["address"] as? String else {
                return nil
            }
            return PairedController(hand: hand, name: entry["name"] as? String ?? "Motion controller", address: address)
        }
        return status
    }

    func controller(hand: String) -> Controller? {
        controllers.first { $0.hand == hand }
    }

    func pairedController(hand: String) -> PairedController? {
        paired.first { $0.hand == hand }
    }

    private static func isProcessRunning(_ processID: Int) -> Bool {
        guard processID > 0 else {
            return false
        }
        return kill(pid_t(processID), 0) == 0 || errno == EPERM
    }
}

enum WmrControllerServiceClient {
    private static let pairMessage: UInt8 = 0x50   // 'P'
    private static let forgetMessage: UInt8 = 0x46 // 'F'

    static var socketPath: String {
        (HomePaths.appSupportDirectory as NSString).appendingPathComponent("wmr_bt.sock")
    }

    /// Look for controllers in pairing mode for `seconds` (0 stops).
    @discardableResult
    static func pair(seconds: Int) -> Bool {
        let value = UInt16(clamping: seconds)
        return send(type: pairMessage, payload: [UInt8(value & 0xff), UInt8(value >> 8)])
    }

    @discardableResult
    static func forgetAll() -> Bool {
        send(type: forgetMessage, payload: [])
    }

    private static func send(type: UInt8, payload: [UInt8]) -> Bool {
        let fd = socket(AF_UNIX, SOCK_STREAM, 0)
        guard fd >= 0 else {
            return false
        }
        defer { close(fd) }

        var address = sockaddr_un()
        address.sun_family = sa_family_t(AF_UNIX)
        let pathBytes = Array(socketPath.utf8)
        let capacity = MemoryLayout.size(ofValue: address.sun_path)
        guard pathBytes.count < capacity else {
            return false
        }
        withUnsafeMutableBytes(of: &address.sun_path) { buffer in
            buffer.copyBytes(from: pathBytes)
            buffer[pathBytes.count] = 0
        }
        let connected = withUnsafePointer(to: &address) { pointer in
            pointer.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                connect(fd, $0, socklen_t(MemoryLayout<sockaddr_un>.size))
            }
        }
        guard connected == 0 else {
            return false
        }

        let length = UInt16(payload.count)
        let frame = [type, 0, UInt8(length & 0xff), UInt8(length >> 8)] + payload
        return frame.withUnsafeBytes { Darwin.send(fd, $0.baseAddress, $0.count, 0) } == frame.count
    }
}

enum WmrControllerServiceLauncher {
    static let toolName = "wmr_btstack"
    static let logPath = (HomePaths.appSupportDirectory as NSString).appendingPathComponent("wmr_btstack.log")

    /// wmr_btstack next to the selected runtime's dylib, where the headset helper also looks for it.
    static func toolPath(runtimeManifestPath: String) -> String? {
        guard let data = try? Data(contentsOf: URL(fileURLWithPath: runtimeManifestPath)),
              let object = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let runtime = object["runtime"] as? [String: Any],
              var libraryPath = runtime["library_path"] as? String else {
            return nil
        }
        if !libraryPath.hasPrefix("/") {
            let manifestDirectory = (runtimeManifestPath as NSString).deletingLastPathComponent
            libraryPath = (manifestDirectory as NSString).appendingPathComponent(libraryPath)
        }
        let path = ((libraryPath as NSString).deletingLastPathComponent as NSString).appendingPathComponent(toolName)
        return FileManager.default.isExecutableFile(atPath: path) ? path : nil
    }

    /// Starts wmr_btstack in reconnect-only mode, detached so it keeps the adapter after Home quits.
    static func start(toolPath: String) throws {
        let fileManager = FileManager.default
        try? fileManager.createDirectory(atPath: HomePaths.appSupportDirectory, withIntermediateDirectories: true)
        if !fileManager.fileExists(atPath: logPath) {
            fileManager.createFile(atPath: logPath, contents: nil)
        }
        let log = try FileHandle(forWritingTo: URL(fileURLWithPath: logPath))
        log.seekToEndOfFile()

        let process = Process()
        process.executableURL = URL(fileURLWithPath: toolPath)
        process.arguments = ["-p", "0"]
        process.standardOutput = log
        process.standardError = log
        process.standardInput = FileHandle.nullDevice
        try process.run()
    }

    static func stop(processID: Int) {
        guard processID > 0 else {
            return
        }
        kill(pid_t(processID), SIGINT)
    }
}
