// SPDX-License-Identifier: MPL-2.0

import Foundation

struct OXRSysServerConfig: Equatable {
    static let minBitrateMbps = 1
    static let maxBitrateMbps = 200
    static let supportedRefreshRates = [60, 72, 80, 90, 120]
    static let minWiredEyeHeightM = 1.0
    static let maxWiredEyeHeightM = 2.2

    var runtimeEnabled = true
    var bitrateMbps = 50
    var refreshRateHz = 72
    var resolutionScale = 0.75
    var keyframeIntervalSec = 2
    var encoderPreset: EncoderPreset = .balanced
    var transport: StreamingTransportSetting = .auto
    var foveatedEncodingPreset: FoveationPresetSetting = .off
    var clientFoveationPreset: ClientFoveationPresetSetting = .auto
    var clientUpscaling = false
    var clientReprojection: ClientReprojectionSetting = .pose
    var abrMode: AbrModeSetting = .bitrate
    var headsetAudio = false
    var wiredHeadset = false
    var wiredDisplayId = 0
    var wiredEyeHeightM = 1.6
    var wiredPositionTracking = true
    var wiredControllerAdapter = false
    var wiredCameraMonitor = true
    var fileLogging = true
    var questLogcat = false

    var headsetMode: HeadsetModeSetting {
        get { wiredHeadset ? .wired : .streaming }
        set { wiredHeadset = newValue == .wired }
    }

    static let defaultText = """
    # OXRSys Runtime Configuration
    # Preferred location: ~/Library/Application Support/OXRSys/oxrsys-runtime.toml

    [general]
    # Enable or disable this runtime without unregistering the manifest.
    # When false, xrCreateInstance returns XR_ERROR_RUNTIME_UNAVAILABLE.
    runtime_enabled = true

    [streaming]
    # H.265 encoding bitrate in Mbps. Lower = less latency but more artifacts.
    # Try 20-30 for lower latency, 50+ for quality.
    bitrate_mbps = 50

    # Preferred headset display refresh rate: 60, 72, 80, 90, or 120.
    refresh_rate_hz = 72

    # Resolution multiplier (0.25 to 1.0). Lower = faster encode, less bandwidth, more blur.
    # 0.5 = half resolution (recommended for WiFi), 1.0 = native resolution.
    resolution_scale = 0.75

    # Keyframe interval in seconds (1-10). Higher = less bandwidth spikes, slower recovery.
    # Default 2 is a good balance. Use 1 for lossy WiFi, 5+ for USB.
    keyframe_interval_sec = 2

    # Encoder speed preset: "quality", "balanced", "speed"
    # speed  = lowest latency, fastest encode, lower quality
    # balanced = default, good mix
    # quality  = best visual quality, slightly higher latency
    encoder_preset = "balanced"

    # Streaming transport: "auto", "wifi", or "usb_adb".
    transport = "auto"

    # Server-side foveated video encoding preset: "off", "light", "medium", or "high".
    foveated_encoding_preset = "off"

    # Headset client foveation override: "auto", "off", "light", "medium", or "high".
    # "auto" leaves the headset client unmanaged by Home.
    client_foveation_preset = "auto"

    # Enable Quest shader upscaling after video decode.
    client_upscaling = false

    # Quest client reprojection for short decode/network gaps: "off", "pose", or "pose_warp".
    client_reprojection = "pose"

    # Adaptive bitrate mode: "off", "bitrate", or "full".
    # "full" may select resolution/foveation/upscaling profiles after session-safe transitions.
    abr_mode = "bitrate"

    # Reserved for headset speaker audio. The runtime does not advertise audio until
    # a platform capture/playback path is attached.
    headset_audio = false

    [wired]
    # Wired headset (Windows Mixed Reality over USB + HDMI/DisplayPort, macOS only).
    # When true and a headset is plugged in, it replaces the streaming client.
    # The panel must be visible to macOS as a display; see docs/platforms/wmr.md
    # for the one-time EDID override.
    wired_headset = false

    # CGDirectDisplayID of the headset panel. 0 = auto-detect.
    wired_display_id = 0

    # Height of the eyes above the floor, in meters: where the head sits
    # without positional tracking, and the starting height with it.
    wired_eye_height_m = 1.6

    # Positional (6DoF) head tracking through Basalt when libbasalt.dylib is
    # installed next to the headset helper; false = orientation only.
    wired_position_tracking = true

    # Windows Mixed Reality motion controllers through a separate USB Bluetooth
    # adapter (macOS's Bluetooth can't pair 1st-gen controllers). The headset
    # helper starts wmr_btstack from next to itself; see docs/platforms/wmr.md.
    wired_controller_adapter = false

    # Open the headset helper's tracking-camera window (both cameras with the
    # features Basalt tracks) on a desktop screen whenever the headset is used.
    wired_camera_monitor = true

    [logging]
    # Write server logs to ~/Library/Application Support/OXRSys/oxrsys-runtime.log.
    file_logging = true

    # Capture Quest/headset logcat to ~/Library/Application Support/OXRSys/oxrsys-headset.log.
    # Requires adb in PATH and a connected device.
    quest_logcat = false
    """

    static func parse(from text: String) -> OXRSysServerConfig {
        var config = OXRSysServerConfig()

        if let value = boolValue("runtime_enabled", in: text) {
            config.runtimeEnabled = value
        }
        if let value = intValue("bitrate_mbps", in: text),
           (Self.minBitrateMbps...Self.maxBitrateMbps).contains(value) {
            config.bitrateMbps = value
        }
        if let value = intValue("refresh_rate_hz", in: text), supportedRefreshRates.contains(value) {
            config.refreshRateHz = value
        }
        if let value = doubleValue("resolution_scale", in: text), value >= 0.25, value <= 1.0 {
            config.resolutionScale = value
        }
        if let value = intValue("keyframe_interval_sec", in: text), (1...10).contains(value) {
            config.keyframeIntervalSec = value
        }
        if let value = stringValue("encoder_preset", in: text), let preset = EncoderPreset(rawValue: value) {
            config.encoderPreset = preset
        }
        if let value = stringValue("transport", in: text), let transport = StreamingTransportSetting(rawValue: value) {
            config.transport = transport
        }
        if let value = stringValue("foveated_encoding_preset", in: text), let preset = FoveationPresetSetting(rawValue: value) {
            config.foveatedEncodingPreset = preset
        }
        if let value = stringValue("client_foveation_preset", in: text), let preset = ClientFoveationPresetSetting(rawValue: value) {
            config.clientFoveationPreset = preset
        }
        if let value = boolValue("client_upscaling", in: text) {
            config.clientUpscaling = value
        }
        if let value = stringValue("client_reprojection", in: text), let mode = ClientReprojectionSetting(rawValue: value) {
            config.clientReprojection = mode
        }
        if let value = stringValue("abr_mode", in: text), let mode = AbrModeSetting(rawValue: value) {
            config.abrMode = mode
        }
        if let value = boolValue("headset_audio", in: text) {
            config.headsetAudio = value
        }
        if let value = boolValue("wired_headset", in: text) {
            config.wiredHeadset = value
        }
        if let value = intValue("wired_display_id", in: text), value >= 0 {
            config.wiredDisplayId = value
        }
        if let value = doubleValue("wired_eye_height_m", in: text),
           (Self.minWiredEyeHeightM...Self.maxWiredEyeHeightM).contains(value) {
            config.wiredEyeHeightM = value
        }
        if let value = boolValue("wired_position_tracking", in: text) {
            config.wiredPositionTracking = value
        }
        if let value = boolValue("wired_controller_adapter", in: text) {
            config.wiredControllerAdapter = value
        }
        if let value = boolValue("wired_camera_monitor", in: text) {
            config.wiredCameraMonitor = value
        }
        if let value = boolValue("file_logging", in: text) {
            config.fileLogging = value
        }
        if let value = boolValue("quest_logcat", in: text) {
            config.questLogcat = value
        }

        return config
    }

    func merged(into currentText: String) -> String {
        var text = currentText.trimmingCharacters(in: .whitespacesAndNewlines)
        if text.isEmpty {
            text = Self.defaultText
        }
        text = removingKeys(Set(["fov_degrees"]), fromSection: "streaming", in: text)

        let sectionValues: [(name: String, keys: [(key: String, value: String)])] = [
            ("general", [
                ("runtime_enabled", boolString(runtimeEnabled)),
            ]),
            ("streaming", [
                ("bitrate_mbps", "\(bitrateMbps)"),
                ("refresh_rate_hz", "\(refreshRateHz)"),
                ("resolution_scale", decimalString(resolutionScale)),
                ("keyframe_interval_sec", "\(keyframeIntervalSec)"),
                ("encoder_preset", "\"\(encoderPreset.rawValue)\""),
                ("transport", "\"\(transport.rawValue)\""),
                ("foveated_encoding_preset", "\"\(foveatedEncodingPreset.rawValue)\""),
                ("client_foveation_preset", "\"\(clientFoveationPreset.rawValue)\""),
                ("client_upscaling", boolString(clientUpscaling)),
                ("client_reprojection", "\"\(clientReprojection.rawValue)\""),
                ("abr_mode", "\"\(abrMode.rawValue)\""),
                ("headset_audio", boolString(headsetAudio)),
            ]),
            ("wired", [
                ("wired_headset", boolString(wiredHeadset)),
                ("wired_display_id", "\(wiredDisplayId)"),
                ("wired_eye_height_m", decimalString(wiredEyeHeightM)),
                ("wired_position_tracking", boolString(wiredPositionTracking)),
                ("wired_controller_adapter", boolString(wiredControllerAdapter)),
                ("wired_camera_monitor", boolString(wiredCameraMonitor)),
            ]),
            ("logging", [
                ("file_logging", boolString(fileLogging)),
                ("quest_logcat", boolString(questLogcat)),
            ]),
        ]

        for sectionValue in sectionValues {
            text = upsertingSection(sectionValue.name, keys: sectionValue.keys, in: text)
        }

        return text.hasSuffix("\n") ? text : text + "\n"
    }
}

private func boolValue(_ key: String, in text: String) -> Bool? {
    guard let value = rawValue(key, in: text)?.lowercased() else {
        return nil
    }
    if ["true", "1", "yes"].contains(value) {
        return true
    }
    if ["false", "0", "no"].contains(value) {
        return false
    }
    return nil
}

private func intValue(_ key: String, in text: String) -> Int? {
    guard let value = rawValue(key, in: text) else {
        return nil
    }
    return Int(value)
}

private func doubleValue(_ key: String, in text: String) -> Double? {
    guard let value = rawValue(key, in: text) else {
        return nil
    }
    return Double(value)
}

private func stringValue(_ key: String, in text: String) -> String? {
    guard let value = rawValue(key, in: text) else {
        return nil
    }
    if value.hasPrefix("\""), value.hasSuffix("\""), value.count >= 2 {
        return String(value.dropFirst().dropLast())
    }
    return value
}

private func rawValue(_ key: String, in text: String) -> String? {
    let pattern = "(?m)^\\s*\(NSRegularExpression.escapedPattern(for: key))\\s*=\\s*(.+?)\\s*$"
    guard let regex = try? NSRegularExpression(pattern: pattern) else {
        return nil
    }
    let range = NSRange(text.startIndex..<text.endIndex, in: text)
    guard let match = regex.firstMatch(in: text, range: range),
          let valueRange = Range(match.range(at: 1), in: text) else {
        return nil
    }
    return String(text[valueRange]).trimmingCharacters(in: .whitespacesAndNewlines)
}

private func decimalString(_ value: Double) -> String {
    String(format: "%.2f", value)
}

private func boolString(_ value: Bool) -> String {
    value ? "true" : "false"
}

private func upsertingSection(_ sectionName: String, keys: [(key: String, value: String)], in text: String) -> String {
    var lines = text.components(separatedBy: .newlines)
    let sectionHeader = "[\(sectionName)]"

    func nextSectionStart(after index: Int) -> Int {
        for offset in (index + 1)..<lines.count where isSectionHeader(lines[offset]) {
            return offset
        }
        return lines.count
    }

    if let sectionIndex = lines.firstIndex(where: { $0.trimmingCharacters(in: .whitespaces) == sectionHeader }) {
        let sectionEnd = nextSectionStart(after: sectionIndex)
        var insertIndex = sectionEnd

        for item in keys {
            let newLine = "\(item.key) = \(item.value)"
            if let existingIndex = (sectionIndex + 1..<sectionEnd).first(where: { lineIndex in
                matchesKey(item.key, line: lines[lineIndex])
            }) {
                lines[existingIndex] = newLine
            } else {
                lines.insert(newLine, at: insertIndex)
                insertIndex += 1
            }
        }
    } else {
        if !lines.isEmpty, !lines.last!.isEmpty {
            lines.append("")
        }
        lines.append(sectionHeader)
        for item in keys {
            lines.append("\(item.key) = \(item.value)")
        }
    }

    return lines.joined(separator: "\n")
}

private func removingKeys(_ keys: Set<String>, fromSection sectionName: String, in text: String) -> String {
    let lines = text.components(separatedBy: .newlines)
    let sectionHeader = "[\(sectionName)]"
    guard let sectionIndex = lines.firstIndex(where: { $0.trimmingCharacters(in: .whitespaces) == sectionHeader }) else {
        return text
    }

    let sectionEnd = ((sectionIndex + 1)..<lines.count).first(where: { isSectionHeader(lines[$0]) }) ?? lines.count
    var indexesToRemove = Set<Int>()
    for index in (sectionIndex + 1)..<sectionEnd where keys.contains(where: { matchesKey($0, line: lines[index]) }) {
        indexesToRemove.insert(index)
        let previousIndex = index - 1
        if previousIndex > sectionIndex, lines[previousIndex].contains("Rendering FOV") {
            indexesToRemove.insert(previousIndex)
        }
    }

    var filtered: [String] = []
    filtered.reserveCapacity(lines.count)
    for (index, line) in lines.enumerated() {
        if indexesToRemove.contains(index) {
            continue
        }
        filtered.append(line)
    }
    return filtered.joined(separator: "\n")
}

private func isSectionHeader(_ line: String) -> Bool {
    let trimmed = line.trimmingCharacters(in: .whitespaces)
    return trimmed.hasPrefix("[") && trimmed.hasSuffix("]")
}

private func matchesKey(_ key: String, line: String) -> Bool {
    let trimmed = line.trimmingCharacters(in: .whitespaces)
    guard let equalsIndex = trimmed.firstIndex(of: "=") else {
        return false
    }
    let lineKey = trimmed[..<equalsIndex].trimmingCharacters(in: .whitespaces)
    return lineKey == key
}
