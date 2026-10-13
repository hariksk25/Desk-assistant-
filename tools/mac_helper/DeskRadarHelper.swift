// Desk Radar Mac helper (v6). Runs on the owner's Mac (LaunchAgent, see install.sh) and talks to the device
// over the home network only:
//   every second  POST http://<device>/mac  {mic, cam, muted, vol, playing, np, app[, cal]}
//   the reply     {"cmd": ["mute", "unmute", "vol+", "vol-", "play", "next", "prev", "focus_on", "focus_off"]}
// Calendar: Apple Calendar via EventKit (the work Google account is already added there), next 24 h, declined
// events skipped, sent only when it changes (and every 5 min). Titles are sent (owner's choice, 12 Oct 2026).
// Mic / camera in use: CoreAudio / CoreMediaIO "running somewhere" flags, no permission needed.
// Mute, volume, now playing, media keys: AppleScript (macOS asks once for Automation access to Music / Spotify).
// Focus: runs the Shortcuts named "DeskRadar Focus On" / "DeskRadar Focus Off" if they exist.
import Foundation
import EventKit
import CoreAudio
import CoreMediaIO
import AppKit

let device = ProcessInfo.processInfo.environment["DESKRADAR_HOST"] ?? "deskradar6.local"
let calHorizon: TimeInterval = 24 * 3600

func log(_ s: String) {
  let f = DateFormatter(); f.dateFormat = "HH:mm:ss"
  FileHandle.standardError.write("[\(f.string(from: Date()))] \(s)\n".data(using: .utf8)!)
}

// ---------- calendar ----------
let store = EKEventStore()
var calAllowed = false
func requestCalendar() {
  let sem = DispatchSemaphore(value: 0)
  if #available(macOS 14.0, *) {
    store.requestFullAccessToEvents { ok, err in calAllowed = ok; if let err = err { log("calendar access error: \(err.localizedDescription)") }; sem.signal() }
  } else {
    store.requestAccess(to: .event) { ok, _ in calAllowed = ok; sem.signal() }
  }
  _ = sem.wait(timeout: .now() + 120)
  log(calAllowed ? "calendar access granted" : "calendar access NOT granted (System Settings > Privacy > Calendars)")
}
func calendarJSON() -> [[String: Any]] {
  guard calAllowed else { return [] }
  store.refreshSourcesIfNecessary()
  let now = Date()
  let pred = store.predicateForEvents(withStart: now.addingTimeInterval(-6 * 3600), end: now.addingTimeInterval(calHorizon), calendars: nil)
  var out: [[String: Any]] = []
  for ev in store.events(matching: pred).sorted(by: { $0.startDate < $1.startDate }) {
    if ev.endDate <= now { continue }
    if let me = ev.attendees?.first(where: { $0.isCurrentUser }), me.participantStatus == .declined { continue }
    if ev.status == .canceled { continue }
    let title = String((ev.title ?? "busy").prefix(46))
    out.append(["s": Int(ev.startDate.timeIntervalSince1970), "e": Int(ev.endDate.timeIntervalSince1970), "t": title,
                "c": String((ev.calendar?.title ?? "").prefix(18)), "a": ev.isAllDay])
    if out.count >= 12 { break }
  }
  return out
}

// ---------- mic / camera in use ----------
func audioDevices() -> [AudioDeviceID] {
  var addr = AudioObjectPropertyAddress(mSelector: kAudioHardwarePropertyDevices, mScope: kAudioObjectPropertyScopeGlobal, mElement: kAudioObjectPropertyElementMain)
  var size: UInt32 = 0
  guard AudioObjectGetPropertyDataSize(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size) == noErr else { return [] }
  var ids = [AudioDeviceID](repeating: 0, count: Int(size) / MemoryLayout<AudioDeviceID>.size)
  guard AudioObjectGetPropertyData(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size, &ids) == noErr else { return [] }
  return ids
}
func hasInput(_ id: AudioDeviceID) -> Bool {
  var addr = AudioObjectPropertyAddress(mSelector: kAudioDevicePropertyStreams, mScope: kAudioObjectPropertyScopeInput, mElement: kAudioObjectPropertyElementMain)
  var size: UInt32 = 0
  return AudioObjectGetPropertyDataSize(id, &addr, 0, nil, &size) == noErr && size > 0
}
func micInUse() -> Bool {
  for id in audioDevices() where hasInput(id) {
    var addr = AudioObjectPropertyAddress(mSelector: kAudioDevicePropertyDeviceIsRunningSomewhere, mScope: kAudioObjectPropertyScopeGlobal, mElement: kAudioObjectPropertyElementMain)
    var v: UInt32 = 0; var size = UInt32(MemoryLayout<UInt32>.size)
    if AudioObjectGetPropertyData(id, &addr, 0, nil, &size, &v) == noErr && v != 0 { return true }
  }
  return false
}
// Which app has the microphone open (macOS 14+: per-process audio objects). Dictation and assistant listeners
// (Siri / CoreSpeech, Wispr Flow, Dictation, Sound Recognition) open the mic for seconds at a time and are not a
// call, so they are ignored; anything else (Zoom, Teams, Meet in a browser, FaceTime, Slack) counts. Older systems
// fall back to the device-level flag above.
let micIgnore = ["com.apple.CoreSpeech", "com.apple.siri", "com.apple.Siri", "com.apple.assistant", "com.apple.SpeechRecognitionCore",
                 "com.apple.accessibility.heard", "com.apple.DictationIM", "com.electron.wispr-flow", "com.wispr"]
func procU32(_ obj: AudioObjectID, _ sel: AudioObjectPropertySelector) -> UInt32 {
  var a = AudioObjectPropertyAddress(mSelector: sel, mScope: kAudioObjectPropertyScopeGlobal, mElement: kAudioObjectPropertyElementMain)
  var v: UInt32 = 0; var sz = UInt32(MemoryLayout<UInt32>.size)
  return AudioObjectGetPropertyData(obj, &a, 0, nil, &sz, &v) == noErr ? v : 0
}
func procStr(_ obj: AudioObjectID, _ sel: AudioObjectPropertySelector) -> String {
  var a = AudioObjectPropertyAddress(mSelector: sel, mScope: kAudioObjectPropertyScopeGlobal, mElement: kAudioObjectPropertyElementMain)
  var v: Unmanaged<CFString>? = nil; var sz = UInt32(MemoryLayout<Unmanaged<CFString>?>.size)
  guard AudioObjectGetPropertyData(obj, &a, 0, nil, &sz, &v) == noErr, let cf = v?.takeRetainedValue() else { return "" }
  return cf as String
}
func micUser() -> String? {   // bundle id of a process running audio input that is not a dictation listener, else nil
  var addr = AudioObjectPropertyAddress(mSelector: kAudioHardwarePropertyProcessObjectList, mScope: kAudioObjectPropertyScopeGlobal, mElement: kAudioObjectPropertyElementMain)
  var size: UInt32 = 0
  guard AudioObjectGetPropertyDataSize(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size) == noErr, size > 0 else { return micInUse() ? "?" : nil }
  var ids = [AudioObjectID](repeating: 0, count: Int(size) / MemoryLayout<AudioObjectID>.size)
  guard AudioObjectGetPropertyData(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size, &ids) == noErr else { return micInUse() ? "?" : nil }
  for p in ids where procU32(p, kAudioProcessPropertyIsRunningInput) != 0 {
    let b = procStr(p, kAudioProcessPropertyBundleID)
    if micIgnore.contains(where: { b.hasPrefix($0) }) { continue }
    return b.isEmpty ? "?" : b
  }
  return nil
}
var micRun = 0; var micApp = ""
func micLive() -> Bool {   // on air once a call app has held the mic for 3 consecutive samples (1 s each)
  if let a = micUser() { micRun += 1; micApp = a; if micRun == 3 { log("mic in use by \(a)") } } else { if micRun >= 3 { log("mic released") }; micRun = 0; micApp = "" }
  return micRun >= 3
}
func cameraInUse() -> Bool {
  var addr = CMIOObjectPropertyAddress(mSelector: CMIOObjectPropertySelector(kCMIOHardwarePropertyDevices), mScope: CMIOObjectPropertyScope(kCMIOObjectPropertyScopeGlobal), mElement: CMIOObjectPropertyElement(kCMIOObjectPropertyElementMain))
  var size: UInt32 = 0
  guard CMIOObjectGetPropertyDataSize(CMIOObjectID(kCMIOObjectSystemObject), &addr, 0, nil, &size) == noErr, size > 0 else { return false }
  var ids = [CMIOObjectID](repeating: 0, count: Int(size) / MemoryLayout<CMIOObjectID>.size); var used: UInt32 = 0
  guard CMIOObjectGetPropertyData(CMIOObjectID(kCMIOObjectSystemObject), &addr, 0, nil, size, &used, &ids) == noErr else { return false }
  for id in ids {
    var a = CMIOObjectPropertyAddress(mSelector: CMIOObjectPropertySelector(kCMIODevicePropertyDeviceIsRunningSomewhere), mScope: CMIOObjectPropertyScope(kCMIOObjectPropertyScopeGlobal), mElement: CMIOObjectPropertyElement(kCMIOObjectPropertyElementMain))
    var v: UInt32 = 0; var u: UInt32 = 0
    if CMIOObjectGetPropertyData(id, &a, 0, nil, UInt32(MemoryLayout<UInt32>.size), &u, &v) == noErr && v != 0 { return true }
  }
  return false
}

// ---------- AppleScript: volume, mute, now playing, media ----------
@discardableResult func osa(_ src: String) -> String {
  var err: NSDictionary?
  let r = NSAppleScript(source: src)?.executeAndReturnError(&err)
  return r?.stringValue ?? ""
}
var savedInput = 75
func volumes() -> (out: Int, input: Int) {
  let s = osa("set v to get volume settings\nreturn (output volume of v as text) & \",\" & (input volume of v as text)")
  let p = s.split(separator: ",").map { Int($0) ?? -1 }
  return (p.count > 0 ? p[0] : -1, p.count > 1 ? p[1] : -1)
}
func playerApp() -> String? {
  for app in ["Spotify", "Music"] {
    if osa("if application \"\(app)\" is running then return \"yes\"") == "yes" { return app }
  }
  return nil
}
func nowPlaying() -> (playing: Bool, np: String, app: String) {
  guard let app = playerApp() else { return (false, "", "") }
  let s = osa("tell application \"\(app)\"\nif player state is playing then\nreturn \"1|\" & (name of current track) & \" - \" & (artist of current track)\nelse\nreturn \"0|\" & (name of current track) & \" - \" & (artist of current track)\nend if\nend tell")
  let parts = s.split(separator: "|", maxSplits: 1).map(String.init)
  return (parts.first == "1", parts.count > 1 ? String(parts[1].prefix(62)) : "", app)
}
func runShortcut(_ name: String) {
  let p = Process(); p.executableURL = URL(fileURLWithPath: "/usr/bin/shortcuts"); p.arguments = ["run", name]
  do { try p.run(); p.waitUntilExit(); log("shortcut \(name): exit \(p.terminationStatus)") } catch { log("shortcut \(name) failed: \(error)") }
}
// ---------- aircraft type photo: Wikipedia page image -> 220 x 124 RGB565 -> /deskradar/photos/<TYPE>.img on the card ----------
func httpGet(_ urlStr: String, accept: String = "*/*") -> Data? {
  guard let url = URL(string: urlStr) else { return nil }
  var req = URLRequest(url: url, timeoutInterval: 20)
  req.setValue("DeskRadarHelper/1 (personal desk display; macOS)", forHTTPHeaderField: "User-Agent"); req.setValue(accept, forHTTPHeaderField: "Accept")
  let sem = DispatchSemaphore(value: 0); var out: Data? = nil
  URLSession.shared.dataTask(with: req) { d, r, _ in if let h = r as? HTTPURLResponse, h.statusCode == 200 { out = d }; sem.signal() }.resume()
  _ = sem.wait(timeout: .now() + 25); return out
}
func uploadToCard(_ data: Data, path: String) -> Bool {
  guard let url = URL(string: "http://\(device)/sdput?path=\(path)") else { return false }
  let boundary = "----DeskRadar\(Int(Date().timeIntervalSince1970))"
  var body = Data()
  body.append("--\(boundary)\r\nContent-Disposition: form-data; name=\"f\"; filename=\"x.img\"\r\nContent-Type: application/octet-stream\r\n\r\n".data(using: .utf8)!)
  body.append(data); body.append("\r\n--\(boundary)--\r\n".data(using: .utf8)!)
  var req = URLRequest(url: url, timeoutInterval: 30); req.httpMethod = "POST"; req.httpBody = body
  req.setValue("multipart/form-data; boundary=\(boundary)", forHTTPHeaderField: "Content-Type")
  let sem = DispatchSemaphore(value: 0); var ok = false
  URLSession.shared.dataTask(with: req) { d, r, _ in if let h = r as? HTTPURLResponse, h.statusCode == 200 { ok = true }; sem.signal() }.resume()
  _ = sem.wait(timeout: .now() + 35); return ok
}
func photoCommand(_ arg: String) {   // "A20N:Airbus A320neo"
  let parts = arg.split(separator: ":", maxSplits: 1).map(String.init)
  guard parts.count == 2, parts[0].count <= 6 else { log("photo: bad request \(arg)"); return }
  let code = parts[0], words = parts[1]
  let title = words.replacingOccurrences(of: " ", with: "_").addingPercentEncoding(withAllowedCharacters: .urlPathAllowed) ?? words
  guard let js = httpGet("https://en.wikipedia.org/api/rest_v1/page/summary/\(title)", accept: "application/json"),
        let j = try? JSONSerialization.jsonObject(with: js) as? [String: Any] else { log("photo: no Wikipedia summary for \(words)"); return }
  var src = (j["thumbnail"] as? [String: Any])?["source"] as? String ?? ""
  if src.isEmpty { log("photo: no image on the page for \(words)"); return }
  // (a /480px- rewrite answered HTTP 400 on some files; the 330 px thumbnail is enough for 220 px)
  guard let img = httpGet(src, accept: "image/*"), let ns = NSImage(data: img), let cg = ns.cgImage(forProposedRect: nil, context: nil, hints: nil) else { log("photo: download failed for \(words)"); return }
  let W = 220, H = 124
  guard let ctx = CGContext(data: nil, width: W, height: H, bitsPerComponent: 8, bytesPerRow: W * 4, space: CGColorSpaceCreateDeviceRGB(), bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue) else { return }
  let sw = CGFloat(cg.width), sh = CGFloat(cg.height); let k = max(CGFloat(W) / sw, CGFloat(H) / sh)   // aspect fill, centred crop
  let dw = sw * k, dh = sh * k
  ctx.setFillColor(CGColor(red: 0, green: 0, blue: 0, alpha: 1)); ctx.fill(CGRect(x: 0, y: 0, width: W, height: H))
  ctx.interpolationQuality = .high
  ctx.draw(cg, in: CGRect(x: (CGFloat(W) - dw) / 2, y: (CGFloat(H) - dh) / 2, width: dw, height: dh))
  guard let px = ctx.data else { return }
  let bytes = px.bindMemory(to: UInt8.self, capacity: W * H * 4)
  var out = Data(); out.append("IMG1".data(using: .utf8)!)
  var w16 = UInt16(W), h16 = UInt16(H); out.append(Data(bytes: &w16, count: 2)); out.append(Data(bytes: &h16, count: 2))
  var credit = Array((String((j["title"] as? String ?? words).prefix(28)) + ", Wikimedia Commons").utf8); credit = Array(credit.prefix(47)); credit += [UInt8](repeating: 0, count: 48 - credit.count)
  out.append(Data(credit))
  var pix = [UInt8](repeating: 0, count: W * H * 2)
  for y in 0..<H { for x in 0..<W {   // CGContext rows are top-down here (no flip applied)
    let i = ((H - 1 - y) * W + x) * 4; let r = UInt16(bytes[i]), g = UInt16(bytes[i + 1]), b = UInt16(bytes[i + 2])
    let v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); let o = (y * W + x) * 2
    pix[o] = UInt8(v & 0xFF); pix[o + 1] = UInt8(v >> 8) } }
  out.append(Data(pix))
  let ok = uploadToCard(out, path: "/deskradar/photos/\(code).img")
  log("photo: \(code) (\(words)) \(ok ? "uploaded" : "upload FAILED") \(out.count) bytes")
}
func perform(_ cmd: String) {
  log("command \(cmd)")
  if cmd.hasPrefix("photo:") { DispatchQueue.global().async { photoCommand(String(cmd.dropFirst(6))) }; return }
  switch cmd {
  case "mute":   let v = volumes(); if v.input > 0 { savedInput = v.input }; osa("set volume input volume 0")
  case "unmute": osa("set volume input volume \(savedInput)")
  case "vol+":   osa("set volume output volume ((output volume of (get volume settings)) + 6)")
  case "vol-":   osa("set volume output volume ((output volume of (get volume settings)) - 6)")
  case "play":   if let a = playerApp() { osa("tell application \"\(a)\" to playpause") }
  case "next":   if let a = playerApp() { osa("tell application \"\(a)\" to next track") }
  case "prev":   if let a = playerApp() { osa("tell application \"\(a)\" to previous track") }
  case "focus_on":  runShortcut("DeskRadar Focus On")
  case "focus_off": runShortcut("DeskRadar Focus Off")
  default: log("unknown command \(cmd)")
  }
}

// ---------- link ----------
func post(_ body: [String: Any]) -> [String]? {
  guard let url = URL(string: "http://\(device)/mac"), let data = try? JSONSerialization.data(withJSONObject: body) else { return nil }
  var req = URLRequest(url: url, timeoutInterval: 4); req.httpMethod = "POST"; req.httpBody = data
  req.setValue("application/json", forHTTPHeaderField: "Content-Type")
  let sem = DispatchSemaphore(value: 0); var cmds: [String]? = nil
  URLSession.shared.dataTask(with: req) { d, _, _ in
    if let d = d, let j = try? JSONSerialization.jsonObject(with: d) as? [String: Any] { cmds = j["cmd"] as? [String] ?? [] }
    sem.signal()
  }.resume()
  _ = sem.wait(timeout: .now() + 5)
  return cmds
}

requestCalendar()
var lastCal = ""; var lastCalSent = Date.distantPast; var linkUp = false; var tick = 0
var media = (playing: false, np: "", app: ""); var vol = (out: -1, input: -1)
log("helper started, device \(device)")
while true {
  let t0 = Date()
  if tick % 3 == 0 { media = nowPlaying(); vol = volumes() }   // AppleScript is slow-ish: every 3 s
  var body: [String: Any] = ["mic": micLive(), "cam": cameraInUse(), "muted": vol.input == 0, "vol": vol.out,
                             "playing": media.playing, "np": media.np, "app": media.app, "micApp": micApp]
  if tick % 30 == 0 || !linkUp {
    let cal = calendarJSON()
    let s = String(data: (try? JSONSerialization.data(withJSONObject: cal, options: [.sortedKeys])) ?? Data(), encoding: .utf8) ?? ""
    if s != lastCal || Date().timeIntervalSince(lastCalSent) > 300 || !linkUp { body["cal"] = cal; lastCal = s; lastCalSent = Date() }
  }
  if let cmds = post(body) {
    if !linkUp { log("device reachable"); linkUp = true }
    for c in cmds { perform(c) }
    if !cmds.isEmpty { vol = volumes() }
  } else if linkUp { log("device not reachable"); linkUp = false }
  tick += 1
  let dt = Date().timeIntervalSince(t0); if dt < 1 { Thread.sleep(forTimeInterval: 1 - dt) }
}
