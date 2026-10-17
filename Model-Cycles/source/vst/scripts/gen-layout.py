#!/usr/bin/env python3
"""core/layout/mc_panel.json -> header C++ per la vista del VST3 del Model:Cycles.

Gli stessi dati che l'AU legge a runtime (au/Extension/PanelView.swift,
MCLayout): qui diventano tabelle compilate, cosi' il plugin non ha bisogno di
un parser JSON. Il JSON e' generato da tools/mc_layout.py eseguendo il disegno
di emu/mcgui.py: 197 oggetti nell'ordine di Tk (poligoni, ovali, rettangoli,
linee, testi), gli oggetti cliccabili, i LED, le costanti dell'interazione.

Il testo di aiuto del JSON e' in italiano (e' quello della GUI Tk): nel plugin
tutti i testi visibili sono in inglese, quindi qui si sostituisce.

    vst/scripts/gen-layout.py core/layout/mc_panel.json OUT.hpp
"""
import json
import sys

# testi visibili del JSON che nel plugin vanno in inglese
HELP_EN = ("FUNC/TRACK/RETRIG: one click holds them for a combination · "
           "right-click = hold · wheel = encoder")
TITLE_EN = "Model:Cycles emulator"

KINDS = {"polygon": "kPolygon", "oval": "kOval", "rectangle": "kRect", "line": "kLine", "text": "kText"}
ANCHORS = {"center": 0, "w": 1, "e": 2}
HITKINDS = {"key": 0, "knob": 1, "pad": 2}
LEDKINDS = {"key": 0, "pad": 1, "dot": 2}

# tasti di Tk (keysym) -> punto di codice dell'evento di tastiera di DPF
SPECIAL = {"space": 32, "Return": 13, "Tab": 9, "Escape": 27, "BackSpace": 8, "period": ord("."),
           "Alt_L": 0xE055, "Alt_R": 0xE056, "Option_L": 0xE055, "Option_R": 0xE056}


def rgb(s):
    v = int(s[1:], 16)
    return "{%d, %d, %d}" % (v >> 16 & 255, v >> 8 & 255, v & 255)


def cstr(s):
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def keycode(k):
    return SPECIAL[k] if k in SPECIAL else ord(k)


def main(src, out):
    j = json.load(open(src, encoding="utf-8"))
    L = []
    w = L.append
    w("// GENERATO da vst/scripts/gen-layout.py da %s: non modificare." % src.split("/")[-1])
    w("#pragma once")
    w("namespace mcl {")
    w("struct RGB { unsigned char r, g, b; };")
    w("enum Kind { kPolygon, kOval, kRect, kLine, kText };")
    w("// font: indice in kFonts; anchor: 0 centro, 1 w (testo a destra del punto), 2 e (a sinistra)")
    w("struct Item { Kind kind; bool smooth, roundCap, hasFill, hasOutline; RGB fill, outline; float w;")
    w("               int off, n; int font, anchor; const char* text; };")
    w("struct FontSpec { float size; bool bold, italic, underline; };")
    w("struct Hit { int id, kind, value; };          // kind: 0 tasto, 1 manopola, 2 pad; id = indice + 1")
    w("struct CodeId { int code, id; };")
    w("struct Mark { int enc, id; float x, y, r; };")
    w("struct LedItem { int led, id, kind; };        // kind: 0 tasto, 1 pad, 2 LED")
    w("struct PadBox { float x, y, w, h; };")
    w("struct KeyMap { unsigned key; int code; };")
    w("static const float kPanelW = %r, kPanelH = %r;" % tuple(float(v) for v in j["panel"]))
    w("static const RGB kBackground = %s;" % rgb(j["background"]))
    for k, v in sorted(j["colors"].items()):
        w("static const RGB kC_%s = %s;" % (k, rgb(v)))
    w("static const char* kTitle = %s;" % cstr(TITLE_EN))

    fonts = j["fonts"]
    names = sorted(fonts["specs"])
    w("// font: " + ", ".join("%d %s" % (i, n) for i, n in enumerate(names)))
    w("static const FontSpec kFonts[] = {")
    for n in names:
        f = fonts["specs"][n]
        w("    {%r, %s, %s, %s}," % (float(f["size"]), str(f["weight"] == "bold").lower(),
                                    str(f["slant"] == "italic").lower(), str(bool(f["underline"])).lower()))
    w("};")
    w("static const float kMinPt = %r, kPxPerPt = %r;" % (float(fonts["min_pt"]), float(fonts["px_per_pt"])))

    pts = []
    items = j["items"]
    w("static const Item kItems[] = {")
    for i, it in enumerate(items):
        p = it["p"]
        off = len(pts)
        pts.extend(float(v) for v in p)
        text = it.get("text", "")
        if text == j["texts"]["help"]:
            text = HELP_EN
        fill, outline = it.get("fill"), it.get("outline")
        w("    {%s, %s, %s, %s, %s, %s, %s, %r, %d, %d, %d, %d, %s},"
          % (KINDS[it["t"]], str(bool(it.get("smooth"))).lower(), str(it.get("cap") == "round").lower(),
             str(bool(fill)).lower(), str(bool(outline)).lower(),
             rgb(fill) if fill else "{0, 0, 0}", rgb(outline) if outline else "{0, 0, 0}",
             float(it.get("w", 1)), off, len(p),
             names.index(it["font"]) if "font" in it else 0, ANCHORS.get(it.get("anchor", "center"), 0),
             cstr(text)))
    w("};")
    w("static const int kNItems = %d;" % len(items))
    w("static const float kPts[] = {")
    for k in range(0, len(pts), 12):
        w("    " + ", ".join(repr(v) for v in pts[k:k + 12]) + ",")
    w("};")
    w("static const int kStatusItem = %d;      // id (indice + 1) della riga di stato" % j["status_item"])

    w("static const Hit kHit[] = {")
    for i, k, v in j["hit"]:
        w("    {%d, %d, %d}," % (i, HITKINDS[k], v))
    w("};")
    w("static const CodeId kOutlineItem[] = {%s};"
      % ", ".join("{%d, %d}" % (int(c), i) for c, i in sorted(j["outline_items"].items(), key=lambda kv: int(kv[0]))))
    o = j["outline"]
    w("static const RGB kOutlineDown = %s, kOutlineChord = %s, kOutlineUp = %s;"
      % (rgb(o["down"]), rgb(o["chord"]), rgb(o["up"])))
    w("static const float kOutlineWidth = %r, kOutlineUpWidth = %r;" % (float(o["width"]), float(o["up_width"])))
    w("static const Mark kMarks[] = {")
    for e, (i, x, y, r) in sorted(j["marks"].items(), key=lambda kv: int(kv[0])):
        w("    {%d, %d, %r, %r, %r}," % (int(e), i, float(x), float(y), float(r)))
    w("};")
    w("static const LedItem kLedItems[] = {")
    for led, lst in sorted(j["led_items"].items(), key=lambda kv: int(kv[0])):
        for i, k in lst:
            w("    {%d, %d, %d}," % (int(led), i, LEDKINDS[k]))
    w("};")
    w("static const int kLedBits[] = {%s};" % ", ".join(str(b) for b in j["led_bits"]))
    s = j["screen"]
    w("static const float kScreenX = %r, kScreenY = %r, kScreenZoom = %r;"
      % (float(s["xy"][0]), float(s["xy"][1]), float(s["zoom"])))
    w("static const RGB kScreenOff = %s, kScreenOn = %s;" % (rgb(s["off"]), rgb(s["on"])))
    w("static const PadBox kPads[] = {%s};"
      % ", ".join("{%r, %r, %r, %r}" % tuple(float(v) for v in p[:4]) for p in j["pads"]))
    i = j["interaction"]
    w("static const int kAutoReleaseMs = %d, kWheelReleaseMs = %d, kDataEnc = %d, kDataPush = %d, kDataPushMs = %d;"
      % (i["auto_release_ms"], i["wheel_release_ms"], i["data_enc"], i["data_push"], i["data_push_ms"]))
    w("static const float kDragPx = %r;" % float(i["drag_px"]))
    w("static const int kTurnDeg = %d, kShiftMult = %d, kPadKeyVelocity = %d;"
      % (i["turn_deg"], i["shift_mult"], i["pad_key_velocity"]))
    w("static const double kVolumeStep = %r, kPadVelLo = %r, kPadVelHi = %r, kPadVelSpan = %r;"
      % (float(i["volume_step"]), float(i["pad_velocity"][0]), float(i["pad_velocity"][1]), float(i["pad_velocity"][2])))
    w("static const int kLatching[] = {%s};" % ", ".join(str(c) for c in i["latching"]))
    w("static const KeyMap kKeyMap[] = {")
    for k, c in sorted(j["keymap"].items()):
        w("    {0x%X, %d},   // %s" % (keycode(k), c, k))
    w("};")
    w("static const KeyMap kPadKeys[] = {")
    for k, c in sorted(j["padkeys"].items()):
        w("    {0x%X, %d},   // %s" % (keycode(k), c, k))
    w("};")
    w("static const float kMinScale = %r;" % float(j["window"]["min_scale"]))
    # i LED per controllo (la vista sulla foto, vst/PhotoLayout.hpp): id del LED (indice di led_bits), -1 = nessuno
    w("struct CtlLed { int code, led; };")
    w("static const CtlLed kKeyLeds[] = {%s};" % ", ".join(
        "{%d, %d}" % (int(c), -1 if k[5] is None else k[5]) for c, k in sorted(j["keys"].items(), key=lambda kv: int(kv[0]))))
    w("static const CtlLed kKnobLeds[] = {%s};" % ", ".join(
        "{%d, %d}" % (int(e), -1 if k[5] is None else k[5]) for e, k in sorted(j["knobs"].items(), key=lambda kv: int(kv[0]))))
    w("static const CtlLed kPadLeds[] = {%s};" % ", ".join(
        "{%d, %d}" % (n + 1, -1 if p[5] is None else p[5]) for n, p in enumerate(j["pads"])))
    w("static const int kPageLedIds[] = {%s};" % ", ".join(str(p[2]) for p in j["page_leds"]))
    w("} // namespace mcl")
    open(out, "w", encoding="utf-8").write("\n".join(L) + "\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
