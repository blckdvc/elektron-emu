#!/usr/bin/env python3
"""core/layout/syntakt_panel.json -> header C++ per la vista del VST3.

Le stesse costanti che l'AU legge a runtime (au/Extension/PanelView.swift,
PanelLayout): qui diventano dati compilati, cosi' il plugin non ha bisogno di
un parser JSON. Il layout viene da tools/gui_layout.py (emu/guiv2.py ed
emu/st_leds.py): non si ritocca a mano.

    vst/scripts/gen-layout.py core/layout/syntakt_panel.json OUT.hpp
"""
import json
import sys


def rgb(s):
    v = int(s[1:], 16)
    return "{%d, %d, %d}" % (v >> 16 & 255, v >> 8 & 255, v & 255)


def main(src, out):
    j = json.load(open(src))
    L = []
    w = L.append
    w("// GENERATO da vst/scripts/gen-layout.py da %s: non modificare." % src.split("/")[-1])
    w("#pragma once")
    w("namespace stl {")
    w("struct RGB { unsigned char r, g, b; };")
    w("struct Key { int code; float x, y, w, h; };")
    w("struct Enc { int turn, push; float x, y, r; };")
    w("struct LedMap { int led, code; };")
    w("struct LedColor { int led; RGB c; };")
    w("struct Name { const char* name; int code; };")
    w("static const float kPanelW = %r, kPanelH = %r;" % tuple(float(v) for v in j["panel"]))
    w("static const RGB kBackground = %s;" % rgb(j["background"]))
    s = j["screen"]
    w("static const float kScreenX = %r, kScreenY = %r, kScreenW = %r, kScreenH = %r;"
      % tuple(float(v) for v in s["xy"] + s["wh"]))
    w("static const RGB kScreenOff = %s, kScreenOn = %s;" % (rgb(s["off"]), rgb(s["on"])))
    keys = sorted(j["keys"].items(), key=lambda kv: int(kv[0]))
    w("static const Key kKeys[] = {")
    for k, (x, y, kw, kh) in keys:
        w("    {%d, %r, %r, %r, %r}," % (int(k), float(x), float(y), float(kw), float(kh)))
    w("};")
    w("static const Enc kEncoders[] = {")
    for e in j["encoders"]:
        x, y, r = e["xyr"]
        w("    {%d, %d, %r, %r, %r}," % (e["turn"], e["push"], float(x), float(y), float(r)))
    w("};")
    v = j["main_volume"]
    x, y, r = v["xyr"]
    w("static const float kVolX = %r, kVolY = %r, kVolR = %r;" % (float(x), float(y), float(r)))
    w("static const double kVolMax = %r, kVolDefault = %r;" % (float(v["max"]), float(v["default"])))
    w("static const RGB kVolTick = %s, kVolText = %s;" % (rgb(v["tick"]), rgb(v["text_color"])))
    w("static const float kVolTextDY = %r, kTickR0 = %r, kTickR1 = %r, kTickWidth = %r;"
      % (float(v["text_dy"]), float(v["tick_r"][0]), float(v["tick_r"][1]), float(v["tick_width"])))
    w("static const double kAngle0 = %r, kAngleSpan = %r;" % (float(v["angle"][0]), float(v["angle"][1])))
    w("static const int kVolHideMs = %d, kVolWheelSteps = %d;" % (v["hide_ms"], v.get("wheel_steps", 2)))
    p = j["page_leds"]
    w("static const float kPageXY[][2] = {%s};" % ", ".join("{%r, %r}" % (float(a), float(b)) for a, b in p["xy"]))
    w("static const int kPageLeds[] = {%s};" % ", ".join(str(i) for i in p["leds"]))
    w("static const float kPageSize = %r;" % float(p["size"]))
    w("struct PageLevel { int value; double level; };")
    w("static const PageLevel kPageLevels[] = {%s};"
      % ", ".join("{%d, %r}" % (int(a), float(b)) for a, b in sorted(p["levels"].items(), key=lambda kv: int(kv[0]))))
    t = j["trk_led"]
    w("static const float kTrkX = %r, kTrkY = %r, kTrkSize = %r;" % (float(t["xy"][0]), float(t["xy"][1]), float(t["size"])))
    w("static const int kTrkLed = %d;" % t["led"])
    w("static const LedMap kLedToControl[] = {%s};"
      % ", ".join("{%d, %d}" % (int(a), b) for a, b in sorted(j["led_to_code"].items(), key=lambda kv: int(kv[0]))))
    # il colore di un LED: per tasto (key_colour, indice del LED) oppure dal nibble alto del valore
    w("static const LedColor kKeyColour[] = {%s};"
      % ", ".join("{%d, %s}" % (int(a), rgb(b)) for a, b in sorted(j["key_colour"].items(), key=lambda kv: int(kv[0]))))
    w("static const LedColor kLedColours[] = {%s};"
      % ", ".join("{%d, %s}" % (int(a), rgb(b)) for a, b in sorted(j["led_colours"].items(), key=lambda kv: int(kv[0]))))
    w("static const int kLitMin = %d, kLitMax = %d;" % tuple(j["lit"]))
    w("static const int kLevels = %d;" % j["levels"])
    g = j["glow"]
    w("static const double kGlowAlpha = %r, kGlowSoft = %r, kRoundEdge = %r;"
      % (float(g["alpha"]), float(g["soft"]), float(g["round_edge"])))
    o = j["outline"]
    w("static const RGB kOutlinePressed = %s, kOutlineLatched = %s;" % (rgb(o["pressed"]), rgb(o["latched"])))
    w("static const float kOutlineWidth = %r;" % float(o["width"]))
    i = j["interaction"]
    w("static const double kDragPx = %r, kFastGain = %r, kFastSpeed = %r, kVolumeStep = %r;"
      % (float(i["drag_px"]), float(i["fast_gain"]), float(i["fast_speed"]), float(i["volume_step"])))
    w("static const int kWheelGain = %d, kWheelClamp = %d, kShiftMult = %d, kTurnClamp = %d;"
      % (i["wheel_gain"], i["wheel_clamp"], i["shift_mult"], i["turn_clamp"]))
    w("static const int kAutoReleaseMs = %d, kPushReleaseMs = %d;" % (i["auto_release_ms"], i["push_release_ms"]))
    w("static const int kLatching[] = {%s};" % ", ".join(str(c) for c in i["latching"]))
    # tastiera del computer ([panel.keys] di devices/syntakt.toml): nome del tasto -> codice
    w("static const Name kKeyboard[] = {%s};"
      % ", ".join('{"%s", %d}' % (k, c) for k, c in sorted(j["keyboard"].items())))
    w("static const float kMinScale = %r;" % float(j["window"]["min_scale"]))
    w("static const RGB kStatusColor = %s;" % rgb(j["status_color"]))
    w("} // namespace stl")
    open(out, "w").write("\n".join(L) + "\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
