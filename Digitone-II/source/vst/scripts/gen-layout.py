#!/usr/bin/env python3
"""core/layout/digitone2_panel.json -> header C++ per la vista del VST3.

Le stesse costanti che l'AU legge a runtime (au/Extension/PanelView.swift,
PanelLayout): qui diventano dati compilati, cosi' il plugin non ha bisogno di
un parser JSON.

    vst/scripts/gen-layout.py core/layout/digitone2_panel.json OUT.hpp
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
    w("namespace dtl {")
    w("struct RGB { unsigned char r, g, b; };")
    w("struct Key { int code; float x, y, w, h; };")
    w("struct Enc { int turn, push; float x, y, r; };")
    w("struct LedMap { int led, code; };")
    w("struct LedColor { int code; RGB c; };")
    w("static const float kPanelW = %r, kPanelH = %r;" % tuple(float(v) for v in j["panel"]))
    w("static const RGB kBackground = %s;" % rgb(j["background"]))
    ph = j.get("photo")
    if ph:       # la foto intera e la parte che si vede (emu/guiv2.py VIEW)
        w("static const float kPhotoW = %r, kPhotoH = %r, kViewX = %r, kViewY = %r;"
          % (float(ph["size"][0]), float(ph["size"][1]), float(ph["view"][0]), float(ph["view"][1])))
    else:
        w("static const float kPhotoW = %r, kPhotoH = %r, kViewX = 0, kViewY = 0;" % tuple(float(v) for v in j["panel"]))
    w("static const int kNLeds = %d;" % j.get("n_leds", 50))
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
    w("static const float kVolX = %r, kVolY = %r, kVolR = %r, kVolMax = %r;" % (float(x), float(y), float(r), float(v["max"])))
    w("static const RGB kVolTick = %s, kVolText = %s;" % (rgb(v["tick"]), rgb(v["text_color"])))
    w("static const float kVolTextDY = %r, kTickR0 = %r, kTickR1 = %r, kTickWidth = %r;"
      % (float(v["text_dy"]), float(v["tick_r"][0]), float(v["tick_r"][1]), float(v["tick_width"])))
    w("static const double kAngle0 = %r, kAngleSpan = %r;" % (float(v["angle"][0]), float(v["angle"][1])))
    w("static const int kVolHideMs = %d;" % v["hide_ms"])
    p = j["page_leds"]
    w("static const float kPageXY[][2] = {%s};" % ", ".join("{%r, %r}" % (float(a), float(b)) for a, b in p["xy"]))
    w("static const int kPageLeds[] = {%s};" % ", ".join(str(i) for i in p["leds"]))
    w("static const float kPageSize = %r;" % float(p["size"]))
    w("static const LedMap kLedToControl[] = {%s};"
      % ", ".join("{%d, %d}" % (int(a), b) for a, b in sorted(j["led_to_control"].items(), key=lambda kv: int(kv[0]))))
    w("static const LedColor kLedColor[] = {%s};"
      % ", ".join("{%d, %s}" % (int(a), rgb(b)) for a, b in sorted(j["led_color"].items(), key=lambda kv: int(kv[0]))))
    w("static const RGB kLedDefault = %s;" % rgb(j["led_default"]))
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
    w("static const float kMinScale = %r;" % float(j["window"]["min_scale"]))
    w("static const RGB kStatusColor = %s;" % rgb(j["status_color"]))
    w("} // namespace dtl")
    open(out, "w").write("\n".join(L) + "\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
