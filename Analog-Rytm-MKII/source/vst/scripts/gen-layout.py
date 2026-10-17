#!/usr/bin/env python3
"""core/layout/rytm_panel.json -> header C++ per la vista del VST3.

Le stesse costanti che l'AU legge a runtime (au/Extension/PanelView.swift,
PanelLayout): qui diventano dati compilati, cosi' il plugin non ha bisogno di
un parser JSON. Coordinate in pixel della foto scalata (image); la finestra ne
mostra `view` (come la GUI v2: emu/guiv2_rytm.py VIEW).

    vst/scripts/gen-layout.py core/layout/rytm_panel.json OUT.hpp
"""
import json
import sys


def rgb(s):
    v = int(s[1:], 16)
    return "{%d, %d, %d}" % (v >> 16 & 255, v >> 8 & 255, v & 255)


def f(v):
    return repr(float(v))


def main(src, out):
    j = json.load(open(src))
    L = []
    w = L.append
    w("// GENERATO da vst/scripts/gen-layout.py da %s: non modificare." % src.split("/")[-1])
    w("#pragma once")
    w("namespace arl {")
    w("struct RGB { unsigned char r, g, b; };")
    w("struct Rect { float x, y, w, h; };            // centro e misura")
    w("struct Key { int code; float x, y, w, h; };")
    w("struct Pad { int id; float x, y, w, h; };")
    w("struct Enc { int turn, push; float x, y, r; };")
    w("struct LedMap { int led, code; };")
    w("struct Colour { int id; RGB c; };")
    w("static const float kImgW = %s, kImgH = %s;" % tuple(f(v) for v in j["image"]))
    v = j["view"]
    w("static const float kViewX = %s, kViewY = %s, kPanelW = %s, kPanelH = %s;"
      % (f(v[0]), f(v[1]), f(v[2] - v[0]), f(v[3] - v[1])))
    w("static const RGB kBackground = %s;" % rgb(j["background"]))
    s = j["screen"]
    w("static const Rect kScreen = {%s}, kBigScreen = {%s};"
      % (", ".join(f(x) for x in s["rect"]), ", ".join(f(x) for x in s["big"])))
    w("static const float kBezelBottom = %s, kBezelPad = %s, kBigPad = %s, kBezelWidth = %s;"
      % (f(s["bezel_bottom"]), f(s["pad"]), f(s["big_pad"]), f(s["bezel_width"])))
    w("static const RGB kBezelFill = %s, kBezelOutline = %s, kScreenOff = %s, kScreenOn = %s;"
      % (rgb(s["bezel_fill"]), rgb(s["bezel_outline"]), rgb(s["off"]), rgb(s["on"])))
    keys = sorted(j["keys"].items(), key=lambda kv: int(kv[0]))
    w("static const Key kKeys[] = {")
    for k, (x, y, kw, kh) in keys:
        w("    {%d, %s, %s, %s, %s}," % (int(k), f(x), f(y), f(kw), f(kh)))
    w("};")
    w("static const Pad kPads[] = {")
    for k, (x, y, pw, ph) in sorted(j["pads"].items(), key=lambda kv: int(kv[0])):
        w("    {%d, %s, %s, %s, %s}," % (int(k), f(x), f(y), f(pw), f(ph)))
    w("};")
    w("static const Enc kEncoders[] = {")
    for e in j["encoders"]:
        x, y, r = e["xyr"]
        w("    {%d, %d, %s, %s, %s}," % (e["turn"], e["push"], f(x), f(y), f(r)))
    w("};")
    v = j["main_volume"]
    x, y, r = v["xyr"]
    w("static const float kVolX = %s, kVolY = %s, kVolR = %s, kVolMax = %s;" % (f(x), f(y), f(r), f(v["max"])))
    w("static const RGB kVolTick = %s, kVolText = %s;" % (rgb(v["tick"]), rgb(v["text_color"])))
    w("static const float kVolTextDY = %s, kTickR0 = %s, kTickR1 = %s, kTickWidth = %s;"
      % (f(v["text_dy"]), f(v["tick_r"][0]), f(v["tick_r"][1]), f(v["tick_width"])))
    w("static const double kAngle0 = %s, kAngleSpan = %s;" % (f(v["angle"][0]), f(v["angle"][1])))
    w("static const int kVolHideMs = %d;" % v["hide_ms"])
    o = j["pad_outline"]
    w("static const RGB kPadOutline = %s;" % rgb(o["color"]))
    w("static const float kPadOutlineWidth = %s;" % f(o["width"]))
    pv = j["pad_velocity"]
    w("static const double kPadVelMin = %s, kPadVelMax = %s, kPadVelTop = %s, kPadVelSpan = %s;"
      % (f(pv["min"]), f(pv["max"]), f(pv["top"]), f(pv["span"])))
    p = j["page_leds"]
    w("static const float kPageXY[][2] = {%s};" % ", ".join("{%s, %s}" % (f(a), f(b)) for a, b in p["xy"]))
    w("static const int kPageLeds[] = {%s};" % ", ".join(str(i) for i in p["leds"]))
    w("static const float kPageSize = %s;" % f(p["size"]))
    w("static const LedMap kLedToControl[] = {%s};"
      % ", ".join("{%d, %d}" % (int(a), b) for a, b in sorted(j["led_to_control"].items(), key=lambda kv: int(kv[0]))))
    w("static const Colour kColours[] = {%s};"
      % ", ".join("{%d, %s}" % (int(a), rgb(b)) for a, b in sorted(j["colours"].items(), key=lambda kv: int(kv[0]))))
    w("static const int kColourDefault = %d;" % j["colour_default"])
    w("static const int kLevels = %d;" % j["levels"])
    g = j["glow"]
    w("static const double kGlowAlpha = %s, kGlowSoft = %s, kRoundEdge = %s;"
      % (f(g["alpha"]), f(g["soft"]), f(g["round_edge"])))
    o = j["outline"]
    w("static const RGB kOutlinePressed = %s, kOutlineLatched = %s;" % (rgb(o["pressed"]), rgb(o["latched"])))
    w("static const float kOutlineWidth = %s;" % f(o["width"]))
    i = j["interaction"]
    w("static const double kDragPx = %s, kFastGain = %s, kFastSpeed = %s, kVolumeStep = %s;"
      % (f(i["drag_px"]), f(i["fast_gain"]), f(i["fast_speed"]), f(i["volume_step"])))
    w("static const int kWheelGain = %d, kWheelClamp = %d, kShiftMult = %d, kTurnClamp = %d;"
      % (i["wheel_gain"], i["wheel_clamp"], i["shift_mult"], i["turn_clamp"]))
    w("static const int kAutoReleaseMs = %d, kPushReleaseMs = %d;" % (i["auto_release_ms"], i["push_release_ms"]))
    w("static const int kLatching[] = {%s};" % ", ".join(str(c) for c in i["latching"]))
    w("static const float kMinScale = %s;" % f(j["window"]["min_scale"]))
    w("static const RGB kStatusColor = %s;" % rgb(j["status_color"]))
    w("} // namespace arl")
    open(out, "w").write("\n".join(L) + "\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
