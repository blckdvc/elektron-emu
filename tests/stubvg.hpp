// stubvg.hpp -- a NanoVG stand-in that writes SVG, to drive SeqView without OpenGL (tests)
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#define START_NAMESPACE_DISTRHO namespace DISTRHO {
#define END_NAMESPACE_DISTRHO }
#define NANOVG_DEJAVU_SANS_TTF "sans"

namespace DISTRHO {

struct Color {
    float r = 0, g = 0, b = 0, a = 1;
    Color() {}
    Color(int R, int G, int B, float A = 1.0f) : r(R / 255.f), g(G / 255.f), b(B / 255.f), a(A) {}
    Color(const Color& x, const Color& y, float t) : r(x.r + (y.r - x.r) * t), g(x.g + (y.g - x.g) * t), b(x.b + (y.b - x.b) * t), a(x.a + (y.a - x.a) * t) {}
    std::string css() const
    {
        char t[64];
        std::snprintf(t, sizeof t, "rgba(%d,%d,%d,%.3f)", int(r * 255), int(g * 255), int(b * 255), a);
        return t;
    }
};

struct NanoImage {
    int id = 0;
    std::vector<uint8_t> px;
    bool isValid() const { return id != 0; }
    void update(const uint8_t* p) { px.assign(p, p + 128 * 64 * 4); }
};

struct Paint { int img = 0; float x, y, w, h; };

class NanoVG {
public:
    enum { ALIGN_LEFT = 1, ALIGN_CENTER = 2, ALIGN_RIGHT = 4, ALIGN_TOP = 8, ALIGN_MIDDLE = 16, ALIGN_BOTTOM = 32 };
    enum Winding { CCW = 1, CW = 2 };
    enum LineCap { BUTT, ROUND, SQUARE };
    std::string out;
    NanoImage* lastImg = nullptr;

    void beginPath() { path.clear(); }
    void rect(float x, float y, float w, float h) { add("M%g %gh%gv%gh%gz", X(x), Y(y), w * s, h * s, -w * s); }
    void roundedRect(float x, float y, float w, float h, float r)
    {
        r = std::min(r, std::min(w, h) / 2) * s;
        const float a = X(x), b = Y(y), W = w * s, H = h * s;
        add("M%g %gh%ga%g %g 0 0 1 %g %gv%ga%g %g 0 0 1 %g %gh%ga%g %g 0 0 1 %g %gv%ga%g %g 0 0 1 %g %gz",
            a + r, b, W - 2 * r, r, r, r, r, H - 2 * r, r, r, -r, r, -(W - 2 * r), r, r, -r, -r, -(H - 2 * r), r, r, r, -r);
    }
    void circle(float x, float y, float r) { add("M%g %ga%g %g 0 1 0 %g 0a%g %g 0 1 0 %g 0", X(x) - r * s, Y(y), r * s, r * s, 2 * r * s, r * s, r * s, -2 * r * s); }
    void arc(float cx, float cy, float r, float a0, float a1, Winding)
    {
        add("M%g %g", X(cx + std::cos(a0) * r), Y(cy + std::sin(a0) * r));
        add("A%g %g 0 %d 1 %g %g", r * s, r * s, (a1 - a0) > M_PI ? 1 : 0, X(cx + std::cos(a1) * r), Y(cy + std::sin(a1) * r));
    }
    void moveTo(float x, float y) { add("M%g %g", X(x), Y(y)); }
    void lineTo(float x, float y) { add("L%g %g", X(x), Y(y)); }
    void fillColor(const Color& c) { fc = c.css(); pnt.img = 0; }
    void fillPaint(const Paint& p) { pnt = p; }
    void strokeColor(const Color& c) { sc = c.css(); }
    void strokeWidth(float w) { sw = w; }
    void lineCap(LineCap) {}
    void fill()
    {
        if (pnt.img && lastImg) {
            char t[256];
            std::snprintf(t, sizeof t, "<g transform='translate(%g %g) scale(%g %g)'>", X(pnt.x), Y(pnt.y), pnt.w * s / 128, pnt.h * s / 64);
            out += t;
            for (int y = 0; y < 64; ++y)
                for (int x = 0; x < 128; ++x) {
                    const uint8_t* p = &lastImg->px[(y * 128 + x) * 4];
                    if (p[0] < 0x80) continue;
                    std::snprintf(t, sizeof t, "<rect x='%d' y='%d' width='1' height='1' fill='#e8f6ff'/>", x, y);
                    out += t;
                }
            out += "</g>";
            pnt.img = 0;
            return;
        }
        out += "<path d='" + path + "' fill='" + fc + "'/>";
    }
    void stroke()
    {
        char t[64];
        std::snprintf(t, sizeof t, "%g", sw * s);
        out += "<path d='" + path + "' fill='none' stroke='" + sc + "' stroke-width='" + t + "' stroke-linecap='round'/>";
    }
    void fontFace(const char*) {}
    void fontSize(float f) { fs = f; }
    void textAlign(int a) { al = a; }
    void text(float x, float y, const char* str, const char*)
    {
        std::string e;
        for (const char* c = str; *c; ++c) e += *c == '<' ? "&lt;" : *c == '&' ? "&amp;" : std::string(1, *c);
        const char* anchor = (al & ALIGN_CENTER) ? "middle" : (al & ALIGN_RIGHT) ? "end" : "start";
        const char* base = (al & ALIGN_MIDDLE) ? "central" : (al & ALIGN_TOP) ? "hanging" : "alphabetic";
        char t[256];
        std::snprintf(t, sizeof t, "<text x='%g' y='%g' font-family='DejaVu Sans' font-size='%g' fill='%s' text-anchor='%s' dominant-baseline='%s'>",
                      X(x), Y(y), fs * s, fc.c_str(), anchor, base);
        out += t + e + "</text>";
    }
    void save() { st.push_back({tx, ty, s}); }
    void restore() { tx = st.back()[0]; ty = st.back()[1]; s = st.back()[2]; st.pop_back(); }
    void translate(float x, float y) { tx += x * s; ty += y * s; }
    void scale(float a, float) { s *= a; }
    NanoImage createImageFromRGBA(int, int, const uint8_t* p, int)
    {
        NanoImage i;
        i.id = 1;
        i.update(p);
        return i;
    }
    Paint imagePattern(float x, float y, float w, float h, float, NanoImage& img, float)
    {
        lastImg = &img;
        return {1, x, y, w, h};
    }

private:
    float X(float x) const { return tx + x * s; }
    float Y(float y) const { return ty + y * s; }
    template <typename... A> void add(const char* f, A... a)
    {
        char t[512];
        std::snprintf(t, sizeof t, f, a...);
        path += t;
    }
    std::string path, fc = "black", sc = "black";
    Paint pnt;
    float sw = 1, fs = 12, tx = 0, ty = 0, s = 1;
    int al = 0;
    std::vector<std::vector<float>> st;
};

} // namespace DISTRHO
