Digitone II (VST3) for Windows
==============================

Install: copy the folder "Digitone II.vst3" to  C:\Program Files\Common Files\VST3  (or use the
installer, which does it for you), then rescan plugins in your DAW.

This plugin does not contain Elektron's firmware. To make it play:
 1. Download the Digitone II firmware (OS 1.11, a .syx file) from the Elektron website
    (elektron.se, Support, Digitone II).
 2. Put it in   %APPDATA%\Elektron Emu\Digitone II\Firmware   (or open the plugin and press
    "Choose firmware...").
 3. Open the plugin. The first start prepares the machine from your firmware and builds the DSP for
    this computer: a few minutes, once. The build needs a small open source compiler (llvm-mingw, about
    190 MB), which the plugin downloads by itself into %LOCALAPPDATA%\Elektron Emu\toolchain, so the
    first start needs an internet connection. The result is kept in
    %APPDATA%\Elektron Emu\Digitone II\Native (and a cache in %LOCALAPPDATA%\Elektron Emu\Digitone II\Cache).

Your machine (projects, sounds, +DRIVE content) is saved automatically in
%APPDATA%\Elektron Emu\Digitone II\Machine. There is one machine, like the hardware: only one plugin
instance plays at a time.
