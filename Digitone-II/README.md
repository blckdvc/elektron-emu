# Digitone II

A VST3 plug-in that emulates the Elektron Digitone II and runs its real operating system. It is not made or supported by Elektron, and it does not include any Elektron firmware.

## Install

On a Mac, copy `plugin/mac/Digitone II.vst3` to `~/Library/Audio/Plug-Ins/VST3/`. If you downloaded it with a browser, macOS will block it until you run `xattr -dr com.apple.quarantine ~/Library/Audio/Plug-Ins/VST3/"Digitone II.vst3"`. On Windows, copy the folder `plugin/windows/Digitone II.vst3` to `C:\Program Files\Common Files\VST3\`. Then rescan plug-ins in your DAW.

## Firmware

You need Digitone II OS 1.11 (other versions don't work yet). Download it from the Elektron support page for your machine, unzip it and put `Digitone_II_OS1.11.syx` in:

- Mac: `~/Library/Application Support/Elektron Emu/Digitone II/Firmware/`
- Windows: `%APPDATA%\Elektron Emu\Digitone II\Firmware\`

The plug-in creates the folder and shows "Open folder" and "Choose firmware..." until the file is there. The first start boots the OS from the .syx and then builds the DSP for your computer: a few minutes, once. The build needs a C compiler. On a Mac it uses Apple's command line tools (the plug-in asks for them and offers to install them if they are missing). On Windows it downloads a small open source compiler (llvm-mingw, about 190 MB) the first time, so it needs an internet connection then. Your machine is saved in `~/Library/Application Support/Elektron Emu/Digitone II` (Windows: `%APPDATA%\Elektron Emu\Digitone II`). It is kept when you close the DAW, even without saving the project. Only one instance at a time, like the real machine.

## Build

On an Apple Silicon Mac with the Xcode command line tools, cmake, ninja and python3:

    cd source
    vst/scripts/build-mac.sh ~/build/dn

The DSP is not part of the build: the plug-in translates and compiles it on the user's machine (`source/vst/NativeBuild.cpp`, `source/core/dsp/dn_gensrc.inc`). The script downloads Unicorn 2.1.4 and DPF at a fixed commit, applies the patches in `patches/` and `vst/patches/`, and builds `~/build/dn/mac/vst-build/bin/Digitone II.vst3`. `vst/scripts/build-windows.sh ~/build/dn` cross-compiles the Windows version with mingw-w64. `dtcore/` is a copy of the engine files shared with the Digitakt II plug-in. Code comments are in Italian.

## Known limits

If the compiler is not available it falls back to a slow interpreter (about 0.5x real time): offline rendering works (in Ableton, Export Audio or Freeze) but live playback stays silent and shows a warning. macOS builds are Apple Silicon only and a few +Drive error messages are still in Italian.
