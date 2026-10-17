# Digitakt II

A VST3 plug-in that emulates the Elektron Digitakt II and runs its real operating system. It is not made or supported by Elektron, and it does not include any Elektron firmware.

## Install

On a Mac, copy `plugin/mac/Digitakt II.vst3` to `~/Library/Audio/Plug-Ins/VST3/`. If you downloaded it with a browser, macOS will block it until you run `xattr -dr com.apple.quarantine ~/Library/Audio/Plug-Ins/VST3/"Digitakt II.vst3"`. On Windows, copy the folder `plugin/windows/Digitakt II.vst3` to `C:\Program Files\Common Files\VST3\`. Then rescan plug-ins in your DAW.

## Firmware

You need Digitakt II OS 1.16 (other versions don't work yet). Download it from the Elektron support page for your machine, unzip it and put `Digitakt_II_OS1.16.syx` in:

- Mac: `~/Library/Application Support/Elektron Emu/Digitakt II/Firmware/`
- Windows: `%APPDATA%\Elektron Emu\Digitakt II\Firmware\`

The plug-in creates the folder and shows "Open folder" and "Choose firmware..." until the file is there. The first start boots the OS from the .syx (about a minute). The factory samples are not included, so it loads 16 simple synthesised sounds on the tracks; load your own samples from the +DRIVE tab. Your machine is saved in `~/Library/Application Support/dtcore` (Windows: `%APPDATA%\dtcore`). It is kept when you close the DAW, even without saving the project. Only one instance at a time, like the real machine.

## Build

On an Apple Silicon Mac with the Xcode command line tools, cmake, ninja and python3:

    cd source
    vst/scripts/build-mac.sh ~/build/dt2

The script downloads Unicorn 2.1.4 and DPF at a fixed commit, applies the patches in `patches/` and `vst/patches/`, and builds `~/build/dt2/mac/vst-build/bin/digitakt2.vst3`. `vst/scripts/build-windows.sh ~/build/dt2` cross-compiles the Windows version with mingw-w64. Code comments are in Italian.

## Known limits

macOS builds are Apple Silicon only. A few +Drive error messages are still in Italian, and the view has only been tried under Wine on Windows.

