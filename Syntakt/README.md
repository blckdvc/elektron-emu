# Syntakt

A VST3 plug-in that emulates the Elektron Syntakt and runs its real operating system. It is not made or supported by Elektron, and it does not include any Elektron firmware.

## Install

On a Mac, copy `plugin/mac/Syntakt.vst3` to `~/Library/Audio/Plug-Ins/VST3/`. If you downloaded it with a browser, macOS will block it until you run `xattr -dr com.apple.quarantine ~/Library/Audio/Plug-Ins/VST3/"Syntakt.vst3"`. On Windows, copy the folder `plugin/windows/Syntakt.vst3` to `C:\Program Files\Common Files\VST3\`. Then rescan plug-ins in your DAW.

## Firmware

You need Syntakt OS 1.41 (other versions don't work yet). Download it from the Elektron support page for your machine, unzip it and put `Syntakt_OS1.41.syx` in:

- Mac: `~/Library/Application Support/Elektron Emu/Syntakt/Firmware/`
- Windows: `%APPDATA%\Elektron Emu\Syntakt\Firmware\`

The plug-in creates the folder and shows "Open folder" and "Choose firmware..." until the file is there. The first start boots the OS from the .syx (under a minute). Your machine is saved in `~/Library/Application Support/Elektron Emu/Syntakt` (Windows: `%APPDATA%\Elektron Emu\Syntakt`). It is kept when you close the DAW, even without saving the project. Only one instance at a time, like the real machine.

## Build

On an Apple Silicon Mac with the Xcode command line tools, cmake, ninja and python3:

    cd source
    vst/scripts/build-mac.sh ~/build/st

The script downloads Unicorn 2.1.4 and DPF at a fixed commit, applies the patches in `patches/` and `vst/patches/`, and builds `~/build/st/mac/vst-build/bin/syntakt.vst3`. `vst/scripts/build-windows.sh ~/build/st` cross-compiles the Windows version with mingw-w64. `dtcore/` is a copy of the engine files shared with the Digitakt II plug-in. Code comments are in Italian.

## Known limits

Tracks 1-8 and the rest of the digital side run the real DSP code; the analog tracks 9-12 are not there. There is no +Drive transfer, macOS builds are Apple Silicon only, and the view has only been tried under Wine on Windows.

