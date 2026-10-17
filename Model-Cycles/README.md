# Model Cycles

A VST3 plug-in that emulates the Elektron Model:Cycles and runs its real operating system. It is not made or supported by Elektron, and it does not include any Elektron firmware.

## Install

On a Mac, copy `plugin/mac/Model Cycles.vst3` to `~/Library/Audio/Plug-Ins/VST3/`. If you downloaded it with a browser, macOS will block it until you run `xattr -dr com.apple.quarantine ~/Library/Audio/Plug-Ins/VST3/"Model Cycles.vst3"`. On Windows, copy the folder `plugin/windows/Model Cycles.vst3` to `C:\Program Files\Common Files\VST3\`. Then rescan plug-ins in your DAW.

## Firmware

You need Model:Cycles OS 1.13 (other versions don't work yet). Download it from the Elektron support page for your machine, unzip it and put `model-cycles_OS1.13.syx` in:

- Mac: `~/Library/Application Support/Elektron Emu/Model Cycles/Firmware/`
- Windows: `%APPDATA%\Elektron Emu\Model Cycles\Firmware\`

The plug-in creates the folder and shows "Open folder" and "Choose firmware..." until the file is there. The first start boots the OS from the .syx (under a minute). Your machine is saved in `~/Library/Application Support/Elektron Emu/Model Cycles` (Windows: `%APPDATA%\Elektron Emu\Model Cycles`). It is kept when you close the DAW, even without saving the project. Only one instance at a time, like the real machine.

## Build

On an Apple Silicon Mac with the Xcode command line tools, cmake, ninja and python3:

    cd source
    vst/scripts/build-mac.sh ~/build/mc

The script downloads Unicorn 2.1.4 and DPF at a fixed commit, applies the patches in `patches/` and `vst/patches/`, and builds `~/build/mc/mac/vst-build/bin/ModelCycles.vst3`. `vst/scripts/build-windows.sh ~/build/mc` cross-compiles the Windows version with mingw-w64. Code comments are in Italian.

## Known limits

The 512 factory presets are not in the OS file, so the preset folders are empty. macOS builds are Apple Silicon only and the view has only been tried under Wine on Windows.

