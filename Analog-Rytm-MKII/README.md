# Analog Rytm MKII

A VST3 plug-in that emulates the Elektron Analog Rytm MKII and runs its real operating system. It is not made or supported by Elektron, and it does not include any Elektron firmware.

## Install

On a Mac, copy `plugin/mac/Analog Rytm MKII.vst3` to `~/Library/Audio/Plug-Ins/VST3/`. If you downloaded it with a browser, macOS will block it until you run `xattr -dr com.apple.quarantine ~/Library/Audio/Plug-Ins/VST3/"Analog Rytm MKII.vst3"`. On Windows, copy the folder `plugin/windows/Analog Rytm MKII.vst3` to `C:\Program Files\Common Files\VST3\`. Then rescan plug-ins in your DAW.

## Firmware

You need Analog Rytm MKII OS 1.73 (other versions don't work yet). Download it from the Elektron support page for your machine, unzip it and put `Analog-Rytm_MKII_OS1.73.syx` in:

- Mac: `~/Library/Application Support/Elektron Emu/Analog Rytm MKII/Firmware/`
- Windows: `%APPDATA%\Elektron Emu\Analog Rytm MKII\Firmware\`

The plug-in creates the folder and shows "Open folder" and "Choose firmware..." until the file is there. The first start boots the OS from the .syx and runs the analog calibration (about two minutes). Your machine is saved in `~/Library/Application Support/Elektron Emu/Analog Rytm MKII` (Windows: `%APPDATA%\Elektron Emu\Analog Rytm MKII`). It is kept when you close the DAW, even without saving the project. Only one instance at a time, like the real machine.

## Build

On an Apple Silicon Mac with the Xcode command line tools, cmake, ninja and python3:

    cd source
    vst/scripts/build-mac.sh ~/build/ar

The script downloads Unicorn 2.1.4 and DPF at a fixed commit, applies the patches in `patches/` and `vst/patches/`, and builds `~/build/ar/mac/vst-build/bin/analogrytm.vst3`. `vst/scripts/build-windows.sh ~/build/ar` cross-compiles the Windows version with mingw-w64. Code comments are in Italian.

## Known limits

The analog voices and the effects are a model we wrote, not the real sound of the machine. macOS builds are Apple Silicon only, a few +Drive error messages are still in Italian, and the view has only been tried under Wine on Windows.

