Syntakt (emulator) - VST3 instrument
====================================

This plugin runs the firmware of an Elektron Syntakt inside your DAW. It does NOT
include any Elektron firmware: you need your own Syntakt firmware file (OS 1.41,
a .syx file, for example from Elektron's website).

Install
  Use the installer (Syntakt-Installer-*.exe on Windows, Syntakt-*.pkg on macOS). It
  puts the plugin in the system VST3 folder and creates the firmware folder:
    Windows  %APPDATA%\Elektron Emu\Syntakt\Firmware\
    macOS    ~/Library/Application Support/Elektron Emu/Syntakt/Firmware/
  Without an installer: copy the "syntakt.vst3" folder (the whole folder) to
    Windows  C:\Program Files\Common Files\VST3\
    macOS    /Library/Audio/Plug-Ins/VST3/  (or ~/Library/Audio/Plug-Ins/VST3/)

First start
  1. Put your Syntakt firmware (.syx) in the firmware folder above. You can also open
     the plugin and click "Choose firmware..." (it copies the file there), or "Open
     folder" to open the folder.
  2. The plugin checks the file. A different firmware version than OS 1.41 is tried
     anyway and a warning is shown.
  3. The very first time, the plugin builds the machine from the firmware ("First
     start", about a minute). It is cached, later starts are immediate.
  Until a firmware is present the plugin is silent and the window says where to put it.

One machine, like the hardware
  There is a single Syntakt: its memory and projects are kept in one file per firmware
  in the Elektron Emu\Syntakt folder, shared by all DAW projects. The plugin resumes where you
  left it, even if you close the DAW without saving. Only one instance can run at a time:
  a second instance stays silent and starts by itself when the first one is closed.

Window and zoom
  Bottom right: 75% 100% 125% 150% ask the DAW to resize the window; the corner handle
  drags it. Computer keyboard: F1-F8, arrows, Enter = YES, Esc/Backspace = NO, 1-8 and
  q w e r t y u i = trigs, Alt/Option = FUNC (held), Tab = TRK, Space = PLAY, . = STOP.

Outputs and parameters
  Three stereo outputs: Main, Main (no Analog FX), Analog FX send. Parameters: Main
  Volume and Sync DAW (Start/Stop and MIDI clock from the DAW transport).
  MIDI in and out: notes, CC, program change, SysEx.

Uninstall
  Windows: Settings > Apps, or the uninstaller in the plugin's folder.
  macOS: run "sudo /Library/Application\ Support/Elektron Emu/Syntakt/uninstall.sh" or delete
  /Library/Audio/Plug-Ins/VST3/syntakt.vst3.
  Your firmware and machine files are never deleted; remove the Elektron Emu\Syntakt folder by hand.
