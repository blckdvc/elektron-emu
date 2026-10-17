Model Cycles (emulator) - VST3 for Windows x64
==============================================

This is an emulator of the Elektron Model:Cycles. It does NOT contain any
Elektron software: you need your own copy of the Model:Cycles firmware
(a .syx file, for example model-cycles_OS1.13.syx, from Elektron's website).

Install
1. Close your DAW.
2. Run ModelCycles-Setup-win64.exe, or copy the whole folder "ModelCycles.vst3"
   (the folder, not just the file inside) to
      C:\Program Files\Common Files\VST3\
3. Start your DAW and rescan plugins. The instrument is called "Model Cycles"
   (by Elektron Emu).

Firmware
- Put your firmware .syx file in
      %APPDATA%\Elektron Emu\Model Cycles\Firmware\
  (typically C:\Users\<you>\AppData\Roaming\Elektron Emu\Model Cycles\Firmware\).
  The folder is created when the plugin first loads. The installer also
  creates it for the user who runs the setup.
- In the plugin window, "Open folder" opens that folder and "Choose
  firmware..." copies a .syx file into it.
- Until a valid firmware is there the plugin stays silent and says so in the
  status line.

First start
- The first time, the plugin boots the firmware from scratch. This takes
  from about 30 seconds to a few minutes, depending on your CPU. The status
  line says "first start". The result is cached in
  %LOCALAPPDATA%\Elektron Emu\Model Cycles\Cache, so later starts are immediate.
- Like the real hardware there is a single machine: your patterns, kits and
  sounds are kept for every project of your DAW, and the plugin resumes where
  you left it, even if you close the DAW without saving. The file is in
  %APPDATA%\Elektron Emu\Model Cycles.
- Only one Model Cycles at a time: a second instance stays silent and starts
  by itself when you close the first one.

Outputs: one stereo output (the mix). Parameters: Main Volume, DAW Sync
(Start/Stop and clock follow the DAW transport). MIDI in and out.

Uninstall
- Use "Apps > Installed apps" (Model Cycles VST3), or delete the folder
  C:\Program Files\Common Files\VST3\ModelCycles.vst3.
  Your firmware, the cache and your machine file are not removed; delete the
  folders %APPDATA%\Elektron Emu\Model Cycles and %LOCALAPPDATA%\Elektron Emu\Model Cycles
  by hand if you want them gone.

Never distribute your firmware or the folders above: they contain Elektron's
software.
