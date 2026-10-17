Digitakt II emulator - VST3 for Windows x64
===========================================

This plug-in runs the Digitakt II operating system inside an emulator. It does
NOT include Elektron's firmware: you need your own copy of the official
Digitakt II OS 1.16 (.syx), downloaded from Elektron's website.

Install
1. Close your DAW.
2. Run DigitaktII-<version>-win64-setup.exe. It installs the plug-in in
      C:\Program Files\Common Files\VST3\digitakt2.vst3
   and creates your firmware folder
      %APPDATA%\Elektron Emu\Digitakt II\Firmware
   (it offers to open it at the end).
3. Download Digitakt II OS 1.16 from elektron.se (Support > Digitakt II >
   Downloads), unzip it and put the .syx file in that folder. Or, in the
   plug-in, click CHOOSE FIRMWARE... and pick the .syx: it is copied there.
4. Open the DAW and rescan plug-ins. The instrument is "Digitakt II" (Elektron Emu).

First start
- The first time, the plug-in boots the firmware from scratch: the status line
  says "first start: booting the firmware". It takes 15 seconds to a couple of
  minutes, once. The result is cached in %LOCALAPPDATA%\dtcore.
- As on the real machine there is one Digitakt: projects, sounds and samples
  stay on its +Drive for every DAW project, and the plug-in always resumes
  where you left it, even if you close without saving. The file is in
  %APPDATA%\dtcore.
- One Digitakt at a time: a second instance stays silent and starts by itself
  when you close the first one.

Window and zoom
- Bottom right: 75% 100% 125% 150% ask the DAW to resize the window. The
  corner handle drags it.
- If the DAW does not resize, the status line says so and the zoom applies
  the next time the window opens.

Outputs: Mix + 16 tracks (stereo). Parameters: Main Volume, Sync DAW
(Start/Stop and clock from the DAW transport).

Uninstall: Settings > Apps > "Digitakt II emulator (VST3)", or run
C:\Program Files\Elektron Emu\Digitakt II\uninstall.exe.
Your firmware and your machine (%APPDATA%\Elektron Emu\Digitakt II, %APPDATA%\dtcore)
are kept.

Independent project, not affiliated with or endorsed by Elektron Music
Machines MAV AB. Elektron and Digitakt are trademarks of Elektron Music
Machines MAV AB, used only to say which hardware OS this software runs.
VST is a trademark of Steinberg Media Technologies GmbH.
Licence: GPL-2.0-or-later (LICENSE.txt). Source code: see the project page.
