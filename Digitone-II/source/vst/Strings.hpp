// Strings.hpp -- tutti i testi che l'utente finale vede nel plugin (vista, messaggi
// di stato, errori, nomi dei parametri): SOLO in inglese, raccolti qui per
// rivederli in un posto solo. Niente traduzioni ne' selettore di lingua. I
// commenti e i documenti di sviluppo restano in italiano.
#pragma once

#include <cstring>
#include <string>

namespace dnstr {

// alcuni messaggi del core (dn_data.c, dn_prepare.c) sono in italiano: si traducono qui
inline std::string coreMsg(std::string m)
{
    static const char* const kMap[][2] = {
        {"il firmware non risponde", "the firmware does not answer"},
        {"il firmware risponde", "the firmware replied with an error"},
        {"risposta corta", "short reply"},
        {"illeggibile", "unreadable"},
        {"non e' ancora disponibile", "is not available yet"},
        {"svuota", "clear"}, {"apri ", "open "}, {"leggi", "read"}, {"scrivi", "write"},
    };
    for (const auto& e : kMap)
        for (size_t p = m.find(e[0]); p != std::string::npos; p = m.find(e[0], p + std::strlen(e[1])))
            m.replace(p, std::strlen(e[0]), e[1]);
    return m;
}

// ---- parametri
static const char* const kParamVolume = "Main Volume";
static const char* const kParamSync = "Sync DAW";
static const char* const kPortMain = "Main";
static const char* const kPortMainL = "Main L";
static const char* const kPortMainR = "Main R";
static const char* const kStateLabel = "Digitone II state";

// ---- riga di stato della vista e scheda (le stesse frasi della Syntakt)
static const char* const kStatusRunning = "Digitone II - emulator (VST3)";
static const char* const kStatusStarting = "engine not started yet";
static const char* const kWarnSlowDsp = "DSP too slow for real time: render offline (see README)";
static const char* const kWarnNativeDsp = "native DSP (personal use)";
inline std::string warnNoNative(const std::string& why)
{
    return "native DSP not available (" + why + "): using the slow interpreter, real time will not hold";
}
static const char* const kWarnNoNativeSkipped = "native DSP skipped: using the slow interpreter, real time will not hold";
static const char* const kStatusNoJit = "the host does not allow executable memory (hardened runtime: allow-jit)";
static const char* const kStatusFirmwareMissing = "firmware missing";
static const char* const kStatusBusy = "Digitone II is already open in another instance: close that one and this one starts by itself";
static const char* const kStatusBooting = "First start: preparing the machine from the firmware (once only)";
static const char* const kStatusBuilding = "First start: building the DSP for this computer (once only, a few minutes)";
static const char* const kStatusNeedTools = "the Apple developer tools are needed to build the DSP";
inline std::string engineError(const std::string& e) { return "engine: " + coreMsg(e); }
inline std::string prepareError(const std::string& e) { return "cannot prepare the machine: " + coreMsg(e); }

static const char* const kCardNoFwTitle = "Digitone II firmware not found";
static const char* const kCardNoFwBody =
    "This plugin does not include the Elektron firmware. Put your Digitone II firmware file (.syx) in the folder below "
    "and the engine starts by itself.";
static const char* const kCardBootTitle = "First start";
inline std::string cardBootBody(int seconds)
{
    return "Preparing the machine from the firmware (this happens once, then it is cached). " + std::to_string(seconds) +
           " s elapsed.";
}
inline std::string cardBuildBody(const std::string& detail, int seconds)
{
    return "Building the DSP for this computer (this happens once, then it is kept): a few minutes. Until it is ready the plugin "
           "cannot play in real time. " + detail + " (" + std::to_string(seconds) + " s elapsed).";
}
static const char* const kCardToolsTitle = "Developer tools needed";
inline std::string cardToolsBody(const std::string& why)
{
    std::string b =
        "To play in real time the plugin builds its DSP on this Mac, once, and that needs Apple's command line developer tools "
        "(free, about 1 GB). Press Install: macOS asks for confirmation and downloads them, and the plugin goes on by itself "
        "when they are ready. Without them it only runs the slow interpreter, fine for rendering offline but not for playing.";
    if (!why.empty()) b += " (" + why + ")";
    return b;
}
static const char* const kToolsInstall = "Install developer tools";
static const char* const kToolsSkip = "Use slow mode";
static const char* const kCardBusyTitle = "Digitone II already open";
static const char* const kCardBusyBody =
    "There is only one machine, like the hardware: this instance stays silent and starts by itself when you close the other one.";
static const char* const kCardErrorTitle = "The engine cannot start";
static const char* const kFwFolderLabel = "Firmware folder:";
static const char* const kFwOpenFolder = "Open folder";
static const char* const kFwChoose = "Choose firmware...";
static const char* const kFwPickTitle = "Choose the Digitone II firmware (.syx)";
static const char* const kFwCopiedNote = "Firmware copied to the folder: the engine is starting";

// ---- messaggi sul firmware
inline std::string fwNotReadable(const std::string& why) { return "not a readable Elektron firmware (" + why + ")"; }
inline std::string fwWrongMachine(const std::string& machine)
{
    return "this is the " + machine + " firmware, not the Digitone II one";
}
static const char* const kFwUnknownMachine = "this firmware is not for the Digitone II";
inline std::string fwUnknownVersion(const std::string& v)
{
    return "firmware version " + (v.empty() ? std::string("?") : v) + " has not been tested: trying it anyway";
}
static const char* const kFwUnreadable = "the file cannot be read";
static const char* const kFwNoFolder = "the firmware folder is not available";
inline std::string fwCannotWrite(const std::string& dir) { return "cannot write to " + dir; }
inline std::string fwCopied(const std::string& name) { return "firmware copied: " + name; }
inline std::string fwFileProblem(const std::string& name, const std::string& why) { return name + ": " + why; }

// ---- +DRIVE
static const char* const kDriveTitle = "+DRIVE";
static const char* const kDriveBtnSyx = "SEND .SYX";
static const char* const kDriveBtnSave = "SAVE";
static const char* const kDriveBtnClear = "CLEAR";
static const char* const kDriveBtnRefresh = "REFRESH";
static const char* const kDriveConfirm = "CONFIRM?";
static const char* const kDriveColName = "NAME";
static const char* const kDriveNotReady = "emulator not ready yet";
static const char* const kDrivePickSyx = "Send SysEx to the Digitone II (MIDI IN)";
static const char* const kDriveSaveTitle = "Save the selected item";
static const char* const kDriveOnlySyx = "nothing to send: only .syx (SysEx) files";
static const char* const kDriveEmpty = "(empty)";
inline std::string driveReading(const std::string& p) { return "reading " + p + " ..."; }
inline std::string driveCounts(int dirs, int items)
{
    return std::to_string(dirs) + " folders, " + std::to_string(items) + " items";
}
inline std::string driveSent(const std::string& name, size_t n) { return "sent " + name + " (" + std::to_string(n) + " bytes)"; }
inline std::string driveSentAll(int n) { return std::to_string(n) + " .syx files sent to MIDI IN"; }
inline std::string driveSaved(size_t n, const std::string& p)
{
    return "saved " + std::to_string(n) + " bytes to " + p;
}
static const char* const kDriveClearing = "clearing ...";
static const char* const kDriveSending = "sending SysEx ...";
static const char* const kDriveCannotWrite = "cannot write the file";

// ---- zoom
static const char* const kZoomNote = "The host did not resize the window: the zoom applies the next time it opens";

// ---- volume
inline std::string volumeText(int pct) { return "vol " + std::to_string(pct) + "%"; }

} // namespace dnstr
