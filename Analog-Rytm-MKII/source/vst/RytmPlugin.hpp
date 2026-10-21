// RytmPlugin.hpp -- la parte audio del VST3 dell'Analog Rytm MKII: il motore dt_rt
// (core/, arcore) come au/Extension/RytmAudioUnit.mm. La vista (RytmUI.cpp) la
// raggiunge con l'accesso diretto di DPF (stesso processo) per schermo, LED,
// pad e pannello. Modellato sul VST3 del Digitakt II (digitakt/emulator/vst).
//
// Il plugin non contiene firmware ne' stati derivati: il firmware e' un .syx
// dell'utente nella cartella Firmware (RytmPaths.hpp), lo stato iniziale si
// genera al primo avvio dal boot freddo (dt_coldboot_session) e va in cache, la
// macchina unica (RAM, +Drive, flash con la calibrazione) in un file per firmware.
#pragma once

#include "DistrhoPlugin.hpp"
#include "arcore.h"
#include "SeqMem.hpp"
#include "DriveModel.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

START_NAMESPACE_DISTRHO

// lo stato del motore per la vista (come EngineStatus della Syntakt): la scheda sopra il
// pannello e la riga di stato in basso
struct RytmViewInfo {
    enum Kind { kStarting, kNoFirmware, kBooting, kBusy, kRunning, kError };
    Kind kind = kStarting;
    std::string text;                     // il dettaglio (errore) o la riga di stato
    std::string firmwareDir;              // dove mettere il firmware (percorso intero)
    std::string firmwareLabel;            // "Analog Rytm MKII OS 1.73" se c'e'
    std::string warning;                  // firmware sconosciuto, file scartati
    std::string phase;                    // fase del primo avvio (inglese, da arcore)
    double bootSeconds = 0;               // da quanto dura il primo avvio
    double progress = 0;                  // 0..1
    uint64_t version = 0;
};

class RytmPlugin : public Plugin {
public:
    enum Params { kParamVolume, kParamSync, kParamCount };
    static const int kBuses = DT_RT_CHANNELS / 2;        // Mix + 8 voci, stereo

    RytmPlugin();
    ~RytmPlugin() override;

    // dal thread della vista
    uint64_t copyScreen(uint8_t screen[128 * 64], uint8_t lit[128], uint8_t pads[12][3]);
    // the pattern read in the OS memory (SeqMem.hpp), false while there is none
    bool seqData(SeqData& d);
    // the sequencer loops on one step page (0..7), -1 all the pages (SeqMem.hpp, loop)
    void setLoopPage(int page);
    int loopPage() const { return fLoopPage.load(); }
    void panel(int kind, int code, int arg);
    double masterVolume() const { return fMaster.load(); }
    void setMasterFromUI(double v);
    RytmViewInfo viewInfo();
    dt_rt* engine() const { return fRt.load(); }       // per il +Drive (RPC dal thread della vista)
    std::shared_ptr<DriveModel> drive() const { return fDrive; }   // nullptr nell'istanza fittizia
    // "Scegli firmware...": copia il .syx nella cartella Firmware e fa partire il motore
    // (dal thread della vista: la copia e' piccola, l'avvio lo fa il thread di servizio)
    bool installFirmware(const std::string& path, std::string& error);
    std::string photoPath() const;                     // foto del pannello, o vuoto
    void requestSave();                                // salva la macchina adesso (thread di servizio)

protected:
    const char* getLabel() const override { return "AnalogRytmMKII"; }
    const char* getDescription() const override { return "Emulatore dell'Analog Rytm MKII (firmware vero dell'utente, arcore)"; }
    const char* getMaker() const override { return "Elektron Emu"; }
    const char* getHomePage() const override { return "https://github.com/"; }
    const char* getLicense() const override { return "GPL-2.0-or-later"; }
    uint32_t getVersion() const override { return d_version(0, 1, 0); }

    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
    void initPortGroup(uint32_t groupId, PortGroup& group) override;
    void initParameter(uint32_t index, Parameter& p) override;
    void initState(uint32_t index, State& s) override;
    float getParameterValue(uint32_t index) const override;
    void setParameterValue(uint32_t index, float value) override;
    String getState(const char* key) const override;
    void setState(const char* key, const char* value) override;

    void activate() override;
    void ioChanged(uint16_t numInputs, uint16_t numOutputs) override;
    void run(const float**, float** outputs, uint32_t frames, const MidiEvent* midi, uint32_t midiCount) override;

private:
    // sincronia col trasporto della DAW: Start/Stop/SPP/clock nel MIDI IN emulato
    struct HostSync {
        bool started = false, wasMoving = false;
        int64_t nextTick = 0;
        double expectedBeat = 0;
        void run(dt_rt* rt, const TimePosition& t, uint32_t frames, double rate);
    };

    // il firmware trovato nella cartella dell'utente
    struct Firmware {
        std::string path, name, sha8, version, label;
        bool valid = false;               // formato .syx/ELE3 riconosciuto
        bool supported = false;           // il motore ha il profilo di questa versione
        std::string error;                // se !valid
    };

    bool startEngine(double rate, uint32_t frames);
    void swapEngine(dt_rt* rt);
    void coldBoot(double rate, uint32_t frames);
    bool scanFirmware(Firmware& out);
    void setView(RytmViewInfo::Kind k, const std::string& text);
    std::string coldBootCachePath() const;

    std::atomic<dt_rt*> fRt{nullptr};

    std::atomic<int> fLoopPage{-1};
    std::atomic<int> fInRun{0};
    std::atomic<double> fMaster{1.0};
    std::atomic<bool> fSync{true};
    uint32_t fNumOutputs = DISTRHO_PLUGIN_NUM_OUTPUTS;
    HostSync fHostSync;
    std::vector<float> fBuf;
    float* fChan[DT_RT_CHANNELS] = {};
    uint32_t fMaxFrames = 0;
    double fRate = 0;
    uint32_t fRtFrames = 0;

    std::vector<uint8_t> fSyx;
    Firmware fFw;
    std::vector<uint8_t> fPendingState;      // stato prima che il motore parta
    std::mutex fLock;                        // fPendingState, fView
    std::recursive_mutex fStartLock;         // startEngine: host, thread di servizio e primo avvio
    RytmViewInfo fView;
    std::string fWarning;                    // firmware sconosciuto o file scartati
    std::atomic<int64_t> fBootStartMs{0};
    std::thread fBoot;
    std::atomic<int> fBootCancel{0};
    std::atomic<bool> fBooting{false};
    std::shared_ptr<DriveModel> fDrive;      // il pannello +Drive (anche senza vista)

    // la macchina unica (come l'hardware): RAM, +Drive e flash in un file sul disco,
    // condiviso da tutti i progetti. Una sola istanza alla volta (lucchetto sul
    // file): le altre restano mute e partono da sole quando quella si chiude.
    struct MachineLock {
#if defined(_WIN32)
        void* h = nullptr;
#else
        int fd = -1;
#endif
        bool acquire(const std::string& path);
        void release();
    };
    std::string machinePath() const;
    bool tryOwn();
    bool saveMachine();
    MachineLock fMachineLock;
    std::atomic<bool> fOwner{false};
    double fWantRate = 0;                    // per partire quando c'e' il firmware / si libera la macchina
    uint32_t fWantFrames = 0;
    std::mutex fSaveLock;
    std::thread fService;
    std::mutex fServiceMx;
    std::condition_variable fServiceCv;
    bool fServiceStop = false;
    std::atomic<bool> fKick{false};           // riprova subito ad avviare (firmware scelto dalla vista)
    std::atomic<bool> fRescan{false};         // ricontrolla la cartella Firmware (dopo "Scegli firmware")
    std::string fSigFailed, fSigBad;          // elenco della cartella all'ultimo esito negativo: se non cambia, niente da rifare
    std::atomic<int64_t> fLastActivityMs{0};  // trasporto in moto o MIDI ricevuto (thread audio)
    std::atomic<bool> fSaveRequested{false};  // getState e il +Drive chiedono, il thread salva

    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(RytmPlugin)
};

END_NAMESPACE_DISTRHO
