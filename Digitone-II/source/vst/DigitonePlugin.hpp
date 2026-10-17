// DigitonePlugin.hpp -- la parte audio del VST3: il motore dt_rt (core/), come
// au/Extension/DigitoneAudioUnit.mm. La vista (DigitoneUI.cpp) la raggiunge con
// l'accesso diretto di DPF (stesso processo) per schermo, LED e pannello.
//
// Il firmware di Elektron non e' mai nel plugin: sta nella cartella dell'utente
// (Platform.hpp). Tutto il lavoro lungo (stato del DSP, sessione iniziale della
// ColdFire) e' nel thread di lavoro, mai in quello della DAW.
#pragma once

#include "DistrhoPlugin.hpp"
#include "dncore.h"
#include "DriveModel.hpp"
#include "Firmware.hpp"
#include "Platform.hpp"
#include "Strings.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

START_NAMESPACE_DISTRHO

// lo stato del motore per la vista (come EngineStatus della Syntakt)
struct EngineStatus {
    enum Kind { Starting, NoFirmware, Booting, Busy, Running, Error, NeedTools } kind = Starting;
    std::string text;               // la riga di stato
    std::string firmwareDir;        // dove mettere il firmware (percorso intero)
    std::string warning;            // firmware sconosciuto, file scartati, DSP nativo
    double bootSeconds = 0;         // da quanto dura il primo avvio
    int percent = -1;               // avanzamento del primo avvio, se noto
    std::string detail;             // la fase del primo avvio (costruzione del DSP), o perche' mancano gli strumenti
};

class DigitonePlugin : public Plugin {
public:
    enum Params { kParamVolume, kParamSync, kParamCount };
    static const int kBuses = 1;            // Main: canali 0/1 di dt_rt (slot TDM 0/1 del DSP)

    DigitonePlugin();
    ~DigitonePlugin() override;

    // dal thread della vista
    uint64_t copyScreen(uint8_t screen[128 * 64], uint8_t leds[DN_LEDS]);
    void panel(int kind, int code, int arg);
    double masterVolume() const { return fMaster.load(); }
    void setMasterFromUI(double v);
    EngineStatus status();
    void saveNow() { requestSave(); }                           // Cmd/Ctrl+S nella vista (GUI v2)
    dt_rt* engine() const { return fRt.load(); }                // per il +DRIVE (RPC dal thread della vista)
    std::shared_ptr<DriveModel> drive() const { return fDrive; }

    // il firmware sta nella cartella dell'utente, mai nel bundle
    // "Choose firmware...": controlla il file, lo copia nella cartella e avvia (dal thread della vista)
    bool installFirmware(const std::string& path, std::string& msg);
    // Mac: il compilatore manca. "Install developer tools" lancia xcode-select --install; "Use slow mode" prosegue con l'interprete
    void installTools();
    void skipNative() { fSkipNative = true; wake(); }

protected:
    const char* getLabel() const override { return "DigitoneII"; }
    const char* getDescription() const override { return "Digitone II emulator (runs your own firmware)"; }
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
    void run(const float**, float** outputs, uint32_t frames, const MidiEvent* midi, uint32_t midiCount) override;

private:
    // sincronia col trasporto della DAW: Start/Stop/SPP/clock nel MIDI IN emulato
    struct SyncEv { uint32_t off; uint8_t n, b[3]; };
    struct HostSync {
        static const int kMax = 512;         // eventi per blocco (a 4096 campioni e 300 BPM ne bastano ~12)
        bool started = false, wasMoving = false;
        int64_t nextTick = 0;
        double expectedBeat = 0;
        SyncEv ev[kMax];
        int count = 0;
        // gli eventi del blocco in ordine di tempo, in ev[0..count): run() li fonde con il
        // MIDI dell'host (dt_rt_midi li vuole in ordine di tempo)
        void run(const TimePosition& t, uint32_t frames, double rate);
        void add(uint32_t off, uint8_t a, uint8_t b = 0, uint8_t c = 0, uint8_t n = 1);
    };

    enum Stage { kIdle, kNeedFirmware, kWaitOwner, kPreparing, kRunning, kError, kNeedTools };

    void workerLoop();
    bool startEngine();                      // solo nel thread di lavoro
    void swapEngine(dt_rt* rt);
    void setStatus(Stage st, const std::string& text);
    void wake();
    bool ensureNative();                     // solo nel thread di lavoro; false = annullato
    static void progressCb(void* ctx, int percent, const char* what);

    std::atomic<dt_rt*> fRt{nullptr};
    std::atomic<int> fInRun{0};
    std::atomic<double> fMaster{0.25};
    std::atomic<bool> fSync{true};
    HostSync fHostSync;
    std::vector<float> fBuf;
    float* fMono[2] = {};
    uint32_t fMaxFrames = 0;
    std::atomic<double> fRate{0};            // frequenza del motore in moto
    uint32_t fRtFrames = 0;
    std::atomic<double> fWantRate{0};        // quella chiesta dall'host (activate)
    std::atomic<uint32_t> fWantFrames{0};

    // firmware
    std::vector<uint8_t> fSyx;               // solo nel thread di lavoro
    dnfw::Info fInfo;
    dnfw::Finder fFinder;
    std::string fFirmwareProblem;            // sotto fLock
    std::atomic<int64_t> fPrepStartMs{0};    // quando e' cominciata la preparazione (0 = no)

    // avvio
    std::atomic<Stage> fStage{kIdle};
    std::atomic<bool> fStartPending{false};  // un avvio e' chiesto o in corso (l'export offline lo aspetta)
    std::atomic<bool> fRetry{false};         // riprova subito (firmware appena messo)
    std::atomic<int> fProgress{-1};
    std::atomic<int> fCancel{0};
    double fTriedRate = 0;                   // solo nel thread di lavoro
    std::vector<uint8_t> fPendingState;      // stato di un progetto vecchio (chiave dncore_state)
    std::string fStatus = dnstr::kStatusStarting;
    std::string fRunningWarning;             // sotto fLock
    std::string fBuildDetail;                // la fase della costruzione del DSP / perche' mancano gli strumenti (sotto fLock)
    std::string fNativeNote;                 // solo nel thread di lavoro: avviso sul DSP nativo per la barra
    bool fPersonalNative = false;            // solo nel thread di lavoro: in uso la libreria personale (dsp-native.dylib)
    bool fNativeFailed = false;              // solo nel thread di lavoro: la costruzione e' fallita in questa istanza, non si ritenta
    std::atomic<bool> fSkipNative{false};    // "Use slow mode": niente DSP nativo
    std::mutex fLock;                        // fPendingState, fStatus, fFirmwareProblem
    std::shared_ptr<DriveModel> fDrive;

    // la macchina unica (come l'hardware): RAM e +Drive in un file sul disco, uno per
    // firmware. Una sola istanza alla volta (lucchetto): le altre restano mute e partono
    // da sole quando la prima si chiude.
    struct MachineLock {
#if defined(_WIN32)
        void* h = nullptr;
#else
        int fd = -1;
#endif
        bool acquire(const dnplat::fs::path& path);
        void release();
    };
    dnplat::fs::path machinePath() const;
    bool tryOwn();
    bool saveMachine();
    void requestSave();
    MachineLock fMachineLock;
    std::atomic<bool> fOwner{false};
    std::mutex fSaveLock;
    std::thread fWorker;
    std::mutex fWakeMx;
    std::condition_variable fWakeCv;
    bool fQuit = false;
    std::atomic<bool> fOffline{false};
    std::atomic<int64_t> fSlowUntilMs{0};     // il DSP e' rimasto indietro (underrun) da poco: avviso nella barra
    std::atomic<int64_t> fLastActivityMs{0};  // trasporto in moto o MIDI ricevuto (thread audio)
    std::atomic<bool> fSaveRequested{false};  // getState e il +DRIVE chiedono, il thread salva

    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DigitonePlugin)
};

END_NAMESPACE_DISTRHO
