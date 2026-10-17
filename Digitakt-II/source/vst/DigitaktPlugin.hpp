// DigitaktPlugin.hpp -- la parte audio del VST3: il motore dt_rt (core/), come
// au/Extension/DigitaktAudioUnit.mm. La vista (DigitaktUI.cpp) la raggiunge
// con l'accesso diretto di DPF (stesso processo) per schermo, LED e pannello.
#pragma once

#include "DistrhoPlugin.hpp"
#include "dtcore.h"
#include "DriveModel.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

START_NAMESPACE_DISTRHO

// lo stato del motore per la vista (come SyntaktPlugin.hpp, la Syntakt e' il riferimento)
struct EngineStatus {
    enum Kind { Starting, NoFirmware, Booting, Busy, Running, Error } kind = Starting;
    std::string text;               // il motivo, per Error
    std::string firmwareDir;        // dove mettere il firmware (percorso intero)
    std::string warning;            // file scartati nella cartella
    double bootSeconds = 0;         // da quanto dura il primo avvio
};

class DigitaktPlugin : public Plugin {
public:
    enum Params { kParamVolume, kParamSync, kParamCount };
    static const int kBuses = 17;           // Mix + 16 tracce, mono dal motore, stereo verso l'host

    DigitaktPlugin();
    ~DigitaktPlugin() override;

    // dal thread della vista
    uint64_t copyScreen(uint8_t screen[128 * 64], uint8_t leds[50]);
    void panel(int kind, int code, int arg);
    double masterVolume() const { return fMaster.load(); }
    void setMasterFromUI(double v);
    EngineStatus status();
    dt_rt* engine() const { return fRt.load(); }       // per il +Drive (RPC dal thread della vista)
    std::shared_ptr<DriveModel> drive() const { return fDrive; }   // nullptr nell'istanza fittizia

    // il firmware sta nella cartella dell'utente, mai nel bundle (README, "Firmware"):
    // ~/Library/Application Support/Elektron Emu/Digitakt II/Firmware, %APPDATA%\Elektron Emu\Digitakt II\Firmware
    static std::string userDir();            // .../Elektron Emu/Digitakt II (creata se manca)
    static std::string firmwareDir();        // .../Elektron Emu/Digitakt II/Firmware (creata se manca)
    bool needsFirmware() const { return fNeedFirmware.load(); }
    std::string firmwareProblem();           // perche' i .syx della cartella non vanno (vuoto se non ce ne sono)
    // "Scegli firmware...": controlla il file, lo copia nella cartella e avvia (dal thread della vista)
    bool installFirmware(const std::string& path, std::string& msg);

protected:
    const char* getLabel() const override { return "DigitaktII"; }
    const char* getDescription() const override { return "Runs the Digitakt II OS supplied by the user (dtcore emulator)"; }
    const char* getMaker() const override { return "Elektron Emu"; }
    const char* getHomePage() const override { return "https://github.com/"; }
    const char* getLicense() const override { return "GPL-2.0-or-later"; }
    uint32_t getVersion() const override { return d_version(0, 8, 0); }

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

    bool startEngine(double rate, uint32_t frames);
    void swapEngine(dt_rt* rt);
    void coldBoot(double rate, uint32_t frames);
    bool locateFirmware();                   // il primo .syx valido della cartella -> fSyx
    void wakeWorker();
    std::string coldBootCachePath() const;

    std::atomic<dt_rt*> fRt{nullptr};
    std::atomic<int> fInRun{0};
    std::atomic<double> fMaster{0.25};
    std::atomic<bool> fSync{true};
    uint32_t fNumOutputs = DISTRHO_PLUGIN_NUM_OUTPUTS;     // l'AU in stereo ne usa 2 (ioChanged)
    HostSync fHostSync;
    std::vector<float> fBuf;
    float* fMono[kBuses] = {};
    uint32_t fMaxFrames = 0;
    double fRate = 0;
    uint32_t fRtFrames = 0;

    std::vector<uint8_t> fSyx;
    std::atomic<bool> fNeedFirmware{false};  // nessun .syx valido nella cartella: la vista lo chiede
    std::atomic<bool> fRetryStart{false};    // firmware appena messo dalla vista: riprova subito
    std::string fFirmwareProblem;            // sotto fLock
    std::vector<std::string> fRejected;      // .syx gia' scartati (nome|misura|data), non si rileggono
    std::vector<uint8_t> fPendingState;      // stato del progetto prima che il motore parta
    std::mutex fLock;                        // fPendingState, fStatus
    std::recursive_mutex fStartLock;         // startEngine: host, setState e thread del primo avvio
    std::string fStatus = "engine not started yet";
    std::thread fBoot;
    std::atomic<int> fBootCancel{0};
    std::atomic<bool> fBooting{false};
    std::atomic<int64_t> fBootStartMs{0};
    std::atomic<bool> fBusy{false};          // la macchina e' di un'altra istanza
    std::atomic<bool> fError{false};         // motore o boot falliti (il motivo in fStatus)
    std::shared_ptr<DriveModel> fDrive;      // il pannello +Drive (anche senza vista)

    // la macchina unica (come l'hardware): RAM e +Drive in un file sul disco,
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
    double fWantRate = 0;                    // per partire quando la macchina si libera
    uint32_t fWantFrames = 0;
    std::mutex fSaveLock;
    std::thread fAutosave;
    std::mutex fAutosaveMx;
    std::condition_variable fAutosaveCv;
    bool fAutosaveStop = false;
    std::atomic<int64_t> fLastActivityMs{0};  // trasporto in moto o MIDI ricevuto (thread audio)
    std::atomic<bool> fSaveRequested{false};  // getState e il +Drive chiedono, il thread salva
    void requestSave();

    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DigitaktPlugin)
};

END_NAMESPACE_DISTRHO
