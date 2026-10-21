// CyclesPlugin.hpp -- la parte audio del VST3: il motore dt_rt (core/), come
// au/Extension/CyclesAudioUnit.mm. La vista (CyclesUI.cpp) la raggiunge con
// l'accesso diretto di DPF (stesso processo) per schermo, LED, pannello e
// per il firmware (FirmwareStore.hpp).
#pragma once

#include "DistrhoPlugin.hpp"
#include "mccore.h"
#include "SeqMem.hpp"
#include "FirmwareStore.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

START_NAMESPACE_DISTRHO

class CyclesPlugin : public Plugin {
public:
    enum Params { kParamVolume, kParamSync, kParamCount };

    // in che punto e' il plugin (per la vista)
    enum Phase {
        kNoFirmware,      // nessun .syx valido nella cartella: audio muto
        kBooting,         // primo avvio: il boot del firmware (una volta sola)
        kBusy,            // la macchina e' di un'altra istanza
        kRunning,
        kError
    };

    CyclesPlugin();
    ~CyclesPlugin() override;

    // dal thread della vista
    uint64_t copyScreen(uint8_t screen[128 * 64], uint8_t leds[8]);
    // the pattern read in the OS memory (SeqMem.hpp), false while there is none
    bool seqData(SeqData& d);
    // the sequencer loops on one step page (0..7), -1 all the pages (SeqMem.hpp, loop)
    void setLoopPage(int page);
    int loopPage() const { return fLoopPage.load(); }
    void panel(int kind, int code, int arg);
    double outputVolume() const { return fVolume.load(); }
    void setVolumeFromUI(double v);
    std::string statusText();
    Phase phase() const { return fPhase.load(); }
    int bootPercent() const;                          // 0..100 nel primo avvio
    std::string firmwareFolder() const;               // la cartella dove va messo il .syx
    std::string firmwareNote();                       // "versione sconosciuta ..." o ""
    // "Scegli firmware...": copia il .syx nella cartella e riparte. -> "" o l'errore
    std::string installFirmware(const std::string& path);
    void rescanFirmware();                            // l'utente ha messo il file a mano

protected:
    const char* getLabel() const override { return "ModelCycles"; }
    const char* getDescription() const override { return "Model:Cycles emulator (runs the Elektron firmware you provide)"; }
    const char* getMaker() const override { return "Elektron Emu"; }
    const char* getHomePage() const override { return "https://github.com/"; }
    const char* getLicense() const override { return "GPL-2.0-or-later"; }
    uint32_t getVersion() const override { return d_version(0, 1, 0); }

    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
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
    struct HostSync {
        bool started = false, wasMoving = false;
        int64_t nextTick = 0;
        double expectedBeat = 0;
        void run(dt_rt* rt, const TimePosition& t, uint32_t frames, double rate);
    };

    bool startEngine(double rate, uint32_t frames);
    void swapEngine(dt_rt* rt);
    void coldBoot(double rate, uint32_t frames);
    bool loadFirmware();                     // fSyx dalla cartella; false = nessuno valido
    std::string coldBootCachePath() const;
    void setStatus(Phase p, const std::string& s);

    std::atomic<dt_rt*> fRt{nullptr};

    std::atomic<int> fLoopPage{-1};
    std::atomic<int> fInRun{0};
    std::atomic<double> fVolume{0.25};
    std::atomic<bool> fSync{true};
    HostSync fHostSync;
    std::vector<float> fBuf;
    float* fOut[2] = {};
    uint32_t fMaxFrames = 0;
    double fRate = 0;
    uint32_t fRtFrames = 0;

    std::vector<uint8_t> fSyx;
    std::string fFwPath, fFwNote, fFwSig;
    std::vector<uint8_t> fPendingState;      // stato del progetto prima che il motore parta
    std::mutex fLock;                        // fPendingState, fStatus, fFw*
    std::recursive_mutex fStartLock;         // startEngine: host, setState e thread del primo avvio
    std::string fStatus = "engine not started yet";
    std::atomic<Phase> fPhase{kNoFirmware};
    std::thread fBoot;
    std::atomic<int> fBootCancel{0};
    std::atomic<bool> fBooting{false};
    std::atomic<int64_t> fBootDone{0};       // istruzioni del primo avvio

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
    std::atomic<int64_t> fLastActivityMs{0}; // trasporto in moto o MIDI ricevuto (thread audio)
    std::atomic<bool> fSaveRequested{false}; // getState lo chiede, il thread salva
    std::atomic<bool> fRescan{false};
    void requestSave();

    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(CyclesPlugin)
};

END_NAMESPACE_DISTRHO
