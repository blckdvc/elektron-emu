// NativeBuild.hpp -- il DSP nativo costruito sul computer dell'utente (vst/README.md, "DSP nativo").
//
// Al primo avvio, dopo il boot dell'interprete (stato DNDS), il plugin traduce il programma DSP in C
// (dn_dsp_gensrc, core/dsp/dn_gensrc.inc), lo compila con il compilatore del computer e carica la
// libreria (dn_dsp_native_load): audio identico all'interprete, ma in tempo reale. Il C, gli oggetti e la
// libreria derivano dal firmware: restano nella cartella dell'utente, mai nel plugin.
//
//   Mac       clang di `xcrun --find clang` (Command Line Tools o un Xcode qualunque); se manca, build()
//             ritorna NeedTools e la vista offre `xcode-select --install`.
//   Windows   llvm-mingw (versione fissa, SHA-256 fissato nel codice) scaricato in
//             %LOCALAPPDATA%\Elektron Emu\toolchain\ al primo avvio.
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "Platform.hpp"

namespace dnnative {

namespace fs = dnplat::fs;

enum class Result { Ready, NeedTools, Failed, Cancelled };

struct Hooks {
    volatile int* cancel = nullptr;
    // percent 0..100 (-1: sconosciuto) e il testo della fase, da un thread qualunque
    std::function<void(int percent, const std::string& text)> progress;
};

// "dsp-<12 esadecimali>-n1.dylib" / ".dll": legata all'impronta della sezione DSP del firmware
// (n1 = versione del traduttore e del runtime: cambia se cambiano)
std::string libName(const std::string& programHash);

// Costruisce `lib` dallo stato DNDS dell'interprete. workRoot: cartella di lavoro (sorgenti e oggetti, tolta
// alla fine); toolRoot: dove sta il compilatore scaricato (Windows). Ritorna:
//   Ready      la libreria e' in `lib`
//   NeedTools  (Mac) manca il compilatore: `detail` dice perche'; riprovare dopo l'installazione
//   Failed     `detail` dice perche' (rete, compilazione, disco)
//   Cancelled  hooks.cancel e' diventato 1
Result build(const uint8_t* dnds, size_t dndsLen, const fs::path& lib, const fs::path& workRoot, const fs::path& toolRoot,
             const Hooks& hooks, std::string& detail);

// Mac: lancia `xcode-select --install` (la finestra di sistema chiede conferma e scarica gli strumenti)
void openToolInstaller();

// il compilatore c'e' e funziona? (build lo controlla comunque). false: why dice perche'
bool compilerAvailable(const fs::path& toolRoot, std::string& why);

} // namespace dnnative
