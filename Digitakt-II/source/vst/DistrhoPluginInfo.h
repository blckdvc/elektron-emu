// DistrhoPluginInfo.h -- il Digitakt II come VST3, CLAP e AUv2 con DPF.
// Stesso motore (dtcore, dt_rt) e stessa faccia dell'AU (au/): 17 uscite
// stereo (Mix + 16 tracce), MIDI in/out, Main Volume e Sync DAW, stato intero
// nel progetto. Vista ridimensionabile a proporzioni fisse e tasti di zoom che
// chiedono all'host una nuova misura (IPlugFrame::resizeView in VST3).
#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND "Elektron Emu"
// l'AUv2 convive con l'AUv3 di au/ (Simo dkt2 "Digitakt II"): altro codice, altro nome
#if defined(DIGITAKT_AU2)
#define DISTRHO_PLUGIN_NAME  "Digitakt II (AU2)"
#else
#define DISTRHO_PLUGIN_NAME  "Digitakt II"
#endif
#define DISTRHO_PLUGIN_URI   "urn:simo:digitakt2"
#define DISTRHO_PLUGIN_CLAP_ID "simo.digitakt2"

#define DISTRHO_PLUGIN_BRAND_ID  Simo
#if defined(DIGITAKT_AU2)
#define DISTRHO_PLUGIN_UNIQUE_ID dk2v
#else
#define DISTRHO_PLUGIN_UNIQUE_ID dkt2
#endif
#define DISTRHO_PLUGIN_AU_TYPE   aumu

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_IS_SYNTH        1
#define DISTRHO_PLUGIN_NUM_INPUTS      0
#define DISTRHO_PLUGIN_NUM_OUTPUTS     34
// solo AU: oltre alle 34 uscite in un bus, lo stereo del Mix (Logic, GarageBand)
#define DISTRHO_PLUGIN_EXTRA_IO        { 0, 2 },
#define DISTRHO_PLUGIN_WANT_DIRECT_ACCESS 1
#define DISTRHO_PLUGIN_WANT_LATENCY    1
#define DISTRHO_PLUGIN_WANT_MIDI_INPUT 1
#define DISTRHO_PLUGIN_WANT_MIDI_OUTPUT 1
#define DISTRHO_PLUGIN_WANT_STATE      1
#define DISTRHO_PLUGIN_WANT_FULL_STATE 1
#define DISTRHO_PLUGIN_WANT_TIMEPOS    1

#define DISTRHO_UI_USE_NANOVG      1
#define DISTRHO_UI_USER_RESIZABLE  1
#define DISTRHO_UI_FILE_BROWSER    1
#define DISTRHO_UI_DEFAULT_WIDTH   712
#define DISTRHO_UI_DEFAULT_HEIGHT  605

#define DISTRHO_PLUGIN_VST3_CATEGORIES "Instrument|Synth|Sampler"
#define DISTRHO_PLUGIN_CLAP_FEATURES   "instrument", "synthesizer", "sampler", "stereo"

#endif
