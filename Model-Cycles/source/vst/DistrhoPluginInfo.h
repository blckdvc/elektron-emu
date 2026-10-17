// DistrhoPluginInfo.h -- il Model:Cycles come VST3 con DPF.
// Stesso motore (mccore, dt_rt) e stessa faccia dell'AU (au/): un bus stereo
// (il mix dell'SSI0, l'unica uscita del firmware), MIDI in/out, Volume e Sync
// DAW. Vista ridimensionabile a proporzioni fisse (840x592 = 75% del pannello
// 1120x760 piu' la riga di 22 px in basso con stato e zoom, come la Syntakt). Il firmware non e' nel
// plugin: lo cerca nella cartella dell'utente (vst/README.md).
#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND "Elektron Emu"
#define DISTRHO_PLUGIN_NAME  "Model Cycles"
#define DISTRHO_PLUGIN_URI   "urn:simo:modelcycles"
#define DISTRHO_PLUGIN_CLAP_ID "simo.modelcycles"

#define DISTRHO_PLUGIN_BRAND_ID  Simo
#define DISTRHO_PLUGIN_UNIQUE_ID mcyv
#define DISTRHO_PLUGIN_AU_TYPE   aumu

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_IS_SYNTH        1
#define DISTRHO_PLUGIN_NUM_INPUTS      0
#define DISTRHO_PLUGIN_NUM_OUTPUTS     2
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
#define DISTRHO_UI_DEFAULT_WIDTH   797
#define DISTRHO_UI_DEFAULT_HEIGHT  554

#define DISTRHO_PLUGIN_VST3_CATEGORIES "Instrument|Synth"
#define DISTRHO_PLUGIN_CLAP_FEATURES   "instrument", "synthesizer", "stereo"

#endif
