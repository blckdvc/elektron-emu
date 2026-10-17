# dncore: il motore del Digitone II in C (piano: docs/AU-PORT.md).
#
# Riusa il motore comune del Digitakt II (digitakt/emulator/core: la macchina
# ColdFire, l'estrazione del firmware, l'impronta SHA-256, il tempo reale e il
# client del +Drive) cosi' com'e', dalla sua cartella, come la Syntakt: qui solo il
# motore del Digitone, che implementa l'API dt_engine_* di dtcore.h, e il DSP.
#
#   core/scripts/build-unicorn.sh /percorso/unicorn-m68k-dn15     (14 patch + hook-index)
#   core/scripts/build-dsp.sh snapshots/dsp_main.snap /percorso/dncore-dsp
#   cmake -S core -B BUILD -G Ninja -DDTCORE_UNICORN_PREFIX=/percorso/unicorn-m68k-dn15 \
#         -DDNCORE_DSP_DIR=/percorso/dncore-dsp
#   cmake --build BUILD
#
# Prodotti: libdncore.a (per il plugin, insieme a DNCORE_DSP_DIR/libdndsp.a, che
# deriva dal firmware e si fa solo nella build locale) e l'eseguibile di prova
# `dncore`. Solo arm64: il DSP ricompilato usa FPCR.
cmake_minimum_required(VERSION 3.20)
project(dncore C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(DTCORE_UNICORN_PREFIX "" CACHE PATH "prefisso della Unicorn con le patch (core/scripts/build-unicorn.sh)")
set(DNCORE_DSP_DIR "" CACHE PATH "il DSP ricompilato (core/scripts/build-dsp.sh): libdndsp.a")
# il motore comune: ../dtcore nell'export pubblico (copia), altrimenti la cartella del Digitakt II
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../dtcore/src/machine.c")
  set(_dtcore_default "${CMAKE_CURRENT_SOURCE_DIR}/../dtcore")
else()
  set(_dtcore_default "${CMAKE_CURRENT_SOURCE_DIR}/../../../digitakt/emulator/core")
endif()
set(DTCORE_DIR "${_dtcore_default}" CACHE PATH
    "il motore comune del Digitakt II (digitakt/emulator/core)")
if(NOT DTCORE_UNICORN_PREFIX)
  message(FATAL_ERROR "DTCORE_UNICORN_PREFIX non impostato: costruisci la Unicorn con core/scripts/build-unicorn.sh PREFIX")
endif()
# DNCORE_DSP_DIR e' facoltativo: senza, il DSP gira con l'interprete SHARC+ (dn_interp.c: parte dal
# solo .syx dell'utente, e' la versione distribuibile). Con DNCORE_DSP_DIR (libdndsp.a, solo
# sviluppo e uso personale) il DSP ricompilato dal firmware e' il motore predefinito.
if(DNCORE_DSP_DIR AND NOT EXISTS "${DNCORE_DSP_DIR}/libdndsp.a")
  message(FATAL_ERROR "DNCORE_DSP_DIR senza libdndsp.a: core/scripts/build-dsp.sh SNAP DIR")
endif()
if(NOT EXISTS "${DTCORE_DIR}/src/machine.c")
  message(FATAL_ERROR "DTCORE_DIR (${DTCORE_DIR}) non e' il core del Digitakt II")
endif()
find_path(UNICORN_INCLUDE_DIR unicorn/unicorn.h PATHS "${DTCORE_UNICORN_PREFIX}/include" NO_DEFAULT_PATH REQUIRED)
find_library(UNICORN_STATIC NAMES libunicorn.a unicorn.lib PATHS "${DTCORE_UNICORN_PREFIX}/lib" "${DTCORE_UNICORN_PREFIX}/lib64" NO_DEFAULT_PATH REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

if(MSVC)
  add_compile_options(/W3 /fp:precise)
else()
  add_compile_options(-Wall -Wextra -Wno-unused-parameter -ffp-contract=off -fno-fast-math)
endif()

set(DNCORE_COMMON ${DTCORE_DIR}/src/fw.c ${DTCORE_DIR}/src/machine.c ${DTCORE_DIR}/src/sha256.c
    ${DTCORE_DIR}/src/rt.c ${DTCORE_DIR}/src/drive.c)
set(DNCORE_SOURCES ${DNCORE_COMMON} src/dn_engine.c src/dn_data.c src/dn_prepare.c dsp/dn_dspback.c)

add_library(dncore STATIC ${DNCORE_SOURCES})
target_include_directories(dncore PUBLIC include ${DTCORE_DIR}/include dsp
                           PRIVATE src ${DTCORE_DIR}/src ${UNICORN_INCLUDE_DIR})
target_link_libraries(dncore PUBLIC ${UNICORN_STATIC} ZLIB::ZLIB Threads::Threads)
target_sources(dncore PRIVATE dsp/dn_interp.c dsp/dn_exec_simd.c dsp/dn_exec_nosimd.c dsp/dn_jit.c dsp/dn_jit_simd.c)
set_source_files_properties(dsp/dn_interp.c dsp/dn_exec_simd.c dsp/dn_exec_nosimd.c dsp/dn_jit.c dsp/dn_jit_simd.c PROPERTIES COMPILE_OPTIONS "-O2;-fno-stack-protector;-fomit-frame-pointer;-fno-math-errno")
if(DNCORE_DSP_DIR)
  target_link_libraries(dncore PUBLIC ${DNCORE_DSP_DIR}/libdndsp.a)
  target_compile_definitions(dncore PRIVATE DN_STATIC_DSP=1)
endif()
if(UNIX AND NOT APPLE)
  target_link_libraries(dncore PUBLIC m dl)
endif()
if(UNIX)
  target_link_libraries(dncore PUBLIC dl)
endif()

add_executable(dncore_cli cli/dncore_cli.c)
target_include_directories(dncore_cli PRIVATE ${DTCORE_DIR}/src)
target_link_libraries(dncore_cli PRIVATE dncore)
set_target_properties(dncore_cli PROPERTIES OUTPUT_NAME dncore)

add_executable(dsptool cli/dsptool.c)
target_include_directories(dsptool PRIVATE ${DTCORE_DIR}/src)
target_link_libraries(dsptool PRIVATE dncore)
if(DNCORE_DSP_DIR)
  target_compile_definitions(dsptool PRIVATE DN_STATIC_DSP=1)
endif()
