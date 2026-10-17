# arcore: il motore del Analog Rytm MKII in C (piano: docs/AU-PORT.md). Deriva da
# dtcore, il motore del Digitakt II (digitakt/emulator/core).
#
#   core/scripts/build-unicorn.sh /percorso/unicorn-m68k-mc
#   cmake -S core -B BUILD -G Ninja -DDTCORE_UNICORN_PREFIX=/percorso/unicorn-m68k-mc
#   cmake --build BUILD
#
# Prodotti: libarcore.a (per il plugin), libarcore_shared (per il Python via
# ctypes) e l'eseguibile di prova `arcore`.
cmake_minimum_required(VERSION 3.20)
project(arcore C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(DTCORE_UNICORN_PREFIX "" CACHE PATH "prefisso della Unicorn con le patch (core/scripts/build-unicorn.sh)")
if(NOT DTCORE_UNICORN_PREFIX)
  message(FATAL_ERROR "DTCORE_UNICORN_PREFIX non impostato: costruisci la Unicorn con core/scripts/build-unicorn.sh PREFIX")
endif()
find_path(UNICORN_INCLUDE_DIR unicorn/unicorn.h PATHS "${DTCORE_UNICORN_PREFIX}/include" NO_DEFAULT_PATH REQUIRED)
find_library(UNICORN_STATIC NAMES libunicorn.a unicorn.lib PATHS "${DTCORE_UNICORN_PREFIX}/lib" "${DTCORE_UNICORN_PREFIX}/lib64" NO_DEFAULT_PATH REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)

if(MSVC)
  add_compile_options(/W3 /fp:precise)
else()
  # niente FMA ne' riordini: stessa aritmetica in virgola mobile del Python
  add_compile_options(-Wall -Wextra -Wno-unused-parameter -ffp-contract=off -fno-fast-math)
endif()

set(ARCORE_SOURCES src/fw.c src/machine.c src/sha256.c src/engine.c src/dma.c src/rt.c src/drive.c src/rytm_dsp_taps.c)
# il motore delle voci (native/rytm_dsp.c, con le prese delle voci separate)
# come lo compila emu/rytmdsp.py (cc -O2 -ffast-math): gli
# ultimi flag vincono su -O3 di Release e su -fno-fast-math qui sopra, cosi'
# l'audio e' lo stesso del Python campione per campione
set_source_files_properties(src/rytm_dsp_taps.c PROPERTIES COMPILE_OPTIONS "-O2;-ffast-math")

add_library(arcore STATIC ${ARCORE_SOURCES})
target_include_directories(arcore PUBLIC include PRIVATE src ${UNICORN_INCLUDE_DIR})
target_link_libraries(arcore PUBLIC ${UNICORN_STATIC} ZLIB::ZLIB Threads::Threads)
if(UNIX AND NOT APPLE)
  target_link_libraries(arcore PUBLIC m dl)
endif()

add_library(arcore_shared SHARED ${ARCORE_SOURCES})
target_compile_definitions(arcore_shared PRIVATE DTCORE_SHARED_BUILD)
target_include_directories(arcore_shared PUBLIC include PRIVATE src ${UNICORN_INCLUDE_DIR})
target_link_libraries(arcore_shared PRIVATE ${UNICORN_STATIC} ZLIB::ZLIB Threads::Threads)
set_target_properties(arcore_shared PROPERTIES OUTPUT_NAME arcore)
if(UNIX AND NOT APPLE)
  target_link_libraries(arcore_shared PRIVATE m dl)
endif()

add_executable(arcore_cli cli/arcore_cli.c)
target_link_libraries(arcore_cli PRIVATE arcore)
set_target_properties(arcore_cli PROPERTIES OUTPUT_NAME arcore)
