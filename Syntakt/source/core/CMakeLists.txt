# stcore: il motore della Syntakt in C (piano: docs/AU-PORT.md).
#
# Riusa il motore comune del Digitakt II (digitakt/emulator/core: la macchina
# ColdFire, l'estrazione del firmware, l'impronta SHA-256, il tempo reale e il
# client del +Drive)
# cosi' com'e', dalla sua cartella: qui solo il motore della Syntakt, che
# implementa l'API dt_engine_* di dtcore.h.
#
#   core/scripts/build-unicorn.sh /percorso/unicorn-m68k-st    (le 15 patch della Syntakt)
#   cmake -S core -B BUILD -G Ninja -DDTCORE_UNICORN_PREFIX=/percorso/unicorn-m68k-st
#   cmake --build BUILD
#
# Prodotti: libstcore.a (per il plugin), libstcore (dinamica, per il Python via
# ctypes) e l'eseguibile di prova `stcore`.
cmake_minimum_required(VERSION 3.20)
project(stcore C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(DTCORE_UNICORN_PREFIX "" CACHE PATH "prefisso della Unicorn con le patch (core/scripts/build-unicorn.sh)")
set(DTCORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../digitakt/emulator/core" CACHE PATH
    "il motore comune del Digitakt II (digitakt/emulator/core)")
if(NOT DTCORE_UNICORN_PREFIX)
  message(FATAL_ERROR "DTCORE_UNICORN_PREFIX non impostato: costruisci la Unicorn con core/scripts/build-unicorn.sh PREFIX")
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
  # niente FMA ne' riordini: l'audio e' confrontato bit per bit col numpy
  add_compile_options(-Wall -Wextra -Wno-unused-parameter -ffp-contract=off -fno-fast-math)
endif()

set(STCORE_COMMON ${DTCORE_DIR}/src/fw.c ${DTCORE_DIR}/src/machine.c ${DTCORE_DIR}/src/sha256.c
    ${DTCORE_DIR}/src/rt.c ${DTCORE_DIR}/src/drive.c)
set(STCORE_SOURCES ${STCORE_COMMON} src/st_engine.c src/st_dma.c)
set(STCORE_INCLUDES include ${DTCORE_DIR}/include src ${DTCORE_DIR}/src)

add_library(stcore STATIC ${STCORE_SOURCES})
target_include_directories(stcore PUBLIC include ${DTCORE_DIR}/include PRIVATE src ${DTCORE_DIR}/src ${UNICORN_INCLUDE_DIR})
target_link_libraries(stcore PUBLIC ${UNICORN_STATIC} ZLIB::ZLIB Threads::Threads)
if(UNIX AND NOT APPLE)
  target_link_libraries(stcore PUBLIC m dl)
endif()

add_library(stcore_shared SHARED ${STCORE_SOURCES})
target_compile_definitions(stcore_shared PRIVATE DTCORE_SHARED_BUILD)
target_include_directories(stcore_shared PUBLIC include ${DTCORE_DIR}/include PRIVATE src ${DTCORE_DIR}/src ${UNICORN_INCLUDE_DIR})
target_link_libraries(stcore_shared PRIVATE ${UNICORN_STATIC} ZLIB::ZLIB Threads::Threads)
set_target_properties(stcore_shared PROPERTIES OUTPUT_NAME stcore)
if(UNIX AND NOT APPLE)
  target_link_libraries(stcore_shared PRIVATE m dl)
endif()

add_executable(stcore_cli cli/stcore_cli.c)
target_include_directories(stcore_cli PRIVATE ${DTCORE_DIR}/src)
target_link_libraries(stcore_cli PRIVATE stcore)
set_target_properties(stcore_cli PROPERTIES OUTPUT_NAME stcore)
