# dtcore: il motore del Digitakt II in C (piano: docs/AU-PORT.md).
#
#   core/scripts/build-unicorn.sh /percorso/unicorn-m68k-dt
#   cmake -S core -B BUILD -G Ninja -DDTCORE_UNICORN_PREFIX=/percorso/unicorn-m68k-dt
#   cmake --build BUILD
#
# Prodotti: libdtcore.a (per il plugin), libdtcore_shared (per il Python via
# ctypes, emu/dtcore.py) e l'eseguibile di prova `dtcore`.
cmake_minimum_required(VERSION 3.20)
project(dtcore C)

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
  # niente FMA ne' riordini: l'audio e' confrontato bit per bit col numpy
  add_compile_options(-Wall -Wextra -Wno-unused-parameter -ffp-contract=off -fno-fast-math)
endif()

set(DTCORE_SOURCES src/fw.c src/machine.c src/sha256.c src/engine.c src/rt.c src/drive.c)

add_library(dtcore STATIC ${DTCORE_SOURCES})
target_include_directories(dtcore PUBLIC include PRIVATE src . ${UNICORN_INCLUDE_DIR})
target_link_libraries(dtcore PUBLIC ${UNICORN_STATIC} ZLIB::ZLIB Threads::Threads)
if(UNIX AND NOT APPLE)
  target_link_libraries(dtcore PUBLIC m dl)
endif()

add_library(dtcore_shared SHARED ${DTCORE_SOURCES})
target_compile_definitions(dtcore_shared PRIVATE DTCORE_SHARED_BUILD)
target_include_directories(dtcore_shared PUBLIC include PRIVATE src . ${UNICORN_INCLUDE_DIR})
target_link_libraries(dtcore_shared PRIVATE ${UNICORN_STATIC} ZLIB::ZLIB Threads::Threads)
set_target_properties(dtcore_shared PROPERTIES OUTPUT_NAME dtcore)
if(UNIX AND NOT APPLE)
  target_link_libraries(dtcore_shared PRIVATE m dl)
endif()

add_executable(dtcore_cli cli/dtcore_cli.c)
target_link_libraries(dtcore_cli PRIVATE dtcore)
set_target_properties(dtcore_cli PROPERTIES OUTPUT_NAME dtcore)
