# The tests of the Elektron Emu view for one machine (tests/run-tests.sh builds the five):
#
#   cmake -S tests -B BUILD -G Ninja -DMACHINE=dt -DDTCORE_UNICORN_PREFIX=/path/unicorn
#   cmake --build BUILD        ->  BUILD/seqtest
#
# MACHINE: dt (Digitakt II), dn (Digitone II), st (Syntakt), ar (Analog Rytm MKII),
# mc (Model:Cycles). The machine's own core is built with it (its CMake project), and
# its panel_layout.hpp is generated as for the plug-in (vst/scripts/gen-layout.py).
cmake_minimum_required(VERSION 3.20)
project(seqtest C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(MACHINE "" CACHE STRING "dt, dn, st, ar or mc")
if(MACHINE STREQUAL "dt")
  set(DIR Digitakt-II)
  set(LIB dtcore)
  set(JSON digitakt2_panel.json)
  set(DEF M_DT)
elseif(MACHINE STREQUAL "dn")
  set(DIR Digitone-II)
  set(LIB dncore)
  set(JSON digitone2_panel.json)
  set(DEF M_DN)
elseif(MACHINE STREQUAL "st")
  set(DIR Syntakt)
  set(LIB stcore)
  set(JSON syntakt_panel.json)
  set(DEF M_ST)
elseif(MACHINE STREQUAL "ar")
  set(DIR Analog-Rytm-MKII)
  set(LIB arcore)
  set(JSON rytm_panel.json)
  set(DEF M_AR)
elseif(MACHINE STREQUAL "mc")
  set(DIR Model-Cycles)
  set(LIB mccore)
  set(JSON mc_panel.json)
  set(DEF M_MC)
else()
  message(FATAL_ERROR "MACHINE must be dt, dn, st, ar or mc")
endif()
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(MC_UNICORN_PREFIX ${DTCORE_UNICORN_PREFIX} CACHE PATH "")   # the Model:Cycles core's name for it

# Syntakt and Digitone II: the shared Digitakt II engine is in source/dtcore
if(EXISTS ${ROOT}/${DIR}/source/dtcore/src/machine.c)
  set(DTCORE_DIR ${ROOT}/${DIR}/source/dtcore CACHE PATH "" FORCE)
endif()
add_subdirectory(${ROOT}/${DIR}/source/core core)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/panel_layout.hpp
  COMMAND ${Python3_EXECUTABLE} ${ROOT}/${DIR}/source/vst/scripts/gen-layout.py
          ${ROOT}/${DIR}/source/core/layout/${JSON} ${CMAKE_CURRENT_BINARY_DIR}/panel_layout.hpp
  DEPENDS ${ROOT}/${DIR}/source/core/layout/${JSON} ${ROOT}/${DIR}/source/vst/scripts/gen-layout.py)

add_executable(seqtest seqtest.cpp ${CMAKE_CURRENT_BINARY_DIR}/panel_layout.hpp)
target_compile_definitions(seqtest PRIVATE ${DEF})
target_include_directories(seqtest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
  ${ROOT}/${DIR}/source/vst ${ROOT}/${DIR}/source/core/include)
if(EXISTS ${ROOT}/${DIR}/source/dtcore/include)
  target_include_directories(seqtest PRIVATE ${ROOT}/${DIR}/source/dtcore/include)
endif()
target_link_libraries(seqtest PRIVATE ${LIB})
