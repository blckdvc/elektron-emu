#!/usr/bin/env bash
# Dipendenze del VST3 della Syntakt per Windows x64, compilate dal Mac (o da Linux) con mingw-w64:
# Unicorn 2.1.4 con le patch di patches/ (le 15 della Syntakt, dall'elenco di
# tools/install-patched-unicorn.sh) + unicorn-2.1.4-win32.patch (dal
# Model:Cycles: qemu_memalign allineato a 16 e buffer del JIT impegnato subito)
# e zlib, entrambe statiche, in un prefisso da passare a CMake.
#
#   vst/scripts/build-deps-windows.sh PREFIX
#
# Requisiti: x86_64-w64-mingw32-gcc (brew install mingw-w64), cmake, ninja, git.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
[[ $# -ge 1 ]] || { echo "uso: $0 PREFIX" >&2; exit 2; }
command -v x86_64-w64-mingw32-gcc >/dev/null || { echo "manca mingw-w64 (brew install mingw-w64)" >&2; exit 1; }
mkdir -p "$1"
prefix=$(cd "$1" && pwd)
work=$prefix/work
mkdir -p "$work"

toolchain=$prefix/mingw.cmake
cat > "$toolchain" <<'CM'
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_C_COMPILER x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_RC_COMPILER x86_64-w64-mingw32-windres)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# niente DLL di mingw accanto al plugin (libwinpthread-1, libgcc, libstdc++)
set(CMAKE_EXE_LINKER_FLAGS_INIT "-static -static-libgcc -static-libstdc++")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-static -static-libgcc -static-libstdc++")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "-static -static-libgcc -static-libstdc++")
CM

# -- zlib ---------------------------------------------------------------------
if [[ ! -f $prefix/lib/libz.a ]]; then
  [[ -d $work/zlib/.git ]] || git clone --quiet --branch v1.3.1 --depth 1 https://github.com/madler/zlib.git "$work/zlib"
  cmake -S "$work/zlib" -B "$work/zlib-build" -G Ninja -DCMAKE_TOOLCHAIN_FILE="$toolchain" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" >/dev/null
  cmake --build "$work/zlib-build" --target zlibstatic >/dev/null
  mkdir -p "$prefix/lib" "$prefix/include"
  cp "$work/zlib-build/libzlibstatic.a" "$prefix/lib/libz.a"
  cp "$work/zlib/zlib.h" "$work/zlib-build/zconf.h" "$prefix/include/"
fi

# -- Unicorn ------------------------------------------------------------------
if [[ ! -f $prefix/lib/libunicorn.a ]]; then
  installer=$root/tools/install-patched-unicorn.sh
  commit=$(sed -n 's/^commit=\([0-9a-f]*\)$/\1/p' "$installer")
  eval "$(sed -n '/^patches=(/,/^)/p;/^patch_shas=(/,/^)/p' "$installer")"
  for i in "${!patches[@]}"; do
    [[ $(shasum -a 256 "${patches[i]}" | awk '{print $1}') == "${patch_shas[i]}" ]] || {
      echo "error: SHA-256 inatteso: ${patches[i]}" >&2; exit 2; }
  done
  src=$work/unicorn
  [[ -d $src/.git ]] || git clone --quiet --branch 2.1.4 --depth 1 https://github.com/unicorn-engine/unicorn.git "$src"
  [[ $(git -C "$src" rev-parse HEAD) == "$commit" ]] || { echo "error: il tag 2.1.4 non e' il commit atteso" >&2; exit 2; }
  git -C "$src" checkout --quiet -- .
  git -C "$src" clean -fdq
  for p in "${patches[@]}" "$root/patches/unicorn-2.1.4-win32.patch"; do
    git -C "$src" apply "$p"
  done
  cmake -S "$src" -B "$work/unicorn-build" -G Ninja -DCMAKE_TOOLCHAIN_FILE="$toolchain" \
    -DCMAKE_BUILD_TYPE=Release -DUNICORN_ARCH=m68k -DUNICORN_BUILD_TESTS=OFF \
    -DBUILD_SHARED_LIBS=OFF -DUNICORN_INSTALL=ON -DCMAKE_INSTALL_PREFIX="$prefix" >/dev/null
  cmake --build "$work/unicorn-build" >/dev/null
  cmake --install "$work/unicorn-build" >/dev/null
fi
ls -la "$prefix/lib"
