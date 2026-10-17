#!/usr/bin/env bash
# Unicorn 2.1.4 con le patch di patches/ come LIBRERIA STATICA (solo m68k) per
# arcore: header + libunicorn.a in un prefisso, da passare a CMake con
# -DDTCORE_UNICORN_PREFIX=... .
#
#   core/scripts/build-unicorn.sh PREFIX [WORKDIR]
#
# La serie di patch e i loro SHA-256 si leggono da tools/install-patched-unicorn.sh
# (una sola lista nel repo: quella che usa anche il Python). WORKDIR (sorgenti e
# build, ~200 MB) va su un disco con spazio: predefinito PREFIX/../unicorn-work.
# Variabili: CMAKE_OSX_ARCHITECTURES (es. "arm64;x86_64"),
# CMAKE_OSX_DEPLOYMENT_TARGET (predefinito 12.0).
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
[[ $# -ge 1 ]] || { echo "uso: $0 PREFIX [WORKDIR]" >&2; exit 2; }
prefix=$1
work=${2:-$(dirname "$prefix")/unicorn-work}
mkdir -p "$prefix" "$work"
prefix=$(cd "$prefix" && pwd)
work=$(cd "$work" && pwd)

installer=$root/tools/install-patched-unicorn.sh
commit=$(sed -n 's/^commit=\([0-9a-f]*\)$/\1/p' "$installer")
eval "$(sed -n '/^patches=(/,/^)/p;/^patch_shas=(/,/^)/p' "$installer")"
[[ ${#patches[@]} -gt 0 && ${#patches[@]} == "${#patch_shas[@]}" ]] || {
  echo "error: lista delle patch illeggibile in $installer" >&2; exit 2; }
for i in "${!patches[@]}"; do
  [[ $(shasum -a 256 "${patches[i]}" | awk '{print $1}') == "${patch_shas[i]}" ]] || {
    echo "error: SHA-256 inatteso: ${patches[i]}" >&2; exit 2; }
done

src=$work/src
if [[ ! -d $src/.git ]]; then
  git clone --quiet --branch 2.1.4 --depth 1 https://github.com/unicorn-engine/unicorn.git "$src"
fi
[[ $(git -C "$src" rev-parse HEAD) == "$commit" ]] || {
  echo "error: il tag 2.1.4 non e' il commit atteso" >&2; exit 2; }
git -C "$src" checkout --quiet -- .
git -C "$src" clean -fdq
for p in "${patches[@]}"; do
  git -C "$src" apply "$p"
done

cmake -S "$src" -B "$work/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DUNICORN_ARCH=m68k -DUNICORN_BUILD_TESTS=OFF \
  -DBUILD_SHARED_LIBS=OFF -DUNICORN_INSTALL=ON \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_OSX_DEPLOYMENT_TARGET="${CMAKE_OSX_DEPLOYMENT_TARGET:-12.0}" \
  ${CMAKE_OSX_ARCHITECTURES:+-DCMAKE_OSX_ARCHITECTURES="$CMAKE_OSX_ARCHITECTURES"} \
  -DCMAKE_INSTALL_PREFIX="$prefix"
cmake --build "$work/build" --config Release
cmake --install "$work/build" --config Release

{
  printf 'unicorn commit=%s\n' "$commit"
  for i in "${!patches[@]}"; do
    printf 'patch=%s sha256=%s\n' "$(basename "${patches[i]}")" "${patch_shas[i]}"
  done
} > "$prefix/DTCORE-UNICORN.txt"
ls -l "$prefix"/lib*/libunicorn*
echo "prefisso: $prefix"
