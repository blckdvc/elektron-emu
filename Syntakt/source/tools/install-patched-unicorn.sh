#!/usr/bin/env bash
# Build the m68k SR-read, code-hook CCR-sync and EMAC MAC-with-load fixes from
# official Unicorn 2.1.4.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
commit=8028ec436f2d9376525352dd38ed9ed6b9f6be10
# Applied in this order, each pinned by SHA-256.
patches=(
  "$root/patches/unicorn-2.1.4-m68k-hook-ccr-sync.patch"
  "$root/patches/unicorn-2.1.4-m68k-emac-mac-load.patch"
  "$root/patches/unicorn-2.1.4-m68k-emac-fractional.patch"
  "$root/patches/unicorn-2.1.4-m68k-mac-regs.patch"
  "$root/patches/unicorn-2.1.4-m68k-notdirty-pages.patch"
  "$root/patches/unicorn-2.1.4-m68k-hookmem-pages.patch"
  "$root/patches/unicorn-2.1.4-m68k-ldst-exit.patch"
  "$root/patches/unicorn-2.1.4-m68k-emac-fused.patch"
  "$root/patches/unicorn-2.1.4-m68k-emac-inline.patch"
  "$root/patches/unicorn-2.1.4-m68k-jit-toggle.patch"
  "$root/patches/unicorn-2.1.4-m68k-tb-budget.patch"
  "$root/patches/unicorn-2.1.4-m68k-tlb-min.patch"
  "$root/patches/unicorn-2.1.4-m68k-emac-spec.patch"
  "$root/patches/unicorn-2.1.4-m68k-hook-index.patch"
  "$root/patches/unicorn-2.1.4-m68k-emac-sat.patch"
)
patch_shas=(
  56de71acf2adbd5ca2f448095478e65e49fd79d378aeb5b5e4217d2c90f52f4e
  ac128dd6836997de55e0d2ad70d7a2978639168090f552c5634da50fddc70bfe
  a7ded0c2d5126f93de43800e060e95a35402e5f45bbd2d307084fe2feb484871
  278375bc1a271c24b57f9df85238a4ed53e35f210055d1fcc20a450e28e31b8c
  e65dd55ba367fe225bb9d3bc0cfbe7628dd87b68978450432e485c84b414b23f
  d23ee5e56ba9c145a5b5df7c2168eb6f349ff48bf73f246e79d9d17da403f6f7
  a3a8fd60e4f3ac4785711455f3c7eeac6375a434dbefcf9fdaf64c31131f43ab
  e0151873cd2a03dfe43147eb1c839cb60a6ddbcbede2383ec427efc9f4c74c81
  e8cabb7b29f51c8c9518a5c29c5e5aa8ca751bd8ac3a3d40aa60ff2c3b8e56bc
  0b6b5b15475da65f0de3c3d8e743114baa0bdcd15f474351a297365d83258482
  a7aa3351f5feeae8991b6d8bf688f8c6b3d0cc22c436069d4ed40373bc6ca38d
  9b83d3b6caad7039d673cb872fbf8aa5df65e5566389aa200df512ef2713d600
  4586d7203ea32a3ca1891f51b2406adbeb394cd880915e26b60b60163627b6fe
  fc32d6112f1917e2cdb20a055e997c92f5210d694a8d925094752aa692d4daea
  3bf897c336fbb526f98326cc94d856d5583c82431a66371e34e6ff8256ffdfd2
)
python=${PYTHON:-$root/.venv/bin/python}
dry_run=false
if [[ ${1:-} == --dry-run ]]; then
  dry_run=true
  shift
fi
[[ $# == 0 ]] || {
  echo "usage: $0 [--dry-run]" >&2
  exit 2
}
[[ -x $python ]] || {
  echo "error: project Python not found: $python (set PYTHON=...)" >&2
  exit 2
}
[[ $("$python" -c 'import unicorn; print(unicorn.__version__)') == 2.1.4 ]] || {
  echo "error: interpreter must have unicorn==2.1.4; run uv sync first" >&2
  exit 2
}
for i in "${!patches[@]}"; do
  [[ $(shasum -a 256 "${patches[i]}" | awk '{print $1}') == "${patch_shas[i]}" ]] || {
    echo "error: unexpected patch SHA-256: ${patches[i]}" >&2
    exit 2
  }
done
command -v git >/dev/null || {
  echo "error: git is required" >&2
  exit 2
}
command -v cmake >/dev/null || {
  echo "error: cmake is required" >&2
  exit 2
}

# These are the exact dynamic-library names selected by Unicorn's bindings.
# Do not glob: official wheels also contain libunicorn.a.
target=$(
  "$python" - <<'PY'
import os
import sys
import unicorn

name = {"darwin": "libunicorn.2.dylib", "linux": "libunicorn.so.2"}.get(sys.platform)
if name is None:
    raise SystemExit("unsupported Unicorn platform: " + sys.platform)
path = os.path.join(os.path.dirname(unicorn.__file__), "lib", name)
if not os.path.isfile(path):
    raise SystemExit("unsupported Unicorn native-library layout: expected " + path)
print(path)
PY
)
case $(uname -s) in
Darwin) built_name=libunicorn.2.dylib ;;
Linux) built_name=libunicorn.so.2 ;;
*)
  echo "error: unsupported platform $(uname -s)" >&2
  exit 2
  ;;
esac
if $dry_run; then
  printf 'dry-run: target=%s\n' "$target"
  printf 'dry-run: expected-build-payload=build/%s\n' "$built_name"
  exit 0
fi

work=$(mktemp -d "${TMPDIR:-/tmp}/digitakt2-unicorn.XXXXXX")
trap 'rm -rf "$work"' EXIT

git clone --quiet --branch 2.1.4 --depth 1 https://github.com/unicorn-engine/unicorn.git "$work/src"
[[ $(git -C "$work/src" rev-parse HEAD) == "$commit" ]] || {
  echo "error: tag 2.1.4 did not resolve to expected commit" >&2
  exit 2
}
for p in "${patches[@]}"; do
  git -C "$work/src" apply --check "$p"
  git -C "$work/src" apply "$p"
done
cmake -S "$work/src" -B "$work/build" -DCMAKE_BUILD_TYPE=Release -DUNICORN_ARCH=m68k -DUNICORN_BUILD_TESTS=OFF
cmake --build "$work/build" --config Release --target unicorn
built=$work/build/$built_name
[[ -f $built && ! -L $built ]] || {
  echo "error: expected real m68k dynamic-library payload not produced: $built" >&2
  exit 2
}
# Replacement is atomic in the library directory, so an interrupted install never leaves a partial dylib.
tmp_target=$(mktemp "$(dirname "$target")/.${built_name}.XXXXXX")
cp "$built" "$tmp_target"
mv -f "$tmp_target" "$target"
printf 'unicorn commit=%s\n' "$commit"
for i in "${!patches[@]}"; do
  printf 'patch=%s sha256=%s\n' "$(basename "${patches[i]}")" "${patch_shas[i]}"
done
printf 'library=%s\nlibrary_sha256=%s\n' "$target" "$(shasum -a 256 "$target" | awk '{print $1}')"
"$python" -m emu.unicorn_compat
