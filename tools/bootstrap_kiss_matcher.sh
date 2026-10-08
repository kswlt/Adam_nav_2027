#!/usr/bin/env bash
set -euo pipefail
# Source revisions are shared with dependencies.lock.yaml. No sudo is used.
dependency_root="${RM_NAV_DEPS_ROOT:-${HOME}/.local/share/adam_nav/deps}"
mkdir -p "${dependency_root}/src"
checkout_revision() {
  local name="$1" url="$2" revision="$3" checkout="${dependency_root}/src/$1"
  if [[ ! -d "${checkout}" ]]; then git clone "${url}" "${checkout}"; fi
  if [[ -n "$(git -C "${checkout}" status --porcelain)" ]]; then
    echo "${name} has local changes; refusing to overwrite them." >&2; exit 1
  fi
  git -C "${checkout}" fetch origin "${revision}"
  git -C "${checkout}" checkout --detach "${revision}"
}
checkout_revision KISS-Matcher https://github.com/MIT-SPARK/KISS-Matcher.git e3440b63340af7414ebd0fdde04e40f02ff4e7d2
checkout_revision ROBIN https://github.com/MIT-SPARK/ROBIN.git 1832f4cc4cba357699b093c2751c0690c29a2442
# Patch only a disposable build-source copy. Keep the original checkout clean.
# Upstream PMC is already pinned; xenium's floating main is pinned here.
robin_build_source="${dependency_root}/build_sources/robin-1832f4cc"
python3 - "${dependency_root}/src/ROBIN" "${robin_build_source}" <<'PY'
import pathlib, shutil, sys
source, destination = map(pathlib.Path, sys.argv[1:])
shutil.copytree(source, destination, dirs_exist_ok=True, ignore=shutil.ignore_patterns('.git'))
path = destination / 'cmake/DownloadExternal.cmake'
text = path.read_text()
old = 'GIT_TAG        main'
assert text.count(old) == 1, 'Upstream dependency declaration changed'
assert '4bbd40ababd8e925c4e1845c509173afe766b443' in text
path.write_text(text.replace(old, 'GIT_TAG        1c449ae953ce2a440b0d16c5ed1181d2754860ab'))
PY
cmake -S "${robin_build_source}" -B "${dependency_root}/build/robin" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${dependency_root}/install" \
  -DBUILD_TESTS=OFF -DUSE_SYSTEM_EIGEN3=ON
[[ "$(git -C "${dependency_root}/build/robin/pmc-src" rev-parse HEAD)" == 4bbd40ababd8e925c4e1845c509173afe766b443 ]]
[[ "$(git -C "${dependency_root}/build/robin/xenium-src" rev-parse HEAD)" == 1c449ae953ce2a440b0d16c5ed1181d2754860ab ]]
cmake --build "${dependency_root}/build/robin" -j2
cmake --install "${dependency_root}/build/robin"
cmake -S "${dependency_root}/src/KISS-Matcher/cpp/kiss_matcher" \
  -B "${dependency_root}/build/kiss_matcher" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${dependency_root}/install" \
  -DCMAKE_PREFIX_PATH="${dependency_root}/install" \
  -DUSE_SYSTEM_TBB=ON -DUSE_SYSTEM_EIGEN3=ON -DUSE_SYSTEM_ROBIN=ON
cmake --build "${dependency_root}/build/kiss_matcher" -j2
cmake --install "${dependency_root}/build/kiss_matcher"
printf 'KISS-Matcher and pinned ROBIN/PMC/xenium installed in %s\n' "${dependency_root}/install"
