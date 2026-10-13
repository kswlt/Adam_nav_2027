#!/usr/bin/env bash
set -euo pipefail
dependency_root="${RM_NAV_DEPS_ROOT:-${HOME}/.local/share/adam_nav/deps}"
mkdir -p "$dependency_root/src"
checkout() {
  local name="$1" url="$2" revision="$3" path="$dependency_root/src/$1"
  if [[ ! -d "$path" ]]; then git clone "$url" "$path"; fi
  if [[ -n "$(git -C "$path" status --porcelain)" ]]; then
    echo "Uncommitted dependency changes: $path" >&2; exit 1
  fi
  git -C "$path" fetch origin "$revision"
  git -C "$path" checkout --detach "$revision"
}
checkout osqp https://github.com/osqp/osqp.git 0dd00a578cf1c2691c5c379965d504c75bf6cfad
git -C "$dependency_root/src/osqp" submodule update --init --recursive
checkout osqp-eigen https://github.com/robotology/osqp-eigen.git 85c37623774c682db396505f0d4ea677040c2557
cmake -S "$dependency_root/src/osqp" -B "$dependency_root/build/osqp" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dependency_root/install" \
  -DUNITTESTS=OFF -DPRINTING=OFF -DPROFILING=ON -DENABLE_MKL_PARDISO=OFF
cmake --build "$dependency_root/build/osqp" -j2
cmake --install "$dependency_root/build/osqp"
cmake -S "$dependency_root/src/osqp-eigen" -B "$dependency_root/build/osqp-eigen" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dependency_root/install" \
  -DCMAKE_PREFIX_PATH="$dependency_root/install" -DBUILD_TESTING=OFF
cmake --build "$dependency_root/build/osqp-eigen" -j2
cmake --install "$dependency_root/build/osqp-eigen"
