#!/usr/bin/env bash
set -euo pipefail
# Fixed upstream stable release; no source patches, Python bindings or unstable module.
dependency_root="${RM_NAV_DEPS_ROOT:-${HOME}/.local/share/adam_nav/deps}"
revision=4f66a491ffc83cf092d0d818b11dc35135521612
checkout="${dependency_root}/src/gtsam"
mkdir -p "${dependency_root}/src"
if [[ ! -d "${checkout}" ]]; then
  git clone --branch 4.2.0 --depth 1 https://github.com/borglab/gtsam.git "${checkout}"
fi
if [[ -n "$(git -C "${checkout}" status --porcelain)" ]]; then
  echo 'Upstream GTSAM checkout has local changes; refusing to overwrite them.' >&2
  exit 1
fi
git -C "${checkout}" fetch origin "${revision}"
git -C "${checkout}" checkout --detach "${revision}"
cmake -S "${checkout}" -B "${dependency_root}/build/gtsam" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG' \
  -DCMAKE_INSTALL_PREFIX="${dependency_root}/install" \
  -DCMAKE_INSTALL_RPATH="${dependency_root}/install/lib" \
  -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_BUILD_PYTHON=OFF
# One compiler bounds peak RAM on the current 7 GiB ASUS machine.
cmake --build "${dependency_root}/build/gtsam" -j1
cmake --install "${dependency_root}/build/gtsam"
