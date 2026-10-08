#!/usr/bin/env bash
set -euo pipefail
# User-space installation; no sudo and no modifications to upstream algorithms.
dependency_root="${RM_NAV_DEPS_ROOT:-${HOME}/.local/share/adam_nav/deps}"
revision=fa0cfc983417c044b6036f71dfa448038bb9c820
checkout="${dependency_root}/src/small_gicp"
mkdir -p "${dependency_root}/src"
if [[ ! -d "${checkout}" ]]; then
  git clone https://github.com/koide3/small_gicp.git "${checkout}"
fi
if [[ -n "$(git -C "${checkout}" status --porcelain)" ]]; then
  echo 'Upstream checkout has local changes; refusing to overwrite them.' >&2
  exit 1
fi
git -C "${checkout}" fetch origin "${revision}"
git -C "${checkout}" checkout --detach "${revision}"
cmake -S "${checkout}" -B "${dependency_root}/build/small_gicp" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${dependency_root}/install" \
  -DBUILD_HELPER=ON -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_WITH_OPENMP=ON
cmake --build "${dependency_root}/build/small_gicp" -j2
cmake --install "${dependency_root}/build/small_gicp"
printf 'Build workspace with: CMAKE_PREFIX_PATH="%s:${CMAKE_PREFIX_PATH:-}" colcon build\n' "${dependency_root}/install"
