#!/usr/bin/env bash
set -euo pipefail
dependency_root="${RM_NAV_DEPS_ROOT:-${HOME}/.local/share/adam_nav/deps}"
revision=688d75cfa780049ae532e5100ca64f46ad8b1a93
checkout="${dependency_root}/src/small_point_lio"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
patch_file="${script_dir}/patches/small_point_lio_raw_state.patch"
mkdir -p "${dependency_root}/src"
if [[ ! -d "${checkout}" ]]; then
  git clone "${RM_NAV_LIO_REPO_URL:-https://github.com/Yancey2023/small_point_lio.git}" "${checkout}"
fi
if [[ -n "$(git -C "${checkout}" status --porcelain)" ]]; then
  # Reruns accept exactly our reviewed patch, never unrelated local changes.
  git -C "${checkout}" diff --binary > "${dependency_root}/lio_current.patch"
  git -C "${checkout}" apply --reverse --check "${patch_file}"
  if [[ "$(git -C "${checkout}" ls-files --others --exclude-standard | wc -l)" != 0 ]]; then
    echo 'Untracked upstream files; refusing to overwrite.' >&2; exit 1
  fi
  git -C "${checkout}" apply --reverse "${patch_file}"
  if [[ -n "$(git -C "${checkout}" status --porcelain)" ]]; then
    git -C "${checkout}" apply "${patch_file}"
    echo 'Additional upstream changes; refusing to overwrite.' >&2; exit 1
  fi
fi
git -C "${checkout}" fetch origin "${revision}"
git -C "${checkout}" checkout --detach "${revision}"
git -C "${checkout}" apply --check "${patch_file}"
git -C "${checkout}" apply "${patch_file}"
sha256sum "${patch_file}"
colcon build --base-paths "${checkout}" \
  --build-base "${dependency_root}/build_lio" --install-base "${dependency_root}/install_lio" \
  --cmake-args -DBUILD_TESTING=OFF
printf 'Source the LIO overlay: %s/install_lio/setup.bash\n' "${dependency_root}"
