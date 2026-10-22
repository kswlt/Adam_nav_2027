#!/usr/bin/env python3
"""Apply the audited Jazzy CMake/nanoflann/driver-launch port to pinned upstream."""
from pathlib import Path
import subprocess
import sys

root = Path(sys.argv[1]).resolve(strict=True)
if subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD']).decode().strip() != '76e126328b670adc92981d9b4900cc059a6f2343':
    raise ValueError('Unexpected Z-Localization revision; re-audit before applying the Jazzy port')
patches = {
    'src/backend/CMakeLists.txt': [(b'project(backend LANGUAGES CXX)', b'project(backend LANGUAGES C CXX)')],
    'src/backend/src/keyframe_map.cpp': [
        (b'std::vector<std::pair<std::size_t, double>> hits;', b'std::vector<nanoflann::ResultItem<std::size_t, double>> hits;'),
        (b'nanoflann::SearchParams()', b'nanoflann::SearchParameters()')],
    'src/bringup/launch/bringup.launch.py': [
        (b"'launch_ROS2', 'msg_MID360_launch.py'", b"'launch', 'msg_MID360_launch.py'")],
}
for relative, replacements in patches.items():
    original = subprocess.check_output(['git', '-C', str(root), 'show', 'HEAD:'+relative]).replace(b'\r\n', b'\n')
    expected = original
    for old, new in replacements:
        if expected.count(old) != 1:
            raise ValueError('Unexpected upstream baseline: '+relative)
        expected = expected.replace(old, new)
    current = (root/relative).read_bytes().replace(b'\r\n', b'\n')
    if current not in (original, expected):
        raise ValueError('Unreviewed upstream changes; refusing overwrite: '+relative)
    (root/relative).write_bytes(expected)
