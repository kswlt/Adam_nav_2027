#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
base=/home/asus/nav_2027_vehicle_config_164/deps_20261020
export MESSAGE_REBUILD_ROOT="$base/pal_ws/src"
python3 - <<'PY'
from pathlib import Path
import os,shutil,xml.etree.ElementTree as ET
for name in ['controller_manager_msgs','control_msgs']:
    upstream=Path('/opt/ros/jazzy/share')/name
    if not upstream.exists():continue
    target=Path(os.environ['MESSAGE_REBUILD_ROOT'])/name;target.mkdir(exist_ok=True)
    shutil.copy2(upstream/'package.xml',target/'package.xml')
    files=[]
    for kind in ['msg','srv','action']:
        for file in (upstream/kind).glob('*.'+kind):
            (target/kind).mkdir(exist_ok=True);shutil.copy2(file,target/kind/file.name);files.append(kind+'/'+file.name)
    dependencies=sorted({item.text for item in ET.parse(upstream/'package.xml').getroot() if item.tag in ('depend','build_depend') and item.text not in ('ament_cmake','rosidl_default_generators')})
    cmake='cmake_minimum_required(VERSION 3.8)\nproject('+name+')\nfind_package(ament_cmake REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)\n'
    cmake+=''.join('find_package('+d+' REQUIRED)\n' for d in dependencies)
    cmake+='rosidl_generate_interfaces(${PROJECT_NAME}\n'+''.join('  "'+f+'"\n' for f in files)+'  DEPENDENCIES '+' '.join(dependencies)+')\nament_export_dependencies(rosidl_default_runtime)\nament_package()\n'
    (target/'CMakeLists.txt').write_text(cmake)
PY
cd "$base/pal_ws"
export MAKEFLAGS=-j2
colcon build --executor sequential --merge-install --install-base "$base/pal_install" --cmake-args -DCMAKE_BUILD_TYPE=Release
echo CONTROL_MESSAGE_REBUILD_PASSED
