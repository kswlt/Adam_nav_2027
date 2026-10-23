"""Endpoint-only occupancy preview; only an explicit operator polygon adds free space."""
import base64
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from import_z_map import read_native_map
from publish_map_bundle import read_pcd, digest
from PIL import Image, ImageDraw


def prepare(native, resolution=.05, min_z=-.10, max_z=1.50):
    native = Path(native).resolve(strict=True)
    if not (native/'keyframes.txt').is_file() or not (native/'clouds').is_dir():
        raise ValueError('必须选择已保存的Z原生地图')
    if not all(math.isfinite(v) for v in (resolution, min_z, max_z)) or not 0 < resolution <= .5 or min_z >= max_z:
        raise ValueError('栅格分辨率或障碍物高度范围不合法')
    draft = native.parent/(native.name+'_draft')
    if not draft.exists():
        subprocess.run([sys.executable, str(TOOLS/'import_z_map.py'), '--native-bundle', str(native),
                        '--output', str(draft), '--calibration-id', 'vehicle164_yaw_axis_fit153cm_debug_v3'], check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    manifest = json.loads((draft/'map_bundle.json').read_text())
    if manifest.get('z_native_map_sha256') != digest(native/'map.pcd'):
        raise ValueError('检查包与原生地图不匹配，请保留原数据并重新导入')
    source = draft/manifest['artifacts']['rebuilt_map.pcd']['path']
    if digest(source) != manifest['artifacts']['rebuilt_map.pcd']['sha256']:
        raise ValueError('检查包地图已经变化')
    points = read_pcd(source)
    xmin = math.floor((min(x for x,y,z in points)-.2)/resolution)*resolution
    ymin = math.floor((min(y for x,y,z in points)-.2)/resolution)*resolution
    width = math.ceil((max(x for x,y,z in points)+.2-xmin)/resolution)+1
    height = math.ceil((max(y for x,y,z in points)+.2-ymin)/resolution)+1
    if width*height > 4000000:
        raise ValueError('检查图超过400万格，请增大分辨率')
    cells = bytearray([205])*(width*height)
    for x,y,z in points:
        if min_z <= z <= max_z:
            ix, iy = int((x-xmin)/resolution), int((y-ymin)/resolution)
            cells[(height-1-iy)*width+ix] = 0
    if 0 not in cells:
        raise ValueError('当前高度范围内没有障碍物，必须检查地面/高度配置')
    path = []
    for line in (native/'keyframes.txt').read_text().splitlines():
        if not line.strip() or line.startswith('#'):
            continue
        row = line.split()
        if len(row) < 9:
            raise ValueError('关键帧轨迹格式损坏')
        x, y = float(row[2]), float(row[3])
        if not math.isfinite(x+y):
            raise ValueError('关键帧轨迹包含非有限值')
        path.append([(x-xmin)/resolution, height-1-(y-ymin)/resolution])
    return {'native': str(native), 'draft': str(draft/'map_bundle.json'), 'width': width, 'height': height,
            'resolution': resolution, 'origin': [xmin,ymin,0], 'min_z': min_z, 'max_z': max_z,
            'source_map_sha256': manifest['z_native_map_sha256'], 'z_range': [min(z for x,y,z in points),max(z for x,y,z in points)],
            'point_count': len(points), 'keyframes': len(path), 'occupied_cells': cells.count(0),
            'free_cells': 0, 'status': 'awaiting_operator_review', 'path': path, '_cells': cells}


def polygon_valid(vertices, width, height):
    if not isinstance(vertices, list) or not 3 <= len(vertices) <= 64:
        raise ValueError('每个区域需要3～64个顶点')
    for v in vertices:
        if (not isinstance(v, list) or len(v)!=2 or
                not all(type(n) in (int,float) and math.isfinite(n) for n in v) or
                not 0 <= v[0] < width or not 0 <= v[1] < height):
            raise ValueError('区域顶点必须位于检查图内')
    if len(set(tuple(v) for v in vertices)) != len(vertices):
        raise ValueError('区域有重复顶点')
    def cross(a,b,c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def intersects(a,b,c,d):
        ab1,ab2,cd1,cd2 = cross(a,b,c),cross(a,b,d),cross(c,d,a),cross(c,d,b)
        if ab1*ab2 < 0 and cd1*cd2 < 0:
            return True
        def on(p,q,r):
            return min(p[0],q[0])<=r[0]<=max(p[0],q[0]) and min(p[1],q[1])<=r[1]<=max(p[1],q[1])
        return any((s==0 and on(p,q,r)) for s,p,q,r in ((ab1,a,b,c),(ab2,a,b,d),(cd1,c,d,a),(cd2,c,d,b)))
    count = len(vertices)
    for i in range(count):
        for j in range(i+1,count):
            if j == i+1 or (i==0 and j==count-1):
                continue
            if intersects(vertices[i],vertices[(i+1)%count],vertices[j],vertices[(j+1)%count]):
                raise ValueError('区域边界自交，请重新圈选')
    if abs(sum(vertices[i][0]*vertices[(i+1)%count][1]-vertices[(i+1)%count][0]*vertices[i][1] for i in range(count))) < 2:
        raise ValueError('区域面积过小')


def reviewed_mask(review, polygons):
    if not isinstance(polygons,list) or not 1 <= len(polygons) <= 20:
        raise ValueError('先圈选1～20个已确认的可通行区域')
    image = Image.new('L',(review['width'],review['height']),205)
    draw = ImageDraw.Draw(image)
    for vertices in polygons:
        polygon_valid(vertices,review['width'],review['height'])
        draw.polygon([tuple(v) for v in vertices],fill=255)
    pixels = bytearray(image.tobytes())
    for i, value in enumerate(review['_cells']):
        if value == 0:
            pixels[i] = 0  # Operator polygons can never erase observed obstacles.
    if 255 not in pixels:
        raise ValueError('圈选区域中没有可用自由格')
    return Image.frombytes('L',image.size,bytes(pixels))


def publish(review, polygons, reviewer, confirmed):
    if confirmed is not True or not isinstance(reviewer,str) or not reviewer.strip() or len(reviewer)>80:
        raise ValueError('需要实际操作者姓名及明确的自由空间/障碍物检查确认')
    if digest(Path(review['native'])/'map.pcd') != review['source_map_sha256']:
        raise ValueError('原生地图已改变，请重新生成检查图')
    image = reviewed_mask(review,polygons)
    native = Path(review['native'])
    staging = Path(tempfile.mkdtemp(prefix=native.name+'_review_',dir=native.parent))
    output = native.parent/(native.name+'_navigation_'+str(time.time_ns()))
    source = 'z-local-session:'+native.name
    try:
        mask = staging/'reviewed.pgm'
        image.save(mask,format='PPM')
        (staging/'alignment.json').write_text(json.dumps({'status':'operator_defined','source_frame':'mapping_odom',
            'target_frame':'map','local_source':source,'coordinate_definition':'session_mapping_odom',
            'translation':[0,0,0],'yaw_rad':0,'scale':1}))
        (staging/'reviewed.json').write_text(json.dumps({'status':'operator_reviewed','coordinate_frame':'map',
            'local_source':source,'resolution':review['resolution'],'origin':review['origin'],
            'reviewer':reviewer.strip(),'review_scope':'free_space_and_obstacles_in_local_map',
            'polygons_image_coordinates':polygons,'obstacle_height_range':[review['min_z'],review['max_z']],
            'source_map_sha256':review['source_map_sha256'],'reviewed_at_unix_ns':time.time_ns()}))
        command = [sys.executable,str(TOOLS/'publish_map_bundle.py'),'--draft',review['draft'],
            '--z-native-bundle',review['native'],'--profile','local_site','--alignment',str(staging/'alignment.json'),
            '--output',str(output),'--free-space-mask',str(mask),'--free-space-metadata',str(staging/'reviewed.json'),
            '--resolution',str(review['resolution']),'--min-z',str(review['min_z']),'--max-z',str(review['max_z'])]
        result = subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=180)
        if result.returncode:
            raise RuntimeError(result.stdout[-8000:])
        return str(output/'map_bundle.json')
    finally:
        shutil.rmtree(staging)


def public(review):
    return {**{k:v for k,v in review.items() if k!='_cells'}, 'cells': base64.b64encode(review['_cells']).decode()}
