#!/usr/bin/env python3
"""Generate read-only layouts using Foxglove's documented layout export structure."""
import json
from pathlib import Path

def layout(frame,navigation):
    visible=['/visualization/live_map_preview','/visualization/lio_path','/visualization/chassis_path']
    if navigation:visible+=['/map','/plan','/visualization/tdt_path','/local_costmap/costmap','/global_costmap/costmap']
    topics={name:{'visible':True} for name in visible}
    # Raw high-bandwidth clouds are available but opt-in; avoid subscribing to
    # raw + deskewed + accumulated full maps by default over Wi-Fi.
    for topic in ('/livox/lidar','/lio/deskewed_odom_cloud','/localization/odom_submap','/visualization/map_cloud','/visualization/tdt_costmap'):
        topics[topic]={'visible':False}
    panels={'3D!scene':{'followTf':frame,'followMode':'follow-pose',
        'scene':{'enableStats':True,'transforms':{'showLabel':True,'axisScale':.3,'lineWidth':1}},
        'transforms':{'frame:'+name:{'visible':True} for name in ('map','odom','base_footprint','chassis','base_link','front_mid360_imu','front_mid360')},
        'topics':topics},
        'RawMessages!tf':{'topicPath':'/tf'},
        'RawMessages!odom':{'topicPath':'/odom'},
        'RawMessages!state':{'topicPath':'/localization/status_reason' if navigation else '/visualization/live_map_status'},
        'RawMessages!tracking':{'topicPath':'/nav/tdt/report' if navigation else '/state/lio_reason'}}
    return {'configById':panels,'globalVariables':{},'userNodes':{},'linkedGlobalVariables':[],
        'playbackConfig':{'speed':1},'layout':{'direction':'row','splitPercentage':72,
        'first':'3D!scene','second':{'direction':'column','splitPercentage':50,
            'first':{'direction':'column','first':'RawMessages!state','second':'RawMessages!tracking'},
            'second':{'direction':'column','first':'RawMessages!tf','second':'RawMessages!odom'}}}}

def main():
    output=Path(__file__).resolve().parents[1]/'src/rm_nav_bringup/config/foxglove';output.mkdir(exist_ok=True)
    online=layout('mapping_odom',False)
    online['configById']['3D!scene']['topics']={topic:{'visible':True} for topic in
        ('/mapping/optimized_map_preview','/mapping/optimized_path','/mapping/loop_graph')}
    online['configById']['3D!scene']['topics']['/mapping/optimized_map']={'visible':False}
    online['configById']['3D!scene']['transforms']={}
    online['configById']['RawMessages!state']['topicPath']='/mapping/online_status'
    online['configById']['RawMessages!tracking']['topicPath']='/mapping/recorder_status'
    online['configById']['RawMessages!tf']['topicPath']='/mapping/mapping_odom_T_odom'
    (output/'mapping_online.json').write_text(json.dumps(online,indent=2)+'\n')
    for name,frame,navigation in [('mapping_low_latency','odom',False),('navigation_tdt','map',True)]:
        (output/(name+'.json')).write_text(json.dumps(layout(frame,navigation),indent=2)+'\n')
if __name__=='__main__':main()
