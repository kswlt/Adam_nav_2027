#!/usr/bin/env python3
"""TEST_ONLY: actual local Bundle -> GICP -> sole map TF -> Nav2/TDT + supervisor.

An ideal holonomic plant and explicit fixture clouds replace physical hardware.
No localization health, map->odom TF, serial or motor commands are fabricated on
the production ROS domain. Do not use this fixture as a real navigation launch.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from rm_nav_interfaces.srv import SubmitGoal
from rm_nav_interfaces.msg import RegistrationEstimate
from smoke_mppi_baseline import IdealPlant, require
from smoke_frozen_map_localization import cloud
import test_publish_map_bundle as fixture_module
import publish_map_bundle as publisher
from PIL import Image


def main():
    domain=os.environ.get('ROS_DOMAIN_ID')
    require(domain not in (None,'','0','97'),'Dedicated TEST_ONLY domain required')
    fixture=fixture_module.PublishBundleTest()
    fixture.setUp()
    points=[]
    for i in range(29):
        for j in range(15):
            a,z=-3.5+i*.25,-.2+j*.12
            points.extend([(4.,a,z),(-4.,a,z),(a,4.,z),(a,-4.,z)])
    for i in range(29):
        for j in range(29):
            points.append((-3.5+i*.25,-3.5+j*.25,-.25))
    publisher.write_pcd(fixture.pcd,points,'mapping_odom; TEST_ONLY synthetic room')
    fixture.refresh_draft()
    fixture.align.write_text(json.dumps({'status':'operator_defined','source_frame':'mapping_odom',
        'target_frame':'map','local_source':'TEST_ONLY_ROOM','coordinate_definition':'session_mapping_odom',
        'translation':[0,0,0],'yaw_rad':0,'scale':1}))
    mask=fixture.root/'fixture_mask.pgm'
    metadata=fixture.root/'fixture_mask.json'
    Image.new('L',(200,200),255).save(mask)
    metadata.write_text(json.dumps({'status':'operator_reviewed','coordinate_frame':'map',
        'local_source':'TEST_ONLY_ROOM','reviewer':'TEST_ONLY_FIXTURE',
        'review_scope':'free_space_and_obstacles_in_local_map','resolution':.05,'origin':[-5,-5,0]}))
    fixture.run_publish('--profile','local_site','--free-space-mask',str(mask),
                        '--free-space-metadata',str(metadata),'--min-z','-.1','--max-z','1.5')
    manifest=fixture.output/'map_bundle.json'
    bundle=json.loads(manifest.read_text())
    rclpy.init()
    plant=IdealPlant(publish_global_tf=False,publish_localization_health=False,
        publish_motion_permission=False,controller_profile='tdt',sensor_reference=True,
        publish_map=False,publish_scan=False,map_version=bundle['version'])
    observed={'healthy':False,'reason':'','status':'','map':None,'estimates':0,'map_tf':0}
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    plant.create_subscription(Bool,'/localization/healthy',lambda m:observed.update(healthy=m.data),latched)
    plant.create_subscription(String,'/localization/status_reason',lambda m:observed.update(reason=m.data),latched)
    plant.create_subscription(String,'/nav/task_status',lambda m:observed.update(status=m.data),10)
    plant.create_subscription(OccupancyGrid,'/map',lambda m:observed.update(map=m),latched)
    plant.create_subscription(RegistrationEstimate,'/localization/estimate',
        lambda m:observed.update(estimates=observed['estimates']+1),10)
    def tf(msg):
        observed['map_tf']+=sum(t.header.frame_id=='map' and t.child_frame_id=='odom' for t in msg.transforms)
    plant.create_subscription(TFMessage,'/tf',tf,10)
    submap=plant.create_publisher(PointCloud2,'/localization/odom_submap',qos_profile_sensor_data)
    scancloud=plant.create_publisher(PointCloud2,'/lio/deskewed_odom_cloud',qos_profile_sensor_data)
    feed={'submap':True}
    def sensors():
        stamp=plant.get_clock().now().to_msg()
        ns=stamp.sec*10**9+stamp.nanosec-30_000_000
        stamp.sec,stamp.nanosec=divmod(ns,10**9)
        msg=cloud(points,'odom',stamp)
        scancloud.publish(msg)
        if feed['submap']:
            submap.publish(msg)
    timer=plant.create_timer(.15,sensors)
    process=None
    report={'scope':'TEST_ONLY_LOCAL_SITE_IDEAL_PLANT','hardware_connected':False,
            'fake_localization_health_publisher':False,'fixture_map_tf_publisher':False}
    logpath=Path('log/local_site_navigation.log')
    logpath.parent.mkdir(exist_ok=True)
    try:
        with logpath.open('w') as log:
            process=subprocess.Popen(['ros2','launch','rm_nav_bringup','sensor_reference_navigation.launch.py',
                'map_bundle:='+str(manifest),'map_profile:=local_site'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            require(plant.wait(lambda:observed['map'] is not None and observed['healthy'],25),
                    'Real map loading/GICP localization failed: '+str({k:v for k,v in observed.items() if k!='map'}))
            require(plant.wait(lambda:observed['estimates']>=2 and observed['map_tf']>0,3),
                    'Map TF was not established by repeated actual matches')
            grid=observed['map']
            require(0 in grid.data and 100 in grid.data,'Reviewed map did not load free and occupied cells')
            for name in ('map_server','planner_server','controller_server','bt_navigator','collision_monitor'):
                require(plant.wait(lambda n=name:plant.active(n),20),'Lifecycle inactive: '+name)
            require(plant.count_publishers('/cmd_vel')==0 and plant.count_publishers('/cmd_yaw_angle')==0,
                    'Unexpected physical command publisher')
            client=plant.create_client(SubmitGoal,'/nav/submit_goal')
            require(client.wait_for_service(timeout_sec=5),'Task supervisor service missing')
            def submit(y):
                request=SubmitGoal.Request()
                request.goal.header.frame_id='map'
                request.goal.header.stamp=plant.get_clock().now().to_msg()
                request.goal.pose.position.y=y
                request.goal.pose.orientation.w=1.
                future=client.call_async(request)
                require(plant.wait(future.done,5),'Task submit timed out')
                require(future.result().accepted,'Supervisor rejected task: '+future.result().reason)
                return future.result().task_id
            task=submit(.8)
            require(plant.wait(lambda:observed['status'].startswith(str(task)+':IDLE:') and
                'terminal status=4' in observed['status'],35),'Local-site navigation did not finish: '+observed['status'])
            require(math.hypot(plant.x,plant.y-.8)<.25,'Ideal plant missed target')
            require(abs(plant.yaw)<1e-6,'Translation-only navigation rotated reference')
            report.update(first_goal_succeeded=True,final_xy=[plant.x,plant.y],
                          actual_gicp_estimates=observed['estimates'],map_tf_messages=observed['map_tf'])
            submit(2.5)
            require(plant.wait(lambda:plant.max_lateral>.01 and ':NAVIGATING:' in observed['status'],10),
                    'Second task did not start')
            feed['submap']=False
            require(plant.wait(lambda:not observed['healthy'] and plant.stopped(),4),
                    'Real localization dropout did not stop navigation')
            report.update(localization_dropout_stopped=True,result='PASS')
            Path('log/local_site_navigation_report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2))
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
        plant.destroy_timer(timer)
        plant.destroy_node()
        rclpy.shutdown()
        fixture.tearDown()


if __name__=='__main__':
    main()
