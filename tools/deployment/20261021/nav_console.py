#!/usr/bin/env python3
"""Chinese terminal UI for the isolated vehicle session; no shell-evaluated inputs."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path('/home/asus/nav_2027_vehicle_config_164')
RUNTIME=ROOT/'runtime_20261021_sensor_reference'
RELEASE=ROOT/'releases/20261021_sensor_reference'
COMPONENTS=('driver','state','foxglove','capture','online','bag','navigation','candidate','physical')
TOPICS={
    '1':('/visualization/live_map_status','累计地图：点数、分辨率、断流'),
    '2':('/mapping/online_status','回环：关键帧、优化、故障'),
    '3':('/state/sensor_reference_reason','定位状态原因'),
    '4':('/nav/sensor_command_status','速度转换、拟合状态与朝向时效'),
    '5':('/hardware/legacy_adam/adapter_status','串口适配状态'),
    '6':('/nav/task_status','导航任务状态'),
    '7':('/lio/sensor_odometry','LIO里程计'),
    '8':('/contact_angle_stamped','电控角度与主机接收时间')}


class Console:
    def __init__(self,dry=False):self.dry=dry

    def call(self,args,ros=False,capture=False):
        if self.dry:
            print('[预演，不执行] '+json.dumps(list(map(str,args)),ensure_ascii=False))
            return '{}'
        env=dict(os.environ,ROS_DOMAIN_ID='97',ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET')
        if ros:
            args=['bash','-c','set -e; source "$1"; source "$2"; shift 2; exec "$@"',
                  'nav_console',str(ROOT/'remote_env_20261020.bash'),str(RELEASE/'install/local_setup.bash'),*args]
        result=subprocess.run(list(map(str,args)),env=env,check=True,text=True,
                              stdout=subprocess.PIPE if capture else None)
        return result.stdout or ''

    def manage(self,action,component,*args,capture=False):
        manager=Path(__file__).with_name('vehicle_session.py')
        return self.call([sys.executable,str(manager),action,component,*args],capture=capture)

    def status(self):
        for name in COMPONENTS:
            state=json.loads(self.manage('status',name,capture=True))
            print(f"{name:12s} {'运行' if state.get('running') else '未运行'}  {state.get('log','')}")
        print('进程运行不等于定位/建图验收通过；选择“消息”查看真实状态。')

    def mapping(self):
        for name in ('physical','navigation','candidate'):self.manage('stop',name)
        archive=RUNTIME/'keyframes_fit153cm_debug_v3'
        before=set(archive.glob('session_*'))
        running=False if self.dry else json.loads(self.manage('status','capture',capture=True)).get('running',False)
        for name in ('driver','state','foxglove','capture','bag'):self.manage('start',name)
        print('建图采集已请求启动；原始录包当前限时180秒，关键帧采集继续运行。')
        if self.dry:
            print('真实运行将等待本次新会话，再启动在线优化；不会自动选旧会话。')
            return
        if running:
            print('采集此前已运行；请用“在线优化”明确选择当前会话。')
            return
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            created=set(archive.glob('session_*'))-before
            if len(created)==1:
                self.manage('start','online',str(created.pop()))
                print('在线优化已请求启动。请查看日志和Foxglove。')
                return
            if len(created)>1:break
            time.sleep(.5)
        print('未确认唯一新会话；未自动启动优化。查看capture日志或手动选择会话。')

    def navigation(self,manifest=None,profile=None):
        manifest=Path(manifest or input('正式MapBundle的map_bundle.json绝对路径：').strip()).expanduser()
        profile=profile or input('地图类型 local_site / official：').strip()
        if not manifest.is_absolute() or not manifest.is_file():raise ValueError('地图文件必须存在且为绝对路径')
        if profile not in ('local_site','official'):raise ValueError('地图类型必须明确选择')
        # Runtime launch still performs the actual published/geometry/map contract checks.
        for name in ('physical','candidate','bag','capture','online'):self.manage('stop',name)
        for name in ('driver','state','foxglove'):self.manage('start',name)
        self.manage('start','navigation',str(manifest),profile)
        self.manage('start','candidate')
        print('已请求影子导航：需要地图初值和定位健康；未开启物理串口。')

    def logs(self):
        name=input('组件 driver/state/foxglove/capture/online/navigation/physical：').strip()
        if name not in COMPONENTS:raise ValueError('未知组件')
        state=json.loads(self.manage('status',name,capture=True))
        if not state.get('log'):raise ValueError('暂无组件日志')
        self.call(['tail','-n','80','-f',state['log']])

    def topics(self):
        for key,(topic,label) in TOPICS.items():print(key,label,topic)
        topic=TOPICS[input('选择消息，Ctrl+C返回菜单：').strip()][0]
        self.call(['ros2','topic','echo',topic],ros=True)

    def online(self):
        sessions=sorted((RUNTIME/'keyframes_fit153cm_debug_v3').glob('session_*'))
        for i,path in enumerate(sessions):print(i,str(path))
        if not sessions:raise ValueError('没有当前配置的关键帧会话')
        self.manage('start','online',str(sessions[int(input('选择会话编号：'))]))

    def export(self):
        import math
        output=Path(input('新的地图检查包绝对路径（目录不得已存在）：').strip()).expanduser()
        low,high=map(float,input('mapping_odom中的障碍物高度范围 min_z max_z（米）：').split())
        if not output.is_absolute() or output.exists():raise ValueError('必须指定尚不存在的绝对目录')
        if not all(map(math.isfinite,(low,high))) or low>=high:raise ValueError('高度范围无效')
        self.call(['python3',str(RELEASE/'tools/export_online_map_review.py'),
                   '--output',str(output),'--min-z',str(low),'--max-z',str(high)],ros=True)
        print('已请求导出检查包；它是draft，不能直接用于正式导航。')

    def foxglove(self):
        print('连接 ws://192.168.1.164:8765\n'
              '累计图：sensor_reference_mapping.json，参考系odom，/visualization/live_map_preview\n'
              '回环图：mapping_online.json，参考系mapping_odom，/mapping/optimized_map_preview\n'
              '导航：sensor_reference_navigation.json，参考系map。布局在release/src/rm_nav_bringup/config/foxglove。\n'
              '点云只开一种；实时地图状态见菜单6→1。每条预览已经是整图。')

    def stop(self):
        for name in ('physical','navigation','candidate','bag','online','capture','foxglove','state','driver'):
            self.manage('stop',name)
        print('仅请求停止本项目管理的组件，不终止原车程序。')

    def physical(self,enabled):
        if enabled:
            self.manage('stop','candidate')
            self.manage('start','physical')
        else:self.manage('stop','physical')

    def goal(self):
        import math
        x,y=map(float,input('目标地图坐标 x y（米）：').split())
        if not all(map(math.isfinite,(x,y))):raise ValueError('目标必须是有限数值')
        payload={'goal':{'header':{'frame_id':'map'},'pose':{'position':{'x':x,'y':y,'z':0.},'orientation':{'w':1.}}}}
        self.call(['ros2','service','call','/nav/submit_goal','rm_nav_interfaces/srv/SubmitGoal',json.dumps(payload)],ros=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run',action='store_true',help='只打印计划，不启动进程')
    parser.add_argument('--action',choices=('status','mapping','navigation','stop'))
    parser.add_argument('--map-bundle')
    parser.add_argument('--map-profile',choices=('local_site','official'))
    args=parser.parse_args()
    ui=Console(args.dry_run)
    if args.action:
        if args.action=='navigation':ui.navigation(args.map_bundle,args.map_profile)
        else:getattr(ui,args.action)()
        return
    actions={'1':ui.status,'2':ui.mapping,'3':ui.online,'4':ui.navigation,'5':ui.logs,
             '6':ui.topics,'7':ui.goal,'8':lambda:ui.physical(True),'9':lambda:ui.physical(False),
             'e':ui.export,'f':ui.foxglove,'s':ui.stop}
    while True:
        print('\n=== 新导航控制台 | Domain97 | 仅本项目 ===\n'
              '1 状态  2 启动建图+录包+在线优化  3 选择在线优化会话\n'
              '4 影子导航（选地图）  5 实时终端日志  6 实时消息\n'
              '7 发送地图目标  8 开启真实串口输出  9 关闭真实串口输出\n'
              'e 导出地图检查包（draft）  f Foxglove配置说明\n'
              's 停止本项目全部组件  q 退出菜单（组件继续运行）')
        try:
            choice=input('选择：').strip().lower()
            if choice=='q':return
            if choice not in actions:print('请选择菜单中的选项');continue
            actions[choice]()
        except KeyboardInterrupt:print('\n返回菜单；后台组件保持现状。')
        except EOFError:return
        except (ValueError,KeyError,IndexError,subprocess.CalledProcessError) as exc:print('操作未完成：',exc)


if __name__=='__main__':main()
