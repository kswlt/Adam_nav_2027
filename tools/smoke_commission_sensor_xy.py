import json,os,pty,struct,subprocess,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from my_serial_py.protocol import RX_FORMAT,TX_FORMAT,TX_SIZE,rm_crc16,modbus_crc16
os.environ['ROS_DOMAIN_ID']='215'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
rclpy.init();node=rclpy.create_node('TEST_ONLY_commission_fixture')
health=[node.create_publisher(Bool,t,1) for t in ['/state/sensor_reference_healthy','/sensors/front_mid360/cloud_healthy']]
pose=node.create_publisher(Odometry,'/lio/sensor_odometry',10)
master,slave=pty.openpty();port=os.ttyname(slave);os.close(slave);os.set_blocking(master,False)
payload=b'\xa5'+struct.pack(RX_FORMAT,4,4,100,200,60,42,1200,1500,9,115.6,0)
rx=payload+struct.pack('<H',rm_crc16(payload))
output=Path('/tmp/commission_sensor_xy_fixture_'+str(time.time_ns()))
speed=float(os.environ.get('COMMISSION_FIXTURE_SPEED','.05'))
duration=float(os.environ.get('COMMISSION_FIXTURE_DURATION','.2' if speed>.2 else '.5'))
extra=['--high-speed-commission'] if speed>.2 else []
process=subprocess.Popen(['python3','tools/commission_sensor_xy.py','--port',port,'--output',str(output),'--test-only-pty','--speed',str(speed),'--duration',str(duration),*extra])
buffer=bytearray();frames=[]
try:
    deadline=time.monotonic()+15
    while process.poll() is None and time.monotonic()<deadline:
        for pub in health:pub.publish(Bool(data=True))
        m=Odometry();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='odom';m.child_frame_id='front_mid360_imu';m.pose.pose.orientation.w=1.;pose.publish(m)
        try:os.write(master,rx)
        except OSError:pass
        rclpy.spin_once(node,timeout_sec=.005)
        try:buffer.extend(os.read(master,8192))
        except (BlockingIOError,OSError):pass
        while len(buffer)>=TX_SIZE:
            if buffer[0]!=0xaa:del buffer[0];continue
            packet=bytes(buffer[:TX_SIZE]);del buffer[:TX_SIZE]
            assert modbus_crc16(packet[:-2])==struct.unpack('<H',packet[-2:])[0]
            frames.append((time.monotonic(),struct.unpack(TX_FORMAT,packet[:-2])))
        time.sleep(.005)
    assert process.wait(timeout=2)==0
    nonzero=[(t,f) for t,f in frames if abs(f[1])>1e-6 or abs(f[2])>1e-6]
    assert nonzero and all(abs(f[1]+speed)<1e-6 and abs(f[2])<1e-6 for _,f in nonzero)
    assert nonzero[-1][0]-nonzero[0][0]<duration+.1
    assert all(max(abs(x) for x in f[1:3])<1e-6 for _,f in frames[-20:])
    assert all(f[3:8]==(0.,0.,3,0,0) for _,f in frames)
    print('PASS TEST_ONLY PTY: real encoder/bridge, bounded pulse, trailing zero frames',len(frames),len(nonzero),str(output))
finally:
    if process.poll() is None:process.terminate();process.wait(timeout=5)
    os.close(master);node.destroy_node();rclpy.shutdown()
