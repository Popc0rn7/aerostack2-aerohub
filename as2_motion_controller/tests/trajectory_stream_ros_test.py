import math
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.msg import ControlMode, PlatformInfo, TrajectoryPoint, TrajectorySetpoints
from as2_msgs.srv import ListControlModes, SetControlMode

rclpy.init()
n=rclpy.create_node('stream_test',namespace='/stream')
platform_modes=[];received=[]
def listing(req,res):res.control_modes=[16,112];return res
def setting(req,res):platform_modes.append(req.control_mode.control_mode);res.success=True;return res
n.create_service(ListControlModes,'platform/list_control_modes',listing)
n.create_service(SetControlMode,'set_platform_control_mode',setting)
# Names are platform/list_control_modes? Resolve below from AS2 names.
info=n.create_publisher(PlatformInfo,'platform/info',10)
ref=n.create_publisher(TrajectorySetpoints,'motion_reference/trajectory',1)
n.create_subscription(TrajectorySetpoints,'actuator_command/trajectory',lambda m:received.append((time.monotonic(),m)),qos_profile_sensor_data)
def pub_info():
 m=PlatformInfo();m.armed=True;m.offboard=True;info.publish(m)
n.create_timer(.02,pub_info)
def wait(pred,timeout=10):
 deadline=time.monotonic()+timeout
 while not pred():
  if time.monotonic()>deadline:raise TimeoutError('test timeout')
  rclpy.spin_once(n,timeout_sec=.002)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.002)
c=n.create_client(SetControlMode,'controller/set_control_mode')
wait(c.service_is_ready)
def mode(value):
 req=SetControlMode.Request();req.control_mode.control_mode=value;req.control_mode.yaw_mode=ControlMode.YAW_ANGLE
 f=c.call_async(req);wait(f.done);assert f.result().success
wait(lambda:ref.get_subscription_count()>0)
summary=[]
for hz in (20,50,100):
 mode(ControlMode.TRAJECTORY)
 received.clear();sent=set();start=time.monotonic();next_sample=start
 while time.monotonic()-start<1.2:
  now=time.monotonic()
  if now>=next_sample:
   m=TrajectorySetpoints();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='stream/odom'
   p=TrajectoryPoint();p.position.x=float(len(sent));p.position.z=1.
   p.twist.x=1.;p.acceleration.z=.5;p.yaw_angle=.2;m.setpoints=[p]
   sent.add((m.header.stamp.sec,m.header.stamp.nanosec));ref.publish(m);last=m
   next_sample+=1./hz
  rclpy.spin_once(n,timeout_sec=.001)
 stream_count=len(received)
 assert 45<=stream_count<=70,(hz,stream_count)
 assert all((m.header.stamp.sec,m.header.stamp.nanosec) in sent for _,m in received)
 xs=[m.setpoints[0].position.x for _,m in received];assert xs==sorted(xs)
 # Duplicate input, including stale sample, must not postpone failsafe.
 end=time.monotonic()+.7
 while time.monotonic()<end:
  ref.publish(last);rclpy.spin_once(n,timeout_sec=.005)
 wait(lambda:platform_modes[-1]==ControlMode.HOVER)
 assert received[-1][0]-start<1.2+.35
 count=len(received)
 # Fresh samples cannot resume a latched mode.
 for _ in range(15):
  last.header.stamp=n.get_clock().now().to_msg();ref.publish(last);spin(.02)
 assert len(received)==count
 summary.append({'input_hz':hz,'output_count_in_1_2s':stream_count,'failsafe_hover':True,'auto_resume':False})
print(summary,flush=True)
n.destroy_node();rclpy.shutdown()
