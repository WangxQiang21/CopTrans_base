import math
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry

def components(msg):
    t=msg.twist.twist
    return t.linear.x,t.linear.y,t.angular.z

rclpy.init()
node=rclpy.create_node('test_ranger_body_interface')
last={}
node.create_subscription(Odometry,'/ranger1/odom',lambda m: last.update(odom=m),5)
node.create_subscription(Int32,'/ranger1/motion_state',lambda m: last.update(mode=m.data),5)
pub=node.create_publisher(Twist,'/ranger1/cmd_vel',1)
def poll(condition, duration, cmd=None):
    deadline=time.monotonic()+duration
    next_pub=0.
    while time.monotonic()<deadline:
        if condition():
            return True
        if cmd is not None and time.monotonic()>=next_pub:
            pub.publish(cmd)
            next_pub=time.monotonic()+.04
        rclpy.spin_once(node,timeout_sec=.02)
    return False
try:
    assert poll(lambda: 'odom' in last and 'mode' in last and pub.get_subscription_count()>0, 20),last
    odom=last['odom']
    print('feedback frames',odom.header.frame_id,odom.child_frame_id,flush=True)
    assert (odom.header.frame_id,odom.child_frame_id)==('map','ranger1/base_link')
    origin=odom.pose.pose.position
    for label,v,w,y,m in [('straight',.12,0.,0.,0), ('turn',.12,.12,0.,0),
                           ('spin',.10,.30,0.,2), ('parallel',0.,0.,.10,1)]:
        assert poll(lambda: max(map(abs,components(last['odom'])))<.005,.9,Twist()),label
        cmd=Twist()
        cmd.linear.x,cmd.linear.y,cmd.angular.z=v,y,w
        expected=(0. if m==2 else v,y,w)
        assert poll(lambda: last['mode']==m and math.dist(components(last['odom']),expected)<.006,
                    3.,cmd),(label,last['mode'],components(last['odom']),expected)
        p=last['odom'].pose.pose.position
        print(label, last['mode'], tuple(round(x,4) for x in components(last['odom'])),
              'ECM pose',tuple(round(x,4) for x in (p.x,p.y,p.z)),flush=True)
    assert poll(lambda: max(map(abs,components(last['odom'])))<.005, 1.6)
    p=last['odom'].pose.pose.position
    assert math.dist((origin.x,origin.y),(p.x,p.y))>.07
    print('watchdog: stopped after command stream ceased',flush=True)
finally:
    node.destroy_node()
    rclpy.try_shutdown()
