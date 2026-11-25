"""Cancel only after observing actual movement on the isolated Gazebo domain."""
import json
from pathlib import Path
import sys
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rclpy.init()
node=rclpy.create_node("hybrid_v3_fault_probe")
pub=node.create_publisher(String,"/hybrid_joint/cancel",10)
count=int(sys.argv[2])
first={};last={};moving_after={};zero_after={}
sent=None
def observe(i,msg):
    moving=max(abs(msg.linear.x),abs(msg.linear.y),abs(msg.angular.z))>1e-5
    t=time.monotonic()
    if moving:first.setdefault(i,t)
    if sent is not None:
        if moving:moving_after[i]=t-sent
        else:zero_after.setdefault(i,t-sent)
    last[i]=moving
for i in range(1,count+1):
    node.create_subscription(Twist,f"/robot{i}/cmd_vel",lambda m,i=i:observe(i,m),10)
end=time.monotonic()+140
while time.monotonic()<end:
    rclpy.spin_once(node,timeout_sec=.02)
    if sent is None and len(first)==count and pub.get_subscription_count():
        sent=time.monotonic()
        pub.publish(String(data="benchmark_injected_cancel"))
    if sent is not None and time.monotonic()-sent>=.7:break
Path(sys.argv[1]).write_text(json.dumps(dict(triggered=sent is not None,moved=sorted(first),last=last,
                                            last_motion_after_s=moving_after,first_zero_after_s=zero_after)))
node.destroy_node();rclpy.shutdown()
