"""Passive DDS listener. Write the C++ tracker diagnostics verbatim, for both ROS and steady clock analysis."""
from contextlib import ExitStack
import gzip
import json
from pathlib import Path
import signal
import sys
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
node=rclpy.create_node("v3_timing_observer")
active=True
def finish(*_):
    global active
    active=False
signal.signal(signal.SIGTERM,finish)
signal.signal(signal.SIGINT,finish)
with ExitStack() as files:
    for i in range(1,int(sys.argv[2])+1):
        handle=files.enter_context(gzip.open(Path(sys.argv[1])/f"robot{i}_controller.jsonl.gz","wt",compresslevel=2))
        def record(msg,out=handle):
            if '"control_timing"' in msg.data:out.write(msg.data+'\n')
        node.create_subscription(String,f"/robot{i}/controller_status",record,100)
    while active:rclpy.spin_once(node,timeout_sec=.15)
node.destroy_node()
rclpy.shutdown()
