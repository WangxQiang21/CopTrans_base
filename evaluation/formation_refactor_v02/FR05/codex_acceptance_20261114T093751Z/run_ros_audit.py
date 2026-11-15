import os,sys,json,time,subprocess,signal,socket
from pathlib import Path
from xmlrpc.client import ServerProxy
R=Path.cwd();OUT=Path(__file__).parent;uri=os.environ["ROS_MASTER_URI"];port=int(uri.rsplit(":",1)[1]);procs=[];logs=[]
# Never attach to an occupied port or touch another test's master.
s=socket.socket();s.bind(("127.0.0.1",port));s.close()
def start(cmd,name):
 f=(OUT/(name+".log")).open("w");logs.append(f);p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
try:
 master=start(["roscore","-p",str(port)],"roscore")
 for _ in range(100):
  try:
   if ServerProxy(uri).getPid("/codex_fr05_audit")[0]==1:break
  except Exception:time.sleep(.1)
 else:raise RuntimeError("private master not ready")
 demo=start(["./run_demo.sh","--map","evaluation/formation_refactor_v02/FR05/empty_map","--scale","1.0","backend:=uniform_v02","rviz:=false","auto_goal:=false","playback_rate:=100"],"run_demo")
 result=subprocess.run([sys.executable,str(OUT/"live_contract_probe.py")],stdout=(OUT/"live_contract_probe.log").open("w"),stderr=subprocess.STDOUT,timeout=90)
 (OUT/"ros_commands.json").write_text(json.dumps({"master_uri":uri,"ROS_HOME":os.environ["ROS_HOME"],"probe_exit_code":result.returncode,"run_demo_pid":demo.pid,"private_roscore_pid":master.pid},indent=2))
 sys.exit(result.returncode)
finally:
 for p in reversed(procs):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in reversed(procs):
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
 for f in logs:f.close()
