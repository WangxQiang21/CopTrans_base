#!/usr/bin/env python3
"""Capture final ROS runs and ask the human to grade each immutable result."""
import copy
from datetime import datetime
import gzip
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import subprocess
import threading
import tarfile
import time
import webbrowser
import numpy as np
from PIL import Image
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from fwis_v3_msgs.msg import PlanSeed, TimedTrajectory
from fwis_v3_optimizer.collection import CollectionStore, identity, atomic_json, serve, utc
from fwis_v3_optimizer.retiming import MotionLimits
from fwis_v3_optimizer.vehicle_execution import vehicle_contract


def git(root, *args):
    try:
        return subprocess.check_output(['git', '-C', str(root), *args], stderr=subprocess.DEVNULL).decode().strip()
    except subprocess.CalledProcessError:
        return 'unknown'


class Collector:
    def __init__(self):
        self.lock = threading.RLock()
        self.rows, self.stamps, self.pending_corridors = {}, {}, {}
        self.map_info = None
        self.backend = rospy.get_param('/fwis_v3_optimizer/planning/backend')
        root = Path(rospy.get_param('~project_root') or Path(__file__).resolve().parents[4]).resolve()
        output = Path(rospy.get_param('~data_root') or root.parent/'expv3_experiment_archive/teacher_collection').expanduser().resolve()
        limits = dict(rospy.get_param('/fwis_v3_optimizer/quality/limits'))
        vehicle = rospy.get_param('/fwis_v3_optimizer/vehicle')
        for k in ('wheelbase', 'track', 'r_min_ack', 'mode_switch_duration', 'mode_setup_time'):
            limits[k] = vehicle[k]
        self.contract = vehicle_contract(vehicle['length'], vehicle['width'],
            rospy.get_param('/fwis_v3_optimizer/collision/margin', .04), MotionLimits(**limits))
        commit, branch = git(root, 'rev-parse', 'HEAD'), git(root, 'branch', '--show-current')
        source = {}
        paths = git(root, 'ls-files', 'catkin_ws/src', 'run_demo.sh', 'tools/collection_panel.html').splitlines()
        for relative in paths:
            p = root/relative
            if p.is_file():
                source[relative] = hashlib.sha256(p.read_bytes()).hexdigest()
        # Include newly added files in a development checkout too.
        for relative in ('catkin_ws/src/fwis_v3_optimizer/scripts/collect_node.py',
                         'catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/collection.py',
                         'catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/vehicle_execution.py'):
            p = root/relative
            source[relative] = hashlib.sha256(p.read_bytes()).hexdigest()
        sid = datetime.now().strftime('%Y%m%dT%H%M%S_%f')+'_'+self.backend
        self.session = {'schema': 'coptrans_collection_session_v1', 'session_id': sid,
            'created_at': utc(), 'branch': branch, 'commit': commit, 'backend': self.backend,
            'git_dirty': git(root, 'status', '--porcelain', '-uno'),
            'source_sha256': identity(source), 'source_files': source,
            'execution_contract': self.contract, 'quality_profile': rospy.get_param('/fwis_v3_optimizer/quality/cost'),
            'map_directory': rospy.get_param('~map_dir'), 'map_scale': rospy.get_param('~map_scale', 1.),
            'use_sim_time': rospy.get_param('/use_sim_time', False)}
        if output == root or root in output.parents:
            raise ValueError('采集数据请保存在仓库外，使用 --data-root 指定外部目录')
        self.store = CollectionStore(output, self.session)
        source_archive = self.store.session_dir/'source.tar.gz'
        with tarfile.open(str(source_archive), 'w:gz', dereference=True) as archive:
            for relative in sorted(source):
                archive.add(str(root/relative), arcname=relative, recursive=False)
        self.session['source_archive'] = source_archive.name
        self.session['source_archive_sha256'] = hashlib.sha256(source_archive.read_bytes()).hexdigest()
        atomic_json(self.store.session_dir/'session.json', self.session)
        self.server, self.url = serve(self.store, root/'tools/collection_panel.html',
                                      rospy.get_param('~port', 0), self.timeout)
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber('/plan/request', PlanSeed, self.on_request, queue_size=10)
        rospy.Subscriber('/plan/timed_trajectory', TimedTrajectory, self.on_trace, queue_size=2)
        rospy.Subscriber('/plan/corridor_markers', MarkerArray, self.on_corridor, queue_size=2)
        rospy.Subscriber('/plan/quality_report', String, self.on_quality, queue_size=10)
        rospy.Subscriber('/plan/search_status', String, self.on_search, queue_size=10)
        rospy.on_shutdown(self.server.shutdown)
        rospy.logwarn('[采集] 打开 %s  数据目录 %s', self.url, output)
        if rospy.get_param('~open_browser', True) and (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            threading.Thread(target=lambda: webbrowser.open(self.url), daemon=True).start()

    def row(self, plan_id):
        return self.rows.setdefault(plan_id, {'schema': 'coptrans_collected_episode_v1',
            'record_id': self.session['session_id']+'-'+str(plan_id), 'plan_id': plan_id,
            'backend': self.backend, 'branch': self.session['branch'], 'created_at': utc(),
            'execution_contract': self.contract, 'timed_trace': [], 'corridors': [],
            'request': None, 'final': False, 'eligible': False, 'outcome': 'planning'})

    def persist(self, row):
        if row.get('final') and 'request_to_capture_wall_s' not in row and row.get('requested_monotonic'):
            row['request_to_capture_wall_s'] = time.monotonic()-row['requested_monotonic']
        if row.get('request') and row.get('map_id'):
            row['task_id'] = identity({'map_id': row['map_id'], 'request': row['request']})
        if row.get('request'):
            self.store.update(row)

    def on_map(self, msg):
        if msg.header.frame_id != 'map' or abs(msg.info.origin.orientation.z) > 1e-10 or abs(msg.info.origin.orientation.w-1.) > 1e-10:
            rospy.logerr('[采集] 不支持非 map 坐标或旋转地图，不保存为示范')
            return
        info = {'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution,
                'origin': [msg.info.origin.position.x, msg.info.origin.position.y], 'data': list(msg.data)}
        key = identity(info)
        with self.lock:
            self.map_info = {'id': key, **{k: info[k] for k in ('width','height','resolution','origin')}}
            directory = self.store.root/'maps';directory.mkdir(exist_ok=True)
            if not (directory/(key+'.json.gz')).exists():
                with gzip.open(str(directory/(key+'.json.gz')), 'wt') as f:
                    json.dump(info, f, separators=(',', ':'))
                pixels = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
                pixels = np.where(pixels < 0, 120, np.where(pixels >= 50, 15, 205)).astype(np.uint8)
                Image.fromarray(np.flipud(pixels)).save(str(directory/(key+'.png')))
            # A latched request may arrive before the map subscription.
            for row in self.rows.values():
                if not row.get('map_id'):
                    row['map_id'], row['map'] = key, copy.deepcopy(self.map_info)
                    self.finish(row)
                elif not row.get('final') and row['map_id'] != key:
                    row['map_changed_during_request'] = True
                    self.finish(row)

    def on_request(self, msg):
        with self.lock:
            row = self.row(int(msg.plan_id))
            row['request'] = {'start': [msg.start_center.x,msg.start_center.y,msg.start_center.theta],
                              'goal': [msg.goal_center.x,msg.goal_center.y,msg.goal_center.theta],
                              'initial_mode': 0, 'initial_speed_mps': 0., 'frame_id': msg.header.frame_id}
            row['request_ros_stamp'] = msg.header.stamp.to_sec()
            row['requested_monotonic'] = time.monotonic()
            if self.map_info:
                row['map_id'], row['map'] = self.map_info['id'], copy.deepcopy(self.map_info)
            self.finish(row)

    def on_trace(self, msg):
        if not msg.points:
            return
        with self.lock:
            row = self.row(int(msg.plan_id))
            row['timed_trace'] = [[p.time_from_start.to_sec(),p.center_pose.x,p.center_pose.y,p.center_pose.theta,p.v,int(p.mode)] for p in msg.points]
            key = str(msg.header.stamp.to_nsec());self.stamps[key] = int(msg.plan_id)
            row['trajectory_ros_stamp'] = msg.header.stamp.to_sec()
            if key in self.pending_corridors:
                row['corridors'] = self.pending_corridors.pop(key);row['corridors_received'] = True
            self.finish(row)

    def on_corridor(self, msg):
        if not msg.markers:
            return
        markers = []
        for m in msg.markers:
            if m.action in (2,3) or not m.points:
                continue
            # Markers are already in map/world coordinates in these planners.
            markers.append({'type': int(m.type), 'namespace':m.ns, 'id':int(m.id),
                            'points': [[p.x,p.y,p.z] for p in m.points],
                            'color': [m.color.r,m.color.g,m.color.b,m.color.a]})
        key = str(msg.markers[0].header.stamp.to_nsec())
        with self.lock:
            if key in self.stamps:
                row = self.row(self.stamps[key]);row['corridors'] = markers;row['corridors_received'] = True;self.finish(row)
            else:
                self.pending_corridors[key] = markers
                if len(self.pending_corridors)>32:
                    self.pending_corridors.pop(next(iter(self.pending_corridors)))

    def on_quality(self, msg):
        try:
            report = json.loads(msg.data)
        except ValueError:
            return
        if 'plan_id' not in report or report.get('status') in ('pending','IN_PROGRESS'):
            return
        with self.lock:
            row = self.row(int(report['plan_id']))
            if row.get('outcome') == 'operator_timeout':
                return
            # Persisted diagnostic carries the effective configuration and source hashes.
            path = report.get('diagnostic_file')
            if path and Path(path).is_file():
                try:
                    saved = json.loads(Path(path).read_text())
                    if saved.get('plan_id') == report['plan_id']:
                        report = {**saved, **report}
                except (ValueError,OSError):
                    pass
            row['quality_report'] = report
            self.finish(row)

    def on_search(self, msg):
        match = re.search(r'\bid=(\d+)', msg.data)
        if not match:
            return
        with self.lock:
            row = self.row(int(match[1]));row['search_status'] = msg.data
            if not msg.data.startswith('search_success'):
                row['outcome'] = 'frontend_failed';row['final'] = True
            self.finish(row)

    def finish(self, row):
        if row.get('outcome') == 'operator_timeout':
            self.persist(row)
            return
        report = row.get('quality_report', {})
        status = report.get('status')
        success = status in ('FEASIBLE','LEGACY_NATIVE_PUBLISHED')
        if success:
            ready = bool(row['timed_trace']) and row.get('corridors_received', False)
            row['final'] = ready
            row['outcome'] = 'published' if ready else 'waiting_for_trajectory_and_corridors'
            audit = report.get('selected', {})
            contract = report.get('execution_contract', {})
            expected_duration = report.get('selected_timing', {}).get('duration_s')
            duration_matches = bool(row['timed_trace']) and expected_duration is not None and abs(row['timed_trace'][-1][0]-expected_duration)<1e-7
            row['eligible'] = bool(ready and not row.get('map_changed_during_request') and row.get('map_id') and contract.get('sha256') == self.contract['sha256']
                and audit.get('status') == 'FEASIBLE' and audit.get('geometry_status') == 'PASS'
                and audit.get('dynamic_status') in ('PASS','KINEMATIC_PROXY_PASS') and duration_matches
                and report.get('execution_display_sweep_clear') is True)
            if not row['eligible']:
                row['eligibility_reason'] = ('规划中地图发生变化' if row.get('map_changed_during_request') else audit.get('reason') or '轨迹、地图或车辆契约尚未完整匹配')
        elif status:
            row['final'] = True;row['eligible'] = False;row['outcome'] = status.lower()
        self.persist(row)

    def timeout(self, value):
        with self.lock:
            row = next((r for r in self.rows.values() if r['record_id']==value.get('record_id')), None)
            if row is None or row.get('final'):
                raise ValueError('任务不存在或已经结束')
            row.update(final=True, eligible=False, outcome='operator_timeout')
            self.persist(row)
            return {'recorded': True, 'note': '仅记录等待超时，不取消正在运行的规划；请等规划结束或重启后再开始下一条。'}


if __name__ == '__main__':
    rospy.init_node('fwis_data_collector')
    Collector()
    rospy.spin()
