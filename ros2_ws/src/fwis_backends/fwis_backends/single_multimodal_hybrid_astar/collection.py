"""ROS-independent storage and local human-review server for collected runs."""
import base64
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

GRADES = ('pending', 'approved', 'comparison', 'rejected')


def utc():
    return datetime.now(timezone.utc).isoformat()


def identity(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name('.'+path.name+'.tmp')
    with temp.open('w') as f:
        json.dump(data, f, ensure_ascii=False, allow_nan=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())
    os.replace(str(temp), str(path))


class CollectionStore:
    def __init__(self, root, session=None, secondary=False):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.session = session
        self.secondary = secondary
        self.lock = threading.RLock()
        if session:
            self.session_dir = self.root/'sessions'/session['session_id']
            atomic_json(self.session_dir/'session.json', session)

    def paths(self):
        if self.session:
            return sorted((self.session_dir/'episodes').glob('*/episode.json'))
        return sorted((self.root/'sessions').glob('*/episodes/*/episode.json'))

    def load(self):
        rows = []
        for p in self.paths():
            try:
                r = json.loads(p.read_text())
                analysis_path = p.parent/'analysis.json'
                if analysis_path.exists():
                    a = json.loads(analysis_path.read_text())
                    if a.get('result_sha256') == r.get('result_sha256'):
                        r['analysis'] = a
                r['record_path'] = str(p)
                rows.append(r)
            except (OSError, ValueError):
                continue
        return rows

    def summary(self, row, path):
        v = {k: row.get(k) for k in ('record_id','plan_id','backend','branch',
             'map_id','request','outcome','final','eligible','label','notes',
             'review_label','review_notes','result_sha256','task_id','updated_at')}
        v['record_path'] = str(path)
        return v

    def update(self, row):
        with self.lock:
            p = self.session_dir/'episodes'/('%06d' % row['plan_id'])/'episode.json'
            if p.exists():
                old = json.loads(p.read_text())
                if old.get('frozen'):
                    return
                for k in ('label', 'notes', 'review_label', 'review_notes'):
                    if k in old:
                        row[k] = old[k]
            row = copy.deepcopy(row)
            row.setdefault('label', 'pending')
            row.setdefault('notes', '')
            row.setdefault('review_label', 'pending')
            row.setdefault('review_notes', '')
            result = {k: row.get(k) for k in ('request', 'map_id', 'execution_contract',
                       'quality_report', 'timed_trace', 'corridors', 'outcome')}
            row['result_sha256'] = identity(result)
            row['updated_at'] = utc()
            atomic_json(p, row)
            atomic_json(p.parent/'summary.json', self.summary(row,p))

    def state(self):
        with self.lock:
            result = []
            for p in self.paths():
                summary_path = p.parent/'summary.json'
                if summary_path.exists():
                    v = json.loads(summary_path.read_text())
                else:
                    row = json.loads(p.read_text())
                    v = self.summary(row,p)
                    atomic_json(summary_path,v)
                a_path = p.parent/'analysis_summary.json'
                if a_path.exists():
                    a = json.loads(a_path.read_text())
                    if a.get('result_sha256') == v.get('result_sha256'):
                        v['analysis'] = a
                result.append(v)
            return {'schema': 'coptrans_collection_state_v1', 'root': str(self.root),
                    'secondary': self.secondary, 'session': self.session,
                    'records': result}

    def find(self, record_id):
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', record_id or ''):
            raise ValueError('invalid_record_id')
        try:
            sid, number = record_id.rsplit('-',1)
            if not re.fullmatch(r'[A-Za-z0-9_]+', sid) or not number.isdecimal() or (self.session and sid != self.session['session_id']):
                raise ValueError('record_not_found')
            p = self.root/'sessions'/sid/'episodes'/('%06d' % int(number))/'episode.json'
            row = json.loads(p.read_text())
            if row['record_id'] != record_id:
                raise ValueError('record_identity_mismatch')
            row['record_path'] = str(p)
            a_path = p.parent/'analysis.json'
            if a_path.exists():
                a = json.loads(a_path.read_text())
                if a.get('result_sha256') == row.get('result_sha256'):
                    row['analysis'] = a
            return row
        except (OSError,KeyError):
            raise ValueError('record_not_found')

    def label(self, value):
        with self.lock:
            row = self.find(value.get('record_id'))
            if row.get('result_sha256') != value.get('result_sha256'):
                raise ValueError('结果已更新，请重新查看后标注')
            if value.get('grade') not in GRADES or not isinstance(value.get('notes'), str):
                raise ValueError('invalid_label')
            if not row.get('final') and not self.secondary:
                raise ValueError('规划尚未结束，不能标注；超时任务请先记录为超时')
            analysis = row.get('analysis')
            eligible = bool(analysis.get('eligible')) if self.secondary and analysis else bool(row.get('eligible'))
            if self.secondary and analysis is None:
                eligible = False
            if value['grade'] == 'approved' and not eligible:
                raise ValueError('共同审核未通过，不能标为优质；可以标为参考或剔除')
            prefix = 'review_' if self.secondary else ''
            row[prefix+'label'] = value['grade']
            row[prefix+'notes'] = value['notes']
            row[prefix+'labeled_at'] = utc()
            row['frozen'] = True
            p = Path(row.pop('record_path'))
            row.pop('analysis', None)
            preview = value.get('preview_png', '')
            if preview:
                if not preview.startswith('data:image/png;base64,'):
                    raise ValueError('invalid_preview')
                data = base64.b64decode(preview.split(',', 1)[1], validate=True)
                if not data.startswith(b'\x89PNG\r\n\x1a\n'):
                    raise ValueError('invalid_preview_png')
                preview_path = p.parent/('review_preview.png' if self.secondary else 'preview.png')
                preview_path.write_bytes(data)
                row[prefix+'preview'] = preview_path.name
            atomic_json(p, row)
            atomic_json(p.parent/'summary.json', self.summary(row,p))
            return {'saved': str(p), 'grade': value['grade']}

    def export(self):
        rows = self.load()
        return {'schema': 'coptrans_collected_selection_v1', 'exported_at': utc(),
                'secondary': self.secondary, 'root': str(self.root),
                'selections': [{k: r.get(k) for k in ('record_id', 'record_path', 'result_sha256',
                    'task_id', 'backend', 'label', 'notes', 'review_label', 'review_notes')}
                    for r in rows]}


def serve(store, template, port=0, timeout_callback=None):
    page = Path(template).read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, data, mime='application/json', status=200):
            body = data if isinstance(data, bytes) else json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            try:
                path = self.path.split('?', 1)[0]
                if path == '/':
                    return self.send(page, 'text/html; charset=utf-8')
                if path == '/api/state':
                    return self.send(store.state())
                if path == '/api/export':
                    return self.send(store.export())
                if path.startswith('/api/record/'):
                    return self.send(store.find(path.rsplit('/', 1)[-1]))
                if re.fullmatch(r'/map/[a-f0-9]{64}\.png', path):
                    return self.send((store.root/'maps'/path.rsplit('/', 1)[-1]).read_bytes(), 'image/png')
                self.send({'error': 'not_found'}, status=404)
            except (ValueError, OSError) as exc:
                self.send({'error': str(exc)}, status=400)

        def do_POST(self):
            # Local browser UI only; JSON avoids cross-origin form submission.
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                return self.send({'error': 'json_required'}, status=400)
            origin = self.headers.get('Origin')
            if origin and origin != 'http://'+self.headers.get('Host', ''):
                return self.send({'error': 'invalid_origin'}, status=403)
            try:
                size = int(self.headers.get('Content-Length', 0))
                if size < 1 or size > 8*1024*1024:
                    raise ValueError('invalid_request_size')
                value = json.loads(self.rfile.read(size))
                if self.path == '/api/label':
                    return self.send(store.label(value))
                if self.path == '/api/timeout' and timeout_callback:
                    return self.send(timeout_callback(value))
                self.send({'error': 'not_found'}, status=404)
            except (ValueError, OSError, KeyError) as exc:
                self.send({'error': str(exc)}, status=400)

    server = ThreadingHTTPServer(('127.0.0.1', int(port)), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, 'http://127.0.0.1:%d/' % server.server_port
