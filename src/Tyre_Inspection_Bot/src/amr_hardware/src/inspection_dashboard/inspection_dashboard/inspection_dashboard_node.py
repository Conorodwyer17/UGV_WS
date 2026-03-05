#!/usr/bin/env python3
"""
Inspection Web Dashboard - Real-time observability for tire inspection missions.

Features:
  - FSM state (inspection_state)
  - Runtime diagnostics (robot pose, distance_to_goal, nav_feedback, proximity_gate)
  - Mission report (when available)
  - Optional Aurora left camera stream
  - Manual start mission button (when start_mission_on_ready=false)

Usage:
  ros2 run inspection_dashboard inspection_dashboard_node
  Then open http://localhost:8085 in a browser.
"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler
from socketserver import TCPServer, ThreadingMixIn
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

_bridge = CvBridge()


def _no_signal(w: int = 640, h: int = 480, label: str = "NO SIGNAL") -> np.ndarray:
    """Placeholder frame when no camera feed."""
    f = np.zeros((h, w, 3), dtype=np.uint8)
    f[:] = (50, 40, 30)
    cv2.rectangle(f, (2, 2), (w - 3, h - 3), (0, 180, 220), 2)
    sz = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)[0]
    cx, cy = (w - sz[0]) // 2, (h + sz[1]) // 2
    cv2.putText(f, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 255), 2, cv2.LINE_AA)
    return f


class InspectionDashboardNode(Node):
    """ROS 2 node that subscribes to inspection topics and serves HTTP."""

    def __init__(self):
        super().__init__("inspection_dashboard")

        self.declare_parameter("port", 8085)
        self.declare_parameter("inspection_state_topic", "/inspection_state")
        self.declare_parameter("runtime_diagnostics_topic", "/inspection_manager/runtime_diagnostics")
        self.declare_parameter("mission_report_topic", "/inspection_manager/mission_report")
        self.declare_parameter("current_goal_topic", "/inspection_manager/current_goal")
        self.declare_parameter("start_mission_topic", "/inspection_manager/start_mission")
        self.declare_parameter("camera_topic", "/slamware_ros_sdk_server_node/left_image_raw")
        self.declare_parameter("enable_camera_stream", True)
        self.declare_parameter("jpeg_quality", 75)

        self._lock = threading.Lock()
        self._inspection_state: str = ""
        self._runtime_diagnostics: Optional[dict] = None
        self._mission_report: Optional[dict] = None
        self._mission_report_raw: Optional[str] = None
        self._current_goal: Optional[dict] = None
        self._frame_camera: Optional[np.ndarray] = None
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        # Subscriptions
        self.create_subscription(
            String,
            self.get_parameter("inspection_state_topic").value,
            self._on_inspection_state,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("runtime_diagnostics_topic").value,
            self._on_runtime_diagnostics,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("mission_report_topic").value,
            self._on_mission_report,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("current_goal_topic").value,
            self._on_current_goal,
            10,
        )

        if self.get_parameter("enable_camera_stream").value:
            self.create_subscription(
                Image,
                self.get_parameter("camera_topic").value,
                self._on_camera,
                5,
            )

        # Publisher for start_mission
        self._start_mission_pub = self.create_publisher(
            Bool,
            self.get_parameter("start_mission_topic").value,
            10,
        )

        self.get_logger().info("Inspection dashboard node ready.")

    def _on_inspection_state(self, msg: String) -> None:
        with self._lock:
            self._inspection_state = msg.data or ""

    def _on_runtime_diagnostics(self, msg: String) -> None:
        try:
            d = json.loads(msg.data) if msg.data else {}
            with self._lock:
                self._runtime_diagnostics = d
        except json.JSONDecodeError:
            pass

    def _on_mission_report(self, msg: String) -> None:
        with self._lock:
            self._mission_report_raw = msg.data or ""
        try:
            lines = (msg.data or "").strip().split("\n")
            d = {}
            for line in lines:
                if "=" in line:
                    k, v = line.split("=", 1)
                    d[k.strip().lower()] = v.strip()
            with self._lock:
                self._mission_report = d
        except Exception:
            with self._lock:
                self._mission_report = {"raw": msg.data or ""}

    def _on_current_goal(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        with self._lock:
            self._current_goal = {"x": p.x, "y": p.y, "z": p.z}

    def _on_camera(self, msg: Image) -> None:
        try:
            frame = _bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._frame_camera = frame
        except Exception:
            pass

    def get_state(self) -> dict:
        with self._lock:
            return {"inspection_state": self._inspection_state}

    def get_diagnostics(self) -> dict:
        with self._lock:
            return self._runtime_diagnostics or {}

    def get_mission_report(self) -> Optional[dict]:
        with self._lock:
            return self._mission_report

    def get_mission_report_raw(self) -> str:
        with self._lock:
            return self._mission_report_raw or ""

    def get_current_goal(self) -> Optional[dict]:
        with self._lock:
            return self._current_goal

    def get_camera_frame(self) -> np.ndarray:
        with self._lock:
            f = self._frame_camera
        return f.copy() if f is not None else _no_signal(640, 480, "CAMERA - NO SIGNAL")

    def publish_start_mission(self) -> None:
        msg = Bool()
        msg.data = True
        self._start_mission_pub.publish(msg)
        self.get_logger().info("Published start_mission=True")


# HTTP Server
class _Handler(BaseHTTPRequestHandler):
    node: InspectionDashboardNode = None  # type: ignore

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self._serve_html()
        elif self.path == "/api/state":
            self._json_response(self.node.get_state())
        elif self.path == "/api/diagnostics":
            self._json_response(self.node.get_diagnostics())
        elif self.path == "/api/mission_report":
            r = self.node.get_mission_report()
            self._json_response(r if r is not None else {})
        elif self.path.startswith("/stream/camera"):
            self._stream_mjpeg()
        elif self.path.startswith("/frame/camera"):
            self._serve_single_frame()
        else:
            self.send_error(404)

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length)) if length else {}
        except Exception:
            body = {}

        if self.path == "/api/start_mission":
            self.node.publish_start_mission()
            self._json_response({"ok": True, "msg": "start_mission published"})
        else:
            self.send_error(404)

    def _json_response(self, data: dict, code: int = 200):
        body = json.dumps(data, default=str).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _serve_html(self):
        body = _HTML.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream_mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, no-store")
        self.end_headers()
        q = self.node._jpeg_quality
        dt = 1.0 / 15.0
        while True:
            try:
                frame = self.node.get_camera_frame()
                _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, q])
                data = jpg.tobytes()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n\r\n" % len(data)).encode())
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
                time.sleep(dt)
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

    def _serve_single_frame(self):
        try:
            frame = self.node.get_camera_frame()
            _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.node._jpeg_quality])
            data = jpg.tobytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)
        except Exception as e:
            self.node.get_logger().error("[frame/camera] %s" % e)
            err = _no_signal(640, 480, "ERROR: %s" % str(e)[:40])
            _, jpg = cv2.imencode(".jpg", err)
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg.tobytes())))
            self.end_headers()
            self.wfile.write(jpg.tobytes())

    def log_message(self, fmt, *args):
        pass


class _ThreadedServer(ThreadingMixIn, TCPServer):
    allow_reuse_address = True
    daemon_threads = True


_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Inspection Dashboard</title>
<style>
:root{--bg:#0a0e14;--card:#141820;--border:#1e2530;--cyan:#00d4ff;--green:#06d6a0;--text:#e0e6f0;--text2:#8899aa;--text3:#4a5568;--font:system-ui,sans-serif;--mono:monospace;}
*,*::before,*::after{margin:0;padding:0;box-sizing:border-box}
html,body{height:100%;background:var(--bg);color:var(--text);font-family:var(--font);overflow:hidden}
.dashboard{display:grid;grid-template-rows:40px 1fr;height:100vh;gap:2px;padding:2px;}
.header{background:var(--card);border:1px solid var(--border);border-radius:6px;display:flex;align-items:center;justify-content:space-between;padding:0 14px;}
.logo{font-size:13px;font-weight:700;font-family:var(--mono);color:var(--cyan);}
.content{display:grid;grid-template-columns:1fr 1fr;gap:2px;min-height:0;overflow:auto;}
.panel{background:var(--card);border:1px solid var(--border);border-radius:6px;padding:12px;overflow:auto;min-height:0;}
.panel-hdr{font-size:10px;font-weight:700;font-family:var(--mono);color:var(--text2);margin-bottom:8px;}
.state-badge{font-size:24px;font-weight:700;font-family:var(--mono);color:var(--cyan);padding:8px 0;}
.tel-row{display:flex;align-items:baseline;gap:6px;margin:4px 0;}
.tel-label{font-size:10px;color:var(--text3);min-width:140px;}
.tel-value{font-size:12px;font-family:var(--mono);}
.cam-wrap{min-height:200px;background:#0d1117;border-radius:4px;overflow:hidden;}
.cam-wrap img{width:100%;height:200px;object-fit:contain;display:block;}
.btn{border:none;border-radius:4px;padding:6px 14px;font-size:11px;font-weight:700;cursor:pointer;margin-top:8px;}
.btn-start{background:#0a3d2a;color:var(--green);border:1px solid var(--green);}
.btn-start:hover{background:#0d5c3f;}
pre{font-size:10px;font-family:var(--mono);color:var(--text2);overflow:auto;max-height:200px;}
</style>
</head>
<body>
<div class="dashboard">
<header class="header"><div class="logo">TIRE INSPECTION DASHBOARD</div><div id="status">Live</div></header>
<div class="content">
<div class="panel">
<div class="panel-hdr">FSM STATE</div>
<div class="state-badge" id="state">--</div>
<div class="panel-hdr">DIAGNOSTICS</div>
<div id="diagnostics"></div>
<button class="btn btn-start" id="btn-start">START MISSION</button>
</div>
<div class="panel">
<div class="panel-hdr">MISSION REPORT</div>
<div id="mission-report"></div>
<div class="panel-hdr">CAMERA</div>
<div class="cam-wrap"><img src="/stream/camera" alt="Camera"></div>
</div>
</div>
</div>
<script>
async function api(path){const r=await fetch(path);return r.json();}
async function poll(){
try{
var s=await api('/api/state');
document.getElementById('state').textContent=s.inspection_state||'--';
var d=await api('/api/diagnostics');
var html='';
if(d.current_state) html+='<div class="tel-row"><span class="tel-label">State</span><span class="tel-value">'+d.current_state+'</span></div>';
if(d.robot_pose) html+='<div class="tel-row"><span class="tel-label">Robot pose</span><span class="tel-value">x='+(d.robot_pose.x?d.robot_pose.x.toFixed(3):'-')+' y='+(d.robot_pose.y?d.robot_pose.y.toFixed(3):'-')+'</span></div>';
if(d.distance_to_goal!=null) html+='<div class="tel-row"><span class="tel-label">Distance to goal</span><span class="tel-value">'+d.distance_to_goal.toFixed(2)+' m</span></div>';
if(d.tires_captured!=null) html+='<div class="tel-row"><span class="tel-label">Tires captured</span><span class="tel-value">'+d.tires_captured+'</span></div>';
if(d.vehicles_detected!=null) html+='<div class="tel-row"><span class="tel-label">Vehicles</span><span class="tel-value">'+d.vehicles_detected+'</span></div>';
if(d.nav_feedback) html+='<div class="tel-row"><span class="tel-label">distance_remaining</span><span class="tel-value">'+(d.nav_feedback.distance_remaining!=null?d.nav_feedback.distance_remaining.toFixed(2):'-')+' m</span></div>';
if(d.proximity_gate_passed!=null) html+='<div class="tel-row"><span class="tel-label">Proximity gate</span><span class="tel-value">'+(d.proximity_gate_passed?'PASSED':'--')+'</span></div>';
document.getElementById('diagnostics').innerHTML=html||'<span class="tel-value">No diagnostics yet</span>';
var mr=await api('/api/mission_report');
var mrEl=document.getElementById('mission-report');
if(mr&&Object.keys(mr).length>0){mrEl.innerHTML='<pre>'+JSON.stringify(mr,null,2)+'</pre>';}else{mrEl.innerHTML='<span class="tel-value">No report yet</span>';}
}catch(e){}
}
document.getElementById('btn-start').onclick=function(){fetch('/api/start_mission',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});};
setInterval(poll,300);
poll();
</script>
</body>
</html>"""


def main(args=None):
    rclpy.init(args=args)
    node = InspectionDashboardNode()
    port = int(node.get_parameter("port").value)
    _Handler.node = node
    server = _ThreadedServer(("0.0.0.0", port), _Handler)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    node.get_logger().info("Inspection Dashboard running at http://localhost:%d" % port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
