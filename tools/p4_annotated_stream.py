#!/usr/bin/env python3
"""Serve a REAL-TIME annotated wrist stream with detection markers (2026-11-22).

Pulls the console MJPEG, overlays the same red/green/yellow markers and grid as
the detection viewer, and serves it as an MJPEG HTTP stream on port 8099 so an
mpv window can show live annotated frames (not a recording).

Markers:
  RED cross     = nominal e5 centre (14,14) board mm - the assumption
  GREEN cross   = detected piece centroid (bright blob) - the measurement
  YELLOW diamond= estimated piece base contact
  ORANGE dots   = PnP square lattice
"""
from __future__ import annotations
import sys,time,threading,urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2,numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from p4_detect_overlay import detect

SOURCE='http://192.168.1.171:8080/stream'
PORT=8099
BOUNDARY=b'--jpgboundary'

latest=b''
lock=threading.Lock()

def puller():
    global latest
    buf=b''
    while True:
        try:
            resp=urllib.request.urlopen(SOURCE,timeout=10)
            while True:
                c=resp.read(8192)
                if not c: break
                buf+=c
                while True:
                    s=buf.find(b'\xff\xd8')
                    if s<0: break
                    e=buf.find(b'\xff\xd9',s+2)
                    if e<0: break
                    jpg=buf[s:e+2]
                    img=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
                    if img is not None:
                        try:
                            ann,_=detect(img)
                            ok,enc=cv2.imencode('.jpg',ann,[cv2.IMWRITE_JPEG_QUALITY,80])
                            if ok:
                                with lock: latest=enc.tobytes()
                        except Exception: pass
                    buf=buf[e+2:]
        except Exception as ex:
            print('pull err',ex,flush=True)
            time.sleep(0.5)

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type','multipart/x-mixed-replace; boundary=jpgboundary')
        self.end_headers()
        try:
            while True:
                with lock: frame=latest
                if frame:
                    self.wfile.write(BOUNDARY+b'\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                    self.wfile.write(frame+b'\r\n')
                time.sleep(0.1)
        except Exception:
            pass
    def log_message(self,*a): pass

if __name__=='__main__':
    threading.Thread(target=puller,daemon=True).start()
    srv=ThreadingHTTPServer(('0.0.0.0',PORT),Handler)
    print(f'annotated stream on http://0.0.0.0:{PORT}/',flush=True)
    srv.serve_forever()
