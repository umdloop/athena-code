#!/usr/bin/env python3
"""
WebRTC camera server for Jetson radio interface.
Serves a test UI at http://0.0.0.0:7001 and handles
WebSocket signaling for GStreamer WebRTC pipelines.

Usage:
    python3 webrtc_server.py

Env vars:
    GST_WEBRTC_PORT   (default 7001)
    GST_CAMERA_WIDTH  (default 640)
    GST_CAMERA_HEIGHT (default 480)
    GST_CAMERA_FPS    (default 30)
"""

import asyncio
import json
import logging
import os
import sys
import glob

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstWebRTC", "1.0")
gi.require_version("GstSdp", "1.0")
from gi.repository import Gst, GstWebRTC, GstSdp, GLib

import websockets.asyncio.server
from aiohttp import web

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("webrtc")

Gst.init(None)

PORT   = int(os.environ.get("GST_WEBRTC_PORT", 7001))
WIDTH  = int(os.environ.get("GST_CAMERA_WIDTH", 640))
HEIGHT = int(os.environ.get("GST_CAMERA_HEIGHT", 480))
FPS    = int(os.environ.get("GST_CAMERA_FPS", 30))

# ── HTML test page ────────────────────────────────────────────────────────────

HTML = """<!doctype html>
<html>
<head>
<meta charset=utf-8>
<title>Cameras</title>
<style>
  body { margin:0; background:#111; color:#fff; font-family:sans-serif }
  #grid { display:grid; grid-template-columns:repeat(5,1fr); gap:6px; padding:6px }
  .tile { position:relative; background:#222 }
  .tile video { width:100%; display:block; min-height:120px }
  .tile span { position:absolute; bottom:4px; left:4px; font-size:11px;
               background:rgba(0,0,0,.6); padding:2px 5px; border-radius:3px }
</style>
</head>
<body>
<div id=grid></div>
<script>
const PORT = location.port || 7001;
const WS   = `ws://${location.hostname}:${parseInt(PORT) + 1}`;

function connect(id) {
  const tile = document.createElement('div');
  tile.className = 'tile';
  const v   = document.createElement('video');
  v.autoplay = true; v.muted = true; v.playsInline = true;
  const lbl = document.createElement('span');
  lbl.textContent = 'cam ' + id;
  tile.append(v, lbl);
  document.getElementById('grid').appendChild(tile);

  const pc = new RTCPeerConnection({ iceServers: [] });
  pc.ontrack = e => { v.srcObject = e.streams[0]; };

  const ws = new WebSocket(WS);

  ws.onopen = () => ws.send(JSON.stringify({ type: 'request', cameraId: id }));

  ws.onmessage = async e => {
    const m = JSON.parse(e.data);
    if (m.type === 'offer') {
      await pc.setRemoteDescription({ type: 'offer', sdp: m.sdp });
      const ans = await pc.createAnswer();
      await pc.setLocalDescription(ans);
      ws.send(JSON.stringify({ type: 'answer', sdp: ans.sdp }));
    } else if (m.type === 'ice' && m.candidate) {
      pc.addIceCandidate({ candidate: m.candidate, sdpMLineIndex: m.sdpMLineIndex });
    } else if (m.type === 'error') {
      lbl.textContent = 'cam ' + id + ' — ' + m.message;
    }
  };

  pc.onicecandidate = e => {
    if (e.candidate) ws.send(JSON.stringify({
      type: 'ice',
      candidate: e.candidate.candidate,
      sdpMLineIndex: e.candidate.sdpMLineIndex
    }));
  };

  ws.onerror = err => console.error('cam', id, err);
  ws.onclose = () => { pc.close(); setTimeout(() => connect(id), 3000); };
}

// fetch live camera list then build grid
fetch(`http://${location.hostname}:7000/cameras`)
  .then(r => r.json())
  .then(d => d.cameras.forEach(c => connect(c.id)))
  .catch(() => {
    // fallback hardcoded IDs
    [2, 4, 7, 8, 10, 13, 14, 16, 20].forEach(connect);
  });
</script>
</body>
</html>
"""

# ── GStreamer WebRTC peer ─────────────────────────────────────────────────────
class Peer:
    def __init__(self, camera_id, send_fn, loop):
        self.camera_id = camera_id
        self._send = send_fn   # coroutine: send_fn(str)
        self._loop = loop
        self.pipe = None
        self.webrtc = None

    def start(self):
        hw = Gst.ElementFactory.find("nvv4l2h264enc") is not None
        enc = "nvv4l2h264enc bitrate=2000000" if hw else "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast"
        
        # Use gst-launch with period separator - webrtcbin in separate bin
        desc = (
            f"webrtcbin name=webrtc bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302 . "
            f"v4l2src device=/dev/video{self.camera_id} io-mode=2 do-timestamp=true ! "
            f"image/jpeg,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! "
            f"jpegdec ! videoconvert ! {enc} ! "
            f"rtph264pay config-interval=-1 pt=96 ! "
            f"webrtc."
        )
        
        log.info(f"[cam {self.camera_id}] {desc}")
        self.pipe = Gst.parse_launch(desc)
        self.webrtc = self.pipe.get_by_name("webrtc")
        
        # connect signals
        self.webrtc.connect("on-negotiation-needed", self._on_negotiation_needed)
        self.webrtc.connect("on-ice-candidate", self._on_ice_candidate)
        
        self.pipe.set_state(Gst.State.PLAYING)
        log.info(f"[cam {self.camera_id}] pipeline PLAYING")

    def stop(self):
        if self.pipe:
            self.pipe.set_state(Gst.State.NULL)

    def _send_nowait(self, msg):
        asyncio.run_coroutine_threadsafe(self._send(json.dumps(msg)), self._loop)

    def _on_negotiation_needed(self, _webrtc):
        log.info(f"[cam {self.camera_id}] negotiation needed")
        promise = Gst.Promise.new_with_change_func(self._on_offer_created)
        self.webrtc.emit("create-offer", None, promise)

    def _on_offer_created(self, promise):
        log.info(f"[cam {self.camera_id}] offer created")
        promise.wait()
        reply = promise.get_reply()
        offer = reply["offer"]
        self.webrtc.emit("set-local-description", offer, Gst.Promise.new())
        sdp_str = offer.sdp.as_text()
        log.info(f"[cam {self.camera_id}] sending offer")
        self._send_nowait({"type": "offer", "sdp": sdp_str})

    def _on_ice_candidate(self, _webrtc, mline, candidate):
        log.info(f"[cam {self.camera_id}] ICE candidate mline={mline}")
        self._send_nowait({"type": "ice", "sdpMLineIndex": mline, "candidate": candidate})

    def set_answer(self, sdp_str):
        _, sdp = GstSdp.SDPMessage.new_from_text(sdp_str)
        answer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdp)
        self.webrtc.emit("set-remote-description", answer, Gst.Promise.new())

    def add_ice(self, mline, candidate):
        self.webrtc.emit("add-ice-candidate", mline, candidate)


# ── WebSocket handler ─────────────────────────────────────────────────────────

async def handle(websocket):
    loop = asyncio.get_event_loop()
    peer = None
    log.info(f"[ws] connected from {websocket.remote_address}")

    try:
        async for raw in websocket:
            msg = json.loads(raw)
            t = msg.get("type")

            if t == "request":
                cam_id = int(msg.get("cameraId", -1))
                dev = f"/dev/video{cam_id}"
                if cam_id < 0 or not os.path.exists(dev):
                    await websocket.send(json.dumps({"type": "error", "message": "camera not found"}))
                    continue
                if peer:
                    peer.stop()
                peer = Peer(cam_id, websocket.send, loop)
                try:
                    peer.start()
                except Exception as e:
                    log.error(f"[cam {cam_id}] {e}")
                    await websocket.send(json.dumps({"type": "error", "message": str(e)}))
                    peer = None

            elif t == "answer" and peer:
                log.info(f"[cam {peer.camera_id}] received answer")
                peer.set_answer(msg["sdp"])

            elif t == "ice" and peer:
                log.info(f"[cam {peer.camera_id}] received ICE candidate")
                peer.add_ice(msg["sdpMLineIndex"], msg["candidate"])

    except Exception as e:
        log.error(f"[ws] {e}")
    finally:
        if peer:
            peer.stop()
        log.info("[ws] disconnected")


# ── HTTP handler for the test page ───────────────────────────────────────────

async def http_index(request):
    return web.Response(text=HTML, content_type="text/html")


# ── main ──────────────────────────────────────────────────────────────────────

async def main():
    # Run GLib main loop in a background thread for GStreamer callbacks
    glib_loop = GLib.MainLoop()
    import threading
    threading.Thread(target=glib_loop.run, daemon=True).start()

    log.info(f"webrtc-camera-server on http://0.0.0.0:{PORT}")
    log.info(f"  open http://localhost:{PORT} in browser")

    # HTTP server for the test page
    app = web.Application()
    app.router.add_get("/", http_index)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", PORT)
    await site.start()

    # WebSocket server on same port (aiohttp handles both)
    async with websockets.asyncio.server.serve(handle, "0.0.0.0", PORT + 1):
        log.info(f"  WebSocket on ws://localhost:{PORT + 1}")
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())
