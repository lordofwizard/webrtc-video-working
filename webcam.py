import asyncio
import json
import logging
import os
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.media import MediaPlayer

ROOT = os.path.dirname(__file__)

class SingletonMediaPlayer:
    _instances = {}

    def __new__(cls, path, format=None, options=None):
        if options is None:
            options = {}
        key = (path, format, tuple(sorted(options.items())))  # Using a tuple for options
        if key not in cls._instances:
            cls._instances[key] = super(SingletonMediaPlayer, cls).__new__(cls)
            cls._instances[key]._initialized = False
        return cls._instances[key]

    def __init__(self, path, format=None, options={}):
        if self._initialized:
            return
        self.player = MediaPlayer(path, format=format, options=options)
        self._initialized = True

    @property
    def video(self):
        return self.player.video


webcam1 = None
webcam2 = None
pcs = set()

class VideoRelayTrack(MediaStreamTrack):
    def __init__(self, track):
        super().__init__()  # Initialize the base class
        self.track = track
        self.kind = track.kind  # Set the correct kind

    async def recv(self):
        frame = await self.track.recv()
        return frame

def create_local_tracks():
    global webcam1, webcam2

    if not webcam1:
        options = {"framerate": "30", "video_size": "1280x720"}
        webcam1 = SingletonMediaPlayer("/dev/video0", format="v4l2", options=options)
    
    if not webcam2:
        options = {"framerate": "30", "video_size": "1280x720"}
        webcam2 = SingletonMediaPlayer("/dev/video4", format="v4l2", options=options)

    return [
        VideoRelayTrack(webcam1.video),
        VideoRelayTrack(webcam2.video)
    ]

async def index(request):
    content = open(os.path.join(ROOT, "index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)

async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print("Connection state is %s" % pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # Relay media sources
    tracks = create_local_tracks()

    for track in tracks:
        pc.addTrack(track)

    await pc.setRemoteDescription(offer)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

async def on_shutdown(app):
    # Close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_post("/offer", offer)
    web.run_app(app, host="0.0.0.0", port=6969)
