import asyncio
import json
import logging
import os
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer, MediaRelay

ROOT = os.path.dirname(__file__)

relay1 = None
relay2 = None
webcam1 = None
webcam2 = None

def create_local_tracks():
    global relay1, relay2, webcam1, webcam2

    options = {"framerate": "30", "video_size": "1280x720"}
    if relay1 is None:
        webcam1 = MediaPlayer("/dev/video0", format="v4l2", options=options)
        relay1 = MediaRelay()
    if relay2 is None:
        webcam2 = MediaPlayer("/dev/video4", format="v4l2", options=options)
        relay2 = MediaRelay()
    return relay1.subscribe(webcam1.video), relay2.subscribe(webcam2.video)

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

    # Open media source
    video1, video2 = create_local_tracks()

    if video1:
        pc.addTrack(video1)
    if video2:
        pc.addTrack(video2)

    await pc.setRemoteDescription(offer)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

pcs = set()

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

