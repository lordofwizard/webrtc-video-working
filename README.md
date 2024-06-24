# Setup a WebRTC 3 Camera Service

### Requirements
1. Robot Running linux and ensure pip is installed
2. Install the following packages

```python
python3 -m pip install aiohttp aiortc opencv-python
```

3. READ [THIS](https://tailscale.com/kb/1247/funnel-examples?q=funnel+examples#expose-a-development-server-to-the-public)
4. Run the `reciever.py` in the background.
5. Start the `start.py` in ROS Accecable network to use it. 
6. USE the tailscale url. to get the video feed on a video server.

_Note_ Make sure you are using a setup of 3 Webcams or find a appropriate commit to  
get the proper number of camera's working. 
