# camera_web.py
from flask import Flask, Response
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
import threading, io
from PIL import Image as PILImage

app = Flask(__name__)
latest_frame = [None]

def cb(msg):
    img = PILImage.frombytes('RGB', (msg.width, msg.height), msg.data)
    buf = io.BytesIO()
    img.save(buf, format='JPEG', quality=70)
    latest_frame[0] = buf.getvalue()

node = Node()
node.subscribe(Image, '/web_camera/image', cb)

@app.route('/stream')
def stream():
    def gen():
        while True:
            if latest_frame[0]:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + latest_frame[0] + b'\r\n')
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

app.run(host='0.0.0.0', port=5000)