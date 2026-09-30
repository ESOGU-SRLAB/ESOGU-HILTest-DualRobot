import eventlet
eventlet.monkey_patch()
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
import time

def cb(msg):
    print("Received frame!", msg.width, msg.height)

node = Node()
node.subscribe(Image, '/web_camera/image', cb)
print("Subscribed!")
while True:
    time.sleep(1)
