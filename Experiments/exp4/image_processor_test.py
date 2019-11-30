import time
from image_processor import ImageProcessor

stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
# stream = 0
img_proc = ImageProcessor(stream, True)
while 1:
    data = img_proc.analyze_objects()
    if 'track' in data:
        del data['track']
    print(data, '\n')
    time.sleep(0.1)
