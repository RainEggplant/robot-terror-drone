import time
from image_processor import ImageProcessor

stream = 'http://192.168.137.55:8080/video'
# stream = 0
img_proc = ImageProcessor(stream, True)
while 1:
    data = img_proc.analyze_objects()
    if 'track' in data:
        del data['track']
    print(data, '\n')
    time.sleep(0.5)
