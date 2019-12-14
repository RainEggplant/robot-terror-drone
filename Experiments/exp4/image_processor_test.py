import time
from image_processor import ImageProcessor

stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
# stream = "http://192.168.137.240:8080/video"
# stream = 0
img_proc = ImageProcessor(stream, True)
while 1:
    data = img_proc.analyze_objects()
    if 'track' in data:
        del data['track']
    if 'track_next' in data:
        del data['track_next']
    if 'ditch' in data:
        data['ditch'] = len(data['ditch']) > 0
    print(data, '\n')
    time.sleep(1)
