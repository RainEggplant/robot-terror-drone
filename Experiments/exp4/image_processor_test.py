import time
from image_processor import ImageProcessor

stream = 'http://192.168.137.76:8080/video'
# stream = 0
img_proc = ImageProcessor(stream, True)
while 1:
    data = img_proc.analyse_objects()
    print(data)
    print()
    time.sleep(0.5)
