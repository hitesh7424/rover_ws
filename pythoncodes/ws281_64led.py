from pi5neo import Pi5Neo
import time

neo = Pi5Neo("/dev/spidev0.0", 64, 800)

while True:
    neo.fill_strip(255, 0, 0)
    neo.update_strip()
    time.sleep(1)

    neo.fill_strip(0, 255, 0)
    neo.update_strip()
    time.sleep(1)

    neo.fill_strip(0, 0, 255)
    neo.update_strip()
    time.sleep(1)