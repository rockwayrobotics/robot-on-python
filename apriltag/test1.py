#!/usr/bin/env python3

import time

import robotpy_apriltag as at

from libcamera import Transform
import picamera2

cam = picamera2.Picamera2()

SIZE = (320, 240)
still1 = cam.create_still_configuration(
    main=dict(size=SIZE, format='YUV420'),
    controls=dict(FrameRate=200), # FrameDurationLimits=(1, 5000)),
    queue=False,
    transform=Transform(hflip=1, vflip=1),
    )

cam.configure(still1)
cam.start()

det = at.AprilTagDetector()
det.addFamily('tag16h5', bitsCorrected=0)
cfg = det.getConfig()
cfg.quadDecimate = 2
det.setConfig(cfg)

now = start = time.time()
reported = start
count = 0
fps = 0
found = False
while now - start < 30:
    img = cam.capture_array()[:SIZE[1],:] # not sure value correct for other sizes
    tags = det.detect(img)
    count += 1
    now = time.time()
    if now - reported > 3:
        fps = count / (now - reported)
        reported = now
        count = 0

    if found != bool(tags):
        found = not found
        if not found:
            print(f'{fps:3.0f} FPS: missing' + ' ' * 40, end='\r')

    if found:
        #breakpoint()
        x = sorted(tags, key=lambda x: -x.getDecisionMargin())[0]
        c = x.getCenter()
        tid = x.getId()
        margin = x.getDecisionMargin()
        print(f'{fps:3.0f} FPS: margin={margin:2.0f} @{c.x:3.0f},{c.y:3.0f} id={tid:2}  ' % tags, end='\r')

cam.stop()

print()

