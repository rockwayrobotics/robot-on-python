#!/usr/bin/env python3

import math
import time

import robotpy_apriltag as at

from libcamera import Transform
import picamera2

degrees = lambda rad: rad * 180 / math.pi

cam = picamera2.Picamera2()

SIZE = (320, 240)
#SIZE = (240, 128)
still1 = cam.create_still_configuration(
    main=dict(size=SIZE, format='YUV420'),
    controls=dict(FrameRate=200), # FrameDurationLimits=(1, 5000)),
    queue=False,
    transform=Transform(hflip=1, vflip=1),
    )

cam.align_configuration(still1)
print('Cam config:\n%s' % '\n'.join(f'{x:>15} = {y}' for x, y in still1.items()))
cam.configure(still1)
cam.start()

det = at.AprilTagDetector()
det.addFamily('tag16h5', bitsCorrected=0)
cfg = det.getConfig()
cfg.quadDecimate = 2
cfg.numThreads = 2
print(f'Apriltags:\n\t{cfg.decodeSharpening=} {cfg.numThreads=} {cfg.quadDecimate=}\n\t{cfg.quadSigma=} {cfg.refineEdges=} {cfg.debug=}')
q = det.getQuadThresholdParameters()
print(f'Quad Threshold:\n\t{degrees(q.criticalAngle)=} {q.deglitch=} {q.maxLineFitMSE=} {q.maxNumMaxima=} {q.minClusterPixels=} {q.minWhiteBlackDiff=}')

det.setConfig(cfg)

now = start = time.time()
reported = start
count = 0
fps = 0
found = False
height = SIZE[0] * 2 // 3
while now - start < 30:
    arr = cam.capture_array()
    img = arr[:height,:]
    tags = det.detect(img)
    count += 1
    now = time.time()
    if now - reported > 1:
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

