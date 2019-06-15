"""
Demonstrating basic usage of Kinect2 cameras.
"""
from libkinect2 import Kinect2
import numpy as np
import cv2

# Init kinect w/color->camera mapping
kinect = Kinect2(use_sensors=['color', 'depth'], use_mappings=[('color', 'camera')])
kinect.connect()
kinect.wait_for_worker()

# 3D Plot (super slow, but works for demo)
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for _, color_img, depth_map, color_cam_map in kinect.iter_frames():
    
    # Display color image as reference
    cv2.imshow('sensors', color_img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

    h, w, _ = color_img.shape

    X = []
    Y = []
    Z = []
    C = []

    # Read (every 10th) point pos and color
    for y_pixel in range(0, h, 10):
        for x_pixel in range(0, w, 10):
            x, z, y = color_cam_map[y_pixel, x_pixel]
            b, g, r = color_img[y_pixel, x_pixel] / 255.0
            X.append(x)
            Y.append(y)
            Z.append(z)
            C.append([r, g, b])

    # Plot
    ax.clear()
    ax.set_xlim((-2,2))
    ax.set_ylim((-2,2))
    ax.set_zlim((-1,2))
    ax.scatter(X, Y, Z, s=1, c=C)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    fig.canvas.draw()
    plt.pause(0.001)

kinect.disconnect()