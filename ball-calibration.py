"""
A utility for calibrating OpenCV-based ball detection
on 5818's 2020 robot, CheeseWrangler.

Connect to camera directly with utility and then transfer
settings to robot code. Variables are: RGB upper and lower
bounds, erode iterations, dilate iterations, contour aspect
ratio minimum and maximum, minimum fill percentage, and
minimum and maximum radius for minimum enclosing circles.
"""

import cv2
import numpy as np
import platform

# Name of options window, stored as field for convenience
opts_window_name = 'Options'


def main():
    """Captures, processes, and displays video feed"""
    # Live camera feed from DirectShow (Windows) or /dev/video0 (Linux)
    # Should be changed if needed (may use builtin camera by default)
    if platform.system() == 'Windows':
        cap = cv2.VideoCapture(cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture('/dev/video2')
    # 640x480@30fps by default, works for calibration
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    cv2.namedWindow(opts_window_name, cv2.WINDOW_NORMAL)
    create_trackbars()
    while True:
        ret, img = cap.read()
        # Crop out top half of picture, no balls in FoV usually
        img = img[int(img.shape[0] * 0.6):, :img.shape[1]]
        # Blur image to make processing easier
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        # Mask colors between lower and upper color ranges, then erode and dilate
        mask = cv2.inRange(blurred, get_trackbar_color(False), get_trackbar_color(True))
        mask = cv2.erode(mask, None, iterations=get_trackbar_data('Erode'))
        mask = cv2.dilate(mask, None, iterations=get_trackbar_data('Dilate'))
        # Find all contours on image (loose)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            # Create bounding rectangles for each contour and filter for
            # actual aspect ratio and fill percentage (min/max)
            _, _, w, h = cv2.boundingRect(c)
            aspect = float(w) / h
            area = cv2.contourArea(c)
            filled = float(area) / (w * h)
            asp_min = get_trackbar_data('Ratio Min')
            asp_max = get_trackbar_data('Ratio Max')
            fill = get_trackbar_data('Fill %')
            if area > 0 and asp_max > aspect > asp_min and filled > fill / 100:
                # Make enclosing circles on bounded contours and filter min/max radius
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                rmi = get_trackbar_data('Radius Min')
                rma = get_trackbar_data('Radius Max')
                if rma > radius > rmi:
                    # Find center of circle from moments, write to image
                    m = cv2.moments(c)
                    center = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
                    cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(img, center, 5, (0, 0, 255), -1)
        # Convert mask grayscale image to color, concat to shown output
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        gui_img = np.concatenate((mask, img), axis=0)
        cv2.imshow('Mask/Output', gui_img)
        # Exit if 'q' key pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def create_trackbars():
    """Creates all options trackbars"""
    create_single_trackbar('Red Min', 0, 255)
    create_single_trackbar('Green Min', 20, 255)
    create_single_trackbar('Blue Min', 20, 255)
    create_single_trackbar('Red Max', 35, 255)
    create_single_trackbar('Green Max', 220, 255)
    create_single_trackbar('Blue Max', 240, 255)
    
    create_single_trackbar('Erode', 20, 40)
    create_single_trackbar('Dilate', 20, 40)
    create_single_trackbar('Ratio Min', 0, 10)
    create_single_trackbar('Ratio Max', 2, 10)
    create_single_trackbar('Fill %', 50, 100)
    create_single_trackbar('Radius Min', 30, 200)
    create_single_trackbar('Radius Max', 130, 200)


def create_single_trackbar(name, val, upper):
    """Call to OpenCV HighGUI trackbar create"""
    cv2.createTrackbar(name, opts_window_name, val, upper, nothing)


def get_trackbar_data(name):
    """Get data from a specific named OpenCV HighGUI trackbar"""
    return cv2.getTrackbarPos(name, opts_window_name)


def get_trackbar_color(is_max):
    """Get color trackbar data for low/high bounds"""
    mod = 'Min'
    if is_max:
        mod = 'Max'
    r = get_trackbar_data('Red ' + mod)
    g = get_trackbar_data('Green ' + mod)
    b = get_trackbar_data('Blue ' + mod)
    return r, g, b


def nothing(x):
    """Blank callback for trackbar creates"""
    pass


if __name__ == "__main__":
    main()
