#!/usr/bin/env python3

"""
planar_homography.py
Author: Hayden Feddock
Date: 1/15/2025

This script listens for images from multiple cameras and computes the planar homography.
"""

import os
import cv2
import numpy as np
import argparse

def select_points(image1, image2, n):
    points1 = []
    points2 = []
    combined_image = np.hstack((image1, image2))

    # Resize the combined image to fit on screen
    screen_width = 1920
    screen_height = 1080
    scale_factor = min(screen_width / combined_image.shape[1], screen_height / combined_image.shape[0])
    resized_combined_image = cv2.resize(combined_image, (0, 0), fx=scale_factor, fy=scale_factor)

    def click_event(event, x, y, flags=None, params=None):
        if event == cv2.EVENT_LBUTTONDOWN:
            if x < resized_combined_image.shape[1] // 2:
                points1.append((int(x / scale_factor), int(y / scale_factor)))
                cv2.circle(resized_combined_image, (x, y), 2, (0, 255, 0), -1)
            else:
                points2.append((int((x - resized_combined_image.shape[1] // 2) / scale_factor), int(y / scale_factor)))
                cv2.circle(resized_combined_image, (x, y), 2, (0, 0, 255), -1)
            if len(points1) == len(points2) and len(points1) > 0:
                cv2.line(resized_combined_image, (int(points1[-1][0] * scale_factor), int(points1[-1][1] * scale_factor)), 
                         (int((points2[-1][0] + image1.shape[1]) * scale_factor), int(points2[-1][1] * scale_factor)), (255, 0, 0), 2)
                print(f"Points selected: {len(points1)} / {n}")
            cv2.imshow("Combined Image", resized_combined_image)

    cv2.imshow("Combined Image", resized_combined_image)
    cv2.setMouseCallback("Combined Image", click_event)
    cv2.waitKey(0)
    return points1, points2

if __name__ == "__main__":

    image1_path = "/home/hayden/data/director_sync/1736894279_398142322_zed_left.jpg"
    image2_path = "/home/hayden/data/director_sync/1736894279_398142322_zed_right.jpg"
    n_points = 8

    image1 = cv2.imread(image1_path)
    image2 = cv2.imread(image2_path)

    print(f"Select {n_points} points.")
    # points1, points2 = select_points(image1, image2, n_points)
    points1 = [(980, 538), (780, 328), (1534, 208), (854, 184), (430, 484), (566, 978), (492, 956), (484, 846), (1696, 624), (1428, 766), (1566, 906), (1838, 932), (470, 248), (1574, 226), (1540, 522), (1358, 564)]
    points2 = [(630, 542), (582, 328), (1486, 210), (656, 184), (144, 484), (362, 976), (274, 956), (252, 844), (1416, 624), (1174, 768), (1468, 906), (1748, 932), (298, 248), (1476, 224), (1490, 516), (1118, 556)]

    print(f"Points 1: {points1}")
    print(f"Points 2: {points2}")

    points1 = np.array(points1, dtype=np.float32)
    points2 = np.array(points2, dtype=np.float32)

    homography_matrix, _ = cv2.findHomography(points1, points2, cv2.RANSAC)
    # F, mask = cv2.findFundamentalMat(points1, points2, cv2.FM_RANSAC)
    print(f"Homography Matrix:\n{homography_matrix}")

    # retval, h1, h2 = cv2.stereoRectifyUncalibrated(points1, points2, F, image1.shape[:2])

    height, width, _ = image2.shape

    # rectified1 = cv2.warpPerspective(image1, h1, (width, height))
    # rectified2 = cv2.warpPerspective(image2, h2, (width, height))

    # combined_image = np.hstack((rectified1, rectified2))


    
    overlay_image = cv2.warpPerspective(image1, homography_matrix, (width, height))
    combined_image = cv2.addWeighted(overlay_image, 0.5, image2, 0.5, 0)

    cv2.imshow("Overlay Image", combined_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
