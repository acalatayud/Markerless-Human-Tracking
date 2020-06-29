import csv
import os

import json
import numpy as np
import cv2
import re

from src.point_helpers import get_cameras


def read_ground_truth(json_file):
    with open(json_file, 'r') as file:
        triangulation = json.load(file)

    frames = []
    for frame in triangulation['reconstruction']['frames']:
        points = np.zeros((8, 3))
        for camera in frame['frameData']:
            for idx, point in enumerate(camera['points']):
                points[idx] += [float(point['x']), float(point['y']), float(point['z'])]
        points = points / len(frame['frameData'])
        frames.append(points)
    return frames

def read_images(dir, cameras):
    camera_corners = []
    for idx, directory in enumerate([f.path for f in os.scandir(dir) if f.is_dir()]):
        files = os.listdir(directory)
        files.sort(key=lambda f: int(re.sub('\D', '', f)))
        charuco_corners = {}
        for filename in files:
            if filename.endswith(".png"):
                detected = detect_charuco(directory + '/' + filename, cameras[idx])
                charuco_corners[int(filename.split(".")[0])] = detected
        camera_corners.append(charuco_corners)
    return camera_corners


def detect_charuco(filename, camera):
    image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard_create(3, 5, 0.27, 0.16, dictionary)
    corners, ids, rejected = cv2.aruco.detectMarkers(image, dictionary, cameraMatrix=camera.camera_matrix, distCoeff=camera.distortion_coefficients)
    corners, ids, rejected, _ = cv2.aruco.refineDetectedMarkers(image, board, corners, ids, rejected, camera.camera_matrix, camera.distortion_coefficients)
    if ids is not None and len(ids) > 0:
        detected = {}
        interpolated = cv2.aruco.interpolateCornersCharuco(corners, ids, image, board, cameraMatrix=camera.camera_matrix, distCoeffs=camera.distortion_coefficients)
        if interpolated[0] < 1:
            return None
        corner_ids = interpolated[2].ravel()
        positions = interpolated[1].reshape((interpolated[1].shape[0], interpolated[1].shape[2]))
        for i in range(len(corner_ids)):
            detected[corner_ids[i]] = positions[i]
        return detected
    return None

def write_marker_position_csv(csv_file, corners):

    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerows([[]] * 3)

        for frame, data in corners.items():
            frame_number = int(frame)
            markers = []

            for i in range(8):
                if data is None or i not in data:
                    x = 0
                    y = 0
                    likelihood = 0
                else:
                    x = data[i][0]
                    y = data[i][1]
                    likelihood = 1
                markers.append([x, y, likelihood])

            writer.writerow([frame_number] + [item for sublist in markers for item in sublist])

    return

cameras = get_cameras("E:/Downloads/PF/Captures/intrinsics.json", "E:/Downloads/PF/Captures/extrinsics.json")
read_ground_truth("E:/Downloads/PF/Captures/reconstruction-scene-0.json")
camera_corners = read_images("E:/Downloads/PF/Captures/scene-0/calibration", cameras)
for index, corners in enumerate(camera_corners):
    write_marker_position_csv('charuco-corners-cam-' + str(index) + '.csv', corners)
