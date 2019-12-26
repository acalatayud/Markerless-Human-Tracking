from src import constants
from src.constants import MARKER_INDICES as INDICES
import csv
import json
import cv2
import numpy as np


class Camera:
    def __init__(self, camera_number, frames, intrinsic_properties, extrinsic_properties):
        self.number = camera_number
        self.frames = frames
        r = extrinsic_properties[camera_number]['rotationVector']
        self.R = cv2.Rodrigues(get_3d_vector(r))[0]
        translation = extrinsic_properties[camera_number]['translationVector']
        self.T = np.array(get_3d_vector(translation))
        matrix_parameters = intrinsic_properties[camera_number]['calibrationMatrix']
        self.camera_matrix = np.matrix([[matrix_parameters['fx'], 0, matrix_parameters['cx']],
                                       [0, matrix_parameters['fy'], matrix_parameters['fy']],
                                       [0, 0, 1]], dtype=float)
        coef = intrinsic_properties[camera_number]['distortionCoefficients']
        self.distortion_coefficients = np.float64([coef['k1'], coef['k2'], coef['p1'], coef['p2'], coef['k3']])
        self.reprojectionError = float(intrinsic_properties[camera_number]['reprojectionError'])


class Frame:
    def __init__(self, markers, frame_number):
        self.markers = markers
        self.frame_number = frame_number


class Marker:
    def __init__(self, x, y, likelihood, marker_key):
        self.x = x
        self.y = y
        self.likelihood = likelihood
        self.marker_key = marker_key


def get_two_best_cameras_for_marker(cameras, frame, marker):
    stereo_cameras = [cameras[0], cameras[1]]
    if len(cameras) > 2:
        for i in range(2, len(cameras - 1)):
            if stereo_cameras[0].frames[frame].markers[marker].likelihood > cameras[i].frames[frame].markers[marker].likelihood:
                if stereo_cameras[0].frames[frame].markers[marker].likelihood > stereo_cameras[1].frames[frame].markers[marker].likelihood:
                    stereo_cameras[0] = cameras[i]
                stereo_cameras[1] = stereo_cameras[0]
            elif stereo_cameras[1].frames[frame].markers[marker].likelihood > cameras[i].frames[frame].markers[marker].likelihood:
                stereo_cameras[1] = cameras[i]
    return stereo_cameras


def triangulate_points(cameras):
    if len(cameras) < 2:
        raise Exception('Triangulation process needs at least two cameras')
    number_of_frames = len(cameras[0].frames)
    number_of_markers = len(cameras[0].frames[0].markers)
    image_size = (640, 480)
    for i in range(number_of_frames - 1):
        for j in range(number_of_markers - 1):
            stereo_cameras = get_two_best_cameras_for_marker(cameras, i, j)
            relative_t = stereo_cameras[1].T - stereo_cameras[0].T
            rotation = stereo_cameras[1].R * np.linalg.inv(stereo_cameras[0].R * relative_t[2])
            r1, r2, p1, p2, q, roi1, roi2 = cv2.stereoRectify(stereo_cameras[0].camera_matrix,
                                                              stereo_cameras[0].distortion_coefficients,
                                                              stereo_cameras[1].camera_matrix,
                                                              stereo_cameras[1].distortion_coefficients, image_size,
                                                              rotation, relative_t)
            undistorted_points = [None, None]
            for index, camera in enumerate(stereo_cameras):
                marker = camera.frames[i].markers[j]
                point = np.float64([marker.x, marker.y])
                undistorted_point = cv2.undistortPoints(point, camera.camera_matrix,
                                                        camera.distortion_coefficients)
                undistorted_points[index] = undistorted_point

            triangulated = cv2.triangulatePoints(p1, p2, undistorted_points[0], undistorted_points[1])
            print(triangulated)


def add_camera(cameras, camera_number, csv_file, intrinsic_file, extrinsic_file):
    return cameras.append(Camera(camera_number, read_marker_position_csv(csv_file),
                                 read_camera_properties(intrinsic_file),
                                 read_camera_properties(extrinsic_file)))


def read_marker_position_csv(csv_file):

    frames = []

    with open(csv_file, 'r') as file:
        lines = csv.reader(file, delimiter=',')
        frame_data = list(lines)[3:]

        for line in frame_data:
            frame_number = int(line[constants.FRAME_NUMBER_INDEX])
            markers = []
            for marker_key, index in INDICES.items():
                x = float(line[(index - 1) * 3 + 1])
                y = float(line[(index - 1) * 3 + 2])
                likelihood = float(line[(index - 1) * 3 + 3])
                key = marker_key
                markers.append(Marker(x, y, likelihood, key))

            frames.append(Frame(markers, frame_number))

    return frames


def read_camera_properties(json_file):
    with open(json_file, 'r') as file:
        properties = json.load(file)

    return properties['calibration']['cameras']


def get_3d_vector(vector):
    return np.float64([float(vector['x']), float(vector['y']), float(vector['z'])])
