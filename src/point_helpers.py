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
        self.R = extrinsic_properties[camera_number]['rotationVector']
        self.T = extrinsic_properties[camera_number]['translationVector']
        self.camera_matrix
        self.distortion_coefficients = intrinsic_properties[camera_number]
        self.reprojectionError = intrinsic_properties[camera_number]





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
    stereo_cameras = []
    stereo_cameras.append(cameras[0])
    stereo_cameras.append(cameras[1])
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
    image_size = [640, 480]
    for i in range(number_of_frames - 1):
        for j in range(number_of_markers - 1):
            stereo_cameras = get_two_best_cameras_for_marker(cameras, i, j)
            cv2.stereoCalibrate(stereo_cameras[0].camera_matrix, stereo_cameras[1].camera_matrix,
                              stereo_cameras[0].distortion_coefficients, stereo_cameras[1].distortion_coefficients,
                              image_size, R, T, E, F)
            cv2.stereoRectify()
            cv2.undistortPoints()
            cv2.undistortPoints()
            cv2.triangulatePoints()



def add_camera(cameras, camera_number, csv_files,intrinsic_file, extrinsic_file):
    return cameras.append(Camera(camera_number, read_marker_position_csv(csv_files), read_cameras_intrinsic_properties(intrinsic_file), read_cameras_extrinsic_properties(extrinsic_file)))

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


def read_cameras_intrinsic_properties(json_file):
    with open(json_file, 'r') as file:
        intrinsic_properties = json.load(file)

    return intrinsic_properties['calibration']['cameras']


def read_cameras_extrinsic_properties(json_file):
    with open(json_file, 'r') as file:
        extrinsic_properties = json.load(file)

    return extrinsic_properties['calibration']['cameras']

def get_3d_vector(vector):
    return [float(vector['x']), float(vector['y']), float(vector['z'])]