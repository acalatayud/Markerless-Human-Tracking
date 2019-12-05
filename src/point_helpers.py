from src import constants
from src.constants import MARKER_INDICES as INDICES
import csv
import json
import cv2

class Camera:
    def __init__(self, name, frames, intrinsic_properties, extrinsic_properties):
        self.name = name
        self.frames = frames
        self.intrinsic_properties
        self.extrinsic_properties

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

def triangulate_points(cameras)
    if (len(cameras) < 2):
        print("Triangulation process needs at least two cameras")
    number_of_frames = len(cameras[0].frames)
    for in range(number_of_frames - 1):
        stereo_cameras = get_two_best_cameras_for_frame()
        cv2.


def add_camera(cameras, csv_files,intrinsic_file, extrinsic_file)
    return camera.append(Camera(csv_file, read_marker_position_csv(csv_file), read_cameras_intrinsic_properties(intrinsic_file), read_cameras_extrinsic_properties(extrinsic_file)))

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
