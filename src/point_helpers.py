from src import constants
from src.constants import MARKER_INDICES as INDICES
import csv
import json
import cv2
import numpy as np


class Camera:
    def __init__(self, camera_number, intrinsic_properties, extrinsic_properties):
        self.number = camera_number
        self.frames = None
        r = extrinsic_properties['rotationVector']
        self.Rvec = get_3d_vector(r)
        self.R = cv2.Rodrigues(self.Rvec)[0]
        translation = extrinsic_properties['translationVector']
        self.T = np.array(get_3d_vector(translation))
        matrix_parameters = intrinsic_properties['calibrationMatrix']
        self.camera_matrix = np.matrix([[matrix_parameters['fx'], 0, matrix_parameters['cx']],
                                       [0, matrix_parameters['fy'], matrix_parameters['cy']],
                                       [0, 0, 1]], dtype=np.float64)
        coef = intrinsic_properties['distortionCoefficients']
        self.distortion_coefficients = np.float64([coef['k1'], coef['k2'], coef['p1'], coef['p2'], coef['k3']])
        self.reprojectionError = float(intrinsic_properties['reprojectionError'])

    def add_frames(self, frames):
        self.frames = frames


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
    triangulated_frames = []
    r1 = 0
    for i in range(number_of_frames - 1):
        triangulated_markers = []
        for j in range(number_of_markers - 1):
            stereo_cameras = get_two_best_cameras_for_marker(cameras, i, j)
            relative_t = np.matmul(stereo_cameras[0].R, stereo_cameras[1].T - stereo_cameras[0].T)
            rotation = np.matmul(stereo_cameras[1].R, np.linalg.inv(stereo_cameras[0].R))
            r1, r2, p1, p2, q, roi1, roi2 = cv2.stereoRectify(stereo_cameras[0].camera_matrix,
                                                              stereo_cameras[0].distortion_coefficients,
                                                              stereo_cameras[1].camera_matrix,
                                                              stereo_cameras[1].distortion_coefficients, image_size,
                                                              rotation, relative_t)
            R = [r1, r2]
            P = [p1, p2]
            undistorted_points = [None, None]
            marker_key = stereo_cameras[0].frames[i].markers[j].marker_key
            for index, camera in enumerate(stereo_cameras):
                marker = camera.frames[i].markers[j]
                point = np.float64([marker.x, marker.y])
                undistorted_point = cv2.undistortPoints(point, camera.camera_matrix,
                                                        camera.distortion_coefficients, R=R[index], P=P[index])
                undistorted_points[index] = undistorted_point

            triangulated_hc = cv2.triangulatePoints(p1, p2, undistorted_points[0], undistorted_points[1])
            triangulated = np.divide(triangulated_hc, triangulated_hc[3][0])
            triangulated = [triangulated[0][0], triangulated[1][0], triangulated[2][0]]
            # PASS TO WORLD COORDINATES FROM RECTIFIED CAMERA COORDINATE SYSTEM?
            triangulated_wc = np.matmul(np.linalg.inv(stereo_cameras[0].R), triangulated) \
                              - stereo_cameras[0].T
            triangulated_markers.append({'point': triangulated, 'marker': marker_key})
        triangulated_frames.append(triangulated_markers)
    return triangulated_frames


def get_cameras(intrinsic_file, extrinsic_file):
    cameras = []
    intrinsic_properties = read_camera_properties(intrinsic_file)
    extrinsic_properties = read_camera_properties(extrinsic_file)
    for camera in intrinsic_properties:
        camera_number = int(camera['cameraNumber'])
        cameras.append(Camera(camera_number, intrinsic_properties[camera_number], extrinsic_properties[camera_number]))
    return cameras


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


def add_frames(paths, cameras):
    for path in paths:
        camera_number = path['camera']
        cameras[camera_number].add_frames(read_marker_position_csv(path['path']))
    return list(filter(lambda c: c.frames is not None, cameras))


def read_camera_properties(json_file):
    with open(json_file, 'r') as file:
        properties = json.load(file)

    return properties['calibration']['cameras']


def get_3d_vector(vector):
    return np.float64([float(vector['x']), float(vector['y']), float(vector['z'])])


def export_csv(triangulated_frames):
    with open('points.csv', mode='w', newline='') as csv_file:
        fields = ['frame', 'marker', 'x', 'y', 'z']
        writer = csv.DictWriter(csv_file, fieldnames=fields)

        writer.writeheader()
        for index, frame in enumerate(triangulated_frames):
            for marker in frame:
                point = marker['point']
                marker_key = marker['marker']
                writer.writerow({'frame': index, 'marker': marker_key, 'x': point[0], 'y': point[1], 'z': point[2]})


def export_xyz(triangulated_frames):
    with open('points.xyz', mode='w', newline='') as xyz_file:

        marker_number = len(triangulated_frames[0])
        for index, frame in enumerate(triangulated_frames):
            xyz_file.write(str(marker_number) + "\n")
            xyz_file.write("\n")
            for marker in frame:
                point = marker['point']
                marker_key = constants.MARKER_INDICES[marker['marker']]
                xyz_file.write(str(marker_key) + ' ' + str(point[0]) + ' ' + str(point[1])
                               + ' ' + str(point[2]) + '\n')


# EXAMPLE CALL: CHANGE PATHS
path = r'C:\Users\lmikolas\Downloads'
path2 = r'C:\Users\lmikolas\Downloads'


frame_paths = [{'camera': 7, 'path': path + r'\scene-2-cam-7DeepCut_resnet101_pf-markerless3dSep11shuffle1_500000.csv'},
         {'camera': 0, 'path': path + r'\\scene-2-cam-0DeepCut_resnet101_pf-markerless3dSep11shuffle1_500000.csv'}]
cameras = get_cameras(path2 + r'\intrinsics.json', path2 + r'\extrinsics.json')
cameras = add_frames(frame_paths, cameras)

triangulated_frames = triangulate_points(cameras)
export_csv(triangulated_frames)
export_xyz(triangulated_frames)
