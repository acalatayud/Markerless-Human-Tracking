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


class Filter:
    def __init__(self, filter):
        self.first = True
        self.filter = filter


def get_best_stereo_cameras_for_marker(cameras, frame, marker):
    max_index_next = 0
    max_index = len(cameras) - 1
    max_likelihood = cameras[0].frames[frame].markers[marker].likelihood + cameras[max_index].frames[frame].markers[marker].likelihood
    min_relative_rotation = np.abs(cv2.Rodrigues(np.matmul(cameras[max_index].R, cameras[0].R.T))[0][1])
    for i in range(0, len(cameras) - 2):
        current_likelihood = cameras[i].frames[frame].markers[marker].likelihood + cameras[i + 1].frames[frame].markers[marker].likelihood
        relative_rotation = np.abs(cv2.Rodrigues(np.matmul(cameras[i].R,  cameras[i + 1].R.T))[0][1])
        if current_likelihood > max_likelihood or (current_likelihood == max_likelihood and (relative_rotation <  min_relative_rotation)):
            max_index = i
            max_index_next = i + 1
            max_likelihood = current_likelihood
    stereo_cameras = [None, None]
    stereo_cameras = [cameras[max_index], cameras[max_index_next]]
    return stereo_cameras


def get_front_back_cameras_for_marker(cameras, frame, marker):
    front = [cameras[3], cameras[0]]
    back = [cameras[1], cameras[2]]
    if front[0].frames[frame].markers[marker].likelihood > 0.7 and front[1].frames[frame].markers[marker].likelihood > 0.7:
        return front
    return back

def get_stereo(cameras, frame, marker, index):
    if index == len(cameras) - 1:
        index_next = 0
    else:
        index_next = index + 1
    stereo_cameras = [cameras[index], cameras[index_next]]
    return stereo_cameras

def get_stereo_pairs(cameras, frame, marker):
    camera_selection_cutoff = 0.999
    camera_pairs = []
    next_index = None
    for i in range(len(cameras) - 1):
        if i == len(cameras) - 1:
            next_index = 0
        else:
            next_index = i + 1
        if cameras[i].frames[frame].markers[marker].likelihood > camera_selection_cutoff and cameras[next_index].frames[frame].markers[marker].likelihood > camera_selection_cutoff:
            camera_pairs.append([cameras[i], cameras[next_index]])
    return camera_pairs

def triangulate_points(cameras, filtered_applied):
    if len(cameras) < 2:
        raise Exception('Triangulation process needs at least two cameras')
    number_of_frames = len(cameras[0].frames)
    number_of_markers = len(cameras[0].frames[0].markers)
    image_size = (640, 480)
    triangulated_frames = []
    latest_triangulated_point = []
    min_likelihood = 0.7

    # set up filter values
    dt = 1.0 / 24
    transition_matrix = np.eye(9, dtype=np.float32)
    transition_matrix[0][3] = dt
    transition_matrix[0][6] = 0.5 * dt * dt
    transition_matrix[1][4] = dt
    transition_matrix[1][7] = 0.5 * dt * dt
    transition_matrix[2][5] = dt
    transition_matrix[2][8] = 0.5 * dt * dt
    measurement_matrix = np.array(
        [(1, 0, 0, 0, 0, 0, 0, 0, 0), (0, 1, 0, 0, 0, 0, 0, 0, 0), (0, 0, 1, 0, 0, 0, 0, 0, 0)], dtype=np.float32)

    # init filters for all markers tracked
    filters = []
    for i in range(number_of_markers - 1):
        kalman_filter = cv2.KalmanFilter(9, 3, 0)
        kalman_filter.transitionMatrix = transition_matrix
        kalman_filter.measurementMatrix = measurement_matrix
        filters.append(Filter(kalman_filter))
    # triangulate points individually
    for i in range(number_of_frames - 1):
        triangulated_markers = []
        for j in range(number_of_markers - 1):
            camera_pairs = get_stereo_pairs(cameras, i, j)
            total_likelihood = 0.0
            triangulated_average = [0, 0, 0 ]
            point_triangulated = False
            for stereo_cameras in camera_pairs:
                total_likelihood += stereo_cameras[0].frames[i].markers[j].likelihood + \
                                    stereo_cameras[1].frames[i].markers[j].likelihood
            for stereo_cameras in camera_pairs:
                relative_translation = np.matmul(stereo_cameras[0].R, -np.matmul(stereo_cameras[1].R.T, stereo_cameras[1].T)
                                       + np.matmul(stereo_cameras[0].R.T, stereo_cameras[0].T))
                relative_rotation = np.matmul(stereo_cameras[1].R, stereo_cameras[0].R.T)
                r1, r2, p1, p2, q, roi1, roi2 = cv2.stereoRectify(stereo_cameras[0].camera_matrix,
                                                                  stereo_cameras[0].distortion_coefficients,
                                                                  stereo_cameras[1].camera_matrix,
                                                                  stereo_cameras[1].distortion_coefficients, image_size,
                                                                  relative_rotation, relative_translation)
                R = [r1, r2]
                P = [p1, p2]
                undistorted_points = [None, None]
                marker_key = stereo_cameras[0].frames[i].markers[j].marker_key
                if stereo_cameras[0].frames[i].markers[j].likelihood > min_likelihood and stereo_cameras[1].frames[i].markers[j].likelihood > min_likelihood:
                    point_triangulated = True
                    for index, camera in enumerate(stereo_cameras):
                        marker = camera.frames[i].markers[j]
                        point = np.float64([marker.x, marker.y])
                        undistorted_point = cv2.undistortPoints(point, camera.camera_matrix,
                                                                camera.distortion_coefficients, R=R[index], P=P[index])
                        undistorted_points[index] = undistorted_point

                    triangulated_hc_camera_frame = cv2.triangulatePoints(p1, p2, undistorted_points[0], undistorted_points[1])
                    triangulated_hc_camera_frame = np.divide(triangulated_hc_camera_frame, triangulated_hc_camera_frame[3][0])
                    triangulated_ec_camera_frame = [triangulated_hc_camera_frame[0][0], triangulated_hc_camera_frame[1][0], triangulated_hc_camera_frame[2][0]]
                    triangulated_ec_world_frame = np.matmul(-stereo_cameras[0].R.T, np.matmul(r1.T, triangulated_ec_camera_frame) + stereo_cameras[0].T)

                    triangulated_average += triangulated_ec_world_frame * ((stereo_cameras[0].frames[i].markers[j].likelihood + stereo_cameras[1].frames[i].markers[j].likelihood) / total_likelihood)

            if point_triangulated:
                if filtered_applied:
                    triangulated_ec_world_frame_formated = np.array(([triangulated_average]), np.float32).T
                    # compensate for the initial state set to 0,0,0 in opencv kalman filter
                    if filters[j].first:
                        for l in range(100):
                            prediction = filters[j].filter.predict()
                            estimated = filters[j].filter.correct(triangulated_ec_world_frame_formated)
                        filters[j].first = False
                    prediction = filters[j].filter.predict()
                    estimated = filters[j].filter.correct(triangulated_ec_world_frame_formated)
                    # append triangulated point
                    triangulated_markers.append(
                        {'point': np.array([estimated[0][0], estimated[1][0], estimated[2][0]]), 'marker': marker_key,
                         'cameras': str(stereo_cameras[0].number) + str(stereo_cameras[1].number)})
                else:
                    triangulated_markers.append({'point': triangulated_average, 'marker': marker_key,
                                                         'cameras': str(stereo_cameras[0].number) + str(
                                                             stereo_cameras[1].number)})

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


def export_xyz(triangulated_frames, cameras):
    with open('points.xyz', mode='w', newline='') as xyz_file:
        for index, frame in enumerate(triangulated_frames):
            xyz_file.write(str(len(triangulated_frames[index]) + len(cameras) + 1) + "\n")
            xyz_file.write("\n")
            for camera_index, camera in enumerate(cameras):
                translation = -np.matmul(camera.R.T, camera.T)
                camera_orientation = np.matmul(camera.R.T, [0, 0, 1])
                xyz_file.write(str(100 + camera_index + 1) + ' ' + str(translation[0]) + ' ' + str(translation[1])
                               + ' ' + str(translation[2]) + ' '
                               + str(camera_orientation[0]) + ' ' + str(camera_orientation[1]) + ' ' + str(camera_orientation[2]) + ' ' + str(0) + '\n')
            xyz_file.write(str(100) +  ' ' + str(0) + ' ' + str(0)
                           + ' ' + str(0) + ' '
                           + str(0) + ' ' + str(0) + ' ' + str(0) + ' ' + str(0) + '\n')
            for marker in frame:
                point = marker['point']
                marker_key = constants.MARKER_INDICES[marker['marker']]
                xyz_file.write(str(marker_key) + ' ' + str(point[0]) + ' ' + str(point[1])
                               + ' ' + str(point[2]) + ' '
                               + str(0) + ' ' + str(0) + ' ' + str(0) + ' ' + marker['cameras'] + '\n')


# EXAMPLE CALL: CHANGE PATHS
path = r'C:\Users\lmikolas\Downloads'
path2 = r'C:\Users\lmikolas\Downloads'


frame_paths = [{'camera': 0, 'path': path + r'\\scene-2-cam-0DeepCut_resnet101_pf-markerless3dSep11shuffle1_500000filtered.csv'},
                {'camera': 1, 'path': path + r'\\scene-2-cam-1DeepCut_resnet101_pf-markerless3dSep11shuffle1_500000filtered.csv'},
                {'camera': 2, 'path': path + r'\\scene-2-cam-2DeepCut_resnet101_pf-markerless3dSep11shuffle1_500000filtered.csv'},
                {'camera': 3, 'path': path + r'\\scene-2-cam-3DeepCut_resnet101_pf-markerless3dSep11shuffle1_500000filtered.csv'},
                {'camera': 4, 'path': path + r'\\scene-2-cam-4DeepCut_resnet101_pf-markerless3dSep11shuffle1_500000filtered.csv'},
                {'camera': 5, 'path': path + r'\\scene-2-cam-5DeepCut_resnet101_pf-markerless3dSep11shuffle1_500000filtered.csv'},
                {'camera': 6, 'path': path + r'\\scene-2-cam-6DeepCut_resnet101_pf-markerless3dSep11shuffle1_500000filtered.csv'},
                {'camera': 7, 'path': path + r'\\scene-2-cam-7DeepCut_resnet101_pf-markerless3dSep11shuffle1_500000filtered.csv'}]
cameras = get_cameras(path2 + r'\intrinsics.json', path2 + r'\extrinsics.json')
cameras = add_frames(frame_paths, cameras)

triangulated_frames = triangulate_points(cameras, True)
export_csv(triangulated_frames)
export_xyz(triangulated_frames, cameras)
