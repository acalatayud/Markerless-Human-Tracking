//
// Created by lmikolas on 2/4/20.
//

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/sfm.hpp>
#include "TriangulationSystem.h"
#include "json.hpp"


void TriangulationSystem::setCameras(std::string pathInstrinsics, std::string pathExtrinsics) {
    // Set intrinsics
    std::ifstream i(pathInstrinsics);
    nlohmann::json j;
    i >> j;
    nlohmann::json camerasIntrinsics = j["calibration"]["cameras"];
    TriangulationSystem::cameras = std::make_shared<std::vector<std::shared_ptr<Camera>>>();
    for (auto it = camerasIntrinsics.begin(); it != camerasIntrinsics.end(); ++it)
    {
        std::shared_ptr<Camera> camera = std::make_shared<Camera>();
        std::cout << "Camera number: " << it.value()["cameraNumber"] << " intrinsic being set.\n";
        camera->setNumber(std::stoi(it.value()["cameraNumber"].get<std::string>()));
        camera->setReprojectionError(std::stod(it.value()["reprojectionError"].get<std::string>()));
        camera->setCameraMatrix(std::stod((std::string)it.value()["calibrationMatrix"]["fx"]), std::stod((std::string)it.value()["calibrationMatrix"]["fy"]),
        std::stod((std::string)it.value()["calibrationMatrix"]["cx"]), std::stod((std::string)it.value()["calibrationMatrix"]["cy"]));
        camera->setDistortionCoefficients(std::stod((std::string)it.value()["distortionCoefficients"]["k1"]), std::stod((std::string)it.value()["distortionCoefficients"]["k2"]),
                                         std::stod((std::string)it.value()["distortionCoefficients"]["p1"]), std::stod(it.value()["distortionCoefficients"]["p2"].get<std::string>()),
                                         std::stod((std::string)it.value()["distortionCoefficients"]["k3"]));
        TriangulationSystem::cameras->push_back(camera);
    }
    // Set extrinsics
    std::ifstream i2(pathExtrinsics);
    i2 >> j;
    nlohmann::json camerasExtrinsics = j["calibration"]["cameras"];
    int cameraCounter = 0;
    for (auto it = camerasExtrinsics.begin(); it != camerasExtrinsics.end(); ++it)
    {
        std::shared_ptr<Camera> camera = TriangulationSystem::cameras->at(cameraCounter++);
        std::cout << "Camera number: " << it.value()["cameraNumber"] << " extrinsic being set.\n";
        camera->setTranslation(std::stod((std::string)it.value()["translationVector"]["x"]), std::stod((std::string)it.value()["translationVector"]["y"]),
                std::stod((std::string)it.value()["translationVector"]["z"]));
        camera->setRotation(std::stod((std::string)it.value()["rotationVector"]["x"]), std::stod((std::string)it.value()["rotationVector"]["y"]),
                              std::stod((std::string)it.value()["rotationVector"]["z"]));

    }
}

void TriangulationSystem::setFrames(const std::map<std::string, std::string>& framePath) {
    for (std::pair<std::string, std::string> myPair : framePath) {
        std::cout << "Camera number " << myPair.first << " frames of path " << myPair.second << " analyzed.\n";
        std::vector<std::vector<std::string>> rows = getRowsOfCSV(myPair.second);
        std::shared_ptr<Camera> camera = TriangulationSystem::cameras->at(std::stoi(myPair.first));
        camera->setFrames(std::make_shared<std::vector<std::shared_ptr<Frame>>>());
        for(int i = 3; i < rows.size(); i++) {
            auto markersOfFrame = std::make_shared<std::vector<std::shared_ptr<Marker>>>();
            for (int j = 1; j <= rows[i].size() - 3; j += 3) {
                markersOfFrame->emplace_back(std::make_shared<Marker>(
                        std::stod(rows[i][j]),
                        std::stod(rows[i][j+1]),
                        std::stod(rows[i][j+2]),
                        j/3));
            }
            camera->getFrames()->emplace_back(std::make_shared<Frame>(i - 3, markersOfFrame));
        }
    }
}

std::shared_ptr<std::vector<std::shared_ptr<Camera>>> TriangulationSystem::getBestCameras(int frame, int marker) {
    auto bestCameras = std::make_shared<std::vector<std::shared_ptr<Camera>>>();
    double min_likelihood = 0.7;
    while (bestCameras->size() < 2) {
        for (const auto &camera : *cameras) {
            double likelihood = camera->getFrames()->at(frame)->getMarkers()->at(marker)->getLikelihood();
            if (likelihood > min_likelihood and std::find(bestCameras->begin(), bestCameras->end(), camera) == bestCameras->end()) {
                bestCameras->emplace_back(camera);
            }
        }
        min_likelihood -= 0.05;
    }
    return bestCameras;
}

void TriangulationSystem::triangulatePoints() {
    if (TriangulationSystem::cameras->size() < 2) {
        std::cout << "Not enough cameras for triangulation.\n";
        exit(1);
    }
    std::shared_ptr<Camera> first = TriangulationSystem::cameras->at(0);
    first->setRelativeTranslation(std::make_shared<cv::Mat>(3,1, CV_64F, double(0)));
    first->setRelativeRotation(std::make_shared<cv::Mat>(cv::Mat::eye(3, 3, CV_64F)));
    for(int i = 1; i < TriangulationSystem::cameras->size(); i++) {
        std::shared_ptr<Camera> second = TriangulationSystem::cameras->at(i);
        cv::Mat aux1 = second->getRotation()->t() * *(second->getTranslation());
        cv::Mat aux2 = first->getRotation()->t() * *(first->getTranslation());
        second->setRelativeTranslation(std::make_shared<cv::Mat>(*(first->getRotation()) * (-aux1 + aux2)));
        second->setRelativeRotation(std::make_shared<cv::Mat>(*(second->getRotation()) * first->getRotation()->t()));
    }
    // set projectionMatrix
    for(const auto& camera : *cameras) {
        camera->setProjectionMatrix(std::make_shared<cv::Mat>(3,4, CV_64F, double(0)));
        cv::sfm::projectionFromKRt(*(camera->getCameraMatrix()), *(camera->getRotation()), *(camera->getTranslation()), *(camera->getProjectionMatrix()));
    }
    // undistort, and triangulate
    int numberOfFrames = cameras->at(0)->getFrames()->size();
    int numberOfMarkers = cameras->at(0)->getFrames()->at(0)->getMarkers()->size();

    for (int frame = 0; frame < numberOfFrames; frame++) {
        std::shared_ptr<std::vector<std::shared_ptr<cv::Point3d>>> frames = std::make_shared<std::vector<std::shared_ptr<cv::Point3d>>>();
        for (int marker = 0; marker < numberOfMarkers; marker++) {

            int visible_cameras = 0;
            for(const auto& camera : *cameras) {
                if (camera->getFrames()->at(frame)->getMarkers()->at(marker)->getLikelihood() > 0)
                    visible_cameras++;
            }

            if (visible_cameras < 2)
                continue;

            // vector containing undistored points
            std::vector<cv::Mat> undistortedPoints;
            std::vector<cv::Mat> projectionMatrices;
            std::shared_ptr<cv::Point3d> triangulatedPoint  = std::make_shared<cv::Point3d>();
            cv::Mat triangulatedArray;
            auto bestCameras = TriangulationSystem::getBestCameras(frame, marker);
            double likelihood = 0;
            int cameraCount = 0;
            for (const auto& camera : *bestCameras) {
                std::shared_ptr<Frame> frameToAnalyze = camera->getFrames()->at(frame);
                std::shared_ptr<Marker> markerToAnalyze = frameToAnalyze->getMarkers()->at(marker);
                std::shared_ptr<cv::Point2d> undistortedPoint = std::make_shared<cv::Point2d>();

                std::vector<cv::Point2d> undistortedPointOutput = {*undistortedPoint};
                std::vector<cv::Point2d> distortedPointInput = {
                        cv::Point2d(markerToAnalyze->getX(), markerToAnalyze->getY())};
                cv::undistortPoints(distortedPointInput, undistortedPointOutput, *(camera->getCameraMatrix()),
                                    *(camera->getDistortionCoefficients()), cv::noArray(),
                                    *(camera->getCameraMatrix()));
                cv::Mat vec = cv::Mat(2, 1, CV_64F);
                vec.at<double>(0, 0) = undistortedPointOutput.at(0).x;
                vec.at<double>(1, 0) = undistortedPointOutput.at(0).y;
                undistortedPoints.emplace_back(vec);
                projectionMatrices.emplace_back(*(camera->getProjectionMatrix()));
                likelihood += markerToAnalyze->getLikelihood();
                cameraCount++;
            }
            cv::sfm::triangulatePoints(undistortedPoints, projectionMatrices, triangulatedArray);
            if (likelihood / cameraCount >= 0.7) {
                triangulatedPoint->x = triangulatedArray.at<double>(0, 0);
                triangulatedPoint->y = triangulatedArray.at<double>(1, 0);
                triangulatedPoint->z = triangulatedArray.at<double>(2, 0);
                frames->emplace_back(triangulatedPoint);
            }
        }
        TriangulationSystem::triangulatedFrames.emplace_back(frames);
    }
}

void TriangulationSystem::exportXYZ() {
    std::ofstream MyFile("points.xyz");
    for (const auto& frame : TriangulationSystem::triangulatedFrames) {
        MyFile << frame->size() << "\n\n";
        int number = 1;
        for (const auto& marker : *frame) {
            MyFile << number << " " << marker->x << " " << marker->y << " " << marker->z << "\n";
            number++;
        }
    }
    MyFile.close();
}

void TriangulationSystem::exportCSV() {
    std::ofstream MyFile("points.csv");
    int frameNumber = 0;
    for (const auto& frame : TriangulationSystem::triangulatedFrames) {
        int number = 1;
        for (const auto& marker : *frame) {
            MyFile << frameNumber << "," << number << "," << marker->x << "," << marker->y << "," << marker->z << "\n";
            number++;
        }
        frameNumber++;
    }
    MyFile.close();
}
