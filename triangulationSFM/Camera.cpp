//
// Created by lmikolas on 2/4/20.
//

#include <opencv2/opencv.hpp>
#include "Camera.h"

void Camera::setRotation(double x, double y, double z) {
    cv::Mat m = cv::Mat(3, 1, CV_64F);
    cv::Mat rotationMatrix = cv::Mat(3,3, CV_64F);
    m.at<double>(0,0) = x;
    m.at<double>(1,0) = y;
    m.at<double>(2,0) = z;
    cv::Rodrigues(m, rotationMatrix);
    Camera::setRotation(std::make_shared<cv::Mat>(rotationMatrix));
}

void Camera::setTranslation(double x, double y, double z) {
    std::shared_ptr<cv::Mat> m = std::make_shared<cv::Mat>(3, 1, CV_64F);
    m->at<double>(0,0) = x;
    m->at<double>(1,0) = y;
    m->at<double>(2,0) = z;
    Camera::setTranslation(m);
}

void Camera::setCameraMatrix(double fx, double fy, double cx, double cy) {
    std::shared_ptr<cv::Mat> m = std::make_shared<cv::Mat>(3, 3, CV_64F);
    m->at<double>(0,0) = fx;
    m->at<double>(1,0) = 0;
    m->at<double>(2,0) = 0;
    m->at<double>(0,1) = 0;
    m->at<double>(1,1) = fy;
    m->at<double>(2,1) = 0;
    m->at<double>(0,2) = cx;
    m->at<double>(1,2) = cy;
    m->at<double>(2,2) = 1;
    Camera::setCameraMatrix(m);
}

void Camera::setDistortionCoefficients(double k1, double k2, double p1, double p2, double k3) {
    std::shared_ptr<cv::Mat> m = std::make_shared<cv::Mat>(1, 5, CV_64F);
    m->at<double>(0,0) = k1;
    m->at<double>(0,1) = k2;
    m->at<double>(0,2) = p1;
    m->at<double>(0,3) = p2;
    m->at<double>(0,4) = k3;
    Camera::setDistortionCoefficients(m);
}

int Camera::getNumber() const {
    return number;
}

void Camera::setNumber(int number) {
    Camera::number = number;
}

double Camera::getReprojectionError() const {
    return reprojectionError;
}

void Camera::setReprojectionError(double reprojectionError) {
    Camera::reprojectionError = reprojectionError;
}

const std::shared_ptr<std::vector<std::shared_ptr<Frame>>> &Camera::getFrames() const {
    return frames;
}

void Camera::setFrames(const std::shared_ptr<std::vector<std::shared_ptr<Frame>>> &frames) {
    Camera::frames = frames;
}

const std::shared_ptr<cv::Mat> &Camera::getRotation() const {
    return rotation;
}

void Camera::setRotation(const std::shared_ptr<cv::Mat> &rotation) {
    Camera::rotation = rotation;
}

const std::shared_ptr<cv::Mat> &Camera::getTranslation() const {
    return translation;
}

void Camera::setTranslation(const std::shared_ptr<cv::Mat> &translation) {
    Camera::translation = translation;
}

const std::shared_ptr<cv::Mat> &Camera::getRelativeRotation() const {
    return relativeRotation;
}

void Camera::setRelativeRotation(const std::shared_ptr<cv::Mat> &relativeRotation) {
    Camera::relativeRotation = relativeRotation;
}

const std::shared_ptr<cv::Mat> &Camera::getRelativeTranslation() const {
    return relativeTranslation;
}

void Camera::setRelativeTranslation(const std::shared_ptr<cv::Mat> &relativeTranslation) {
    Camera::relativeTranslation = relativeTranslation;
}

const std::shared_ptr<cv::Mat> &Camera::getCameraMatrix() const {
    return cameraMatrix;
}

void Camera::setCameraMatrix(const std::shared_ptr<cv::Mat> &cameraMatrix) {
    Camera::cameraMatrix = cameraMatrix;
}

const std::shared_ptr<cv::Mat> &Camera::getDistortionCoefficients() const {
    return distortionCoefficients;
}

void Camera::setDistortionCoefficients(const std::shared_ptr<cv::Mat> &distortionCoefficients) {
    Camera::distortionCoefficients = distortionCoefficients;
}

Camera::~Camera() {

}

const std::shared_ptr<cv::Mat> &Camera::getProjectionMatrix() const {
    return projectionMatrix;
}

void Camera::setProjectionMatrix(const std::shared_ptr<cv::Mat> &projectionMatrix) {
    Camera::projectionMatrix = projectionMatrix;
}
