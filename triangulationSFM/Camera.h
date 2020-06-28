//
// Created by lmikolas on 2/4/20.
//

#ifndef TRIANGULATION_CAMERA_H
#define TRIANGULATION_CAMERA_H


#include <regex>
#include <opencv2/core/mat.hpp>
#include "Frame.h"

class Camera {
    int number;
    double reprojectionError;
    std::shared_ptr<std::vector<std::shared_ptr<Frame>>> frames;
    std::shared_ptr<cv::Mat> rotation;
    std::shared_ptr<cv::Mat> translation;
    std::shared_ptr<cv::Mat> relativeRotation;
    std::shared_ptr<cv::Mat> relativeTranslation;
    std::shared_ptr<cv::Mat> cameraMatrix;
    std::shared_ptr<cv::Mat> projectionMatrix;
public:
    const std::shared_ptr<cv::Mat> &getProjectionMatrix() const;

    void setProjectionMatrix(const std::shared_ptr<cv::Mat> &projectionMatrix);

private:
    std::shared_ptr<cv::Mat> distortionCoefficients;
public:
    int getNumber() const;

    void setNumber(int number);

    double getReprojectionError() const;

    void setReprojectionError(double reprojectionError);

    const std::shared_ptr<std::vector<std::shared_ptr<Frame>>> &getFrames() const;

    void setFrames(const std::shared_ptr<std::vector<std::shared_ptr<Frame>>> &frames);

    const std::shared_ptr<cv::Mat> &getRotation() const;

    void setRotation(const std::shared_ptr<cv::Mat> &rotation);

    const std::shared_ptr<cv::Mat> &getTranslation() const;

    void setTranslation(const std::shared_ptr<cv::Mat> &translation);

    const std::shared_ptr<cv::Mat> &getRelativeRotation() const;

    void setRelativeRotation(const std::shared_ptr<cv::Mat> &relativeRotation);

    const std::shared_ptr<cv::Mat> &getRelativeTranslation() const;

    void setRelativeTranslation(const std::shared_ptr<cv::Mat> &relativeTranslation);

    const std::shared_ptr<cv::Mat> &getCameraMatrix() const;

    void setCameraMatrix(const std::shared_ptr<cv::Mat> &cameraMatrix);

    const std::shared_ptr<cv::Mat> &getDistortionCoefficients() const;

    void setDistortionCoefficients(const std::shared_ptr<cv::Mat> &distortionCoefficients);

    void setRotation(double x, double y, double z);

    void setTranslation(double x, double y, double z);

    void setCameraMatrix(double fx, double fy, double cx, double cy);

    void setDistortionCoefficients(double k1, double k2, double p1, double p2, double k3);

    virtual ~Camera();
};


#endif //TRIANGULATION_CAMERA_H
