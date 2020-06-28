//
// Created by lmikolas on 2/4/20.
//

#include "Frame.h"

const std::shared_ptr<std::vector<std::shared_ptr<Marker>>> &Frame::getMarkers() const {
    return markers;
}

void Frame::setMarkers(const std::shared_ptr<std::vector<std::shared_ptr<Marker>>> &markers) {
    Frame::markers = markers;
}

int Frame::getFrameNumber() const {
    return frameNumber;
}

void Frame::setFrameNumber(int frameNumber) {
    Frame::frameNumber = frameNumber;
}

Frame::Frame(int frameNumber, const std::shared_ptr<std::vector<std::shared_ptr<Marker>>> &markers) : frameNumber(frameNumber),
                                                                                     markers(markers) {}
