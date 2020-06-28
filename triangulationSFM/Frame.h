//
// Created by lmikolas on 2/4/20.
//

#ifndef TRIANGULATION_FRAME_H
#define TRIANGULATION_FRAME_H


#include <vector>
#include <memory>
#include "Marker.h"

class Frame {
    int frameNumber;
    std::shared_ptr<std::vector<std::shared_ptr<Marker>>> markers;
public:
    Frame(int frameNumber, const std::shared_ptr<std::vector<std::shared_ptr<Marker>>> &markers);

    const std::shared_ptr<std::vector<std::shared_ptr<Marker>>> &getMarkers() const;

    void setMarkers(const std::shared_ptr<std::vector<std::shared_ptr<Marker>>> &markers);

    int getFrameNumber() const;

    void setFrameNumber(int frameNumber);

};


#endif //TRIANGULATION_FRAME_H
