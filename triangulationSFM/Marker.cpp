//
// Created by lmikolas on 2/4/20.
//

#include "Marker.h"

Marker::Marker(double x, double y, double likelihood, int markerKey) : x(x), y(y), likelihood(likelihood),
                                                                       markerKey(markerKey) {}

double Marker::getX() const {
    return x;
}

void Marker::setX(double x) {
    Marker::x = x;
}

double Marker::getY() const {
    return y;
}

void Marker::setY(double y) {
    Marker::y = y;
}

double Marker::getLikelihood() const {
    return likelihood;
}

void Marker::setLikelihood(double likelihood) {
    Marker::likelihood = likelihood;
}

int Marker::getMarkerKey() const {
    return markerKey;
}

void Marker::setMarkerKey(int markerKey) {
    Marker::markerKey = markerKey;
}

void Marker::setWorldCoord(double wx, double wy, double wz) {
    Marker::wx = wx;
    Marker::wy = wy;
    Marker::wz = wz;
}

double Marker::getWx() const {
    return wx;
}

double Marker::getWy() const {
    return wy;
}

double Marker::getWz() const {
    return wz;
}

Marker::~Marker() {

}