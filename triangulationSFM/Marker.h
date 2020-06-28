//
// Created by lmikolas on 2/4/20.
//

#ifndef TRIANGULATION_MARKER_H
#define TRIANGULATION_MARKER_H


class Marker {
    double x;
    double y;
    double likelihood;
    int markerKey;
public:
    double getWx() const;

    double getWy() const;

    double getWz() const;

private:
    double wx, wy, wz;
public:
    Marker(double x, double y, double likelihood, int markerKey);

    double getX() const;

    void setX(double x);

    double getY() const;

    void setY(double y);

    double getLikelihood() const;

    void setLikelihood(double likelihood);

    int getMarkerKey() const;

    void setMarkerKey(int markerKey);

    void setWorldCoord(double wx, double wy, double wz);

    virtual ~Marker();

};


#endif //TRIANGULATION_MARKER_H
