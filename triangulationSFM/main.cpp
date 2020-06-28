#include <iostream>
#include <opencv2/opencv.hpp>
#include "TriangulationSystem.h"


int main() {
    // set intrinsic and extrinsics path
    std::string path_intrinsics = "../intrinsics.json";
    std::string path_extrinsics = "../extrinsics.json";
    // set frames paths
    std::map<std::string, std::string>  framePaths;
    framePaths["0"] = "../deeplabcut-results/scene-2-cam-0DLC_resnet50_pf-markerless3dSep11shuffle1_550000filtered.csv";
    framePaths["1"] = "../deeplabcut-results/scene-2-cam-1DLC_resnet50_pf-markerless3dSep11shuffle1_550000filtered.csv";
    framePaths["2"] = "../deeplabcut-results/scene-2-cam-2DLC_resnet50_pf-markerless3dSep11shuffle1_550000filtered.csv";
    framePaths["3"] = "../deeplabcut-results/scene-2-cam-3DLC_resnet50_pf-markerless3dSep11shuffle1_550000filtered.csv";
    framePaths["4"] = "../deeplabcut-results/scene-2-cam-4DLC_resnet50_pf-markerless3dSep11shuffle1_550000filtered.csv";
    framePaths["5"] = "../deeplabcut-results/scene-2-cam-5DLC_resnet50_pf-markerless3dSep11shuffle1_550000filtered.csv";
    framePaths["6"] = "../deeplabcut-results/scene-2-cam-6DLC_resnet50_pf-markerless3dSep11shuffle1_550000filtered.csv";
    framePaths["7"] = "../deeplabcut-results/scene-2-cam-7DLC_resnet50_pf-markerless3dSep11shuffle1_550000filtered.csv";

    TriangulationSystem triangulationSystem = TriangulationSystem();
    // read camera data from JSON
    triangulationSystem.setCameras(path_intrinsics, path_extrinsics);
    // add frames to each camera
    triangulationSystem.setFrames(framePaths);
    // triangulatePoints
    triangulationSystem.triangulatePoints();
    // export for showing
    triangulationSystem.exportXYZ();
    return 0;
}
