//
// Created by hanlonm on 06.04.22.
//
#include "HoleDetector.h"
namespace fs = boost::filesystem;

int main (int argc, char** argv)
{

    std::string data_path = "/../data/";
    fs::path current = fs::current_path();
    auto path = current.string() + data_path;

    HoleDetector holeDetector (path + "hololens.pcd",
                               path + "floorplan.jpg" );

    holeDetector.getFloorplanCloud(true, path + "floorplan.pcd");
    holeDetector.detectHoles();
    holeDetector.visualize();
}