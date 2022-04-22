//
// Created by hanlonm on 06.04.22.
//
#include "HoleDetector.h"
namespace fs = boost::filesystem;

int main (int argc, char** argv)
{

    fs::path path = fs::current_path();
    auto config_path = path.string() + "/cfg/";
    auto data_path = path.string() + "/data/talstrasse/";

    HoleDetector holeDetector (path.string(), config_path + "config.yaml");

    holeDetector.getFloorplanCloud(true, data_path + "Floorplan/floorplan.pcd");
    holeDetector.detectHoles();
    holeDetector.visualize();
}