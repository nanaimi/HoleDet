//
// Created by hanlonm on 06.04.22.
//
#include "HoleDetector.h"

int main (int argc, char** argv)
{
    HoleDetector holeDetector ("/home/hanlonm/HoleDet/Data/hololens.pcd",
                               "/home/hanlonm/HoleDet/Data/floorplan.jpg" );
    holeDetector.detectHoles();
    holeDetector.getFloorplanCloud(true, "/home/hanlonm/HoleDet/Data/floorplan.pcd");
    holeDetector.visualize();
}