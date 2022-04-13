//
// Created by hanlonm on 06.04.22.
//
#include "HoleDetector.h"

int main (int argc, char** argv)
{
    HoleDetector holeDetector ("/home/maurice/ETH/HoleDet/data/hololens.pcd",
                               "/home/maurice/ETH/HoleDet/data/floorplan.jpg" );
    holeDetector.detectHoles();
    holeDetector.getFloorplanCloud(false, "/home/maurice/ETH/HoleDet/data/floorplan.pcd");
    holeDetector.visualize();
}