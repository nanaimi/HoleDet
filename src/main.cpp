//
// Created by hanlonm on 06.04.22.
//
#include "HoleDetector.h"

int main (int argc, char** argv)
{
    HoleDetector holeDetector ("/home/hanlonm/HoleDet/Data/hololens.pcd");
    holeDetector.detectHoles();
    holeDetector.visualize();
}