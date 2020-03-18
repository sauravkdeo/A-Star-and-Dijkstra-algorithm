/**
 * @file      include/map.h
 * @brief     Header file for building map
 * @author    Saurav Kumar
 * @copyright 2019
 *
 **BSD 3-Clause License
 *
 *Copyright (c) 2018, Saurav Kumar
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef INCLUDE_MAP_H_
#define INCLUDE_MAP_H_
#include <../include/coord.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <fstream>

using std::vector;

using std::priority_queue;

class Map {
 private:
  int option;
  int x;
  int y;
  int resolution;
  int robotradius;
  int clearance;
  Coord startingCoord;
  Coord endCoord;
  int gridmapheight;
  int gridmapwidth;
  vector<vector<int>> gridmap;
  cv::Mat frame;


 public:
  Map(int, int, int, int);
  ~Map();

  void definepolygonobstacle(int, vector<vector<int>>);
  void definecurveobstacle(Coord, int, int);
  void addobstaclebuffer();
  void joinconcaveshapes();
  void formobstacleboundary();

  int getgridmapheight();
  int getgridmapwidth();

  void setgridmap(int, int, int);
  int getgridmap(int, int);

  void Printmap();
  void savemap();
  void rotatemap();

  void seemap();
  void updatemap(Coord, Coord, int);
  void fillmap(Coord , int);

  void buildmap();
  Coord getendcoord();
  Coord getstartingCoord();
  int getoption();
  int getresolution();
  void setresolution(int);
  //  void setinputresolution(int);
  bool checksanity(Coord);
  bool checkboundary(Coord);
  bool takeinput(Coord , Coord);

  void setx(int);
  int getx();
  void sety(int);
  int gety();
};


#endif  // INCLUDE_MAP_H_
